#include <Arduino.h>
#include <WiFi.h> // Part of ESP32 board libraries
#include <WebServer.h> // Part of ESP32 board libraries
#include <WebSocketsServer.h> // Installed "WebSockets" library
#include <Wire.h>
#include <SparkFun_GridEYE_Arduino_Library.h>
#include <TFT_eSPI.h>

#include "interpolation.h"

#define BUTTON1_PIN 0
#define BUTTON2_PIN 35

#define MODE_OVERVIEW  0
#define MODE_SELECTION 1

#define SCREENX TFT_HEIGHT
#define SCREENY TFT_WIDTH

#define SENSOR_SIZE 8
#define AMG88xx_PIXEL_ARRAY_SIZE SENSOR_SIZE * SENSOR_SIZE
#define INTERPOLATED_SIZE 32

#define BORDER_WIDTH 2
// assume SCREENX > SCREENY – change for your case!
#define PIXSIZE ((SCREENY - 2 * BORDER_WIDTH) / INTERPOLATED_SIZE)

#define SCALE_STEPS 4

#define TEXT_X_OFFSET (INTERPOLATED_SIZE*PIXSIZE + 6*BORDER_WIDTH)

const uint16_t camColors[] = {0x480F,
	                          0x400F,0x400F,0x400F,0x4010,0x3810,0x3810,0x3810,0x3810,0x3010,0x3010,
	                          0x3010,0x2810,0x2810,0x2810,0x2810,0x2010,0x2010,0x2010,0x1810,0x1810,
	                          0x1811,0x1811,0x1011,0x1011,0x1011,0x0811,0x0811,0x0811,0x0011,0x0011,
	                          0x0011,0x0011,0x0011,0x0031,0x0031,0x0051,0x0072,0x0072,0x0092,0x00B2,
	                          0x00B2,0x00D2,0x00F2,0x00F2,0x0112,0x0132,0x0152,0x0152,0x0172,0x0192,
	                          0x0192,0x01B2,0x01D2,0x01F3,0x01F3,0x0213,0x0233,0x0253,0x0253,0x0273,
	                          0x0293,0x02B3,0x02D3,0x02D3,0x02F3,0x0313,0x0333,0x0333,0x0353,0x0373,
	                          0x0394,0x03B4,0x03D4,0x03D4,0x03F4,0x0414,0x0434,0x0454,0x0474,0x0474,
	                          0x0494,0x04B4,0x04D4,0x04F4,0x0514,0x0534,0x0534,0x0554,0x0554,0x0574,
	                          0x0574,0x0573,0x0573,0x0573,0x0572,0x0572,0x0572,0x0571,0x0591,0x0591,
	                          0x0590,0x0590,0x058F,0x058F,0x058F,0x058E,0x05AE,0x05AE,0x05AD,0x05AD,
	                          0x05AD,0x05AC,0x05AC,0x05AB,0x05CB,0x05CB,0x05CA,0x05CA,0x05CA,0x05C9,
	                          0x05C9,0x05C8,0x05E8,0x05E8,0x05E7,0x05E7,0x05E6,0x05E6,0x05E6,0x05E5,
	                          0x05E5,0x0604,0x0604,0x0604,0x0603,0x0603,0x0602,0x0602,0x0601,0x0621,
	                          0x0621,0x0620,0x0620,0x0620,0x0620,0x0E20,0x0E20,0x0E40,0x1640,0x1640,
	                          0x1E40,0x1E40,0x2640,0x2640,0x2E40,0x2E60,0x3660,0x3660,0x3E60,0x3E60,
	                          0x3E60,0x4660,0x4660,0x4E60,0x4E80,0x5680,0x5680,0x5E80,0x5E80,0x6680,
	                          0x6680,0x6E80,0x6EA0,0x76A0,0x76A0,0x7EA0,0x7EA0,0x86A0,0x86A0,0x8EA0,
	                          0x8EC0,0x96C0,0x96C0,0x9EC0,0x9EC0,0xA6C0,0xAEC0,0xAEC0,0xB6E0,0xB6E0,
	                          0xBEE0,0xBEE0,0xC6E0,0xC6E0,0xCEE0,0xCEE0,0xD6E0,0xD700,0xDF00,0xDEE0,
	                          0xDEC0,0xDEA0,0xDE80,0xDE80,0xE660,0xE640,0xE620,0xE600,0xE5E0,0xE5C0,
	                          0xE5A0,0xE580,0xE560,0xE540,0xE520,0xE500,0xE4E0,0xE4C0,0xE4A0,0xE480,
	                          0xE460,0xEC40,0xEC20,0xEC00,0xEBE0,0xEBC0,0xEBA0,0xEB80,0xEB60,0xEB40,
	                          0xEB20,0xEB00,0xEAE0,0xEAC0,0xEAA0,0xEA80,0xEA60,0xEA40,0xF220,0xF200,
	                          0xF1E0,0xF1C0,0xF1A0,0xF180,0xF160,0xF140,0xF100,0xF0E0,0xF0C0,0xF0A0,
	                          0xF080,0xF060,0xF040,0xF020,0xF800};

float pixels[SENSOR_SIZE * SENSOR_SIZE];
float dest2d[INTERPOLATED_SIZE * INTERPOLATED_SIZE];

TFT_eSPI tft = TFT_eSPI(SCREENY, SCREENX);
TFT_eSprite crosshair = TFT_eSprite(&tft);

GridEYE grideye;

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

const char* ssid = "ThermalCam";
const char* password = "1234567890";

uint8_t mode = MODE_OVERVIEW;
bool btn1_down = true;
bool btn2_down = true;

/**
 * @brief original library has a bug that causes negative values to 
 * 		  reported incorrectly. This is a workaround until the library
 *        is fixed.
 * 
 * @param i pixel index
 * @return float temperature in degree Celsius
 */
float readTemperature(unsigned char i) {
	int16_t temperature = grideye.getPixelTemperatureRaw(i);

	if(temperature & (1 << 11))
	{
		//thanks to GitRayman for this fix
		temperature |= 0b1111 << 12;
	}

	float degreesC = temperature * 0.25;

	return degreesC;
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

char webpage[] PROGMEM = R"=====(
<html>
<head>
	<style>
		#thermcam td {
		  width: 20;
		  height: 20;
		}
	</style>
	<script>
		var Socket;
		var streamframestimer;
		var sizesensor = [8,8];
		var minTsensor = 0.0;
		var maxTsensor = 80.0;

		var minT = 15.0; // initial low range of the sensor (this will be blue on the screen)
		var maxT = 50.0; // initial high range of the sensor (this will be red on the screen)

		var currentFrame = [];
		var minTframe = maxTsensor;
		var maxTframe = minTsensor;
		var minTframelocation = [0,0];
		var maxTframelocation = [0,0];

		var camColors = [
		"1500BF", "0900BF", "0003C0", "0010C0", "001DC1", "002AC2", "0037C2", "0044C3",
		"0052C3", "005FC4", "006DC5", "007AC5", "0088C6", "0095C6", "00A3C7", "00B1C8",
		"00BFC8", "00C9C4", "00C9B8", "00CAAB", "00CB9E", "00CB90", "00CC83", "00CC76",
		"00CD68", "00CE5B", "00CE4D", "00CF40", "00CF32", "00D024", "00D116", "00D109",
		"05D200", "13D200", "21D300", "2FD400", "3DD400", "4CD500", "5AD500", "69D600",
		"78D700", "86D700", "95D800", "A4D800", "B3D900", "C2DA00", "D1DA00", "DBD500",
		"DBC700", "DCB900", "DDAA00", "DD9C00", "DE8E00", "DE7F00", "DF7100", "E06200",
		"E05300", "E14400", "E13500", "E22600", "E31700", "E30800", "E40006", "E50015",
		];

		function mapto( val, minin, maxin, minout ,maxout ){
			if ( val >= maxin ){
				return maxout;
			} else if ( val <= minin ){
				return minout;
			} else {
				return ((( ( maxout - minout )/( maxin - minin ) ) * val ) + ( minout - ( ( maxout - minout )/( maxin - minin ) ) * minin ));
			}
		}

		function adjusttemrange(range,adjustment){
			if ( range == "min" ){
				if ( adjustment == 0){
					minT = minTframe;
				} else {
					minT += adjustment;
				}
			}
			if ( range == "max" ){
				if ( adjustment == 0){
					maxT = maxTframe;
				} else {
					maxT += adjustment;
				}
			}
			if ( minT < minTsensor ){
				minT = minTsensor;
			}
			if ( maxT > maxTsensor ){
				maxT = maxTsensor;
			}
			if ( minT >= maxT ){
				minT = maxT - 1.0;
			}
			fillthermcamtable(currentFrame);
			//document.getElementById("minT").innerHTML = minT.toFixed(2) + "C";
			document.getElementById("minT").innerHTML = minT.toFixed(2);
			//document.getElementById("maxT").innerHTML = maxT.toFixed(2) + "C";
			document.getElementById("maxT").innerHTML = maxT.toFixed(2);
		}

		function ctof(tempc){
			return (tempc * ( 9 / 5 ) ) + 32;
		}

		function buildthermcamtable(){
			var thermcamTable = document.getElementById("thermcam");
			for(i=0;i<sizesensor[0];i++){
				var row = thermcamTable.insertRow(i);
				for(j=0;j<sizesensor[1];j++){
					var cell = row.insertCell(j);
					cell.id = "tccell"+((i*8)+j);
				}
			}
		}

		function fillthermcamtable(pixelarray){
			minTframe = maxTsensor;
			maxTframe = minTsensor;
			// Fill table
			// Temps in C and F in cells
			// BG color of cells use camColors
			// Update minTframe and maxTframe
			for(i=0;i<sizesensor[0];i++){
				for(j=0;j<sizesensor[1];j++){
					var idx = ((i*8)+j);
					var cell = document.getElementById("tccell"+idx);
					var camcolorvalue = Math.round( mapto( pixelarray[idx], minT , maxT, 0 ,63 ) );
					// cell.innerHTML = pixelarray[idx].toFixed(2) + "C " + ctof( pixelarray[idx] ).toFixed(2) + "F";
					cell.innerHTML = Math.round( pixelarray[idx] )
					cell.style.backgroundColor = "#" + camColors[camcolorvalue];
					cell.className = "camcolorvalue" + camcolorvalue;
					if ( pixelarray[idx] < minTframe ){
						minTframe = pixelarray[idx];
						minTframelocation = [j,i];
					}
					if ( pixelarray[idx] > maxTframe ){
						maxTframe = pixelarray[idx];
						maxTframelocation = [j,i];
					}
				}
			}
			//document.getElementById("minTframedata").innerHTML = minTframe.toFixed(2) + "C " + ctof( minTframe ).toFixed(2) + "F (" + minTframelocation[0] + "," + minTframelocation[1] + ")";
			document.getElementById("minTframedata").innerHTML = minTframe.toFixed(2) + " (" + minTframelocation[0] + "," + minTframelocation[1] + ")";
			//document.getElementById("maxTframedata").innerHTML = maxTframe.toFixed(2) + "C " + ctof( maxTframe ).toFixed(2) + "F (" + maxTframelocation[0] + "," + maxTframelocation[1] + ")";
			document.getElementById("maxTframedata").innerHTML = maxTframe.toFixed(2) + " (" + maxTframelocation[0] + "," + maxTframelocation[1] + ")";
		}

		function init() {
			// Build table
			buildthermcamtable();
			//document.getElementById("minT").innerHTML = minT.toFixed(2) + "C";
			document.getElementById("minT").innerHTML = minT.toFixed(2);
			//document.getElementById("maxT").innerHTML = maxT.toFixed(2) + "C";
			document.getElementById("maxT").innerHTML = maxT.toFixed(2);

			Socket = new WebSocket('ws://' + window.location.hostname + ':81/');
			Socket.onmessage = function(event){
				if ( event.data[0] == '[' ){
					currentFrame = JSON.parse(event.data);
					fillthermcamtable(currentFrame);
				}
			}
		}

		function sendText(){
			Socket.send(document.getElementById("txBar").value);
			document.getElementById("txBar").value = "";
		}

		function getframe(){
			Socket.send("F");
		}

		function streamframes(){
			streamframestimer = setTimeout( streamframes , 200 );
			getframe();
		}

		function stopstreamframes(){
			clearTimeout( streamframestimer );
		}

	</script>
</head>
<body onload="init()">
	<table id="thermcam">
	</table>
	<hr/>
	<table id="thermcamcontrols">
		<tr>
			<td>Min</td>
			<td id="minT"></td>
			<td><button type="button" onclick="adjusttemrange('min',-5);">-5</button></td>
			<td><button type="button" onclick="adjusttemrange('min',-1);">-1</button></td>
			<td><button type="button" onclick="adjusttemrange('min', 0);">Min</button></td>
			<td><button type="button" onclick="adjusttemrange('min',+1);">+1</button></td>
			<td><button type="button" onclick="adjusttemrange('min',+5);">+5</button></td>
			<td id="minTframedata"></td>
		</tr>
		<tr>
			<td>Max</td>
			<td id="maxT"></td>
			<td><button type="button" onclick="adjusttemrange('max',-5);">-5</button></td>
			<td><button type="button" onclick="adjusttemrange('max',-1);">-1</button></td>
			<td><button type="button" onclick="adjusttemrange('max', 0);">Max</button></td>
			<td><button type="button" onclick="adjusttemrange('max',+1);">+1</button></td>
			<td><button type="button" onclick="adjusttemrange('max',+5);">+5</button></td>
			<td id="maxTframedata"></td>
		</tr>
	</table>
	<hr/>
	<div>
		<button type="button" onclick="getframe();">Get Frame</button>
		<button type="button" onclick="streamframes();">Start Stream Frames</button>
		<button type="button" onclick="stopstreamframes();">Stop Stream Frames</button>
	</div>
</body>
</html>
)=====";

void setup() {
  	
	Serial.begin(115200);

	pinMode(BUTTON1_PIN, INPUT);
  	pinMode(BUTTON2_PIN, INPUT);

	WiFi.softAP(ssid, password);
	
	Serial.print("Camera Stream Ready! Go to: http://");
	Serial.println(WiFi.softAPIP()); //WiFi.localIP());

	server.on("/",[](){
		server.send_P(200, "text/html", webpage);  
	});
	server.begin();
	webSocket.begin();
	webSocket.onEvent(webSocketEvent);

	Wire.begin();
	grideye.begin();

	tft.init();
	tft.setRotation(3);
	tft.fillScreen(0x0000);
	tft.fillRect(0, 0,
	             INTERPOLATED_SIZE * PIXSIZE + 2 * BORDER_WIDTH,
	             INTERPOLATED_SIZE * PIXSIZE + 2 * BORDER_WIDTH,
	             0xFFFF);
	tft.fillRect(BORDER_WIDTH, BORDER_WIDTH,
	             INTERPOLATED_SIZE * PIXSIZE,
	             INTERPOLATED_SIZE * PIXSIZE,
	             0x0000);
	tft.setCursor(0, 0);

	crosshair.createSprite(3 * PIXSIZE, 3 * PIXSIZE);
	crosshair.drawRect(0, 0, 3 * PIXSIZE, 3 * PIXSIZE, TFT_WHITE);
	crosshair.drawRect(1, 1, 3 * PIXSIZE - 2, 3 * PIXSIZE - 2, TFT_WHITE);
}

float lowTemp = 0;
float highTemp = 30;
float kFast = 0.100;
float kSlow = 0.025;

uint32_t colorMap(float temp) {
	float lum;
	if (temp <= lowTemp) {
		lum = 0;
	} else if (temp >= highTemp) {
		lum = 1;
	} else {
		lum = (temp - lowTemp) / (highTemp - lowTemp);
	}
	int index = lum * ((sizeof(camColors) / sizeof(uint16_t)) - 1 );
	return camColors[index];
}

void adjustLimits(float low, float high) {
	float k;
	float spread = highTemp - lowTemp;
	k = low < lowTemp ? kFast : spread > 10 ? kSlow : 0;
	lowTemp = lowTemp - k * (lowTemp - low);
	k = high > highTemp ? kFast : spread > 10 ? kSlow : 0;
	highTemp = highTemp - k * (highTemp - high);
}

String formatTemperature(float temp) {
	if (temp > 80) {
		return String(">+80.0C");
	} else
		if (temp < -20) {
			return String("<-20.0C");
		}
		
	char buffer[20];
	sprintf(buffer, "%+06.2fC", temp);
	return String(buffer); 
}

void renderOverview(float min, float max) {
	tft.setTextColor(0xFFFF, 0x0000);
	tft.setTextSize(2);

	tft.setCursor(TEXT_X_OFFSET, 0);
	tft.print("Min:");
	tft.setCursor(TEXT_X_OFFSET, 16);
	tft.print(formatTemperature(min));

	tft.setCursor(TEXT_X_OFFSET, 36);
	tft.print("Max: ");
	tft.setCursor(TEXT_X_OFFSET, 36 + 16);
	tft.print(formatTemperature(max));

    for (int i = 0; i < SCALE_STEPS; i++) {
        float t = lowTemp + i * (highTemp - lowTemp) / (SCALE_STEPS-1);
        tft.fillRect(TEXT_X_OFFSET, 2*36 + i*16 + 2, 12, 12, colorMap(t));
    	tft.setCursor(TEXT_X_OFFSET + 16, 2*36 + i*16);
    	tft.print(formatTemperature(t));
    }
}

void renderSelection() {
	tft.setTextColor(0xFFFF, 0x0000);
	tft.setTextSize(2);
	tft.setCursor(TEXT_X_OFFSET, TFT_WIDTH / 2 - 2);

	float val = get_point(dest2d, INTERPOLATED_SIZE, INTERPOLATED_SIZE, INTERPOLATED_SIZE / 2, INTERPOLATED_SIZE / 2);
	uint32_t color = colorMap(val);

	tft.fillRect(TEXT_X_OFFSET, SCREENY / 2 - 3, 12, 12, color);
	tft.setCursor(TEXT_X_OFFSET + 16, SCREENY / 2 - 3 - 2);
	tft.print(formatTemperature(val));

	crosshair.pushSprite((INTERPOLATED_SIZE / 2) * PIXSIZE + BORDER_WIDTH - PIXSIZE, (INTERPOLATED_SIZE / 2) * PIXSIZE + BORDER_WIDTH - PIXSIZE, TFT_BLACK);
}

void onBtn1Press() {
}

void onBtn2Press() {
	switch (mode) {
		case MODE_OVERVIEW:
			mode = MODE_SELECTION;
			break;
		case MODE_SELECTION:
			mode = MODE_OVERVIEW;
			break;
	}

	tft.fillRect(TEXT_X_OFFSET, 0, TFT_HEIGHT - TEXT_X_OFFSET, TFT_WIDTH - 1, TFT_BLACK);
}

void handleButtons() {
	uint8_t btn1_pressed = digitalRead(BUTTON1_PIN);
  	uint8_t btn2_pressed = digitalRead(BUTTON2_PIN);

	if (btn1_pressed && !btn1_down) {
		onBtn1Press();
	}
	if (btn2_pressed && !btn2_down) {
		onBtn2Press();
	}

	btn1_down = btn1_pressed > 0;
	btn2_down = btn2_pressed > 0;
}

void loop() {

	unsigned long t0 = millis();
	tft.startWrite();

	handleButtons();

	float min = 1000;
	float max = -1000;
	for(unsigned char i = 0; i < 64; i++) {
		float val = readTemperature(i);
		pixels[i] = pixels[i] - 0.5 * ( pixels[i] - val );
		if (val < min) { min = val; }
		if (val > max) { max = val; }
	}
	adjustLimits(min, max);

	webSocket.loop();
	server.handleClient();
	if(Serial.available() > 0){
		char c[] = {(char)Serial.read()};
		webSocket.broadcastTXT(c, sizeof(c));
	}

	interpolate_image(pixels, SENSOR_SIZE, SENSOR_SIZE, dest2d, INTERPOLATED_SIZE, INTERPOLATED_SIZE);

	for (int y=0; y<INTERPOLATED_SIZE; y++) {
		for (int x=0; x<INTERPOLATED_SIZE; x++) {
			float val = get_point(dest2d, INTERPOLATED_SIZE, INTERPOLATED_SIZE, x, y);
			uint32_t color = colorMap(val);

			if (mode == MODE_SELECTION) {
				// update all pixels except the ones that are used in
				// the crosshair to prevent flickering

				int d_center_x = x - INTERPOLATED_SIZE / 2;
				int d_center_y = y - INTERPOLATED_SIZE / 2;
				if (abs(d_center_x) <= 1 && abs(d_center_y) <= 1
					&& !(d_center_x == 0 && d_center_y == 0)) {
					int d_x = d_center_x < 0 ? PIXSIZE / 2 : 0;
					int d_y = d_center_y < 0 ? PIXSIZE / 2 : 0;
					int d_width = abs(d_center_x) > 0 ? PIXSIZE / 2 : 0;
					int d_height = abs(d_center_y) > 0 ? PIXSIZE / 2 : 0;
					tft.fillRect(
							BORDER_WIDTH + PIXSIZE * x + d_x, BORDER_WIDTH + PIXSIZE * y + d_y, 
							PIXSIZE - d_width, PIXSIZE - d_height, 
							color
						);
					continue;
				}
			}
			tft.fillRect(BORDER_WIDTH + PIXSIZE * x, BORDER_WIDTH + PIXSIZE * y, PIXSIZE, PIXSIZE, color);
		}
	}

	switch (mode) {
		case MODE_OVERVIEW:
			renderOverview(min, max);
			break;
		case MODE_SELECTION:
			renderSelection();
			break;
	}

	tft.endWrite();

	unsigned long t = millis() - t0;
	delay( t < 100 ? 100 - t : 1 );
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length){
  if(type == WStype_TEXT){
    if(payload[0] == 'F'){ // i.e. "Frame"
		//amg.readPixels(pixels);

		String thermalFrame = "[";

		for (int i = 1; i < AMG88xx_PIXEL_ARRAY_SIZE + 1; i++) { // Starts at 1 so that (i % 8) will be 0 at end of 8 pixels and newline will be added
			thermalFrame += pixels[i - 1];
			if ( i != AMG88xx_PIXEL_ARRAY_SIZE ){
				thermalFrame += ", ";
				if ( i % 8 == 0 ) thermalFrame += "\n";
			}
		}
		thermalFrame += "]";

		char tf[thermalFrame.length()+1];

		thermalFrame.toCharArray( tf, thermalFrame.length()+1 );

		Serial.println(tf);

		webSocket.broadcastTXT(tf, sizeof(tf)-1);

    } else{
      for(int i = 0; i < length; i++)
        Serial.print((char) payload[i]);
      Serial.println();
    }
  }
}
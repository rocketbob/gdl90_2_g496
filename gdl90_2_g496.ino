#include <WiFi.h>
#include <WiFiUDP.h>

unsigned int UDPPort = 4000;	// local port to listen on
const int bufferSize = 450;
byte packetBuffer[bufferSize];	//buffer to hold incoming packet
byte messageBuffer[bufferSize];

const int maxTargets = 8;

float mLat = 0;  // My position variables
float mLon = 0;
int mAltitude = 0;

int tStatus = 0;	// Traffic variables 
float tLat = 0;
float tLon = 0;
int tAltitude = 0;
int tAltitudeDelta = 0;
int tVertVelocity = 0;
int tHeading = 0;
int tIcon[maxTargets] = { 0,0,0,0,0,0,0,0 };
byte tClimb[maxTargets] = { 0,0,0,0,0,0,0,0 };
int tDir[maxTargets] = { 0,0,0,0,0,0,0,0 };
int tHundredsAltitude[maxTargets] = { 0,0,0,0,0,0,0,0 };
byte tByteDistance[maxTargets] = { 0,0,0,0,0,0,0,0 };
float tDistance = 0;

byte tNumberOf = 0;
int timer = 0;

byte checksum = 0;

WiFiUDP Udp;

void setup() {
	timer = millis();
	Serial2.begin(9600);	// Serial port to Garmin 396/496 (pin D4 on NodeMCU)
	Serial.begin(115200);	// Debug serial port (usb on NodeMCU)
	WiFi.begin("stratux");	// ADS-B SSID
	//WiFi.begin("Ping-4D6F");
	Serial.println();
	Serial.print("Wait for WiFi");
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}
	Serial.println("");
	Serial.println("WiFi connected");
	Serial.println("IP address: " + WiFi.localIP().toString());
	Udp.begin(UDPPort);
}

float calculateDegrees(int highByte, int midByte, int lowByte) {
	int position = 0;

	float xx;

	position = highByte;
	position <<= 8;
	position |= midByte;
	position <<= 8;
	position |= lowByte;
	position &= 0xFFFFFFFF;

	if ((position & 0x800000) != 0) {
		int yy;

		position |= 0xFF000000;

		yy = (int)position;
		xx = (float)(yy);
	}
	else {
		xx = (position & 0x7FFFFF);
	}

	xx *= 2.1457672E-5;

	return xx;
}

float findTrafficDirection(float latD, float lonD) {	// convert pythag angle to bearing
	if (latD >= 0 && lonD >= 0) {
		float result = 180 / PI*(atan(lonD / latD));
		return result;
	}
	if (latD < 0 && lonD >= 0) {
		float result = 180 / PI*(abs(atan(latD / lonD))) + 90;
		return result;
	}
	if (latD < 0 && lonD < 0) {
		float result = 180 / PI*(atan(lonD / latD)) + 180;
		return result;
	}
	if (latD >= 0 && lonD < 0) {
		float result = 180 / PI*(abs(atan(latD / lonD))) + 270;
		return result;
	}
}

void writeByte(byte byteInput) {
	Serial2.write(byteInput);
	checksum = checksum + byteInput;
	if (byteInput == 0x10) {
		Serial2.write(0x10);        //Duplicate the byte if same as DLE - not in checksum
	}
}

void writeInt(int intInput) {   //Split int into two bytes
	byte low = intInput;
	byte high = intInput >> 8;
	writeByte(low);
	writeByte(high);
}

void sendNOTIS() {
	checksum = 0;                 //Reset checksum
	Serial2.write(0x10);          //DLE - not in checksum
	writeByte(0x8C);              //Message ID
	writeByte(0x08);              //Data length
	writeByte(0x04);              //?
	writeByte(0x00);              //?
	writeByte(0x00);              //3F for traffic not available
	writeByte(0x00);              //?
	writeInt(0);                  //Compass heading (degrees)
	writeByte(0x00);              //?3f
	writeByte(0x00);              //?
	Serial2.write(-checksum);     //Checksum
	Serial2.write(0x10);          //DLE - not in checksum
	Serial2.write(0x03);          //ETX - not in checksum
	//Serial.println("NOTIS");
}


void sendTIS() {

	
	checksum = 0;	//Reset checksum

	if (tNumberOf == 0) {
		sendNOTIS();
	}
	else {
		Serial2.write(0x10);			//DLE - not in checksum
		writeByte(0x8C);				//Message ID
		writeByte(8 + (8 * tNumberOf));	//Data length
		writeByte(0x04);				//?
		writeByte(0x00);				//?
		writeByte(tNumberOf);			//Number of targets
		writeByte(0x00);				//?
		writeInt(0);					//Compass heading (degrees) when sending relative bearing traffic
		writeByte(0x00);				//?
		writeByte(0x00);				//?

		for (int t = 0; t < tNumberOf; t++) {
			writeByte(0x00);				//Traffic 00=normal 01=alert 02=normal?
			writeByte(tClimb[t]);			//Traffic vertical direction 0=level 1=climbing 2=descending
			writeByte(tIcon[t]);			//Traffic icon heading 0=N 1=NE 2=E 3=SE 4=S 5=SW 6=W 7=NW 8=NONE? 9=NONE
			writeByte(tByteDistance[t]);	//Traffic distance (nm*8)
			writeInt(tDir[t]);				//Traffic azimuth (degrees)
			writeInt(tHundredsAltitude[t]);	//Traffic altitude (feet/100)	
		}

		/*
		writeByte(0x00);	//Traffic 00=normal 01=alert 02=normal?
		writeByte(0x01);	//Traffic vertical direction 0=level 1=climbing 2=descending
		writeByte(0x09);	//Traffic icon heading 0=N 1=NE 2=E 3=SE 4=S 5=SW 6=W 7=NW 8=NONE? 9=NONE
		writeByte(0x10);	//Traffic distance (nm*8)
		writeInt(0);		//Traffic azimuth (degrees)
		writeInt(25);		//Traffic altitude (feet/100)
		*/

		Serial2.write(-checksum);	//Checksum
		Serial2.write(0x10);		//DLE - not in checksum
		Serial2.write(0x03);		//ETX - not in checksum
	}
}

void loop() {

	// if there's data available, read a packet
	int packetSize = Udp.parsePacket();
	if (packetSize) {

		// read the packet into packetBufffer
		int len = Udp.read(packetBuffer, bufferSize);
		if (len > 0) {
			packetBuffer[len] = 0;
		}
		int m = 0;
		for (int i = 0; i < len; i++) {	// Unpack buffer.  Remove extra bytes.
			if (packetBuffer[i] == 0x7D) {
				i++;
				if (packetBuffer[i] == 0x5E) {
					messageBuffer[m] = 0x7E;
				}
				if (packetBuffer[i] == 0x5D) {
					messageBuffer[m] == 0x7D;
				}
			}
			messageBuffer[m] = packetBuffer[i];
			m++;
		}
		/*// Print hex values--------------------------------------------
		for (int j = 0; j < len; j++) {

			Serial.print(" ");
			if (messageBuffer[j] < 0x10) { Serial.print("0"); }
			Serial.print(messageBuffer[j], HEX);
		}
		Serial.println("");
		//*/
		for (int j = 0; j < len; j++) {
			if (messageBuffer[j] == 0x7E) {	// Watch for DLE

				switch (messageBuffer[j + 1]) {
					case 10: { //ownship message

						// Lat Lon
						mLat = calculateDegrees((int)(messageBuffer[j + 6] & 0xFF), (int)(messageBuffer[j + 7] & 0xFF), (int)(messageBuffer[j + 8] & 0xFF));
						mLon = calculateDegrees((int)(messageBuffer[j + 9] & 0xFF), (int)(messageBuffer[j + 10] & 0xFF), (int)(messageBuffer[j + 11] & 0xFF));

						// Altitude
						int upper = (((int)messageBuffer[j + 12]) & 0xFF) << 4;
						int lower = (((int)messageBuffer[j + 13]) & 0xFF) >> 4;
						mAltitude = upper + lower;
						mAltitude *= 25;
						//mAltitude -= 1000;

						j = j + 31;
						/*Serial.print("M ");
						Serial.print(mLat,6);
						Serial.print(" ");
						Serial.print(mLon,6);
						Serial.print(" ");
						Serial.print(mAltitude);
						Serial.println(""); */
						break;
					}

					case 20: { // Traffic message

						// Alert status
						tStatus = (messageBuffer[j + 2] & 0xF0) >> 4;	// Alert status is meaningless in Stratux.  Must determine traffic status on our own.

						// Lat Lon
						tLat = calculateDegrees((int)(messageBuffer[j + 6] & 0xFF), (int)(messageBuffer[j + 7] & 0xFF), (int)(messageBuffer[j + 8] & 0xFF));	// Parse Lat/Lon in degrees
						tLon = calculateDegrees((int)(messageBuffer[j + 9] & 0xFF), (int)(messageBuffer[j + 10] & 0xFF), (int)(messageBuffer[j + 11] & 0xFF));
						
						// Altitude
						int upper = (((int)messageBuffer[j + 12]) & 0xFF) << 4;
						int lower = (((int)messageBuffer[j + 13]) & 0xFF) >> 4;
						tAltitude = upper + lower;
						tAltitude *= 25;
						//tAltitude -= 1000;
						tAltitudeDelta = tAltitude - mAltitude;	// Calculate relative altitude of traffic
						tHundredsAltitude[tNumberOf] = tAltitudeDelta / 100; //Convert feet into 100's of feet
					
						// Vertical velocity
						int upperb = ((int)messageBuffer[j + 16] & 0x0F) << 8;
						int lowerb = (int)messageBuffer[j + 17] & 0xFF;
						tVertVelocity = (upperb | lowerb);
						if (tVertVelocity >= 3 && tVertVelocity <= 510) { tClimb[tNumberOf] = 1; }	// Convert vert velocity to climb/descent arrow byte
						if (tVertVelocity >= 3586 && tVertVelocity <= 4093) { tClimb[tNumberOf] = 2; }
						if (tVertVelocity < 3) { tClimb[tNumberOf] = 0; }
						if (tVertVelocity > 4093) { tClimb[tNumberOf] = 0; }
						if (tVertVelocity == 2048) { tClimb[tNumberOf] = 0; }

						// Heading
						tHeading = (((int)messageBuffer[j + 18] & 0xFF)) * 360 / 256;

						if ((((messageBuffer[j + 18] + 16) / 32) & 0x0F) == 8) {	//  Constrain tHeading to 1 nibble
							tIcon[tNumberOf] = (((messageBuffer[j + 18] + 16) / 32) & 0x0F) - 8;
						}
						else {
						tIcon[tNumberOf] = (((messageBuffer[j + 18] + 16) / 32) & 0x0F);
						}

						j = j + 31;
						/*Serial.print("T ");
						Serial.print(tLat,6);
						Serial.print(" ");
						Serial.print(tLon,6);
						Serial.print(" ");
						Serial.print(tAltitude);
						Serial.print("ft ");
						Serial.println("");*/
						float tLatDelta = ((tLat - mLat) * 60);	// Lat distance (nm)
						float tLonDelta = ((mLon - tLon) * 60)*cos(mLat);	// Lon distance (nm)

						tDistance = sqrt(abs(tLonDelta*tLonDelta) + abs(tLatDelta*tLatDelta));  // Calculate pythag distance
						tByteDistance[tNumberOf] = (tDistance * 8);  //Convert miles to 1/8th miles

						tDir[tNumberOf] = (int)(findTrafficDirection(tLatDelta, tLonDelta));

					
					
						/*Serial.print(tLatDelta);
						Serial.print(" ");
						Serial.print(tLonDelta);
						Serial.print(" ");*/
						Serial.print(tStatus);
						Serial.print(" ");
						Serial.print(tDistance, 1);
						Serial.print("nm ");
						//Serial.print(tByteDistance[tNumberOf], HEX);
						//Serial.print("-8s "); 
						Serial.print(tDir[tNumberOf]);
						Serial.print("deg ");
						Serial.print(tAltitudeDelta);
						Serial.print("ft ");
						Serial.print(tClimb[tNumberOf]);
						Serial.print(" ");
						Serial.print(tHeading);
						Serial.print("deg ");
						Serial.print(tIcon[tNumberOf]);
						Serial.print("dir "); Serial.println("");

						if (tDistance > 31.8) {	// ignore distances greater than 1 byte in 8ths 
							tIcon[tNumberOf] = 0;	// clear data from ignored traffic
							tClimb[tNumberOf] = 0;
							tDir[tNumberOf] = 0;
							tHundredsAltitude[tNumberOf] = 0;
							tByteDistance[tNumberOf] = 0;
							tNumberOf--;
						}

						tNumberOf++;
						if (tNumberOf > maxTargets) { tNumberOf = maxTargets; }

						break;
					}
				}
			}
		}
	}

	if ((millis() - timer) > 1000) {	// send traffic every second
		sendTIS();
		timer = millis();	// reset timer from last traffic transmission
		Serial.println("sending traffic");
		tNumberOf = 0;

		for (int i = 0; i < maxTargets; i++) {	// clear sent traffic data
			tIcon[i] = 0;
			tClimb[i] = 0;
			tDir[i] = 0;
			tHundredsAltitude[i] = 0;
			tByteDistance[i] = 0;
		}

	}

}
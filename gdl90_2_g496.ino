#include <WiFi.h>
#include <WiFiUDP.h>
#include <HardwareSerial.h>

unsigned int UDPPort = 4000;	// local port to listen on
const int bufferSize = 450;
byte packetBuffer[bufferSize];	//buffer to hold incoming packet
byte messageBuffer[bufferSize];
const int wifiSelectPin = 4;
const int trafficAudioPin = 13;  //traffic audio output trip pin
int trafficAudioTimer = millis();
bool trafficAudioState = false;
const int maxTargets = 8;
const int warnAltitude = 9;  //warning altitude in 100ft (9)
const byte warnDistance = 16;  // warning distance in 1/8's of miles (16)
const int warnMinAltitude = 1;  //min warning altitude in 100ft to avoid ownships warnings
const byte warnMinDistance = 4;  //min warn distance in 1/8's of miles
float maxTrafficDistance = 30;  //max display distance in nm (20)
int maxTrafficAltitude = 50;  //max display altitude difference in 100's of feet (40)
int tPersist = 30000;  //traffic persist in milliseconds

float mLat = 40.128464;  // My position variables
float mLon = -86.228897;
int mAltitude = 930;
int mGS = 50;

int tStatus = 0;	// Traffic variables
int tAddressType = 0;
int tAddress = 0;
float tLat = 0;
float tLon = 0;
int tAltitude = 0;
int tAltitudeDelta = 0;
int tVertVelocity = 0;
int tHeading = 0;
int tTimestamp[maxTargets] = { 0,0,0,0,0,0,0,0 };
int tICAO[maxTargets] = { 0,0,0,0,0,0,0,0 };
int tIcon[maxTargets] = { 0,0,0,0,0,0,0,0 };
byte tClimb[maxTargets] = { 0,0,0,0,0,0,0,0 };
int tDir[maxTargets] = { 0,0,0,0,0,0,0,0 };
int tHundredsAltitude[maxTargets] = { 0,0,0,0,0,0,0,0 };
byte tByteDistance[maxTargets] = { 0,0,0,0,0,0,0,0 };
float tDistance = 0;

byte tNumberOf = 0;
byte tWriteNumber = 0;
int timer = 0;

byte checksum = 0;
int myICAO = 0xA8E6E0;  // drew address
//int myICAO = 0xA0835D;  // test


WiFiUDP Udp;

void setup() {
	pinMode(trafficAudioPin, OUTPUT);
	digitalWrite(trafficAudioPin, LOW);
	delay(100);
	digitalWrite(trafficAudioPin, HIGH);


	timer = millis();
	Serial1.begin(9600);	// Serial port to Garmin 396/496 (pin D4 on NodeMCU)
	Serial.begin(115200);	// Debug serial port (usb on NodeMCU)
	pinMode(wifiSelectPin, INPUT);

	if (digitalRead(wifiSelectPin) == LOW) {
		WiFi.begin("stratux");	// ADS-B SSID
		Serial.println();
		Serial.println("Stratux");
		maxTrafficDistance = 30; // set max distance and alt longer for bench testing (30)
		maxTrafficAltitude = 490; //(490)

	}
	else {
		//WiFi.begin("Ping-4D6F"); //Drew
		WiFi.begin("Ping-EA8D"); //Tom
		Serial.println();
		Serial.println("Ping");
	}
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
	//Serial.print(" findTdir");
	if (latD == 0 && lonD == 0) {
		float result = 0;
		return result;
	}
	if (latD == 0 && lonD > 0) {
		float result = 90;
		return result;
	}
	if (latD == 0 && lonD < 0) {
		float result = 270;
		return result;
	}
	if (lonD == 0 && latD > 0) {
		float result = 0;
		return result;
	}
	if (lonD == 0 && latD < 0) {
		float result = 180;
		return result;
	}
	if (latD > 0 && lonD > 0) {
		//if (abs(latD) == 0) latD = .001; 
		float result = 180 / PI * (atan(lonD / latD));
		return result;
	}
	if (latD < 0 && lonD > 0) {
		//if (abs(lonD) == 0) lonD = .001;
		float result = (180 / PI * (atan((lonD) / (latD)))) + 180;
		return result;
	}
	if (latD < 0 && lonD < 0) {
		//if (abs(latD) == 0) latD = .001; 
		float result = (180 / PI * (atan((lonD) / (latD)))) + 180;
		return result;
	}
	if (latD > 0 && lonD < 0) {
		//if (abs(lonD) == 0) lonD = .001;
		float result = (180 / PI * (atan(lonD / (latD)))) + 360;
		/*
		Serial.print(" ");
		Serial.print("atan");
		Serial.print(" ");
		Serial.print(atan(latD/abs(lonD)));
		Serial.print(" ");
		*/
		return result;
	}
}

void writeByte(byte byteInput) {
	Serial1.write(byteInput);
	checksum = checksum + byteInput;
	if (byteInput == 0x10) {
		Serial1.write(0x10);        //Duplicate the byte if same as DLE - not in checksum
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
	Serial1.write(0x10);          //DLE - not in checksum
	writeByte(0x8C);              //Message ID
	writeByte(0x08);              //Data length
	writeByte(0x04);              //?
	writeByte(0x00);              //?
	writeByte(0x00);              //3F for traffic not available
	writeByte(0x00);              //?
	writeInt(0);                  //Compass heading (degrees)
	writeByte(0x00);              //?3f
	writeByte(0x00);              //?
	Serial1.write(-checksum);     //Checksum
	Serial1.write(0x10);          //DLE - not in checksum
	Serial1.write(0x03);          //ETX - not in checksum
	//Serial.println("NOTIS");
}


void sendTIS() {


	checksum = 0;	//Reset checksum

	if (tNumberOf == 0) {
		sendNOTIS();
	}
	else {
		Serial1.write(0x10);			//DLE - not in checksum
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

			if (tByteDistance[t] <= warnDistance && tHundredsAltitude[t] <= warnAltitude) { //Set traffic warning if within warn dist and altitude
				if (tByteDistance[t] >= warnMinDistance && tHundredsAltitude[t] >= warnMinAltitude && mGS >= 20) {
					writeByte(0x01);
					triggerTrafficAudioOut();
				}
				else {
					writeByte(0x00);
				}
			}
			else {
				writeByte(0x00);			//Traffic 00=normal 01=alert 02=normal?
			}

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

		Serial1.write(-checksum);	//Checksum
		Serial1.write(0x10);		//DLE - not in checksum
		Serial1.write(0x03);		//ETX - not in checksum
	}
}

void clearLastTraffic() {
	Serial.println(" clrLast");
	tIcon[tNumberOf] = 0;	// clear data from ignored traffic
	tClimb[tNumberOf] = 0;
	tDir[tNumberOf] = 0;
	tHundredsAltitude[tNumberOf] = 0;
	tByteDistance[tNumberOf] = 0;
	tNumberOf--;
}
void triggerTrafficAudioOut() {
	if (trafficAudioState == false && millis() > (trafficAudioTimer + (10 * 1000))) {
		digitalWrite(trafficAudioPin, LOW);
		trafficAudioState = true;
		//Serial.print("audio");
	}
}

void trafficAudioOut() {


	if (millis() > (trafficAudioTimer + 500 + (10 * 1000))) {
		digitalWrite(trafficAudioPin, HIGH);
		trafficAudioState = false;
		trafficAudioTimer = millis();
	}
}

void loop() {

	trafficAudioOut();

	// if there's data available, read a packet
	int packetSize = Udp.parsePacket();
	if (packetSize) {

		// read the packet into packetBufffer
		int len = Udp.read(packetBuffer, bufferSize);
		if (len > 0) {
			packetBuffer[len] = 0;
		}

		int m = 0;
		for (int i = 0; i < len; i++) { // Unpack buffer. Remove extra bytes.
			if (packetBuffer[i] == 0x7D) { // escape control character
				i++; // look at next character
				if (packetBuffer[i] == 0x5E) { // this is 0x7E ^ 0x20, which is an escaped 0x7E
					messageBuffer[m] = 0x7E;
					//Serial.print("Replaced "); Serial.print(packetBuffer[i], HEX); Serial.print(" with "); Serial.println(messageBuffer[m], HEX);
				}
				else if (packetBuffer[i] == 0x5D) { // this is 0x7D ^ 0x20, which is an escaped 0x7D
					messageBuffer[m] = 0x7D;
					//Serial.print("Replaced "); Serial.print(packetBuffer[i], HEX); Serial.print(" with "); Serial.println(messageBuffer[m], HEX);
				}
			}
			else { // all OTHER characters
				messageBuffer[m] = packetBuffer[i]; // copy unescaped characters
			}
			m++;
		}
		len = m; // this is the new length of the message with escape symbols (0x7D) removed

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
					int lower = (((int)messageBuffer[j + 13]) & 0xF0) >> 4;
					mAltitude = upper + lower;
					mAltitude *= 25;
					mAltitude -= 1000;

					// Groundspeed
					int uppergs = ((int)messageBuffer[j + 15] & 0xFF) << 4;
					int lowergs = ((int)messageBuffer[j + 16] & 0xF0) >> 4;
					mGS = uppergs + lowergs;

					if (digitalRead(wifiSelectPin) == LOW) {// to simulate GPS and baro alt during bench testing
						mLat = 40.128464;
						mLon = -86.228897;
						mAltitude = 930;
						mGS = 50;
					}


					j = j + 31;
					//**
					Serial.print("M ");
					Serial.print(mLat, 6);
					Serial.print(" ");
					Serial.print(mLon, 6);
					Serial.print(" ");
					Serial.print(mAltitude);
					Serial.print(" ");
					Serial.print(mGS);
					Serial.println("");
					//**
					break;
				}

				case 20: { // Traffic message

					//if (tNumberOf >= maxTargets) { tNumberOf--; }

					// Alert status
					tStatus = (messageBuffer[j + 2] & 0xF0) >> 4;	// Alert status is meaningless in Stratux.  Must determine traffic status on our own.

					// Address
					tAddressType = (int)(messageBuffer[j + 2] & 0x0F);
					tAddress = ((((int)messageBuffer[j + 3]) & 0xFF) << 16) + ((((int)(messageBuffer[j + 4]) & 0xFF) << 8)) + ((((int)messageBuffer[j + 5]) & 0xFF));

					// Lat Lon
					tLat = calculateDegrees((int)(messageBuffer[j + 6] & 0xFF), (int)(messageBuffer[j + 7] & 0xFF), (int)(messageBuffer[j + 8] & 0xFF));	// Parse Lat/Lon in degrees
					tLon = calculateDegrees((int)(messageBuffer[j + 9] & 0xFF), (int)(messageBuffer[j + 10] & 0xFF), (int)(messageBuffer[j + 11] & 0xFF));
					float tLatDelta = ((tLat - mLat) * 60);	// Lat distance (nm)
					float tLonDelta = ((tLon - mLon) * 60) * cos(mLat / 360 * 2 * PI);	// Lon distance (nm)
					tDistance = sqrt(abs(tLonDelta * tLonDelta) + abs(tLatDelta * tLatDelta));  // Calculate pythag distance


					// Altitude
					int upper = (((int)messageBuffer[j + 12]) & 0xFF) << 4;
					int lower = (((int)messageBuffer[j + 13]) & 0xF0) >> 4;
					tAltitude = upper + lower;
					tAltitude *= 25;
					tAltitude -= 1000;
					tAltitudeDelta = tAltitude - mAltitude;	// Calculate relative altitude of traffic

					// Vertical velocity
					int upperb = ((int)messageBuffer[j + 16] & 0x0F) << 8;
					int lowerb = (int)messageBuffer[j + 17] & 0xFF;
					tVertVelocity = (upperb | lowerb);

					// Heading
					tHeading = (((int)messageBuffer[j + 18] & 0xFF)) * 360 / 256;

					// ignore distance traffic
					if (tDistance > maxTrafficDistance || (tAltitudeDelta / 100) > maxTrafficAltitude || (tAltitudeDelta / 100) < maxTrafficAltitude * -1 || ((byte)((int)tDistance * 8) < warnMinDistance && (tAltitudeDelta / 100) < warnMinAltitude)) {
						//tNumberOf++;

						Serial.println("");
						Serial.println("deleted");
						break;
					}

					// check for my ICAO address
					if (tAddress == myICAO) {
						Serial.println("");
						Serial.println("deleted my ICAO");
						break;
					}

					// check for duplicate icao address
					bool overWrite = false;
					for (int i = 0; i < tNumberOf; i++) {
						if (tAddress == tICAO[i]) {
							overWrite = true;
							tWriteNumber = i;
						}
					}

					// if display full then overwrite more distant traffic
					if (tNumberOf >= maxTargets && overWrite == false) {
						byte tMostDistant = 0;
						int tMostDistantNumber = 0;
						for (int i = 0; i < tNumberOf; i++) {
							if (tByteDistance[i] > tMostDistant) {
								tMostDistant = tByteDistance[i];
								tMostDistantNumber = i;
							}
						}
						if (((byte)((int)tDistance * 8)) <= tMostDistant) {
							overWrite = true;
							tWriteNumber = tMostDistantNumber;
						}
						else {
							break;
						}
					}


					if (overWrite == false) {
						tWriteNumber = tNumberOf;
					}


					// write data into arrays

					tICAO[tWriteNumber] = tAddress;

					tByteDistance[tWriteNumber] = (byte)((int)tDistance * 8);  //Convert miles to 1/8th miles

					tDir[tWriteNumber] = (int)(findTrafficDirection(tLatDelta, tLonDelta));

					tHundredsAltitude[tWriteNumber] = tAltitudeDelta / 100; //Convert feet into 100's of feet

					if (tVertVelocity >= 3 && tVertVelocity <= 510) { tClimb[tWriteNumber] = 1; }	// Convert vert velocity to climb/descent arrow byte
					if (tVertVelocity >= 3586 && tVertVelocity <= 4093) { tClimb[tWriteNumber] = 2; }
					if (tVertVelocity < 3) { tClimb[tWriteNumber] = 0; }
					if (tVertVelocity > 4093) { tClimb[tWriteNumber] = 0; }

					if ((((messageBuffer[j + 18] + 16) / 32) & 0x0F) == 8) {	//  Constrain tHeading to 1 nibble
						tIcon[tWriteNumber] = (((messageBuffer[j + 18] + 16) / 32) & 0x0F) - 8;
					}
					else {
						tIcon[tWriteNumber] = (((messageBuffer[j + 18] + 16) / 32) & 0x0F);
					}

					tTimestamp[tWriteNumber] = millis();


					j = j + 31;

					Serial.print("T ");
					Serial.print(tStatus);
					Serial.print(" ");
					Serial.print(tAddressType);
					Serial.print(" ");
					Serial.print(tAddress, HEX);
					Serial.print(" ");
					Serial.print(tLat, 6);
					Serial.print(" ");
					Serial.print(tLon, 6);
					Serial.print(" ");
					Serial.print(tAltitude);
					Serial.print("ft ");
					//Serial.println("");

					Serial.print(" ");
					Serial.print(tLatDelta);
					Serial.print(" ");
					Serial.print(tLonDelta);
					Serial.print(" ");
					Serial.print("");
					/*Serial.print(tLatDelta);
					Serial.print(" ");
					Serial.print(tLonDelta);
					Serial.print(" ");*/
					//Serial.print(tStatus);
					//Serial.print(" ");
					Serial.print(tDistance, 1);
					Serial.print("nm ");
					//Serial.print(tByteDistance[tNumberOf], HEX);
					//Serial.print("-8s "); 
					Serial.print(tDir[tWriteNumber]);
					Serial.print("deg ");
					Serial.print(tAltitudeDelta / 100);
					Serial.print("00ft ");
					Serial.print(tClimb[tWriteNumber]);
					Serial.print(" ");
					Serial.print(tHeading);
					Serial.print("deg ");
					Serial.print(tIcon[tWriteNumber]);
					Serial.print("dir ");
					Serial.print(tTimestamp[tWriteNumber]);
					Serial.println(" ");



					if (overWrite == false) {

						tNumberOf++;
					}

					//if (tNumberOf > maxTargets) { tNumberOf--; }

					break;
				}
				}
			}
		}
	}

	if ((millis() - timer) > 1000) {	// send traffic every second
		sendTIS();
		timer = millis();	// reset timer from last traffic transmission
		Serial.println("");

		//Serial.println("");
		Serial.print("sending ");
		Serial.println(tNumberOf);
		/*
		Serial.println("");
		Serial.print(tLat);
		Serial.print(" ");
		Serial.println(tLon);
		Serial.print(mLat);
		Serial.print(" ");
		Serial.println(mLon);
		*/

		//tNumberOf = 0;
		int tempNum = tNumberOf;
		for (int i = 0; i < tempNum; i++) {	// clear old traffic data


			Serial.print(tTimestamp[i]);
			Serial.print(" ");

			if (millis() >= (tTimestamp[i] + tPersist)) {  // is traffic record over 30 sec old?

				if (i == tNumberOf && tNumberOf != 0) {  //  is expired traffic the last record?
					tICAO[i] = 0;
					tIcon[i] = 0;
					tClimb[i] = 0;
					tDir[i] = 0;
					tHundredsAltitude[i] = 0;
					tByteDistance[i] = 0;
					tTimestamp[i] = 0;

					tNumberOf--;
				}
				if (i == tNumberOf && tNumberOf == 0) {  //  clear 0th record
					tICAO[i] = 0;
					tIcon[i] = 0;
					tClimb[i] = 0;
					tDir[i] = 0;
					tHundredsAltitude[i] = 0;
					tByteDistance[i] = 0;
					tTimestamp[i] = 0;

				}
				if (i != tNumberOf && tNumberOf != 0) {
					tICAO[i] = tICAO[tNumberOf - 1];
					tIcon[i] = tIcon[tNumberOf - 1];  // copy info from last traffic record to current one
					tClimb[i] = tClimb[tNumberOf - 1];
					tDir[i] = tDir[tNumberOf - 1];
					tHundredsAltitude[i] = tHundredsAltitude[tNumberOf - 1];
					tByteDistance[i] = tByteDistance[tNumberOf - 1];
					tTimestamp[i] = tTimestamp[tNumberOf - 1];

					tICAO[tNumberOf - 1] = 0;
					tIcon[tNumberOf - 1] = 0;  // delete last traffic record
					tClimb[tNumberOf - 1] = 0;
					tDir[tNumberOf - 1] = 0;
					tHundredsAltitude[tNumberOf - 1] = 0;
					tByteDistance[tNumberOf - 1] = 0;
					tTimestamp[tNumberOf - 1] = 0;

					tNumberOf--;
				}
			}

		}
		Serial.println(" ");
	}

}
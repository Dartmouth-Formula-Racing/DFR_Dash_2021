/*
Debouncing timer array:
0 - Drive button
1 - Neutral button
2 - Reverse button
3 - toggle 1 (idk what these are)
4 - toggle 2

CAN data transmit format:
[drivebutton (0/1), neutralbutton (0/1), reversebutton (0/1), toggle1 (0/1/2), toggle2 (0/1/2)]
*/

// Enable debugging over serial
#define DEBUG 1
#define DEBUG_BAUD 9600

#include <ASTCanLib.h>
#include <SerLCD.h>
#include "constants.h"

enum drive_state {DRIVE, NEUTRAL, REVERSE};
enum drive_state inputDriveState = NEUTRAL; // Arduino input drive state
enum drive_state realDriveState = NEUTRAL; // CVC drive state

int buttonStateDrive = LOW;
int buttonStateNeutral = LOW;
int buttonStateReverse = LOW;
int toggleState1 = 0;
int toggleState2 = 0;

st_cmd_t txMsg;
st_cmd_t rxMsg;

uint8_t sendData[8] = {0,0,0,0,0,0,0,0};
uint8_t txBuffer[8] = {};
uint8_t rxBuffer[8] = {};
char dBuffer[LCDCols * LCDRows];
unsigned long lastTx = 0;
unsigned long lastDisplay = 0;
unsigned long lastLoop = 0;

SerLCD lcd;

unsigned long count = 0;

unsigned long debounceTimers[5] = {0,0,0,0,0};
bool debounceActive[5] = {false,false,false,false,false};

// Function prototypes
char* subString(char* fullString, int startPos, int endPos);
void parseRx(st_cmd_t* rxMsg);
bool lcdWrite(SerLCD* lcd, char* toPrint);

// Set debugging state to 0
#ifndef DEBUG
#define DEBUG 1
#endif
// Set debugging serial speed to 9600 baud 
#ifndef DEBUG_BAUD
#define DEBUG_BAUD 9600
#endif
// Defining serial output buffer
#if (DEBUG > 0)
char serialBuffer[64];
#endif

void setup() {
	canInit(500000); 				// This line needs to be before any serial stuff
	Wire.begin();
	lcd.begin(Wire);
	lcd.setBacklight(0x00FFFFFF);

	#if (DEBUG > 0)
	Serial.begin(DEBUG_BAUD);
	Serial.println("Serial debugging active");
	lcdWrite(&lcd, "DEBUGGING");
	#endif

	pinMode(buttonPinDrive, INPUT);  
	pinMode(buttonPinNeutral, INPUT);
	pinMode(buttonPinReverse, INPUT); 
	pinMode(LEDPinDrive, OUTPUT);
	pinMode(LEDPinNeutral, OUTPUT);
	pinMode(LEDPinReverse, OUTPUT);

	digitalWrite(LEDPinDrive, LOW);
	digitalWrite(LEDPinNeutral, LOW);
	digitalWrite(LEDPinReverse, LOW);

	// Setting up buffers
	txMsg.pt_data = &txBuffer[0];	// reference tx message data to tx buffer
	rxMsg.pt_data = &rxBuffer[0];	// reference rx message data to rx buffer

	// Initializing rx CAN packet
	rxMsg.id.ext   = MESSAGE_ID;
	rxMsg.ctrl.ide = MESSAGE_PROTOCOL;
	rxMsg.dlc      = MESSAGE_LENGTH;
	rxMsg.ctrl.rtr = MESSAGE_RTR;

}

void loop() {
	clearBuffer(rxBuffer);

	// Receiving data first
	// ============================================= RECEIVING CAN DATA =============================================
	// Wait for some data to be received
	// while(can_cmd(&rxMsg) != CAN_CMD_ACCEPTED);

	// Wait for command to finish
	// while(can_cmd(&rxMsg) == CAN_STATUS_NOT_COMPLETED);
	// Message data should now be available in rxMsg
	
	// parseRx(&rxMsg);
	
	// ============================================ UPDATING DISPLAY/LEDS ============================================
	if (millis() - lastDisplay > displayCooldown) {
		lastDisplay = millis();
		
		count++;
		sprintf(dBuffer, "Dashboard LCD works! %d", count);
		lcdWrite(&lcd, dBuffer);
	}

	digitalWrite(LEDPinDrive, buttonStateDrive);
	digitalWrite(LEDPinNeutral, buttonStateNeutral);
	digitalWrite(LEDPinReverse, buttonStateReverse);

	// if ((millis()/1000) % 3 == 0) {
	// 	lcd.setBacklight(0x00FF0000);
	// } else if ((millis()/1000) % 3 == 1) {
	// 	lcd.setBacklight(0x0000FF00);
	// } else if ((millis()/1000) % 3 == 2) {
	// 	lcd.setBacklight(0x000000FF);
	// }
	// ============================================= READING INPUT STATE =============================================
	
	// Debouncing
	if (debounceActive[0] == false) {
		if (digitalRead(buttonPinDrive) != buttonStateDrive) {
			debounceActive[0] = true;
			debounceTimers[0] = millis();
		}
	} else if (debounceActive[0] == true) {
		if (digitalRead(buttonPinDrive) == buttonStateDrive) {
			debounceActive[0] = false;
			debounceTimers[0] = millis();
		}
		else if (millis() - debounceTimers[0] >= debounceTime) {
			debounceActive[0] = false;
			buttonStateDrive = digitalRead(buttonPinDrive);
		}
	}

	if (debounceActive[1] == false) {
		if (digitalRead(buttonPinNeutral) != buttonStateNeutral) {
			debounceActive[1] = true;
			debounceTimers[1] = millis();
		}
	} else if (debounceActive[1] == true) {
		if (digitalRead(buttonPinNeutral) == buttonStateNeutral) {
			debounceActive[1] = false;
			debounceTimers[1] = millis();
		}
		else if (millis() - debounceTimers[1] >= debounceTime) {
			debounceActive[1] = false;
			buttonStateNeutral = digitalRead(buttonPinNeutral);
		}
	}

	if (debounceActive[2] == false) {
		if (digitalRead(buttonPinReverse) != buttonStateReverse) {
			debounceActive[2] = true;
			debounceTimers[2] = millis();
		}
	} else if (debounceActive[2] == true) {
		if (digitalRead(buttonPinReverse) == buttonStateReverse) {
			debounceActive[2] = false;
			debounceTimers[2] = millis();
		}
		else if (millis() - debounceTimers[2] >= debounceTime) {
			debounceActive[2] = false;
			buttonStateReverse = digitalRead(buttonPinReverse);
		}
	}
		
	// TODO: Add debouncing for toggles

	// ============================================ TRANSMITTING CAN DATA ============================================
	memcpy(sendData, emptyPacket, 8);
	sendData[0] = buttonStateDrive;
	sendData[1] = buttonStateNeutral;
	sendData[2] = buttonStateReverse;
	sendData[3] = toggleState1;
	sendData[4] = toggleState2;

	memcpy(txBuffer, sendData, 8);

	// Setup CAN packet.
	txMsg.ctrl.ide = MESSAGE_PROTOCOL;  // Set CAN protocol (0: CAN 2.0A, 1: CAN 2.0B)
	txMsg.id.ext   = MESSAGE_ID;        // Set message ID
	txMsg.dlc      = MESSAGE_LENGTH;    // Data length: 8 bytes
	txMsg.ctrl.rtr = MESSAGE_RTR;       // Set rtr bit

	// Sending packet
	txMsg.cmd = CMD_TX_DATA;
	while(can_cmd(&txMsg) != CAN_CMD_ACCEPTED);
	while(can_get_status(&txMsg) == CAN_STATUS_NOT_COMPLETED);
	// Packet sent


	// =========================================== WAITING UNTIL NEXT LOOP ===========================================
	while (millis() - lastLoop < dashLoopTime);
	lastLoop = millis();
}

void parseRx(st_cmd_t* rxMsg) {
	#if (DEBUG > 0)
	serialPrintData(rxMsg);
	#endif
}


// TODO: Rewrite function to only replace relevant characters instead of whole screen
bool lcdWrite(SerLCD* lcd, char* toPrint) {
	int line = 0;
	bool messageFits = true;

	// Check if the message fits on the screen
	if (sizeof(toPrint) / sizeof(toPrint[0]) > (LCDRows * LCDCols)) {
		messageFits = false;
	}

	lcd->clear();
	lcd->setCursor(0, 0);

	// Check if a line break needs to be inserted
	if (sizeof(toPrint) / sizeof(toPrint[0]) > LCDCols) {
		#if (DEBUG > 0)
		Serial.println("Line needs to be split");
		#endif

		char* firstLine = subString(toPrint, 0, (LCDCols - 1));

		#if (DEBUG > 0)
		Serial.println(sprintf("First line: %s", firstLine));
		#endif

		lcd->print(firstLine);
		free(firstLine);

		line++;

		toPrint += LCDCols; // Remove first LCDCols characters from string
		#if (DEBUG > 0)
		Serial.println(sprintf("Second line: %s", toPrint));
		#endif
		lcd->setCursor(0, line);
		lcd->print(toPrint);
	} else {
		lcd->print(toPrint);
	}
	
	return messageFits;
}

// Use free() as soon as possible to reclaim memory
char* subString(char* fullString, int startPos, int endPos) {
	char* outString = (char*) malloc (endPos - startPos);
	memcpy(outString, &fullString[startPos], endPos);
	return outString;
}

#if (DEBUG > 0)
void serialPrintData(st_cmd_t *msg){
	char textBuffer[50] = {0};
	if (msg->ctrl.ide>0){
		sprintf(textBuffer,"id %d ",msg->id.ext);
	}
	else
	{
		sprintf(textBuffer,"id %04x ",msg->id.std);
	}
	Serial.print(textBuffer);

	//  IDE
	sprintf(textBuffer,"ide %d ",msg->ctrl.ide);
	Serial.print(textBuffer);
	//  RTR
	sprintf(textBuffer,"rtr %d ",msg->ctrl.rtr);
	Serial.print(textBuffer);
	//  DLC
	sprintf(textBuffer,"dlc %d ",msg->dlc);
	Serial.print(textBuffer);
	//  Data
	sprintf(textBuffer,"data ");
	Serial.print(textBuffer);

	for (int i =0; i<msg->dlc; i++){
		sprintf(textBuffer,"%02X ",msg->pt_data[i]);
		Serial.print(textBuffer);
	}
	Serial.print("\r\n");
}
#endif

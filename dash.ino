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
#define DEBUG 0
#define DEBUG_BAUD 1000000

#include <ASTCanLib.h>
#include <SerLCD.h>
#include "constants.h"
#include "car_state.h"
#include "dash_screens.h"

enum CAN_MODE {TX, RX};

uint8_t buttonStateDrive = LOW;
uint8_t buttonPulseDrive = LOW;
uint8_t buttonStateNeutral = LOW;
uint8_t buttonPulseNeutral = LOW;
uint8_t buttonStateReverse = LOW;
uint8_t buttonPulseReverse = LOW;
uint8_t toggleState1 = 0;
uint8_t toggleState2 = 0;
uint8_t toggleByte = 0;

st_cmd_t txMsg;
st_cmd_t rxMsg;
st_cmd_t abortMsg;

uint8_t sendData[8] = {0,0,0,0,0,0,0,0};
uint8_t txBuffer[8] = {};
uint8_t rxBuffer[8] = {};
uint8_t abortBuffer[8] = {};
uint32_t lastTx = 0;
uint32_t lastDisplay = 0;
uint32_t lastLoop = 0;
uint16_t dashView = 2;

bool CANStep1 = false;
bool CANStep2 = false;
bool CANBusy = false;
bool CANFault = false;
enum CAN_MODE CANMode;

SerLCD lcd;

bool warning = false;
bool fault = false;
char warningReason[LCDCols * LCDRows - 9];
char faultReason[LCDCols * LCDRows - 7];

uint32_t debounceTimers[5] = {0,0,0,0,0};
bool debounceActive[5] = {false,false,false,false,false};

Car_Data carData(LCDCols, LCDRows);

// Function prototypes
void parseRx(st_cmd_t* rxMsg);
void dashStartup(SerLCD lcd, bool skip, uint8_t LCDCols);
void generateDisplay(char** dashDisplays, char* outString, uint32_t size, uint8_t view, Car_Data carData);
void updateDisplay(SerLCD lcd, char** dashDisplays, uint8_t LCDCols, uint8_t view, Car_Data carData);
void simplePrintLCD(SerLCD lcd, char* text, uint8_t LCDCols, uint8_t LCDRows);

// Set debugging state to 0 if not already set
#ifndef DEBUG
#define DEBUG 0
#endif
// Set debugging serial speed to 9600 baud if not already set
#ifndef DEBUG_BAUD
#define DEBUG_BAUD 9600
#endif

// Defining serial output buffer
#if (DEBUG > 0)
char serialBuffer[64];
#endif

void setup() {
	canInit(500000); // This needs to be called before any serial stuff

	Wire.begin();
	lcd.begin(Wire);
	pinMode(LEDPinReadyToDriveR, OUTPUT);
	pinMode(LEDPinReadyToDriveG, OUTPUT);
	pinMode(LEDPinPrechargeR, OUTPUT);
	pinMode(LEDPinPrechargeG, OUTPUT);
	pinMode(LEDPinTempWarning, OUTPUT);
	pinMode(LEDPinDrive, OUTPUT);
	pinMode(LEDPinNeutral, OUTPUT);
	pinMode(LEDPinReverse, OUTPUT);
	pinMode(buttonPinDrive, INPUT);
	pinMode(buttonPinNeutral, INPUT);
	pinMode(buttonPinReverse, INPUT);

	#if (DEBUG > 0)
		Serial.begin(DEBUG_BAUD);
		Serial.println("\n\n\nSerial debugging enabled\n\n\n");
	#endif

	dashStartup(lcd, false, LCDCols);

	// Setting up buffers
	txMsg.pt_data = &txBuffer[0];	// reference tx message data to tx buffer
	rxMsg.pt_data = &rxBuffer[0];	// reference rx message data to rx buffer
	abortMsg.pt_data = &abortBuffer[0];	// reference abort message data to abort buffer
}

void loop() {
	// ============================================= READING INPUT STATE =============================================
	// Setting pulsed button states to low
	if (buttonPulseDrive == HIGH) {
		buttonPulseDrive = LOW;
	}
	if (buttonPulseNeutral == HIGH) {
		buttonPulseNeutral = LOW;
	}
	if (buttonPulseReverse == HIGH) {
		buttonPulseReverse = LOW;
	}

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
			if (buttonStateDrive == HIGH) {
				buttonPulseDrive = HIGH;
			}
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
			if (buttonStateNeutral == HIGH) {
				buttonPulseNeutral = HIGH;
			}
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
			if (buttonStateReverse == HIGH) {
				buttonStateReverse = HIGH;
			}
		}
	}

	// ================================== SWITCHING SCREENS WITH TOGGLES EVENTUALLY ==================================
	if (buttonPulseDrive == HIGH) {
		if (dashView < (sizeof(dashDisplays)/sizeof(dashDisplays[0]) - 1)) {
			dashView++;
		} else {
			dashView = 0;
		}
	}

	// =================================================== CAN I/O ===================================================
	// ============================================= RECEIVING CAN DATA ==============================================
	if (!CANFault) {
		if (!CANBusy) {
			CANStep1 = false;
			CANStep1 = false;
			CANBusy = true;
			if (millis() - lastTx > txCooldown) {
				CANMode = TX;
				clearBuffer(txBuffer);
				memcpy(sendData, emptyPacket, 8);
				sendData[0] = buttonStateDrive;
				sendData[1] = buttonStateNeutral;
				sendData[2] = buttonStateReverse;
				sendData[3] = toggleState1;
				sendData[4] = toggleState2;
				sendData[7] = toggleByte;
				toggleByte = toggleByte == 0 ? 1 : 0;
				memcpy(txBuffer, sendData, 8);
				// Set up TX CAN packet.
				txMsg.ctrl.ide 	= MESSAGE_PROTOCOL; // Set CAN protocol (0: CAN 2.0A, 1: CAN 2.0B)
				txMsg.id.ext   	= MESSAGE_ID;   	// Set message ID
				txMsg.dlc      	= MESSAGE_LENGTH;   // Data length: 8 bytes
				txMsg.ctrl.rtr 	= MESSAGE_RTR;      // Set rtr bit
				txMsg.cmd 		= CMD_TX_DATA;		// Set TX command
				#if (DEBUG > 0)
				Serial.println("Starting CAN TX");
				#endif
				can_cmd(&txMsg);
			} else {
				CANMode = RX;
				clearBuffer(rxBuffer);
				// Set up RX CAN packet.
				rxMsg.ctrl.ide 	= MESSAGE_PROTOCOL; // Set CAN protocol (0: CAN 2.0A, 1: CAN 2.0B)
				rxMsg.id.ext   	= MESSAGE_ID;   	// Set message ID
				rxMsg.dlc      	= MESSAGE_LENGTH;   // Data length: 8 bytes
				rxMsg.ctrl.rtr 	= MESSAGE_RTR;      // Set rtr bit
				rxMsg.cmd 		= CMD_RX_DATA;		// Set RX command
				#if (DEBUG > 0)
				Serial.println("Starting CAN RX");
				#endif
				can_cmd(&rxMsg);
			}
		} else {
			if (CANMode == TX) {
				// Check if command was accepted
				if (can_cmd(&txMsg) == CAN_CMD_ACCEPTED) {
					#if (DEBUG > 0)
					Serial.println("CAN TX accepted");
					#endif
					CANStep1 = true;
				}
				// Check if command finished
				if (can_cmd(&txMsg) == CAN_STATUS_COMPLETED) {
					#if (DEBUG > 0)
					Serial.println("CAN TX done");
					#endif
					CANStep2 = true;
				}
				// Attempt to recover if CAN refuses command
				if (can_cmd(&txMsg) == CAN_CMD_REFUSED) {
					#if (DEBUG > 0)
					Serial.println("CAN TX refused. Resetting...");
					#endif
					// Set up abort CAN packet.
					abortMsg.ctrl.ide 	= MESSAGE_PROTOCOL; // Set CAN protocol (0: CAN 2.0A, 1: CAN 2.0B)
					abortMsg.id.ext   	= MESSAGE_ID;   	// Set message ID
					abortMsg.dlc      	= MESSAGE_LENGTH;   // Data length: 8 bytes
					abortMsg.ctrl.rtr 	= MESSAGE_RTR;      // Set rtr bit
					abortMsg.cmd 		= CMD_ABORT;		// Set abort command
					while(can_cmd(&abortMsg) != CAN_CMD_ACCEPTED);
					#if (DEBUG > 0)
					Serial.println("CAN abort accepted");
					#endif
					CANStep1 = true;
					while(can_cmd(&abortMsg) == CAN_STATUS_NOT_COMPLETED);
					#if (DEBUG > 0)
					Serial.println("CAN aborted");
					#endif
					// CAN failed, car is not safe to drive
					if (can_get_status(&abortMsg) == CAN_STATUS_ERROR) {
						CANFault = true;
						fault = true;
						snprintf(faultReason, (LCDRows * LCDCols - 7), "CAN BROKE TURN OFF CAR");
						#if (DEBUG > 0)
						Serial.println("CAN failed");
						#endif
					}
					CANStep2 = true;
				}
			} else if (CANMode == RX) {
				// Check if command was accepted
				if (can_cmd(&rxMsg) == CAN_CMD_ACCEPTED) {
					#if (DEBUG > 0)
					Serial.println("CAN RX accepted");
					#endif
									CANStep1 = true;
				}
				// Check if command was finished
				if (can_cmd(&rxMsg) == CAN_STATUS_COMPLETED) {
					#if (DEBUG > 0)
					Serial.println("CAN RX done");
					#endif
					CANStep2 = true;
				}
				// Attempt to recover if CAN refuses command
				if (can_cmd(&rxMsg) == CAN_CMD_REFUSED) {
					#if (DEBUG > 0)
					Serial.println("CAN RX refused. Resetting...");
					#endif
					// Set up abort CAN packet.
					abortMsg.ctrl.ide 	= MESSAGE_PROTOCOL; // Set CAN protocol (0: CAN 2.0A, 1: CAN 2.0B)
					abortMsg.id.ext   	= MESSAGE_ID;   	// Set message ID
					abortMsg.dlc      	= MESSAGE_LENGTH;   // Data length: 8 bytes
					abortMsg.ctrl.rtr 	= MESSAGE_RTR;      // Set rtr bit
					abortMsg.cmd 		= CMD_ABORT;		// Set abort command
					while(can_cmd(&abortMsg) != CAN_CMD_ACCEPTED);
					#if (DEBUG > 0)
					Serial.println("CAN abort accepted");
					#endif
					CANStep1 = true;
					while(can_cmd(&abortMsg) == CAN_STATUS_NOT_COMPLETED);
					#if (DEBUG > 0)
					Serial.println("CAN aborted");
					#endif
					// CAN failed, car is not safe to drive
					if (can_get_status(&abortMsg) == CAN_STATUS_ERROR) {
						CANFault = true;
						fault = true;
						snprintf(faultReason, (LCDRows * LCDCols - 7), "CAN BROKE TURN OFF CAR");
						#if (DEBUG > 0)
						Serial.println("CAN failed");
						#endif
					}
					CANStep2 = true;
				}
			}
			if (CANStep1 && CANStep2) {
				CANBusy = false;
			}
		}
	}

	// Interpreting CVC data
	parseRx(&rxMsg);

	// ============================================ UPDATING DISPLAY/LEDS ============================================
	if (warning || fault) {
		if (warning) {
			dashView = 0;
			snprintf(carData.error, carData.maxError, warningReason);
		}
		if (fault) {
			dashView = 1;
			snprintf(carData.error, carData.maxError, faultReason);
		}
	}
	if (millis() - lastDisplay > displayCooldown) {
		updateDisplay(lcd, dashDisplays, LCDCols, dashView, carData);
		lastDisplay = millis();
	}

	digitalWrite(LEDPinDrive, buttonStateDrive);
	digitalWrite(LEDPinNeutral, buttonStateNeutral);
	digitalWrite(LEDPinReverse, buttonStateReverse);


	// =========================================== WAITING UNTIL NEXT LOOP ===========================================
	while (millis() - lastLoop < dashLoopTime);
	lastLoop = millis();
}


// =============================================== FUNCTION DEFINITIONS ==============================================
void simplePrintLCD(SerLCD lcd, char* text, uint8_t LCDCols, uint8_t LCDRows) {
	int character = 0;
    int length = strlen(text);
    char rowBuffer[LCDCols + 1];
    for (int i = 0; i < LCDRows; i++) {
        for (int j = 0; j < LCDCols; j++) {
            if (character < length) {
                rowBuffer[j] = text[character];
            } else {
                rowBuffer[j] = ' ';
            }
            character++;
        }
		rowBuffer[character] = '\0';
		#if (DEBUG > 0)
		Serial.println(rowBuffer);
		#endif
		lcd.setCursor(0, i);
		lcd.print(rowBuffer);
    }
}

void updateDisplay(SerLCD lcd, char** dashDisplays, uint8_t LCDCols, uint8_t view, Car_Data carData) {
	char dBuffer[LCDCols*LCDRows + 1];

	if (view == 0) { // Warning view
		lcd.setFastBacklight(yellow);
	} else if (view == 1) {
		lcd.setFastBacklight(red);
	} else {
		lcd.setFastBacklight(white);
	}
	generateDisplay(dashDisplays, dBuffer, LCDCols*LCDRows + 1, dashView, carData);
	simplePrintLCD(lcd, dBuffer, LCDCols, LCDRows);
}

void generateDisplay(char** dashDisplays, char* outString, uint32_t size, uint8_t view, Car_Data carData) {
	if (view == 0) {
		// Warning screen
		// 23 character string
		snprintf(outString, size, dashDisplays[view], carData.error);
	} else if (view == 1) {
		// Fault screen
		// 25 character string
		snprintf(outString, size, dashDisplays[view], carData.error);
	} else if (view == 2) {
		char str_v[5];
		char str_i[5];
		char state[8];

		// Speed (MPH), Drive state, Voltage, Current
		// 3 digit integer, 8 character string, 5 digit float, 5 digit float
		dtostrf(carData.voltageHigh, 4, 1, str_v);
		dtostrf(carData.current, 4, 1, str_i);
		carData.stateStr(state);

		snprintf(outString, size, dashDisplays[view], carData.speed, state, str_v, str_i);
	} else if (view == 3) {
		// Battery temperature, Motor temperature, Motor driver temperature
		// 3 digit int, 3 digit int, 3 digit int
		snprintf(outString, size, dashDisplays[view], carData.tempBat, carData.tempMotor, carData.tempDriver);
	} else {
		snprintf(outString, size, "Invalid display ID (%d)", view);
	}
}

void parseRx(st_cmd_t* rxMsg) {
	#if (DEBUG > 0)
	SerialPrintCAN(rxMsg);
	#endif
}


// Testing everything on dashboard power up
void dashStartup(SerLCD lcd, bool skip, uint8_t LCDCols) {
	byte red, green, blue = 0;
	int16_t angle = 0;
	int16_t barSize = 0;
	int16_t newBarSize = 0;
	char loadBar[LCDCols] = {0};
	digitalWrite(LEDPinReadyToDriveR, HIGH);
	digitalWrite(LEDPinReadyToDriveG, HIGH);
	digitalWrite(LEDPinPrechargeR, HIGH);
	digitalWrite(LEDPinPrechargeG, HIGH);
	digitalWrite(LEDPinTempWarning, HIGH);
	digitalWrite(LEDPinDrive, HIGH);
	digitalWrite(LEDPinNeutral, HIGH);
	digitalWrite(LEDPinReverse, HIGH);

	lcd.clear();
	#if (DEBUG > 0)
		lcd.print("SERIAL DEBUG ON");
	#else
		lcd.print("    Starting    ");
	#endif

	if (!skip) {

		loadBar[0] = '\xff';
		for (angle = 0; angle < 360; angle += 6) {
			if (angle < 60) {
				red = 255;
				green = round(angle*4.25-0.01);
				blue = 0;
			} else if (angle < 120) {
				red = round((120-angle)*4.25-0.01);
				green = 255;
				blue = 0;
			} else if (angle < 180) {
				red = 0;
				green = 255;
				blue = round((angle-120)*4.25-0.01);
			} else if (angle < 240) {
				red = 0;
				green = round((240-angle)*4.25-0.01);
				blue = 255;
			} else if (angle < 300) {
				red = round((angle-240)*4.25-0.01);
				green = 0;
				blue = 255;
			} else {
				red = 255, green = 0;
				blue = round((360-angle)*4.25-0.01);
			}

			newBarSize = ((int)(LCDCols * ((float)angle/360))) + 1;
			if (newBarSize != barSize) {
				barSize = newBarSize;
				lcd.setCursor(0, 1);
				loadBar[barSize] = '\xff';
				lcd.print(loadBar);
			}

			lcd.setFastBacklight(red, green, blue);
		}
	}

	lcd.setFastBacklight(white);
	lcd.setCursor(0, 0);
	lcd.print("                ");
	lcd.setCursor(0, 0);
	delay(50);
	lcd.print("Dashboard loaded");
	digitalWrite(LEDPinReadyToDriveR, LOW);
	digitalWrite(LEDPinReadyToDriveG, LOW);
	digitalWrite(LEDPinPrechargeR, LOW);
	digitalWrite(LEDPinPrechargeG, LOW);
	digitalWrite(LEDPinTempWarning, LOW);
	digitalWrite(LEDPinDrive, LOW);
	digitalWrite(LEDPinNeutral, LOW);
	digitalWrite(LEDPinReverse, LOW);
	delay(250);
}

#if (DEBUG > 0)
void SerialPrintCAN(st_cmd_t *msg){
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

	for (int16_t i =0; i<msg->dlc; i++){
		sprintf(textBuffer,"%02X ",msg->pt_data[i]);
		Serial.print(textBuffer);
	}
	Serial.print("\r\n");
}
#endif
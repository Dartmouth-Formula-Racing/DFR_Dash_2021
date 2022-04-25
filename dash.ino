/**
 * @file
 * @author  Andrei Gerashchenko <Andrei.Gerashchenko.UG@dartmouth.edu>
 * @version 0.6
 *
 * 
 *
 * @section DESCRIPTION
 *
 * This file contains code for the dashboard of the 2022 DFR car.
 */

#include <ASTCanLib.h>
#include <SerLCD.h>
#include "constants.h"
#include "car_state.h"
#include "dash_screens.h"

// Enable debugging over serial
#define DEBUG 3
#define DEBUG_BAUD 1000000
#define FORCE_ERROR_VIEW 0

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

uint8_t sendData[8] = {0,0,0,0,0,0,0,0};
uint8_t txBuffer[8] = {};
uint8_t rxBuffer[8] = {};
uint32_t lastTx = 0;
uint32_t lastDisplay = 0;
uint32_t lastLoop = 0;
uint16_t dashView = 2;

bool CANStep1 = false;
bool CANStep2 = false;
bool CANBusy = false;
bool CANFault = false;
uint32_t CANErrors = 0;
enum CAN_MODE CANMode;

SerLCD lcd;
uint8_t lcd_r = 0;
uint8_t lcd_g = 0;
uint8_t lcd_b = 0;

bool warning = false;
bool fault = false;
char warningReason[LCDCols * LCDRows - 9];
char faultReason[LCDCols * LCDRows - 7];


/*
Debouncing timer array:
0 - Drive button
1 - Neutral button
2 - Reverse button
3 - toggle 1 (idk what these are)
4 - toggle 2
*/
uint32_t debounceTimers[5] = {0,0,0,0,0};
bool debounceActive[5] = {false,false,false,false,false};

Car_Data carData(LCDCols, LCDRows);

// Function prototypes
void parseRx(st_cmd_t* rxMsg);
void dashStartup(SerLCD lcd, bool skip, uint8_t LCDCols);
void generateDisplay(char** dashDisplays, char* outString, uint32_t size, uint8_t view, Car_Data carData);
void updateDisplay(SerLCD lcd, char** dashDisplays, uint8_t LCDCols, uint8_t view, Car_Data carData);
void simplePrintLCD(SerLCD lcd, char* text, uint8_t LCDCols, uint8_t LCDRows);
void setLCDBacklight(SerLCD lcd, uint32_t rgb);
void setLCDBacklight(SerLCD lcd, uint8_t r, uint8_t g, uint8_t b);
char* strlshift(char *s);

// Set debugging state to 0 if not already set
#ifndef DEBUG
#define DEBUG 0
#endif
#ifndef FORCE_ERROR_VIEW
#define FORCE_ERROR_VIEW 1
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
		Serial.print("TX CMD status direct: ");
		Serial.print(txMsg.status);
		Serial.print(" | from can_get_status: ");
		Serial.println(can_get_status(&txMsg));
	#endif

	dashStartup(lcd, false, LCDCols);

	// Setting up buffers
	txMsg.pt_data = &txBuffer[0];	// reference tx message data to tx buffer
	rxMsg.pt_data = &rxBuffer[0];	// reference rx message data to rx buffer
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
	
	if (!CANBusy) {
		CANStep1 = false;
		CANStep1 = false;
		CANBusy = true;
		if (millis() - lastTx > txCooldown && CANMode == RX) {
			CANMode = TX;
			clearBuffer(txBuffer);
			memcpy(sendData, emptyPacket, 8);
			/*
			CAN data transmit format:

			[drivebutton (0/1), 
			neutralbutton (0/1), 
			reversebutton (0/1), 
			toggle1 (0/1/2), 
			toggle2 (0/1/2),
			toggleByte (0/1)]
			*/

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
			#if (DEBUG > 1)
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
			#if (DEBUG > 1)
			Serial.println("Starting CAN RX");
			#endif
			can_cmd(&rxMsg);
		}
	} else {
		if (CANMode == TX) {
			// Check if command was accepted
			if (can_cmd(&txMsg) == CAN_CMD_ACCEPTED) {
				#if (DEBUG > 1)
				Serial.println("CAN TX accepted");
				#endif
				CANStep1 = true;
			}
			// Check if command finished
			if (can_cmd(&txMsg) == CAN_STATUS_COMPLETED) {
				#if (DEBUG > 1)
				Serial.println("CAN TX done");
				#endif
				CANStep2 = true;
				CANErrors = 0;
			}
			// Attempt to recover if CAN refuses command
			if (can_cmd(&txMsg) == CAN_CMD_REFUSED) {
				CANErrors++;
				#if (DEBUG > 1)
				Serial.println("CAN TX refused. Resetting...");
				#endif
				txMsg.cmd 		= CMD_ABORT;		// Set abort command
				while(can_cmd(&txMsg) != CAN_CMD_ACCEPTED);
				#if (DEBUG > 1)
				Serial.println("CAN abort accepted");
				#endif
				CANStep1 = true;
				while(can_cmd(&txMsg) == CAN_STATUS_NOT_COMPLETED);
				#if (DEBUG > 1)
				Serial.println("CAN aborted");
				#endif
				CANStep2 = true;
			}
		} else if (CANMode == RX) {
			// Check if command was accepted
			if (can_cmd(&rxMsg) == CAN_CMD_ACCEPTED) {
				#if (DEBUG > 1)
				Serial.println("CAN RX accepted");
				#endif
								CANStep1 = true;
			}
			// Check if command was finished
			if (can_cmd(&rxMsg) == CAN_STATUS_COMPLETED) {
				#if (DEBUG > 1)
				Serial.println("CAN RX done");
				#endif
				CANStep2 = true;
				CANErrors = 0;
				// Interpreting CVC data
				parseRx(&rxMsg);
			}
			// Attempt to recover if CAN refuses command
			if (can_cmd(&rxMsg) == CAN_CMD_REFUSED) {
				CANErrors++;
				#if (DEBUG > 1)
				Serial.println("CAN RX refused. Resetting...");
				#endif
				rxMsg.cmd 		= CMD_ABORT;		// Set abort command
				while(can_cmd(&rxMsg) != CAN_CMD_ACCEPTED);
				#if (DEBUG > 1)
				Serial.println("CAN abort accepted");
				#endif
				CANStep1 = true;
				while(can_cmd(&rxMsg) == CAN_STATUS_NOT_COMPLETED);
				#if (DEBUG > 1)
				Serial.println("CAN aborted");
				#endif
				CANStep2 = true;
			}
		}
		if (CANStep1 && CANStep2) {
			CANBusy = false;
		}
		if (CANErrors >= CANErrorThreshold) {
			// CAN failed, car is not safe to drive
			CANFault = true;
			snprintf(faultReason, (LCDRows * LCDCols - 7), "CAN FAULT PRESS E-STOP");
			#if (DEBUG > 1)
			Serial.println("CAN failed");
			#endif
		} else if (CANFault) {
			CANFault = false;
			#if (DEBUG > 1)
			Serial.println("CAN fault cleared");
			#endif
		}
	}
	// ========================================== CHECKING FOR ERRORS/FAULTS =========================================
	fault = (CANFault); // (CANFault || SomeOtherFault || AnotherFault)
	// warning = ();
	
	// ============================================ UPDATING DISPLAY/LEDS ============================================
	
	if (warning || fault) {
		if (warning) {
			#if (FORCE_ERROR_VIEW)
			dashView = 0;
			#endif
			snprintf(carData.error, carData.maxError, warningReason);
		}
		if (fault) {
			#if (FORCE_ERROR_VIEW)
			dashView = 1;
			#endif
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
/**
* Overload for separate r,g,b setLCBacklight
*
* @param lcd An initialized SerLCD object
* @param rgb A 32 bit unsigned integer representing an rgb value
* @return none.
*/
void setLCDBacklight(SerLCD lcd, uint32_t rgb) {
	// convert from hex triplet to byte values
	uint8_t r = (rgb >> 16) & 0x0000FF;
	uint8_t g = (rgb >> 8) & 0x0000FF;
	uint8_t b = rgb & 0x0000FF;

  	setLCDBacklight(lcd, r, g, b);
}

/**
* A function for changing the RGB backlight of the LCD only
* when the color will be different from what is already set
*
* @param lcd An initialized SerLCD object
* @param r An 8 bit unsigned integer for the red portion
* @param g An 8 bit unsigned integer for the blue portion
* @param b An 8 bit unsigned integer for the green portion
* @return none.
*/
void setLCDBacklight(SerLCD lcd, uint8_t r, uint8_t g, uint8_t b) {
	if (r != lcd_r || g != lcd_g || b != lcd_b) {
		#if (DEBUG > 2)
		Serial.print("New LCD RGB: r = ");
		Serial.print(r);
		Serial.print(" | g = ");
		Serial.print(g);
		Serial.print(" | b = ");
		Serial.println(b);
		#endif
		lcd_r = r;
		lcd_g = g;
		lcd_b = b;
		lcd.setFastBacklight(r, g, b);
	}
}

char* strlshift(char *s) {
    uint32_t n = strlen(s);
	uint32_t i;

	for (i = 0; i < (n - 1); i++) {
		s[i] = s[i + 1];
	}
	s[n - 1] = ' ';

    return s;
}

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
		
		// Removing leading space
		if (rowBuffer[0] == ' ') {
			#if (DEBUG > 0)
			Serial.println("Before string shift");
			Serial.println(rowBuffer);
			#endif
			strlshift(rowBuffer);
		}
		#if (DEBUG > 0)
		Serial.println(rowBuffer);
		#endif

		
		#if (DEBUG > 1)
		Serial.println("Buffer ints: ");
		for (int i = 0; i < strlen(rowBuffer); i++) {
			Serial.print((int)rowBuffer[i]);
			Serial.print(", ");
		}
		Serial.println("");
		#endif
		lcd.setCursor(0, i);
		lcd.print(rowBuffer);
    }
}

void updateDisplay(SerLCD lcd, char** dashDisplays, uint8_t LCDCols, uint8_t view, Car_Data carData) {
	char dBuffer[LCDCols*LCDRows + 1];

	if (view == 0) { // Warning view
		setLCDBacklight(lcd, yellow);
		#if (DEBUG > 0)
		Serial.println("Set backlight to yellow");
		#endif
	} else if (view == 1) {
		setLCDBacklight(lcd, red);
		#if (DEBUG > 0)
		Serial.println("Set backlight to red");
		#endif
	} else {
		setLCDBacklight(lcd, white);
		#if (DEBUG > 0)
		Serial.println("Set backlight to white");
		#endif
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
		
		// Fixing slightly dumb dtostrf implementation bug
		while (str_v[0] == ' ') {
			strlshift(str_v);
		}
		dtostrf(carData.current, 4, 1, str_i);
		while (str_i[0] == ' ') {
			strlshift(str_i);
		}
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


/**
* Interprets a received CAN packet
*
* @param rxMsg an st_cmd_t* structure.
* @return none.
*/
/*
CAN data receive format (big-endian):

Packet 1:
ID			0x7FE
[0] 		Bits 0-3 -> CVC state
[1]			Bits 8-11 -> CVC fault
[2,3,4,5] 	Bits 16-47 -> 32 bit accumulator voltage
[6,7] 		Bits 48-63 -> 16 bit accumulator current

Packet 2:
ID			0x7FF
[]

*/
void parseRx(st_cmd_t* rxMsg) {
	#if (DEBUG > 0)
	SerialPrintCAN(rxMsg);
	#endif
	uint32_t id;
	if (rxMsg->ctrl.ide > 0) {
		id = rxMsg->id.ext;
	} else {
		id = rxMsg->id.std;
	}

	if (id == 0x7FE) {
		carData.cvcState = (cvc_state)rxMsg->pt_data[0];
		carData.cvcFault = (cvc_fault)rxMsg->pt_data[1];
		carData.voltageHigh = ((float)(rxMsg->pt_data[2] << 24 | rxMsg->pt_data[3] << 16 | rxMsg->pt_data[4] << 8 | rxMsg->pt_data[5]))/100.0;
		carData.current = ((float)(rxMsg->pt_data[6] << 8 | rxMsg->pt_data[7]))/100.0;
	} else if (id == 0x7FF) {

	}
}


/**
* Tests the dashboard LEDs and LCD on dashboard
*	startup.
*
* @param rxMsg ASTCanLib st_cmd_t* structure.
* @param skip Determines whether the startup animation 
* 	should be skipped.
* @param LCDCols 8 bit integer that sets the number of 
* 	LCD columns.
* @return none.
*/
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

			setLCDBacklight(lcd, red, green, blue);
		}
	}

	setLCDBacklight(lcd, white);
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
/**
* Formats and prints an ASTCanLib st_cmd_t message to
* 	the serial port
*
* @param msg ASTCanLib st_cmd_t* structure.
* @return none.
*/
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
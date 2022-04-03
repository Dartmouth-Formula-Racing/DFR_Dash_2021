/*
CAN drive states:
Default - 0
00000000 - Neutral (Used to be drive??)
10000000 - Drive
20000000 - Reverse

State priority:
1. Neutral
2. Drive
3. Reverse
Will send request for neutral if all 3 buttons are pressed
*/


#include <ASTCanLib.h>
#include <LiquidCrystal.h>
#include "constants.h"

enum drive_state {DRIVE, NEUTRAL, REVERSE};
enum drive_state inputDriveState = NEUTRAL; // Arduino input drive state
enum drive_state realDriveState = NEUTRAL; // CVC drive state

int buttonStateDrive = HIGH; // LOW true
int buttonStateNeutral = HIGH;
int buttonStateReverse = HIGH;

st_cmd_t txMsg;
st_cmd_t rxMsg;

uint8_t sendData[8] = {0,0,0,0,0,0,0,0};
uint8_t txBuffer[8] = {};
uint8_t rxBuffer[8] = {};
unsigned long lastTx = 0;
unsigned long lastDisplay = 0;
unsigned long lastLoop = 0;

// Works over SPI right now, need to switch to I2C. Missing hardware for this??
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


void setup() {
	canInit(500000); 				// This line needs to be before any serial stuff

	pinMode(buttonPinDrive, INPUT_PULLUP);  
	pinMode(buttonPinNeutral, INPUT_PULLUP);
	pinMode(buttonPinReverse, INPUT_PULLUP); 

	// Setting up buffers
	txMsg.pt_data = &txBuffer[0];	// reference tx message data to tx buffer
	rxMsg.pt_data = &rxBuffer[0];	// reference rx message data to rx buffer

	// Initializing rx CAN packet
	rxMsg.id.ext   = MESSAGE_ID;
	rxMsg.ctrl.ide = MESSAGE_PROTOCOL;
	rxMsg.dlc      = MESSAGE_LENGTH;
	rxMsg.ctrl.rtr = MESSAGE_RTR;

	// set up the LCD's number of columns and rows:	
	lcd.begin(LCDCols, LCDRows);
	// Print to first line
	lcd.print("Dashboard");

	// Set cursor to second line
	lcd.setCursor(0, 1);
	lcd.print("LCD works!");

}

void loop() {
	clearBuffer(rxBuffer);

	// Receiving data first
	// ============================================= RECEIVING CAN DATA =============================================
	// Wait for some data to be received
	while(can_cmd(&rxMsg) != CAN_CMD_ACCEPTED);
	// Wait for command to finish
	while(can_cmd(&rxMsg) == CAN_STATUS_NOT_COMPLETED);
	// Message data should now be available in rxMsg
	
	parseRx(&rxMsg);
	
	// ============================================ UPDATING DISPLAY/LEDS ============================================
	if (millis() - lastDisplay < txCooldown) {

		lastDisplay = millis();
	}
	// ============================================= READING INPUT STATE =============================================
	
	// ============================================ TRANSMITTING CAN DATA ============================================
	

	// =========================================== WAITING UNTIL NEXT LOOP ===========================================
	while (millis() - lastLoop < lastLoop);
	lastLoop = millis();
}

void parseRx(st_cmd_t* rxMsg) {

}

void displayUpdate() {
	
}
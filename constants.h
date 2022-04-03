#ifndef CONSTANTS_H
#define CONSTANTS_H


const int dashLoopTime = 10;        // (milliseconds) | Defines the minimum dashboard loop time

// CAN definitions
#define MESSAGE_ID        123       // Message ID
#define MESSAGE_PROTOCOL  1         // CAN protocol (0: CAN 2.0A, 1: CAN 2.0B)
#define MESSAGE_LENGTH    8         // Data length: 8 bytes
#define MESSAGE_RTR       0         // rtr bit
const uint8_t emptyPacket[8] = {0,0,0,0,0,0,0,0};
const int txCooldown = 50;			// (milliseconds) | Defines the minimum time between CAN transmissions
const int displayCooldown = 1000;	// (milliseconds) | Defines the minimum time between display updates

// Button definitions
const int debounceTime = 15; // (milliseconds) | How long a button must be pressed before its state is applied
// Button data pin definitions
// Connect other end of buttons to ground
const int buttonPinDrive = 12;     
const int buttonPinNeutral = 16;
const int buttonPinReverse = 15;

// Button LED pin definitions
const int LEDPinDrive = 9;
const int LEDPinNeutral = 10;
const int LEDPinReverse = 11;
// LCD constants
const int LCDRows = 2;
const int LCDCols = 16;
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;


#endif
#ifndef CONSTANTS_H
#define CONSTANTS_H

const uint16_t dashLoopTime = 0;        // (milliseconds) | Defines the minimum dashboard loop time


// CAN definitions
#define MESSAGE_ID        123       // Message ID
#define MESSAGE_PROTOCOL  1         // CAN protocol (0: CAN 2.0A, 1: CAN 2.0B)
#define MESSAGE_LENGTH    8         // Data length: 8 bytes
#define MESSAGE_RTR       0         // rtr bit
const uint8_t emptyPacket[8] = {0,0,0,0,0,0,0,0};
const uint16_t txCooldown = 50;			// (milliseconds) | Defines the minimum time between CAN transmissions
const uint16_t displayCooldown = 200;	// (milliseconds) | Defines the minimum time between display updates
const uint32_t CANErrorThreshold = 5;   // Throw fault after 5 consecutive errors


// Button definitions
const uint16_t debounceTime = 15; // (milliseconds) | How long a button must be pressed before its state is applied
// Button data pin definitions
// Connect other end of buttons to ground
const uint16_t buttonPinDrive = 12;     
const uint16_t buttonPinNeutral = 13;
const uint16_t buttonPinReverse = 14;


// Button LED pin definitions
const uint16_t LEDPinReadyToDriveR = 4;
const uint16_t LEDPinReadyToDriveG = 5;
const uint16_t LEDPinPrechargeR = 6;
const uint16_t LEDPinPrechargeG = 7;
const uint16_t LEDPinTempWarning = 8;
const uint16_t LEDPinDrive = 9;
const uint16_t LEDPinNeutral = 10;
const uint16_t LEDPinReverse = 11;


// LCD constants
const uint16_t LCDRows = 2;
const uint16_t LCDCols = 16;
const uint16_t rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
const uint32_t red = 0x00FF0000;
const uint32_t green = 0x0000FF00;
const uint32_t blue = 0x000000FF;
const uint32_t yellow = 0x00FFFF00;
const uint32_t white = 0x00FFFFFF;


#endif
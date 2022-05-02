#ifndef CAR_STATE_H
#define CAR_STATE_H

typedef enum cvc_state_e
{
	CVC_ERROR,
	GLV_FAULT,
	READY,
	PRECHARGE,
	BUZZER,
	DRIVE,
	NEUTRAL,
	REVERSE,
	CHARGING,
	CHARGE_ERROR,
	CHARGE_DONE
} cvc_state;
typedef enum cvc_fault_e 
{
    CVC_OK,
    CVC_WARNING,
    CVC_RST_FAULT,
    CVC_HARD_FAULT
} cvc_fault;

class Car_Data {
public:
    Car_Data(int LCDCols, int LCDRows); // Constructor
    ~Car_Data(); // Destructor
    // Four byte of total voltage of all cells in the battery pack, encoded in 0.01 V
    // (for example, decimal value 70501 from all four bytes means 705.01 V.)
    float voltageHigh = (float)0.0;
    
    // LV battery voltage
    float voltageLow = (float)0.0;

    // Momentary current value (data type is signed 16 bit word). Encoded in 0.1 A units.
    // For example, decimal word value -4098 means -409.8 A (discharging).
    // A decimal value of 173 means 17.3 A (charging).
    float current = (float)0.0;

    int motorRPM = 0;
    float speed = 0;
    int tempBat = 0;
    // Signed integer, actual temperature (in °C) times 10 
    float tempMotor = (float)0.0;
    // Signed integer, actual temperature (in °C) times 10 
    float tempInverter = (float)0.0;

    cvc_state cvcState = NEUTRAL; // State of CVC
    cvc_fault cvcFault = CVC_OK;

    char* error; // Reason for error
    int maxError;

    void stateStr(char* out);

    
};

#endif

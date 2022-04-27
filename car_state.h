#ifndef CAR_STATE_H
#define CAR_STATE_H

typedef enum cvc_state_e {PRECHARGE, DRIVE, NEUTRAL, REVERSE} cvc_state;
typedef enum cvc_fault_e {CVC_OK, CVC_WARNING, CVC_RST_FAULT, CVC_HARD_FAULT} cvc_fault;

class Car_Data {
public:
    float voltageLow = (float)0.0;  // HV battery voltage
    float voltageHigh = (float)0.0; // LV battery voltage - probably no sensor
    float current = (float)0.0;     // Maybe?

    int speed = 0;
    int tempBat = 0;
    int tempMotor = 0;
    int tempDriver = 0;

    cvc_state cvcState = NEUTRAL; // State of CVC
    cvc_fault cvcFault = CVC_OK;

    char* error; // Reason for error
    int maxError;

    void stateStr(char* out);

    Car_Data(int LCDCols, int LCDRows); // Constructor
    ~Car_Data(); // Destructor
};

#endif
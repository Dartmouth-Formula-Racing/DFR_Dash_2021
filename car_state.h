#ifndef CAR_STATE_H
#define CAR_STATE_H

enum drive_state {DRIVE, NEUTRAL, REVERSE};

class Car_Data {
public:
    float voltageLow = (float)99.9;   // HV battery voltage
    float voltageHigh = (float)009.9;  // LV battery voltage
    float current = (float)999.9;

    int speed = 999;
    int tempBat = 999;
    int tempMotor = 999;
    int tempDriver = 999;

    enum drive_state driveState = NEUTRAL; // Drive state of CVC

    char* error; // Reason for error
    int maxError;

    void stateStr(char* out);

    Car_Data(int LCDCols, int LCDRows); // Constructor
    ~Car_Data(); // Destructor
};

#endif
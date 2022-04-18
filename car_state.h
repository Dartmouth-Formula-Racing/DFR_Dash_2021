#ifndef CAR_STATE_H
#define CAR_STATE_H

enum drive_state {DRIVE, NEUTRAL, REVERSE};

class Car_Data {
public:
    float voltageLow = (float)0.0;   // HV battery voltage
    float voltageHigh = (float)0.0;  // LV battery voltage
    float current = (float)0.0;

    int speed = 0;
    int tempBat = 0;
    int tempMotor = 0;
    int tempDriver = 0;

    enum drive_state driveState = NEUTRAL; // Drive state of CVC

    char* error; // Reason for error
    int maxError;

    void stateStr(char* out);

    Car_Data(int LCDCols, int LCDRows); // Constructor
    ~Car_Data(); // Destructor
};

#endif
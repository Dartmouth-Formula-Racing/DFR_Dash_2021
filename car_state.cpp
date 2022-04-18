#include "car_state.h"
#include <Arduino.h>

Car_Data::Car_Data(int LCDCols, int LCDRows) {
    this->error = new char[LCDCols * LCDRows];
    this->maxError = LCDCols * LCDRows;
    int i = 0;
    for (i = 0; i < maxError - 1; i++) {
        error[i] = ' ';
    }
    error[i] = '\0';
}

Car_Data::~Car_Data() {}

void Car_Data::stateStr(char* out) {
    if (this->driveState == DRIVE) {
        snprintf(out, 6, "DRIVE");
    } else if (this->driveState == NEUTRAL) {
        snprintf(out, 8, "NEUTRAL");
    } else if (this->driveState == REVERSE) {
        snprintf(out, 8, "REVERSE");
    }
}
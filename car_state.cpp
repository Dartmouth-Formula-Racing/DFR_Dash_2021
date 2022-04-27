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
    if (this->cvcState == PRECHARGE) { 
        snprintf(out, 8, "PRECHRG"); 
    } else if (this->cvcState == DRIVE) {
        snprintf(out, 6, "DRIVE");
    } else if (this->cvcState == NEUTRAL) {
        snprintf(out, 8, "NEUTRAL");
    } else if (this->cvcState == REVERSE) {
        snprintf(out, 8, "REVERSE");
    } else {
        snprintf(out, 8, "UNKNOWN");
    }
}
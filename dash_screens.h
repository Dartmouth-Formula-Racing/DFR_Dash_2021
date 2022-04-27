#ifndef DASH_SCREENS_H
#define DASH_SCREENS_H

// Designed for a 16x2 LCD
// Character/digit counts include decimal poins

// Speed (MPH), Drive state, Voltage, Current
// 3 digit integer, 8 character string, 5 character string, 5 character string
// White or green backlight
/*

Example:
__________________
|100 MPH  NEUTRAL|
|287.3 V  794.3 A|
‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
*/
char ds_vbat_ibat_vel_screen_t[] = "%-3i MPH %8s%-5s V  %5s A";


// Battery temperature, Motor temperature, Motor driver temperature
// 3 digit int, 3 digit int, 3 digit int
// White or green backlight
/*

Example:
__________________
|TEMPS    DRV 130|
|BAT 327||MTR 231|
‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
*/
char tbat_tmotor_tinverter_t[] = "TEMPS(C) DRV:%-3iBAT:%-3i  MTR:%-3i";

// Charging screen
// 6 character string, 3 digit int
// Green backlight
/*
Example: 
__________________
|    CHARGING    |
|999.99 V   100 %|
‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
*/
char charging_screen_t[] = "    CHARGING    %-6s  V  %3i %";

// Warning screen
// 23 character string
// Yellow backlight
/*

Example: 
__________________
|Warning: Battery|
|temperature high|
‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
*/
char warning_screen_t[] = "Warning: %s";


// Fault screen
// 25 character string
// Red backlight
/*

Example: 
__________________
|Fault: Battery  |
|overheated      |
‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
*/
char fault_screen_t[] = "Fault: %s";




// Adding all of the displays to a single array
char *dashDisplays[] = {warning_screen_t, fault_screen_t, ds_vbat_ibat_vel_screen_t, tbat_tmotor_tinverter_t, charging_screen_t};
enum displays {warning_screen, fault_screen, ds_vbat_ibat_vel_screen, tbat_tmotor_tinverter, charging_screen};


#endif
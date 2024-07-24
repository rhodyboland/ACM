#ifndef ACM_H
#define ACM_H

#include <PicoMQTT.h>
#include "acm.hpp"

/*
Sends up to date switch, dimmer and priority information to the client
would be good to make this only update when user connects.
*/
void update_inputs();

/*
Callback function for subscriptions
*/
void callback(char*, char*);

/*
ADC is non linear on the ESP32 
Function provides a more accurate reading

Returns:
    (double) adjusted voltage

*/
double ReadVoltage(byte);

/*
Read the current from the selected channel in milliamps

Returns:
    (float) current in mA for given channel
*/
float readCurrent(int);

/*
Read current sense of highside switches
*/
void readCS();

/*
Publishes updated data to broker
*/
void sendData();

/*
Initialise acm IO
*/
void init_acm();

#endif
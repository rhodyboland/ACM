#include "acm.hpp"

extern PicoMQTT::Server mqtt;

#define MSG_BUFFER_SIZE (50)
#define INBUILT

// current samples per hour
#define PERHOUR 1800
// battery size in mAh
#define BATSIZE 75000

// Sample counts
// 100 ms main sample time
#define SAMPLE_INTERVAL 100
// 10 100ms samples
#define NUM_SAMPLES 10
// 240 1s samples
#define NUM_SAMPLES_LONG 240

// IO pins
#define VIN_PIN 1
#define Q1_SEN_PIN 45
#define Q1_SEL1_PIN 46
#define Q1_SEL0_PIN 47
#define Q1_CS_PIN 3

// Calibration value for VIN
#define voltCal 0.96046;

// Switched output declaration
const int numOutputs = 6;  // Number of outputs (out0 to out5)
const int outputPin[numOutputs] = {38, 37, 36, 35, 16, 17};

// arrays used for saving output states
int priorities[numOutputs];
int switch_states[numOutputs];
int dimmer_states[numOutputs];

// Sample vairables
unsigned long lastSampleTime = 0;

// data variables
float sumVolt = 0.0;
float sumVoltLong = 0.0;
int sampleCount = 0;
int sampleCountLong = 0;
unsigned long lastReadTime = 0;
int currentChannel = 0;
float totalCurrent = 0;
float cumulativeCurrent = 0;
float remainingmAh = BATSIZE;
unsigned long lastMsg = 0;
bool current_ready = false;
bool battery_critical = false;
float wh = 0.0;
float avgVolt = 0.0;

char msg[MSG_BUFFER_SIZE];

// prefixes for sending to coresponding topic
const char* outputPrefix = "Remote/acm/outputs/out";
const char* dimmerPrefix = "Remote/acm/outputs/dimmer/out";
const char* priorityPrefix = "Remote/acm/outputs/priorities/out";

/*
Sends up to date switch, dimmer and priority information to the client
would be good to make this only update when user connects.
*/
void update_inputs() {
    for (int i = 0; i < numOutputs; i++) {
        String pubString1 = "acm/outputs/out";
        pubString1 += String(i, DEC);
        snprintf(msg, MSG_BUFFER_SIZE, "%d", switch_states[i]);
        mqtt.publish(pubString1.c_str(), msg);

        String pubString2 = "acm/outputs/dimmer/out";
        pubString2 += String(i, DEC);
        snprintf(msg, MSG_BUFFER_SIZE, "%d", dimmer_states[i]);
        mqtt.publish(pubString2.c_str(), msg);

        String pubString3 = "acm/outputs/priorities/out";
        pubString3 += String(i, DEC);
        snprintf(msg, MSG_BUFFER_SIZE, "%d", priorities[i]);
        mqtt.publish(pubString3.c_str(), msg);
    }
}

/*
Callback function for subscriptions
*/
void callback(char* topic, char* payload) {
    unsigned int length = strlen(payload);
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';

    // Check if the received topic matches the expected format

    // Output switches callback from subscribtion
    if (strstr(topic, outputPrefix) != NULL) {
        // Extract the output number from the topic
        int outputNum = atoi(topic + strlen(outputPrefix));
        // Check if the output number is within the valid range
        if (outputNum >= 0 && outputNum < numOutputs) {
            int dataValue = atoi(message);
            String pubString = "acm/outputs/dimmer/out";
            pubString += String(outputNum, DEC);
            if (dataValue == 0) {
                // User commanded OFF
                analogWrite(outputPin[outputNum], 0);
                mqtt.publish(pubString.c_str(), "0");

                // save output to dimmer and switch arrays
                dimmer_states[outputNum] = 0;
                switch_states[outputNum] = 0;
            } else if (dataValue == 1) {
                // User commanded ON
                mqtt.publish(pubString.c_str(), "255");
                analogWrite(outputPin[outputNum], 255);

                // save output to dimmer and switch arrays
                dimmer_states[outputNum] = 255;
                switch_states[outputNum] = 1;
            }
        }

        // Dimmer control slider callback
    } else if (strstr(topic, dimmerPrefix) != NULL) {
        int outputNum = atoi(topic + strlen(dimmerPrefix));
        // Check if the output number is within the valid range
        if (outputNum >= 0 && outputNum < numOutputs) {
            int dataValue = atoi(message);
            if (dataValue > -1 && dataValue < 256) {
                // Set dimmer level
                analogWrite(outputPin[outputNum], dataValue);
                // save to dimmer array
                dimmer_states[outputNum] = dataValue;
            }
        }

        // priorities switch callback
    } else if (strstr(topic, priorityPrefix) != NULL) {
        int outputNum = atoi(topic + strlen(priorityPrefix));
        // Check if the output number is within the valid range
        if (outputNum >= 0 && outputNum < numOutputs) {
            int dataValue = atoi(message);
            // save to priority array
            priorities[outputNum] = dataValue;
        }
    }
}

/*
ADC is non linear on the ESP32
Function provides a more accurate reading

Returns:
    (double) adjusted voltage

*/
double ReadVoltage(byte pin) {
    double reading = analogRead(pin);
    if (reading < 1 || reading > 4095) return 0;
    return -0.000000000000016 * pow(reading, 4) +
           0.000000000118171 * pow(reading, 3) -
           0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading +
           0.034143524634089;
}

/*
Read the current from the selected channel in milliamps

Returns:
    (float) current in mA for given channel
*/
float readCurrent(int channel) {
    digitalWrite(Q1_SEL0_PIN, channel & 0x01);
    digitalWrite(Q1_SEL1_PIN, (channel >> 1) & 0x01);
    delay(5);

    int voltage = analogReadMilliVolts(Q1_CS_PIN);
    float current = (float)voltage / (float)1000 * (float)5050;
    return current;
}

/*
Read current sense of highside switches
*/
void readCS() {
    unsigned long currentMillis = millis();
    // If it's time to read the next channel and there are still channels left
    if (currentMillis - lastReadTime >= 500 && currentChannel <= 3) {
        char msg[MSG_BUFFER_SIZE];
        String pubString = "acm/data/current/out";
        pubString += String(currentChannel, DEC);
        float current = readCurrent(currentChannel);
        totalCurrent += current;
        snprintf(msg, MSG_BUFFER_SIZE, "%.2f", current);
        mqtt.publish(pubString.c_str(), msg);

        // Update the last read time and move on to the next channel
        lastReadTime = currentMillis;
        currentChannel++;
    }
    if (currentChannel > 3) {
        current_ready = true;
    }
}

/*
Publishes updated data to broker
*/
void sendData() {
    unsigned long currentMillis = millis();
    // Sampling logic voltage sense
    if (currentMillis - lastSampleTime >= SAMPLE_INTERVAL &&
        sampleCount < NUM_SAMPLES) {
        float rawVolt = (float)ReadVoltage(VIN_PIN);
        sumVolt += ((rawVolt * (3585.0 + 14930.0)) / 3585.0);
        sampleCount++;
        lastSampleTime = currentMillis;
    }

    // current sense
    if (current_ready == true) {
        char msg[MSG_BUFFER_SIZE];

        // Send Total Current
        snprintf(msg, MSG_BUFFER_SIZE, "%.2f", totalCurrent);
        mqtt.publish("acm/data/current/total", msg);

        // Calculate and Send Instantaneous mAh
        float instantaneousmAh = totalCurrent / PERHOUR;
        snprintf(msg, MSG_BUFFER_SIZE, "%.2f", instantaneousmAh);
        mqtt.publish("acm/data/current/instantaneousmAh", msg);

        // Calculate and Send Cumulative Total Current
        cumulativeCurrent += instantaneousmAh;
        snprintf(msg, MSG_BUFFER_SIZE, "%.2f", cumulativeCurrent);
        mqtt.publish("acm/data/current/cumulative", msg);

        // Calculate and Send remaining mAh
        remainingmAh = BATSIZE - cumulativeCurrent;
        snprintf(msg, MSG_BUFFER_SIZE, "%.2f", remainingmAh);
        mqtt.publish("acm/data/current/remainingmAh", msg);

        // Calculate Wh used
        wh += ((instantaneousmAh / 1000) * avgVolt);
        snprintf(msg, MSG_BUFFER_SIZE, "%.2f", wh);
        mqtt.publish("acm/data/current/Wh", msg);

        current_ready = false;
        // Reset for the next cycle
        currentChannel = 0;
        totalCurrent = 0;
    }

    // voltage sending and logic
    if (sampleCount == NUM_SAMPLES) {
        // battery voltage
        avgVolt = (sumVolt / NUM_SAMPLES) * voltCal;
        snprintf(msg, MSG_BUFFER_SIZE, "%.2f", avgVolt);
        mqtt.publish("acm/data/voltage/inputVoltage", msg);

        // battery percentage base upon voltage
        float batLevel = ((avgVolt)-11.9) * 100;
        if (batLevel > 100) {
            batLevel = 100.0;
        }
        snprintf(msg, MSG_BUFFER_SIZE, "%.2f", batLevel);
        mqtt.publish("acm/data/voltage/batLevel", msg);

        // long sample for auto cutout etc.
        if (sampleCountLong < NUM_SAMPLES_LONG) {
            sampleCountLong++;
            sumVoltLong += avgVolt;
        } else {
            // long term voltage average
            float avgVoltLong = sumVoltLong / (NUM_SAMPLES_LONG);
            snprintf(msg, MSG_BUFFER_SIZE, "%.2f", avgVoltLong);
            mqtt.publish("acm/data/diag", msg);

            // first stage auto cutout
            // turn off all non-priority outputs
            if (avgVoltLong < 11.9) {
                for (int i = 0; i < 6; i++) {
                    if (priorities[i] == 0) {
                        // Turn OFF the output for non-priority items
                        analogWrite(outputPin[i], 0);

                        String pubString = "acm/data/outputs/out";
                        pubString += String(i, DEC);
                        mqtt.publish(pubString.c_str(), "0");

                        dimmer_states[i] = 0;
                        switch_states[i] = 0;
                    }
                }

                // second stage auto cutout
                // turn off all outputs
            }
            if (avgVoltLong < 11.1) {
                for (int i = 0; i < 6; i++) {
                    // Turn OFF the output for all items
                    analogWrite(outputPin[i], 0);

                    String pubString = "acm/data/outputs/out";
                    pubString += String(i, DEC);
                    mqtt.publish(pubString.c_str(), "0");

                    // save to dimmer and switch arrays
                    dimmer_states[i] = 0;
                    switch_states[i] = 0;
                    battery_critical = true;
                }

                // voltage above full logic
            }
            if (avgVoltLong > 12.9) {
                cumulativeCurrent = 0.0;

                // voltage has returned above first stage cutout
                // turn back on only priorities
            }
            if (avgVoltLong > 12.2) {
                for (int i = 0; i < 6; i++) {
                    if (priorities[i] == 1 && battery_critical == true) {
                        // Turn ON the output for priority items
                        analogWrite(outputPin[i], 255);
                        String pubString = "acm/data/outputs/out";
                        pubString += String(i, DEC);
                        mqtt.publish(pubString.c_str(), "1");

                        // save to dimmer and switch arrays
                        dimmer_states[i] = 255;
                        switch_states[i] = 1;
                    }
                }
                battery_critical = false;
            }
            sampleCountLong = 0;
            sumVoltLong = 0.0;
        }

        update_inputs();

        // Reset for the next cycle
        sampleCount = 0;
        sumVolt = 0.0;
        lastMsg = currentMillis;
    }
}

/*
Initialise acm IO
*/
void init_acm() {
    for (int i = 0; i < numOutputs; i++) {
        pinMode(outputPin[i], OUTPUT);
    }

    pinMode(VIN_PIN, INPUT);
    pinMode(Q1_SEN_PIN, OUTPUT);
    pinMode(Q1_SEL0_PIN, OUTPUT);
    pinMode(Q1_SEL1_PIN, OUTPUT);
    pinMode(Q1_CS_PIN, INPUT);

    // sensing enable
    digitalWrite(Q1_SEN_PIN, HIGH);
}
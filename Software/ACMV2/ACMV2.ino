#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ----- Constants and Definitions -----
#define RGB_BRIGHTNESS 255 // Default brightness for the RGB LED
#define V_SENSE 2 // Pin for voltage sensing

// ----- ESP32 OUTPUTS -----
#define LC1 17
#define LC2 18
#define LC3 19
#define LC4 35
#define LC5 36
#define LC6 37
#define LC7 38
#define LC8 39
#define MC1 40
#define MC2 41

// ----- SPI IO-Expander MCP23S17 OUTPUTS -----
#define MCP_ChipSelect 10
#define Q1_SEn 0
#define Q1_SEL0 1
#define Q1_SEL1 2
#define Q1_RST 3
#define Q2_SEn 4
#define Q2_SEL0 5
#define Q2_SEL1 6
#define Q2_RST 7
#define D_SEn 8
#define D_SEL 9
#define D_RST 10
#define INV_CTRL 15

// ----- ESP32 INPUTS -----
#define INV_STATE 1
#define Q1_CS 3
#define Q2_CS 4
#define D_CS 5
#define SENSE_1 6
#define SENSE_2 7
#define EXT_SW1 42
#define EXT_SW2 45

// ----- I/O -----
#define SPARE1 14
#define SPARE2 15
#define SPARE3 16

// USART 1 5V (Victron)
#define U1_RX 8
#define U1_TX 9

// USART 2 3v3 (JK BMS)
#define U2_RX 46
#define U2_TX 47

// CAN SN65HVD230DR
#define CAN_TX 20
#define CAN_RX 21

// MCP23S17 Setup
Adafruit_MCP23X17 mcp;

// ----- BLE Definitions -----
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Configuration and Status Variables
float critVoltage = 10.0;
float cutOutVoltage = 11.8;
float cutInVoltage = 12.2;
bool autoCutoffEnabled = true;
bool alwaysOnChannels[10] = {false, false, false, false, false, false, false, false, false, false};
bool priorityChannels[10] = {false, false, false, false, false, false, false, false, false, false}; // Priority channels
bool lowCurrentStates[8] = {false, false, false, false, false, false, false, false};
bool mediumCurrentStates[2] = {false, false};
int lowCurrentBrightness[8] = {255, 255, 255, 255, 255, 255, 255, 255};

// Voltage divider ratio and calibration factor for voltage sensing
const float voltageDividerRatio = 5.68;
const float voltageCalibrationFactor = 1.00;

// Rolling average buffer for voltage readings
const int voltageBufferSize = 10;
float voltageBuffer[voltageBufferSize] = {0.0};
int voltageBufferIndex = 0;
unsigned long lastVoltageReadMillis = 0; // Last voltage read timestamp

const int currentBufferSize = 10; // Rolling average size for current values

// Buffers to store last 10 current readings for each channel
float lowCurrentBuffers[8][currentBufferSize] = {0};
int lowCurrentIndex[8] = {0};

// Buffers to store last 10 current readings for each medium current channel
float mediumCurrentBuffers[2][currentBufferSize] = {0};
int mediumCurrentIndex[2] = {0};

bool errorPresent = false;
int errorCode = 0;
bool outputsDisabled = false;
float batteryVoltage = 0; // Initial battery voltage
float totatlCurrent = 0;

// ----- Function Prototypes -----
void setLEDColor(char color);
void flashLED(char color, int times, int delayTime);
void pulseLED(char color, int pulseDuration);
void handleAutoShutdownWarning();
void checkBatteryVoltage();
void handleConnectionIndicator();
void handleCommandReceivedIndicator();
void handleErrorIndicator(int errorCode);
void disableOutputs();
void manageErrorState();
float readRealVoltage();
void sendSensorData();
int getLowCurrentPin(int index);
int getMediumCurrentPin(int index);
float readQuadCurrent(int channel, int selectPin, int sensePin);
float readDualCurrent(int channel);
void applyConfiguration(const std::string &config);

// Function to get the corresponding pin for low current channels
int getLowCurrentPin(int index) {
    switch (index) {
        case 1: return LC1;
        case 2: return LC2;
        case 3: return LC3;
        case 4: return LC4;
        case 5: return LC5;
        case 6: return LC6;
        case 7: return LC7;
        case 8: return LC8;
        default: return -1;
    }
}

// Function to get the corresponding pin for medium current channels
int getMediumCurrentPin(int index) {
    switch (index) {
        case 1: return MC1;
        case 2: return MC2;
        default: return -1;
    }
}

// Function to read and calculate the real voltage from the V_SENSE pin
float readRealVoltage() {
    // Read the analog value from the voltage sense pin
    int analogValue = analogRead(V_SENSE);
    
    // Convert the analog value to a voltage in volts
    float measuredVoltage = (analogValue * 3.3 / 4095.0) * voltageDividerRatio;
    
    // Apply the calibration factor for fine-tuning
    return measuredVoltage * voltageCalibrationFactor;
}



// Function to convert a float to a hex string (2 bytes for voltage/current)
String floatToHex(float value, int scale) {
    return String((int)(value * scale), HEX);
}

// Function to convert a state to a hex string (1 or 0)
String stateToHex(bool state) {
    return state ? "1" : "0";
}

// Function to send sensor data over BLE
void sendSensorData() {
    // Voltage and Current (Hex representation)
    String batteryVoltageHex = floatToHex(batteryVoltage, 100); // Voltage scaled by 100 (e.g., 12.34 -> 0C4A)
    String solarVoltageHex = floatToHex(24.0, 100);
    String solarCurrentHex = floatToHex(4.0, 100);
    String solarPowerHex = floatToHex(100.0, 100);    

    // Load Channels (Low current)
    String loadChannelsSection = "L:";
    for (int i = 0; i < 8; i++) {
        String stateHex = stateToHex(lowCurrentStates[i]);
        String brightnessHex = String(lowCurrentBrightness[i], HEX); // Brightness in hex
        float current = (i < 4) ? readQuadCurrent(i, Q1_SEL0, Q1_CS) : readQuadCurrent(i - 4, Q2_SEL0, Q2_CS); // Current for each channel
        String currentHex = floatToHex(current, 1000); // Current scaled by 1000 (e.g., 449.28 -> 1C1A)
        loadChannelsSection += stateHex + brightnessHex + currentHex + ((i < 7) ? "," : ";"); // Comma separated
    }

    // Medium Current Channels
    String mediumChannelsSection = "M:";
    for (int i = 0; i < 2; i++) {
        String stateHex = stateToHex(mediumCurrentStates[i]);
        float current = readDualCurrent(i); // Current for medium channels
        String currentHex = floatToHex(current, 1000);
        mediumChannelsSection += stateHex + currentHex + ((i < 1) ? "," : ";");
    }

    String currentUsageHex = floatToHex(totatlCurrent, 100); // Current usage scaled by 100
    // Build Voltage & Current section
    String voltageCurrentSection = "V:" + batteryVoltageHex + "," + currentUsageHex + "," + solarVoltageHex + "," + solarCurrentHex + "," + solarPowerHex + ";";
    
    // Combine all sections
    String dataPacket = voltageCurrentSection + loadChannelsSection + mediumChannelsSection;
    totatlCurrent = 0;
    // Send the data
    pCharacteristic->setValue(dataPacket.c_str());
    pCharacteristic->notify();

    Serial.println("Data sent: " + dataPacket); // Debug print
}

// Function to read quad current from high side drivers with rolling average
float readQuadCurrent(int channel, int selectPin, int sensePin) {
    mcp.digitalWrite(selectPin, channel & 0x01);
    mcp.digitalWrite(selectPin + 1, (channel >> 1) & 0x01);
    delay(10);
    int adcValue = analogRead(sensePin);
    float voltage = adcValue * (3.3 / 4095.0) * 1000;
    float current = (voltage / 1000.0) * 5050;
    if(selectPin == Q2_SEL0) {
      channel += 4;
    }
    // Serial.print(channel);
    // Serial.print(" raw: ");
    // Serial.println(current);
    // Update the rolling buffer for this channel
    lowCurrentBuffers[channel][lowCurrentIndex[channel]] = current;
    lowCurrentIndex[channel] = (lowCurrentIndex[channel] + 1) % currentBufferSize;

    // Calculate the rolling average
    float sum = 0.0;
    for (int i = 0; i < currentBufferSize; i++) {
        sum += lowCurrentBuffers[channel][i];
    }
    // Serial.print(channel);
    // Serial.print(" average: ");
    // Serial.println(sum/currentBufferSize);
    float avCurrent = sum / currentBufferSize;
    totatlCurrent += avCurrent;
    return avCurrent;
}

// Function to read dual current from high side drivers with rolling average
float readDualCurrent(int channel) {
    mcp.digitalWrite(D_SEL, channel & 0x01);
    delay(10);
    int adcValue = analogRead(D_CS);
    float voltage = adcValue * (3.3 / 4095.0) * 1000;
    float current = (voltage / 1000.0) * 9150;

    // Update the rolling buffer for this medium channel
    mediumCurrentBuffers[channel][mediumCurrentIndex[channel]] = current;
    mediumCurrentIndex[channel] = (mediumCurrentIndex[channel] + 1) % currentBufferSize;

    // Calculate the rolling average
    float sum = 0.0;
    for (int i = 0; i < currentBufferSize; i++) {
        sum += mediumCurrentBuffers[channel][i];
    }
    float avCurrent = sum / currentBufferSize;
    totatlCurrent += avCurrent;
    return avCurrent;
}


// Function to disable all outputs due to low battery
void disableOutputs() {
    for (int i = 1; i <= 8; i++) {
        digitalWrite(getLowCurrentPin(i), LOW);
    }
    for (int i = 1; i <= 2; i++) {
        digitalWrite(getMediumCurrentPin(i), LOW);
    }
    outputsDisabled = true;
    flashLED('R', 3, 300);
}

// Function to check battery voltage and trigger warnings
void checkBatteryVoltage() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastVoltageReadMillis >= 100) { // Check if 100 ms have passed
        lastVoltageReadMillis = currentMillis;
        batteryVoltage = readRealVoltage();
    }

    if (batteryVoltage < critVoltage) {
        handleAutoShutdownWarning();
    } else if (batteryVoltage < cutOutVoltage+0.2) {
        flashLED('Y', 3, 500);
    } else if (batteryVoltage < cutOutVoltage && !outputsDisabled) {
        disableOutputs();
    } else if (batteryVoltage >= cutInVoltage && outputsDisabled) {
        outputsDisabled = false;
        setLEDColor('G');
    }
}

// Function to indicate new connection
void handleConnectionIndicator() {
    flashLED('G', 1, 250);
}

// Function to indicate a command has been received
void handleCommandReceivedIndicator() {
    flashLED('B', 1, 250);
}

// Function to handle error indication (persistent)
void handleErrorIndicator(int code) {
    errorCode = code;
    errorPresent = true;
    manageErrorState();
}

// Manage persistent error state indication
void manageErrorState() {
    if (errorPresent) {
        if (errorCode == 1) {
            flashLED('R', 2, 300);
        } else if (errorCode == 2) {
            flashLED('M', 3, 300);
        }
    }
}

// Function to handle auto shutdown warning
void handleAutoShutdownWarning() {
    pulseLED('R', 1000);
}

// Function to set the RGB LED color
void setLEDColor(char color) {
    switch (color) {
        case 'W':
            neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, RGB_BRIGHTNESS, RGB_BRIGHTNESS);
            break;
        case 'R':
            neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);
            break;
        case 'G':
            neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);
            break;
        case 'B':
            neopixelWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS);
            break;
        case 'Y':
            neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, RGB_BRIGHTNESS, 0);
            break;
        case 'C':
            neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, RGB_BRIGHTNESS);
            break;
        case 'M':
            neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, RGB_BRIGHTNESS);
            break;
        case 'O':
            neopixelWrite(RGB_BUILTIN, 0, 0, 0);
            break;
        default:
            neopixelWrite(RGB_BUILTIN, 0, 0, 0);
            break;
    }
}

// Function to flash the LED a specific number of times
void flashLED(char color, int times, int delayTime) {
    for (int i = 0; i < times; i++) {
        setLEDColor(color);
        delay(delayTime);
        setLEDColor('O');
        delay(delayTime);
    }
}

// Function to create a pulsing effect on the LED
void pulseLED(char color, int pulseDuration) {
    for (int i = 0; i < RGB_BRIGHTNESS; i++) {
        neopixelWrite(RGB_BUILTIN, (color == 'R') ? i : 0, (color == 'G') ? i : 0, (color == 'B') ? i : 0);
        delay(pulseDuration / RGB_BRIGHTNESS);
    }
    for (int i = RGB_BRIGHTNESS; i >= 0; i--) {
        neopixelWrite(RGB_BUILTIN, (color == 'R') ? i : 0, (color == 'G') ? i : 0, (color == 'B') ? i : 0);
        delay(pulseDuration / RGB_BRIGHTNESS);
    }
}

// Function to apply configuration from received command
void applyConfiguration(const std::string &config) {
    size_t pos = 0;
    size_t next_pos = 0;

    Serial.print("Config received: ");
    Serial.println(config.c_str());

    pos = config.find("CO");
    if (pos != std::string::npos) {
        pos += 2;
        next_pos = config.find(" ", pos);
        std::string cutOutStr = config.substr(pos, next_pos - pos);
        cutOutVoltage = atof(cutOutStr.c_str());
        Serial.print("Cut Out Voltage: ");
        Serial.println(cutOutVoltage);
    }

    pos = config.find("CI");
    if (pos != std::string::npos) {
        pos += 2;
        next_pos = config.find(" ", pos);
        std::string cutInStr = config.substr(pos, next_pos - pos);
        cutInVoltage = atof(cutInStr.c_str());
        Serial.print("Cut In Voltage: ");
        Serial.println(cutInVoltage);
    }

    pos = config.find("AC");
    if (pos != std::string::npos) {
        pos += 2;
        next_pos = config.find(" ", pos);
        autoCutoffEnabled = config.substr(pos, next_pos - pos) == "1";
        Serial.print("Auto Cutoff Enabled: ");
        Serial.println(autoCutoffEnabled);
    }

    pos = config.find("AO");
    if (pos != std::string::npos) {
        pos += 2;
        next_pos = config.find(" ", pos);
        std::string aoString = config.substr(pos, next_pos - pos);
        for (int i = 0; i < 10; ++i) {
            alwaysOnChannels[i] = aoString[i] == '1';
        }
    }
    
    pos = config.find("PR");
    if (pos != std::string::npos) {
        pos += 2;
        next_pos = config.find(" ", pos);
        std::string prString = config.substr(pos, next_pos - pos);
        for (int i = 0; i < 10; ++i) {
            priorityChannels[i] = prString[i] == '1';
        }
        Serial.print("Priority Channels: ");
        for (int i = 0; i < 10; ++i) {
            Serial.print(priorityChannels[i]);
        }
        Serial.println();
    }
}

// BLE Server Callback for connection events
class MyCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        handleConnectionIndicator(); // New device connected
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        // Start advertising again
        BLEDevice::startAdvertising();
        flashLED('R', 1, 250); // Device disconnected
    }
};

// BLE Characteristic Callback for handling data received from the app
class CharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = std::string(pCharacteristic->getValue().c_str());
        if (value.length() > 0) {
            Serial.print("Received command: ");
            Serial.println(value.c_str());
            handleCommandReceivedIndicator(); // Command received

            char command = value[0];
            switch (command) {
                case 'L': { // Control low current channels
                    int lcIndex = value[1] - '1';
                    lowCurrentStates[lcIndex] = (value[2] == '1');
                    digitalWrite(getLowCurrentPin(lcIndex + 1), lowCurrentStates[lcIndex] ? HIGH : LOW);
                    break;
                }
                case 'M': { // Control medium current channels
                    int mcIndex = value[1] - '1';
                    mediumCurrentStates[mcIndex] = (value[2] == '1');
                    digitalWrite(getMediumCurrentPin(mcIndex + 1), mediumCurrentStates[mcIndex] ? HIGH : LOW);
                    break;
                }
                case 'B': { // Set brightness for low current outputs
                    int lcIndex = value[1] - '1';
                    lowCurrentBrightness[lcIndex] = std::stoi(value.substr(2));
                    if (lowCurrentStates[lcIndex]) {
                        analogWrite(getLowCurrentPin(lcIndex + 1), lowCurrentBrightness[lcIndex]);
                    }
                    break;
                }
                case 'E': { // Error indicator command
                    int code = std::stoi(value.substr(1));
                    handleErrorIndicator(code);
                    break;
                }
                case 'C': { // Configuration command
                    applyConfiguration(value);
                    break;
                }
                default:
                    handleErrorIndicator(1); // Unknown command error
                    break;
            }
        }
    }
};


void setup() {
    Serial.begin(115200);

    // Initialize MCP23S17 with SPI
    if (!mcp.begin_SPI(MCP_ChipSelect)) {
        Serial.println("Failed to initialize MCP23S17");
        while (1);
    }

    // Set up MCP23S17 pin modes
    mcp.pinMode(Q1_SEn, OUTPUT);
    mcp.pinMode(Q1_SEL0, OUTPUT);
    mcp.pinMode(Q1_SEL1, OUTPUT);
    mcp.pinMode(Q1_RST, OUTPUT);
    mcp.pinMode(Q2_SEn, OUTPUT);
    mcp.pinMode(Q2_SEL0, OUTPUT);
    mcp.pinMode(Q2_SEL1, OUTPUT);
    mcp.pinMode(Q2_RST, OUTPUT);
    mcp.pinMode(D_SEn, OUTPUT);
    mcp.pinMode(D_SEL, OUTPUT);
    mcp.pinMode(D_RST, OUTPUT);

    pinMode(LC1, OUTPUT);
    pinMode(LC2, OUTPUT);
    pinMode(LC3, OUTPUT);
    pinMode(LC4, OUTPUT);
    pinMode(LC5, OUTPUT);
    pinMode(LC6, OUTPUT);
    pinMode(LC7, OUTPUT);
    pinMode(LC8, OUTPUT);
    pinMode(MC1, OUTPUT);
    pinMode(MC2, OUTPUT);

    pinMode(V_SENSE, INPUT); // Setup the voltage sense pin as input
    mcp.digitalWrite(Q1_SEn, HIGH);
    mcp.digitalWrite(Q2_SEn, HIGH);
    BLEDevice::init("ESP32_ACM");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_NOTIFY |
                        BLECharacteristic::PROPERTY_WRITE_NR
                      );

    pCharacteristic->setCallbacks(new CharacteristicCallbacks());
    pCharacteristic->setValue("0000");
    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    BLEDevice::startAdvertising();

    // Initialize the voltage buffer
    for (int i = 0; i < voltageBufferSize; i++) {
        voltageBuffer[i] = 0.0;
    }

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < currentBufferSize; j++) {
            lowCurrentBuffers[i][j] = 0.0;
        }
    }
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < currentBufferSize; j++) {
            mediumCurrentBuffers[i][j] = 0.0;
        }
    }

    xTaskCreate(
        [](void*) {
            while (true) {
                sendSensorData();
                vTaskDelay(500 / portTICK_PERIOD_MS); // Send data every 500 ms
            }
        },
        "SendSensorData",
        4096,
        nullptr,
        1,
        nullptr
    );
}

void loop() {
    checkBatteryVoltage(); // Continuously monitor battery voltage

    // Check error state periodically
    manageErrorState();
}

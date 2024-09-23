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
float cutOutVoltage = 12.0;
float cutInVoltage = 12.4;
bool autoCutoffEnabled = true;
bool alwaysOnChannels[10] = {false, false, false, false, false, false, false, false, false, false};
bool lowCurrentStates[8] = {false, false, false, false, false, false, false, false};
bool mediumCurrentStates[2] = {false, false};
int lowCurrentBrightness[8] = {255, 255, 255, 255, 255, 255, 255, 255};

// Voltage divider ratio and calibration factor for voltage sensing
const float voltageDividerRatio = 5.68;
const float voltageCalibrationFactor = 1.01975;

// Rolling average buffer for voltage readings
const int voltageBufferSize = 10;
float voltageBuffer[voltageBufferSize] = {0.0};
int voltageBufferIndex = 0;
unsigned long lastVoltageReadMillis = 0; // Last voltage read timestamp

bool errorPresent = false;
int errorCode = 0;
bool outputsDisabled = false;
float batteryVoltage = 12.5; // Initial battery voltage

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

class CharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue().c_str();
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

// Function to get the low current pin based on index
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

// Function to get the medium current pin based on index
int getMediumCurrentPin(int index) {
    switch (index) {
        case 1: return MC1;
        case 2: return MC2;
        default: return -1;
    }
}

// Function to read the real voltage from the voltage divider
float readRealVoltage() {
    int adcValue = analogRead(V_SENSE);
    float measuredVoltage = (adcValue * 3.3) / 4095.0;
    float inputVoltage = measuredVoltage * voltageDividerRatio * voltageCalibrationFactor;
    voltageBuffer[voltageBufferIndex] = inputVoltage;
    voltageBufferIndex = (voltageBufferIndex + 1) % voltageBufferSize;

    float sum = 0.0;
    for (int i = 0; i < voltageBufferSize; i++) {
        sum += voltageBuffer[i];
    }
    return sum / voltageBufferSize;
}

// Function to send sensor data over BLE
void sendSensorData() {
    float voltage = batteryVoltage; // Use last read battery voltage
    float current = 5.67;
    bool solarCharging = true;
    float solarPower = 100.0;
    float solarVoltage = 24.0;
    float solarCurrent = 4.0;

    mcp.digitalWrite(Q1_SEn, HIGH);
    float lowCurrents[4];
    for (int i = 0; i < 4; i++) {
        lowCurrents[i] = readQuadCurrent(i, Q1_SEL0, Q1_CS);
    }
    mcp.digitalWrite(Q1_SEn, LOW);

    mcp.digitalWrite(Q2_SEn, HIGH);
    for (int i = 4; i < 8; i++) {
        lowCurrents[i] = readQuadCurrent(i - 4, Q2_SEL0, Q2_CS);
    }
    mcp.digitalWrite(Q2_SEn, LOW);

    mcp.digitalWrite(D_SEn, HIGH);
    float mediumCurrents[2];
    for (int i = 0; i < 2; i++) {
        mediumCurrents[i] = readDualCurrent(i);
    }
    mcp.digitalWrite(D_SEn, LOW);

    char buffer[512]; // Increased buffer size to accommodate all values
    snprintf(buffer, sizeof(buffer), 
             "BV%.2f CU%.2f SC%d SP%.1f SV%.1f SA%.1f "
             "LC1S%d LC2S%d LC3S%d LC4S%d LC5S%d LC6S%d LC7S%d LC8S%d "
             "MC1S%d MC2S%d "
             "LC1B%d LC2B%d LC3B%d LC4B%d LC5B%d LC6B%d LC7B%d LC8B%d "
             "LC1C%.2f LC2C%.2f LC3C%.2f LC4C%.2f LC5C%.2f LC6C%.2f LC7C%.2f LC8C%.2f "
             "MC1C%.2f MC2C%.2f",
             voltage, current, solarCharging,
             solarPower, solarVoltage, solarCurrent,
             lowCurrentStates[0], lowCurrentStates[1], lowCurrentStates[2], lowCurrentStates[3],
             lowCurrentStates[4], lowCurrentStates[5], lowCurrentStates[6], lowCurrentStates[7],
             mediumCurrentStates[0], mediumCurrentStates[1],
             lowCurrentBrightness[0], lowCurrentBrightness[1], lowCurrentBrightness[2], lowCurrentBrightness[3],
             lowCurrentBrightness[4], lowCurrentBrightness[5], lowCurrentBrightness[6], lowCurrentBrightness[7],
             lowCurrents[0], lowCurrents[1], lowCurrents[2], lowCurrents[3],
             lowCurrents[4], lowCurrents[5], lowCurrents[6], lowCurrents[7],
             mediumCurrents[0], mediumCurrents[1]);
    pCharacteristic->setValue(buffer);
    pCharacteristic->notify();
}

// Function to read quad current from high side drivers
float readQuadCurrent(int channel, int selectPin, int sensePin) {
    mcp.digitalWrite(selectPin, channel & 0x01);
    mcp.digitalWrite(selectPin + 1, (channel >> 1) & 0x01);
    delay(10);
    int adcValue = analogRead(sensePin);
    float voltage = adcValue * (3.3 / 4095.0) * 1000;
    return (voltage / 1000.0) * 5050;
}

// Function to read dual current from high side drivers
float readDualCurrent(int channel) {
    mcp.digitalWrite(D_SEL, channel & 0x01);
    delay(10);
    int adcValue = analogRead(D_CS);
    float voltage = adcValue * (3.3 / 4095.0) * 1000;
    return (voltage / 1000.0) * 9150;
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
    flashLED('Y', 3, 300);
}

// Function to check battery voltage and trigger warnings
void checkBatteryVoltage() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastVoltageReadMillis >= 100) { // Check if 100 ms have passed
        lastVoltageReadMillis = currentMillis;
        batteryVoltage = readRealVoltage();
    }

    if (batteryVoltage < 11.8) {
        pulseLED('R', 1000);
        handleAutoShutdownWarning();
    } else if (batteryVoltage < 12.0) {
        flashLED('R', 3, 500);
    } else if (batteryVoltage < 12.2 && !outputsDisabled) {
        disableOutputs();
    } else if (batteryVoltage >= 12.2 && outputsDisabled) {
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
}

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

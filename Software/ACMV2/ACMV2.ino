#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ----- Constants and Definitions -----
#define RGB_BRIGHTNESS 255 // Default brightness for the RGB LED
#define V_SENSE 2          // Pin for voltage sensing

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
#define thermisterSeriesRes 9880

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
float batteryVoltage = 0; // This is the main voltage we feed into the system
float totalCurrent = 0;

float sensor1 = 0;

// Maximum block size to prevent buffer overflow
#define MAX_BLOCK_SIZE 512

// Buffer to store incoming block data
char blockBuffer[MAX_BLOCK_SIZE];
int blockIndex = 0;

// Watchdog variables (for Victron)
unsigned long lastSerialDataMillis = 0; 
const unsigned long WATCHDOG_TIMEOUT = 3000; 
bool serialConnectionActive = false; // Flag for Victron connection

// ----- Victron Fields -----
unsigned int PID; // Product ID
unsigned int FW;  // Firmware version
String SER_num;   // Serial number
int V;            // main battery voltage
int I;            // main battery current
int VPV;          // panel voltage (mV)
int PPV;          // Panel Power (W)
int CS;           // Current state (of operation)
unsigned long OR; // Off reason
int ERR;          // Error code
String LOAD_state;// Load state (ON/OFF)
int IL;           // Load Current (mA)
int H19;          // Yield total (0.01kWh)
int H20;          // Yield today (0.01kWh)
int H21;          // Max power today (W)
int H22;          // Yield Yesterday (0.01kWh)
int H23;          //Max power yesterday (W)
int HSDS;         // Day sequence number (0-364)

// Enum to manage reading states
enum State { 
  READING_LINES,
  BLOCK_COMPLETE
};
State currentState = READING_LINES;

// Temporary storage for the current line being read
String currentLine = "";
uint32_t checksumSum = 0;

// ----------------------------------------------------------------------
// JK-BMS Integration
// ----------------------------------------------------------------------
HardwareSerial SerialBMS(2);  // Use UART #2 on ESP32-S3

// Full request frame for JK-BMS:
static const uint8_t JK_BMS_REQUEST[] = {
  0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00,
  0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x68, 0x00, 0x00, 0x01, 0x29
};

// We read up to 512 bytes from BMS
uint8_t bmsBuf[512];
uint16_t bmsLen = 0;
unsigned long lastBmsDataMillis = 0;
bool bmsConnected = false; // Flag for BMS connection

// BMS data
float bmsVoltage = -1.0f;
float bmsCurrent = 0.0f;
float bmsPower   = 0.0f;
uint8_t bmsSoc   = 0;
// We parse up to 4 cells for demonstration
float cellVoltages[4] = {0,0,0,0};
float avgCellVoltage   = 0.0f;
int16_t mosTemp        = -999;
int16_t batTemp1       = -999;
int16_t batTemp2       = -999;

// Helper: read for up to `timeoutMs` from BMS
uint16_t readBmsFrame(uint16_t timeoutMs) {
  uint16_t idx = 0;
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (SerialBMS.available()) {
      if (idx < 512) {
        bmsBuf[idx++] = SerialBMS.read();
      } else {
        // overflow discard
        SerialBMS.read();
      }
      start = millis(); // reset timeout
    }
  }
  return idx;
}

// Read a 16-bit big-endian integer from buf[offset..offset+1]
uint16_t readU16BE(const uint8_t *buf) {
  return (uint16_t(buf[0]) << 8) | buf[1];
}

// Convert JK “raw temperature” to signed degrees C
int16_t convertJKTemperature(uint16_t rawVal) {
  if (rawVal <= 100) return rawVal;
  return rawVal - 200; 
}

// Current: top bit => charge vs discharge, rest => x0.1A
float convertJKCurrent(uint16_t rawValBE) {
  bool isCharge = (rawValBE & 0x8000) != 0;
  uint16_t magnitude = (rawValBE & 0x7FFF);
  float amps = magnitude * 0.1f;
  return isCharge ? amps : -amps;
}

bool parseBmsFrame(const uint8_t *buf, uint16_t len) {
  if (len < 10) return false;
  if (buf[0] != 0x4E || buf[1] != 0x57) return false;
  // For demonstration, parse these tokens: 0x79 (cells), 0x80..82 temps, 0x83 volt, 0x84 current, 0x85 SOC
  // We'll reset them each parse:
  for (int c = 0; c < 4; c++) cellVoltages[c] = 0.0f;
  avgCellVoltage = 0.0f;
  bmsVoltage = -1.0f;
  bmsCurrent = 0.0f;
  bmsPower   = 0.0f;
  bmsSoc     = 0;
  mosTemp    = -999;
  batTemp1   = -999;
  batTemp2   = -999;

  uint16_t i = 0;
  while (i + 1 < len) {
    uint8_t token = buf[i];
    if (token == 0x79 && i + 1 < len) {
      uint8_t blockSize = buf[i+1];
      uint16_t needed = i + 2 + blockSize;
      if (needed > len) break;

      uint8_t cellCount = blockSize / 3; 
      float sumC = 0.0f;
      for (uint8_t c=0; c<cellCount && c<4; c++) {
        uint16_t off = i + 2 + c*3;
        // [cellIndex, hiVolt, loVolt]
        uint16_t mv = (buf[off+1]<<8) | buf[off+2];
        float cv = mv * 0.001f;
        cellVoltages[c] = cv;
        sumC += cv;
      }
      if (cellCount > 0) {
        avgCellVoltage = sumC / cellCount;
      }
      i = needed;
    }
    else if (token == 0x80 && i+2 < len) {
      uint16_t rawT = readU16BE(&buf[i+1]);
      mosTemp = convertJKTemperature(rawT);
      i += 3;
    }
    else if (token == 0x81 && i+2 < len) {
      uint16_t rawT = readU16BE(&buf[i+1]);
      batTemp1 = convertJKTemperature(rawT);
      i += 3;
    }
    else if (token == 0x82 && i+2 < len) {
      uint16_t rawT = readU16BE(&buf[i+1]);
      batTemp2 = convertJKTemperature(rawT);
      i += 3;
    }
    else if (token == 0x83 && i+2 < len) {
      uint16_t raw10mV = readU16BE(&buf[i+1]);
      bmsVoltage = raw10mV * 0.01f;
      i += 3;
    }
    else if (token == 0x84 && i+2 < len) {
      uint16_t rawCur = readU16BE(&buf[i+1]);
      bmsCurrent = convertJKCurrent(rawCur);
      i += 3;
    }
    else if (token == 0x85 && i+1 < len) {
      bmsSoc = buf[i+1];
      i += 2;
    }
    else {
      i++;
    }
  }

  bmsPower = bmsVoltage * bmsCurrent;

  // Mark success if we at least have a voltage > 0
  if (bmsVoltage > 0.0f) {
    return true;
  }
  return false;
}

// Poll the JK-BMS, read & parse data
void pollBms() {
  // 1) Clear leftover
  while (SerialBMS.available()) {
    SerialBMS.read();
  }
  // 2) Send the full request
  SerialBMS.write(JK_BMS_REQUEST, sizeof(JK_BMS_REQUEST));
  // 3) Wait & read
  uint16_t length = readBmsFrame(1000); 
  if (length == 0) {
    // No data => possibly disconnected
    return;
  }
  if (parseBmsFrame(bmsBuf, length)) {
    bmsConnected = true;
    lastBmsDataMillis = millis();
  }
}

// ----------------------------------------------------------------------
// Function Prototypes from Original Code
// ----------------------------------------------------------------------
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

// ----------------------------------------------------------------------
// BLE Setup
// ----------------------------------------------------------------------
class MyCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        handleConnectionIndicator(); // New device connected
    }
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        BLEDevice::startAdvertising();
        flashLED('R', 1, 250); // Device disconnected
    }
};

class CharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = std::string(pCharacteristic->getValue().c_str());
        if (value.length() > 0) {
            Serial.print("Received command: ");
            Serial.println(value.c_str());
            handleCommandReceivedIndicator(); // Command received

            char command = value[0];
            switch (command) {
                case 'L': { 
                    int lcIndex = value[1] - '1';
                    lowCurrentStates[lcIndex] = (value[2] == '1');
                    digitalWrite(getLowCurrentPin(lcIndex + 1), lowCurrentStates[lcIndex] ? HIGH : LOW);
                    break;
                }
                case 'M': { 
                    int mcIndex = value[1] - '1';
                    mediumCurrentStates[mcIndex] = (value[2] == '1');
                    digitalWrite(getMediumCurrentPin(mcIndex + 1), mediumCurrentStates[mcIndex] ? HIGH : LOW);
                    break;
                }
                case 'B': { 
                    int lcIndex = value[1] - '1';
                    lowCurrentBrightness[lcIndex] = std::stoi(value.substr(2));
                    if (lowCurrentStates[lcIndex]) {
                        analogWrite(getLowCurrentPin(lcIndex + 1), lowCurrentBrightness[lcIndex]);
                    }
                    break;
                }
                case 'E': { 
                    int code = std::stoi(value.substr(1));
                    handleErrorIndicator(code);
                    break;
                }
                case 'C': { 
                    applyConfiguration(value);
                    break;
                }
                default:
                    handleErrorIndicator(1); 
                    break;
            }
        }
    }
};

void getSensor() {
  
  float reading = analogRead(SENSE_1);
 
  // Serial.print("Analog reading "); 
  // Serial.println(reading);
 
  // convert the value to resistance
  reading = (4095 / reading)  - 1;     // (1023/ADC - 1) 
  reading = thermisterSeriesRes / reading;  // 10K / (1023/ADC - 1)
  // Serial.print("Thermistor resistance "); 

  float steinhart;
  steinhart = reading / 9000;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= 3950;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (25 + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15; 
  // Serial.println(steinhart);
  sensor1 = steinhart;
}

// ----------------------------------------------------------------------
// Helper: parse the complete Victron block
// ----------------------------------------------------------------------
void parseVictronBlock(char* buffer) {
  char tempBuffer[MAX_BLOCK_SIZE];
  strncpy(tempBuffer, buffer, MAX_BLOCK_SIZE);
  tempBuffer[MAX_BLOCK_SIZE - 1] = '\0'; 

  // Reset fields
  PID = 0;
  FW  = 0;
  SER_num = "";
  V=I=VPV=PPV=CS=ERR=IL=H19=H20=H21=H22=H23=HSDS=0;
  OR=0;
  LOAD_state="";

  // Tokenize by \r\n
  char* line = strtok(tempBuffer, "\r\n");
  while (line != NULL) {
    char* tabPos = strchr(line, '\t');
    if (tabPos != NULL) {
      *tabPos = '\0'; 
      char* label = line;
      char* value = tabPos + 1;

      if (strcmp(label, "PID") == 0) {
        PID = strtol(value, NULL, 16);
      }
      else if (strcmp(label, "FW") == 0) {
        FW = atoi(value);
      }
      else if (strcmp(label, "SER#") == 0) {
        SER_num = String(value);
      }
      else if (strcmp(label, "V") == 0) {
        V = atoi(value);
      }
      else if (strcmp(label, "I") == 0) {
        I = atoi(value);
      }
      else if (strcmp(label, "VPV") == 0) {
        VPV = atoi(value);
      }
      else if (strcmp(label, "PPV") == 0) {
        PPV = atoi(value);
      }
      else if (strcmp(label, "CS") == 0) {
        CS = atoi(value);
      }
      else if (strcmp(label, "OR") == 0) {
        OR = strtoul(value, NULL, 16);
      }
      else if (strcmp(label, "ERR") == 0) {
        ERR = atoi(value);
      }
      else if (strcmp(label, "LOAD") == 0) {
        LOAD_state = String(value);
      }
      else if (strcmp(label, "IL") == 0) {
        IL = atoi(value);
      }
      else if (strcmp(label, "H19") == 0) {
        H19 = atoi(value);
      }
      else if (strcmp(label, "H20") == 0) {
        H20 = atoi(value);
      }
      else if (strcmp(label, "H21") == 0) {
        H21 = atoi(value);
      }
      else if (strcmp(label, "H22") == 0) {
        H22 = atoi(value);
      }
      else if (strcmp(label, "H23") == 0) {
        H23 = atoi(value);
      }
      else if (strcmp(label, "HSDS") == 0) {
        HSDS = atoi(value);
      }
    }

    line = strtok(NULL, "\r\n");
  }
}

// ----------------------------------------------------------------------
// Setup
// ----------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    // BMS Serial2 at 115200
    SerialBMS.begin(115200, SERIAL_8N1, U2_RX, U2_TX);
    Serial.println("Serial2 (JK-BMS) initialized at 115200 baud");

    // Initialize MCP23S17 (SPI)
    if (!mcp.begin_SPI(MCP_ChipSelect)) {
        Serial.println("Failed to initialize MCP23S17");
        while (1);
    }

    // Setup MCP
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

    pinMode(V_SENSE, INPUT); 
    mcp.digitalWrite(Q1_SEn, HIGH);
    mcp.digitalWrite(Q2_SEn, HIGH);

    // BLE init
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

    // Initialize voltage/current buffers
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

    // Create a task to send sensor data frequently
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

// ----------------------------------------------------------------------
// Main Loop
// ----------------------------------------------------------------------
void loop() {
    // 1) Poll the JK-BMS ~every 1s
    static unsigned long lastBmsPoll = 0;
    if (millis() - lastBmsPoll >= 1000) {
      lastBmsPoll = millis();
      pollBms();
      getSensor();
    }
    // If we haven't received new BMS data in >3s, mark bmsConnected false
    if (bmsConnected && (millis() - lastBmsDataMillis > 3000)) {
      bmsConnected = false;
    }

    // 2) Check battery voltage logic
    checkBatteryVoltage(); 
    manageErrorState();
    

    // 3) Process Victron Serial (Serial1) data
    while (Serial1.available()) {
        char incomingByte = Serial1.read();
        checksumSum += static_cast<unsigned char>(incomingByte);

        if (blockIndex < MAX_BLOCK_SIZE - 1) {
            blockBuffer[blockIndex++] = incomingByte;
            lastSerialDataMillis = millis();
            serialConnectionActive = true;
        } else {
            Serial.println("Error: Block buffer overflow");
            blockIndex = 0;
            checksumSum = 0;
            currentLine = "";
            return;
        }

        if (incomingByte == '\n') {
            if (currentLine.endsWith("\r")) {
                currentLine.remove(currentLine.length() - 1);
            }
            // Check for "Checksum" line
            if (currentLine.startsWith("Checksum\t")) {
                if ((checksumSum % 256) == 0) {
                    blockBuffer[blockIndex] = '\0';
                    parseVictronBlock(blockBuffer);
                } else {
                    Serial.println("Error: Checksum invalid");
                }
                blockIndex = 0;
                checksumSum = 0;
            } 
            currentLine = "";
        } else {
            currentLine += incomingByte;
        }
    }
    // Watchdog for Victron
    if (millis() - lastSerialDataMillis > WATCHDOG_TIMEOUT) {
        if (serialConnectionActive) {
            serialConnectionActive = false;
            Serial.println("Watchdog: No serial data received for 3 seconds.");
        }
    }
}

// ----------------------------------------------------------------------
// Adjusted checkBatteryVoltage: uses BMS if connected, else fallback
// ----------------------------------------------------------------------
void checkBatteryVoltage() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastVoltageReadMillis >= 100) {
        lastVoltageReadMillis = currentMillis;

        // If we have BMS data, use it. Otherwise, read from the voltage divider
        if (bmsConnected && bmsVoltage > 0.0f) {
          batteryVoltage = bmsVoltage;
        } else {
          batteryVoltage = readRealVoltage();
        }
    }

    if (batteryVoltage < critVoltage) {
        handleAutoShutdownWarning();
    } else if (batteryVoltage < cutOutVoltage + 0.2) {
        flashLED('Y', 3, 500);
    } else if (batteryVoltage < cutOutVoltage && !outputsDisabled) {
        disableOutputs();
    } else if (batteryVoltage >= cutInVoltage && outputsDisabled) {
        outputsDisabled = false;
        setLEDColor('G');
    }
}

// ----------------------------------------------------------------------
// Update sendSensorData to include BMS connectivity & values
// ----------------------------------------------------------------------
String floatToHex(float value, int scale);
String stateToHex(bool state);

void sendSensorData() {
    // If bmsConnected => use BMS current for BLE or keep your existing logic?
    // We'll just share BMS current in a new field. Or you can do whatever you want.

    // In your original code, you assemble voltageCurrentSection with:
    //   batteryVoltage (from the global), totalCurrent, solarVoltage, solarCurrent, ...
    // We'll add the BMS connect flag similarly to how we do "connectionFlag" for victron.

    // Voltage & Current:
    String batteryVoltageHex = floatToHex(batteryVoltage, 100);
    String cellAvgHex = floatToHex(avgCellVoltage, 100);
    // totalCurrent is from your readQuadCurrent & readDualCurrent sums
    String currentUsageBMSHex = floatToHex(bmsCurrent, 100);
    String currentUsageHex = floatToHex(totalCurrent, 100);
    // Solar from your code:
    String solarVoltageHex = floatToHex(VPV, 100);
    String solarCurrentHex = floatToHex(I, 100);
    String solarPowerHex = floatToHex(PPV, 100);
    String solarStateHex = floatToHex(CS, 1);

    String batTemp1Hex = floatToHex(batTemp1, 100);

    String batSocHex = floatToHex(bmsSoc, 100);
    // Serial.println(sensor1);
    String sensor1Hex = floatToHex(sensor1, 100);

    // Victron connection flag:
    String connectionFlag = serialConnectionActive ? "1" : "0";

    // **Add BMS flag**:
    String bmsFlag = bmsConnected ? "1" : "0";

    // Combine in the voltageCurrentSection
    // We'll add bmsFlag at the end
    String voltageCurrentSection = "V:" + batteryVoltageHex + "," + currentUsageBMSHex + "," + currentUsageHex + "," 
                                     + solarVoltageHex + "," + solarCurrentHex + "," 
                                     + solarPowerHex + "," + solarStateHex + "," + cellAvgHex  + "," + batTemp1Hex + "," + batSocHex + "," + sensor1Hex + "," 
                                     + connectionFlag + "," + bmsFlag + ";" ;

    // Low current channels
    String loadChannelsSection = "L:";
    for (int i = 0; i < 8; i++) {
        String stateHex = stateToHex(lowCurrentStates[i]);
        String brightnessHex = String(lowCurrentBrightness[i], HEX);
        float current = (i < 4) ? readQuadCurrent(i, Q1_SEL0, Q1_CS) 
                                : readQuadCurrent(i - 4, Q2_SEL0, Q2_CS);
        String currentHex = floatToHex(current, 1000);
        loadChannelsSection += stateHex + brightnessHex + currentHex + ((i < 7) ? "," : ";");
    }

    // Medium current channels
    String mediumChannelsSection = "M:";
    for (int i = 0; i < 2; i++) {
        String stateHex = stateToHex(mediumCurrentStates[i]);
        float current = readDualCurrent(i);
        String currentHex = floatToHex(current, 1000);
        mediumChannelsSection += stateHex + currentHex + ((i < 1) ? "," : ";");
    }

    // Build final string
    String dataPacket = voltageCurrentSection + loadChannelsSection + mediumChannelsSection;
    totalCurrent = 0; // Reset after each send

    pCharacteristic->setValue(dataPacket.c_str());
    pCharacteristic->notify();

    Serial.println("Data sent: " + dataPacket);
}

// Helper for hex conversions
String floatToHex(float value, int scale) {
    return String((int)(value * scale), HEX);
}
String stateToHex(bool state) {
    return state ? "1" : "0";
}

// ----------------------------------------------------------------------
// The rest are your original utility functions
// ----------------------------------------------------------------------
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

void handleAutoShutdownWarning() {
    pulseLED('R', 1000);
}

void setLEDColor(char color) {
    // Placeholder for your neopixelWrite or LED code
    // e.g. neopixelWrite(RGB_BUILTIN, r, g, b)
}

void flashLED(char color, int times, int delayTime) {
    for (int i = 0; i < times; i++) {
        setLEDColor(color);
        delay(delayTime);
        setLEDColor('O');
        delay(delayTime);
    }
}

void pulseLED(char color, int pulseDuration) {
    // Basic example
    for (int i = 0; i < RGB_BRIGHTNESS; i++) {
        // e.g. neopixelWrite(RGB_BUILTIN, i,0,0) if color=='R'
        delay(pulseDuration / RGB_BRIGHTNESS);
    }
    for (int i = RGB_BRIGHTNESS; i >= 0; i--) {
        // ...
        delay(pulseDuration / RGB_BRIGHTNESS);
    }
}

void handleConnectionIndicator() {
    flashLED('G', 1, 250);
}

void handleCommandReceivedIndicator() {
    flashLED('B', 1, 250);
}

void handleErrorIndicator(int code) {
    errorCode = code;
    errorPresent = true;
    manageErrorState();
}

void manageErrorState() {
    if (errorPresent) {
        if (errorCode == 1) {
            flashLED('R', 2, 300);
        } else if (errorCode == 2) {
            flashLED('M', 3, 300);
        }
    }
}

// Reads the actual sense pin for fallback
float readRealVoltage() {
    int analogValue = analogRead(V_SENSE);
    float measuredVoltage = (analogValue * 3.3 / 4095.0) * voltageDividerRatio;
    return measuredVoltage * voltageCalibrationFactor;
}

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

int getMediumCurrentPin(int index) {
    switch (index) {
        case 1: return MC1;
        case 2: return MC2;
        default: return -1;
    }
}

float readQuadCurrent(int channel, int selectPin, int sensePin) {
    mcp.digitalWrite(selectPin, channel & 0x01);
    mcp.digitalWrite(selectPin + 1, (channel >> 1) & 0x01);
    delay(10);
    int adcValue = analogRead(sensePin);
    float voltage = adcValue * (3.3 / 4095.0) * 1000;
    float current = (voltage / 1000.0) * 5050;
    if (selectPin == Q2_SEL0) channel += 4;

    lowCurrentBuffers[channel][lowCurrentIndex[channel]] = current;
    lowCurrentIndex[channel] = (lowCurrentIndex[channel] + 1) % currentBufferSize;

    float sum = 0.0;
    for (int i = 0; i < currentBufferSize; i++) {
        sum += lowCurrentBuffers[channel][i];
    }
    float avCurrent = sum / currentBufferSize;
    totalCurrent += avCurrent;
    return avCurrent;
}

float readDualCurrent(int channel) {
    mcp.digitalWrite(D_SEL, channel & 0x01);
    delay(10);
    int adcValue = analogRead(D_CS);
    float voltage = adcValue * (3.3 / 4095.0) * 1000;
    float current = (voltage / 1000.0) * 9150;

    mediumCurrentBuffers[channel][mediumCurrentIndex[channel]] = current;
    mediumCurrentIndex[channel] = (mediumCurrentIndex[channel] + 1) % currentBufferSize;

    float sum = 0.0;
    for (int i = 0; i < currentBufferSize; i++) {
        sum += mediumCurrentBuffers[channel][i];
    }
    float avCurrent = sum / currentBufferSize;
    totalCurrent += avCurrent;
    return avCurrent;
}

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

# ACM V2
**Second generation of the ACM**

This device is intended to be a control module for accessories that have been fitted to a camping/touring/4wdriving oriented vehicle. These controllable accessories could be lighting, fridge, air commpressor, water pump, tv, and more. 
The device can take inputs from various sources and use internal logic to provide monitoring to the user.
The device is to be app controlled from a mobile phone.

### Hardware:
  - 3rd generation of custom PCB with high side switches from STM. Powered by ESP32-S3
    - STM High side switches with current sensing and auto disconnect
      - 2x VNQ9025AJTR Quad Channel 30A max
      - 1x VND9008AJTR Dual Channel 67A max
  - RGB LED WS2812B
  - ESP32-S3-WROOM1-N16R8
      - USB C Programming and debug
      - Reset and boot buttons
      - Auto selecting 12v or USB 5v 3v3 regulator
  - Inputs for 2 sensors and 2 12v switches/inputs
  - Outputs Term blocks
  - High current 12V input
  - CAN for expansion

<img src="/Hardware/Images/IMG_4668.jpg" width=50% height=50%>

### Software:
#### Platform IO using arduino framework
  - BLE
#### Features (configurable in app)
- Switches for each channel
- Battery voltage and percentage
- Total and individual current measurement
- Auto cut-out of outputs for low battery voltage with priority selection and auto cut-in
- Power used and remaining
- Victron VE Serial input and parsing (MPPT etc.)
- JK BMS input.
- Low side switched inverter control (Highside uses standard out)
- Diagnostic outputs
- CAN to esp32 display coming

#### Custom IOS App ####

# ACM V1
**First generation of the ACM, mainly as a learning development and hardware test.**

This device is intended to be a control module for accessories that have been fitted to a camping/touring/4wdriving oriented vehicle. These controllable accessories could be lighting, fridge, air commpressor, water pump, tv, and more. 
The device can take inputs from various sources and use internal logic to provide monitoring to the user.
The device is to be app controlled from a mobile phone.

### Hardware:
  - 2nd generation of custom PCB with high side switches from STM. Powered by ESP32-S3. First gen used STM32 and ESP32-C3 for wifi.
    - STM High side switches with current sensing and auto disconnect
      - VNQ9025AJTR Quad Channel 30A max
      - VND9008AJTR Dual Channel 67A max
      - Indication LEDs
  - ESP32-S3-DevKitC-1U-N8R2
  - Inputs for sensors and switches
  - Outputs Term blocks
  - High current 12V input with fuse

<img src="/Hardware/Images/IMG_4668.jpg" width=50% height=50%>

### Software:
#### Platform IO using arduino framework
  - PicoMQTT for simultanious broker and client on ESP32
  - Connects to wifi and sets up an MQTT broker, starts publishing data and subscribes to channels to recieve commands.
#### Features (configurable in app)
- Switches for each channel
- Battery voltage and percentage
- Total and individual current measurement
- Auto cut-out of outputs for low battery voltage with priority selection and auto cut-in
- Power used and remaining
- Diagnostic text channels


Used an app for IOS called "IoT OnOff" which supports custom layouts and doesn't look too bad.

<img src="/Software/IMG_4670.jpg" width=50% height=50%>

### Evaluation
The device was tested for a couple of months in a 4wd vehicle with a wifi router. LED lights, inverter and fridge were all tested
  - MQTT communication
    - was reliable and worked very well but;
      - Needs dedicated wifi network (ESP32 may be able to become hotspot also but atleast on iphones this means no network connection/having to constantly switch)
      - Good quality apps with nice UI are limited for this usecase.
  - Outputs
    - Quad channel preformed excellently. Ran fridge not stop, small LED lights and phone chargers simultaneously. Average constant use approx 10-15A.
    - Dual channel did not provide near rated current. Needs substantially more cooling. Ran inverter at approx. 150W for 10s before thermal cutout.
    - Tested PWM dimming on Quad channel. Proved to work well when frequency was selected to reduce noise. Not recommended, led driver would be prefered.
    - Current sensing seemed mostly correct in testing. PWM use causes readings to be inaccurate.
#### Plans
  - PCB full re-design
  - Add input and parsing for Victron products. (MPPT charger etc.)
  - Battery BMS input (serial/can)
  - High current outputs re-evaluation
  

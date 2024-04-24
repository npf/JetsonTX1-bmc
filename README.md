## TX1 BMC with serial console over lan via telnet and power on/off

The BMC microcontroller used is a WEMOS D1 Mini (ESP8266 with WiFi)
- https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/

Controlling a Jetson TX1
- https://elinux.org/Jetson_TX1
- https://developer.download.nvidia.com/assets/embedded/secure/jetson/TX1/docs/JetsonTX1_Developer_Kit_Carrier_Board_Specification.pdf
- https://jetsonhacks.com/nvidia-jetson-tx1-j21-header-pinout/

### Wiring:
Common GND
- TX1 J21.6 <-> WEMOS GND

Power control
- TX1 J6 (Power header)
    - J6.1 <-> NPN Collector (V+)
    - J6.2 <-> NPN Emitter (GND)
- WEMOS D2 <-> NPN Base via R = 10kOhm

Serial console
- TX1 J21.8 <-> WEMOS D7
- TX1 J21.10 <-> WEMOS D8

Debug
- WEMOS USB <-> PC USB 38400 bauds

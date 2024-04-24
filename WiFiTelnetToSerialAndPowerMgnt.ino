/*
  WiFiTelnetToSerialAndPowerMgmt - esp8266 based BMC for Jetson TX1

  Copyright (c) 2024 Pierre Neyron. All rights reserved.

  Fork and enhance the WiFiTelnetToSerial, part of the ESP8266WiFi
  library for Arduino environment.

  Copyright (c) 2015 Hristo Gochkov. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/

/*
  TX1 BMC with serial console over lan via telnet and power on/off

  The BMC microcontroller used is a WEMOS D1 Mini (ESP8266 with WiFi)
    https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
  Controlling a Jetson TX1
    https://elinux.org/Jetson_TX1
    https://developer.download.nvidia.com/assets/embedded/secure/jetson/TX1/docs/JetsonTX1_Developer_Kit_Carrier_Board_Specification.pdf
    https://jetsonhacks.com/nvidia-jetson-tx1-j21-header-pinout/

  Wiring:
    Common GND
      TX1 J21.6 <-> WEMOS GND
    Power control
      TX1 J6 (Power header)
        J6.1 <-> NPN Collector (V+)
        J6.2 <-> NPN Emitter (GND)
      WEMOS D2 <-> NPN Base via R = 10kOhm
    Serial console
      TX1 J21.8 <-> WEMOS D7
      TX1 J21.10 <-> WEMOS D8
    Debug
      WEMOS USB <-> PC USB 38400 bauds
*/

#include <ESP8266WiFi.h>

#include <algorithm>  // std::min

#ifndef STASSID
#define STASSID "myWiFi"
#define STAPSK "myPSK"
#endif

#define BAUD_SERIAL 115200
#define BAUD_LOGGER 38400
#define RXBUFFERSIZE 1024

#define POWER_INTERVAL 30000
#define POWER_DELAY_ON 200
#define POWER_DELAY_OFF 8000
#define KEY_CMD '&'
#define KEY_POWER_ON '1'
#define KEY_POWER_OFF '0'
#define KEY_POWER_REBOOT '8'
#define KEY_SERIAL_FLUSH '7'
#define KEY_SERIAL_RESET '9'

#define PIN_POWER 4 // WEMOS D2 is PIN 4 is down (0V) at boot
#define PIN_LED LED_BUILTIN 
////////////////////////////////////////////////////////////

#include <SoftwareSerial.h>
SoftwareSerial* logger = nullptr;

#define STACK_PROTECTOR 512  // bytes

// how many clients should be able to telnet to this ESP8266
#define MAX_SRV_CLIENTS 2
const char* ssid = STASSID;
const char* password = STAPSK;

const int port = 23;

WiFiServer server(port);
WiFiClient serverClients[MAX_SRV_CLIENTS];

unsigned long nextLowMillis = 0;
unsigned long nextHighMillis = 0;
unsigned long currentMillis = 0;

bool autoPowerOn = true;

void setup() {
  pinMode(PIN_LED, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
  digitalWrite(PIN_LED, LOW);  // Turn the LED ON (PULLED UP)
  pinMode(PIN_POWER, OUTPUT);  // Initialize the TX1 POWER pin as an output
  digitalWrite(PIN_POWER, LOW);  // Do not power ON TX1

  Serial.begin(BAUD_SERIAL);
  Serial.setRxBufferSize(RXBUFFERSIZE);

  Serial.swap();
  // Hardware serial is now on RX:GPIO13 TX:GPIO15
  // use EspSoftwareSerial on regular RX(3)/TX(1) for logging
  logger = new SoftwareSerial(3, 1);
  logger->begin(BAUD_LOGGER);
  logger->enableIntTx(false);
  logger->printf("\n\n[%lu] Using EspSoftwareSerial for logging\n", millis());
  logger->printf("[%lu] ", millis(), ESP.getFullVersion());
  logger->println(ESP.getFullVersion());
  logger->printf("[%lu] Serial baud: %d (8n1: %d KB/s)\n", millis(), BAUD_SERIAL, BAUD_SERIAL * 8 / 10 / 1024);
  logger->printf("[%lu] Serial receive buffer size: %d bytes\n", millis(), RXBUFFERSIZE);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  logger->printf("\n[%lu] Connecting to ", millis());
  logger->println(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    logger->print('.');
    delay(500);
  }
  logger->println();
  logger->printf("[%lu] Connected, address=", millis());
  logger->println(WiFi.localIP());

  // start server
  server.begin();
  server.setNoDelay(true);

  logger->printf("[%lu] Ready! Use 'telnet ", millis());
  logger->print(WiFi.localIP());
  logger->printf(" %d' to connect\n", port);
  nextHighMillis = millis() + POWER_INTERVAL;
  nextLowMillis = 0;
  logger->printf("[%lu] Next power-on trigger after %lu\n", millis(), nextHighMillis);
}

bool is_past(unsigned long t) {
  //return (t < currentMillis); //No overflow management -> bug after 49 days
  //return (t < currentMillis) ^ (abs((long long) t - (long long) currentMillis)) >= (1 << (sizeof(t) * 8 - 1)));
  unsigned long delta = (t > currentMillis)?(t - currentMillis):(currentMillis - t);
  return (t < currentMillis) ^ (delta >= (1 << (sizeof(t) * 8 - 1)));
}

void loop() {
  //delay(3000);
  currentMillis = millis();
  if (nextLowMillis && is_past(nextLowMillis)) {
    if (autoPowerOn) {
      nextHighMillis = currentMillis + POWER_INTERVAL;
      if (nextHighMillis) nextHighMillis++; // avoid 0 when overflow
      logger->printf("[%lu] Untrigger, next power-on trigger after %lu\n", currentMillis, nextHighMillis);
    } else {
      nextHighMillis = 0;
      logger->printf("[%lu] Untrigger, no next power-on trigger\n", currentMillis);
    }
    nextLowMillis = 0;
    digitalWrite(PIN_LED, LOW); // LED ON
    digitalWrite(PIN_POWER, LOW); // UNTRIGGER
  } else if (nextHighMillis && is_past(nextHighMillis)) {
    nextLowMillis = currentMillis + POWER_DELAY_ON;
    if (nextLowMillis) nextLowMillis++; // avoid 0 when overflow
    nextHighMillis = 0;
    logger->printf("[%lu] Trigger power-on, untrigger after %lu\n", currentMillis, nextLowMillis);
    digitalWrite(PIN_LED, HIGH); // LED OFF
    digitalWrite(PIN_POWER, HIGH); // TRIGGER
  }
  
  // check if there are any new clients
  if (server.hasClient()) {
    // find free/disconnected spot
    int i;
    for (i = 0; i < MAX_SRV_CLIENTS; i++)
      if (!serverClients[i]) {  // equivalent to !serverClients[i].connected()
        serverClients[i] = server.accept();
        logger->printf("[%lu] New telnet client: index ", currentMillis);
        logger->println(i);
        break;
      }

    // no free/disconnected spot so reject
    if (i == MAX_SRV_CLIENTS) {
      server.accept().println("busy");
      // hints: server.accept() is a WiFiClient with short-term scope
      // when out of scope, a WiFiClient will
      // - flush() - all data will be sent
      // - stop() - automatically too
      logger->printf("[%lu] Server is busy with %d active connections\n", currentMillis, MAX_SRV_CLIENTS);
    }
  }

  // check TCP clients for data
#if 0
  // Incredibly, this code is faster than the buffered one below - #4620 is needed
  // loopback/3000000baud average 348KB/s
  for (int i = 0; i < MAX_SRV_CLIENTS; i++)
    while (serverClients[i].available() && Serial.availableForWrite() > 0) {
      // working char by char is not very efficient
      Serial.write(serverClients[i].read());
    }
#else
  // loopback/3000000baud average: 312KB/s
  for (int i = 0; i < MAX_SRV_CLIENTS; i++)
    while (serverClients[i].available() && Serial.availableForWrite() > 0) {
      size_t maxToSerial = std::min(serverClients[i].available(), Serial.availableForWrite());
      maxToSerial = std::min(maxToSerial, (size_t)STACK_PROTECTOR);
      uint8_t buf[maxToSerial];
      size_t tcp_got = serverClients[i].read(buf, maxToSerial);

      if (tcp_got >= 2 && (char) buf[0] == KEY_CMD) {
        logger->printf("[%lu] Read %c%c\n", currentMillis, (char) buf[0], (char) buf[1]);
        switch ((char) buf[1]) {
          case KEY_POWER_OFF:
            autoPowerOn = false;
            logger->printf("[%lu] Unactivate auto power-on\n", currentMillis);
            // Then, go on with the reboot code (no break)
          case KEY_POWER_REBOOT:
            nextLowMillis = currentMillis + POWER_DELAY_OFF;
            if (nextLowMillis) nextLowMillis++; // avoid 0 when overflow
            nextHighMillis = 0;
            logger->printf("[%lu] Trigger power-off, untrigger after %lu\n", currentMillis, nextLowMillis);
            digitalWrite(PIN_LED, HIGH); // LED OFF
            digitalWrite(PIN_POWER, HIGH); // TRIGGER
            break;
          case KEY_POWER_ON:
            autoPowerOn = true;
            logger->printf("[%lu] Activate auto power-on\n", currentMillis);
            if (!nextLowMillis) {
              nextHighMillis = currentMillis;
              if (nextHighMillis) nextHighMillis++; // avoid 0 when overflow
            }
            break;
          case KEY_SERIAL_RESET:
            logger->printf("[%lu] Reset serial line\n", currentMillis);
            Serial.end();
            delay(100);
            Serial.begin(BAUD_SERIAL);
            Serial.setRxBufferSize(RXBUFFERSIZE);
            break;
          case KEY_SERIAL_FLUSH:
            logger->printf("[%lu] Flush serial line\n", currentMillis);
            Serial.flush();
            break;
        } 
      } else {
        size_t serial_sent = Serial.write(buf, tcp_got);
        if (serial_sent != maxToSerial) {
          logger->printf("[%lu] Len mismatch: available:%zd tcp-read:%zd serial-write:%zd\n", currentMillis, maxToSerial, tcp_got, serial_sent);
        }
      }
    }
#endif

  // determine maximum output size "fair TCP use"
  // client.availableForWrite() returns 0 when !client.connected()
  int maxToTcp = 0;
  for (int i = 0; i < MAX_SRV_CLIENTS; i++)
    if (serverClients[i]) {
      int afw = serverClients[i].availableForWrite();
      if (afw) {
        if (!maxToTcp) {
          maxToTcp = afw;
        } else {
          maxToTcp = std::min(maxToTcp, afw);
        }
      } else {
        // warn but ignore congested clients
        logger->printf("[%lu] One client is congested\n", currentMillis);
      }
    }

  // check UART for data
  size_t len = std::min(Serial.available(), maxToTcp);
  len = std::min(len, (size_t)STACK_PROTECTOR);
  if (len) {
    uint8_t sbuf[len];
    int serial_got = Serial.readBytes(sbuf, len);
    // push UART data to all connected telnet clients
    for (int i = 0; i < MAX_SRV_CLIENTS; i++)
      // if client.availableForWrite() was 0 (congested)
      // and increased since then,
      // ensure write space is sufficient:
      if (serverClients[i].availableForWrite() >= serial_got) {
        size_t tcp_sent = serverClients[i].write(sbuf, serial_got);
        if (tcp_sent != len) { 
          logger->printf("[%lu] Len mismatch: available:%zd serial-read:%zd tcp-write:%zd\n", currentMillis, len, serial_got, tcp_sent);
        }
      }
  }
}

/* HW Configuration ############################################################################################
    Single antenna, IMU, & dual antenna code for AgOpenGPS
    If dual right antenna is for position (must enter this location in AgOpen), left Antenna is for heading & roll

    Connection plan:
    WT32 Serial 2 RX (5) to F9P Position receiver TX1 (Position data)
    WT32 Serial 2 TX (17) to F9P Position receiver RX1 (RTCM data for RTK)

    WT32 Serial1 RX (2) to IMU SCL pin for RVC mode (IMU data)

    WT32 CAN-R (33) to CAN-R driver (CAN data)
    WT32 CAN-T (32) to CAN-T driver (CAN data)

    WT32 I/O Pin WAS (35 in case of internal reader) to I/O pin was-high (AMP23 input pin 2)

    WT32 I/O Pin SDA (15 in case of external reader) to ADS1115 SDA
    WT32 I/O Pin SCL (14 in case of external reader) to ADS1115 SCL
    ADS1115 A0 Pin (in case of external reader) to I/O pin was-high (AMP23 input pin 2)

    WT32 I/O Pin work (1) to button opto (AMP23 input pin 9)
    WT32 I/O Pin steer (36) to button opto (AMP23 input pin 8)
    WT32 I/O Pin remote (39) to button opto (AMP23 input pin 10)

    WT32 I/O Pin dir (0) to cytron (1)
    WT32 I/O Pin pwm (4) to cytron (2)
    WT32 I/O Pin pwm2 (12) to cytron (3) for cytron modified

    Configuration of receiver
    Position F9P
    CFG-RATE-MEAS - 100 ms -> 10 Hz
    CFG-UART1-BAUDRATE 460800
    Serial 1 In - RTCM (Correction Data from AOG)
    Serial 1 Out - NMEA GGA
    CFG-UART2-BAUDRATE 460800
    Serial 2 Out - RTCM 1074,1084,1094,1230,4072.0 (Correction data for Heading F9P, Moving Base)  
    1124 is not needed (China’s BeiDou system) - Save F9P brain power 

    Heading F9P
    CFG-RATE-MEAS - 100 ms -> 10 Hz
    CFG-UART1-BAUDRATE 460800
    Serial 1 Out - UBX-NAV-RELPOSNED
    CFG-UART2-BAUDRATE 460800
    Serial 2 In RTCM
*/

#define ASYNC_UDP_WT32_ETH01_DEBUG_PORT      Serial
// Use from 0 to 4. Higher number, more debugging messages and memory usage.
#define _ASYNC_UDP_WT32_ETH01_LOGLEVEL_      1

#include <AsyncUDP_WT32_ETH01.h>

#include "JsonDB.h"
#include <LittleFS.h>
#include "Autosteering.h"
#include "WebserverHelper.h"
/////////////////////////////////////////////

// Global Variables ##########################################################################
JsonDB db("/configuration.json"); // Create a database for data interaction
AsyncUDP udpAutosteer;                // A UDP instance to let us send and receive packets over UDP for Autosteer
AsyncUDP udpNtrip;                    // A UDP instance to receive packets over UDP for Ntrip
Autosteering aog;                     // Create empty main processing object for autosteering
//############################################################################################

void setup(){
  Serial.begin(115200);// Serial for debugging TX0/RX0 on WT32
  delay(3000);
  while (!Serial);
  Serial.print("\nStarting AOG pcb board on " + String(ARDUINO_BOARD));
  Serial.println(" with " + String(SHIELD_TYPE));
  Serial.println(ASYNC_UDP_WT32_ETH01_VERSION);

  // Mount file system. Initialize.
  if(!LittleFS.begin(true)){
    Serial.println("LittleFS Mount Failed");
    return;
  }

  // Configure Resources from "configuration.json" file
  db.begin(LittleFS);//(LittleFS, true) for reset configuration files
  // init webserver setup mode, only when the three buttons are pressed siimultaneously at the ESP begining
  if((digitalRead(db.conf.remote_pin)==LOW) && (digitalRead(db.conf.work_pin)==LOW) && (digitalRead(db.conf.steer_pin)==LOW)){
    db.saveConfiguration();
    db.saveSteerSettings();
    db.saveSteerConfiguration();
    setServerMode();
  }
  // Set up main objects
  aog.begin(&db, &udpAutosteer, true, true);

  // Init Network
  WT32_ETH01_onEvent();
  ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER);
  ETH.config(db.conf.eth_ip, db.conf.eth_gateway, db.conf.eth_subnet, db.conf.eth_dns);
  WT32_ETH01_waitForConnect();
  Serial.print("AOG board started @ IP address: ");
  Serial.println(ETH.localIP());

  // Register UDP callback functions to server & ports
  if (udpAutosteer.listen(db.conf.server_autosteer_port)){
    Serial.printf("UDP connected to autosteer port (%d)\n", db.conf.server_autosteer_port);
    udpAutosteer.onPacket([](AsyncUDPPacket packet){ aog.parseUdp(packet);});
  }

  if (udpNtrip.listen(db.conf.server_ntrip_port)){
    Serial.printf("UDP connected to ntrip port (%d)\n",db.conf.server_ntrip_port);
    udpNtrip.onPacket([](AsyncUDPPacket packet){ aog.udpNtrip(packet);});
  }

  Serial.println(F("\nSetup complete, waiting for AgOpenGPS ######################################################################\n"));
}

void loop(){
  aog.run();
}
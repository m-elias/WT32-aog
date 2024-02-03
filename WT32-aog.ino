/* HW Configuration ############################################################################################
    Single antenna, IMU, & dual antenna code for AgOpenGPS
    If dual right antenna is for position (must enter this location in AgOpen), left Antenna is for heading & roll

    Connection plan:
    WT32 Serial 2 RX (5) to F9P Position receiver TX1 (Position data)
    WT32 Serial 2 TX (17) to F9P Position receiver RX1 (RTCM data for RTK)

    WT32 Serial1 RX (2) to IMU SCL pin for RVC mode (IMU data). Tx (21)?

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

    SW Configuration  #############################################################################################
    - Arduino v2.2.1
    - ArduinoJson v6.21.3
    - SimpleKalmanFilter v0.1
    For ESP32 family
    - esp32 v2.0.11
    - AsyncUDP_WT32_ETH01 v2.1.0
    - ESPAsyncWebServer v1.2.3 (https://github.com/me-no-dev/ESPAsyncWebServer.git)
    - ACAN_ESP32 1.1.0
    For Teensy family
    - Teensyduino v1.58.1
    - QNEthernet v0.24.0 (using the one on the repository included with the code)
    - AsyncUDP_Teensy41 v1.2.1 (using the one on the repository included with the code)
    - AsyncWebServer_Teensy41 v1.6.2 (with all dependencies) (using the one on the repository included with the code)
    - ACAN_T4 1.1.6
*/
#define FIRMWARE_VERSION  "v0.0.1"
#ifdef ARDUINO_BOARD
  #define MICRO_VERSION     1 //1: WT32_ETH01; 2: Teensy41
 #else
  #define MICRO_VERSION     2 //1: WT32_ETH01; 2: Teensy41
#endif

// Libraries setup
#include <LittleFS.h>

#if MICRO_VERSION == 1
  #define ASYNC_UDP_WT32_ETH01_DEBUG_PORT      Serial
  #define _ASYNC_UDP_WT32_ETH01_LOGLEVEL_      1 // Use from 0 to 4. Higher number, more debugging messages and memory usage.

  // External Libraries in use
  #include <AsyncUDP_WT32_ETH01.h>
#endif
#if MICRO_VERSION == 2
 #define SHIELD_TYPE     "Teensy4.1 QNEthernet"
 #define USE_NATIVE_ETHERNET         false
 #define USE_QN_ETHERNET             true
 #define USING_DHCP            false
 #define SD_CONFIG SdioConfig(FIFO_SDIO) // Use Teensy SDIO

 // External Libraries in use
 #include "lib/vendor/QNEthernet/src/QNEthernet.h"       // https://github.com/ssilverman/QNEthernet
 using namespace qindesign::network;

 LittleFS_Program lfs;
#endif

// Internal Libraries in use
#include "JsonDB.h"
#include "Autosteering.h"
/////////////////////////////////////////////

// Global Variables ##########################################################################
JsonDB db("/configuration.json");     // Create a database for data interaction
AsyncUDP udpAutosteer;                // A UDP instance to let us send and receive packets over UDP for Autosteer
AsyncUDP udpNtrip;                    // A UDP instance to receive packets over UDP for Ntrip
Autosteering aog;                     // Create empty main processing object for autosteering
//############################################################################################

#include "WebserverHelper.h"

void setup(){
  Serial.begin(115200);// Serial for debugging TX0/RX0 on WT32
  delay(3000);
  while (!Serial);
  Serial.printf("\nStarting AIO Firmware on %s with release:%s\n",MICRO_VERSION==1?"WT32-ETH01":"Teensy 4.1", FIRMWARE_VERSION);

 #if MICRO_VERSION == 1
  // Mount file system. Initialize.
  if(!LittleFS.begin(true)){
    Serial.println("LittleFS Mount Failed");
    return;
  }
  // Configure Resources from "configuration.json" file
  db.begin(LittleFS);//(LittleFS, true) for reset configuration files

  // Init Network
  WT32_ETH01_onEvent();
  ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER);
  ETH.config(db.conf.eth_ip, db.conf.eth_gateway, db.conf.eth_subnet, db.conf.eth_dns);
  WT32_ETH01_waitForConnect();
  Serial.print("AOG board started @ IP address: ");
  Serial.println(ETH.localIP());
 #endif
 #if MICRO_VERSION == 2
  if(CrashReport){
    while (!Serial);
    Serial.print(CrashReport);
  }

  // Mount file system. Initialize.
  if (!lfs.begin(960*1024)) {// checks that the LittFS program has started with the disk size specified
    Serial.printf("Error starting %s\n", "PROGRAM FLASH DISK");
    Serial.println("LittleFS Mount Failed");
    return;
  }
  Serial.println("LittleFS initialized.");
  
  // Configure Resources from "configuration.json" file
  db.begin(lfs,true);//(lfs, true) for reset configuration files

  // Init Network
  if(USING_DHCP){
    // Start the Ethernet connection, using DHCP
    Serial.print("Initialize Ethernet using DHCP => ");
    Ethernet.begin();
  }else{
    // Start the Ethernet connection, using static IP
    Serial.printf("Initialize Ethernet using static IP => ip:%d.%d.%d.%d, gateway:%d.%d.%d.%d\n", db.conf.eth_ip[0], db.conf.eth_ip[1], db.conf.eth_ip[2], db.conf.eth_ip[3], db.conf.eth_gateway[0], db.conf.eth_gateway[1], db.conf.eth_gateway[2], db.conf.eth_gateway[3]);
    Ethernet.begin(db.conf.eth_ip, db.conf.eth_subnet, db.conf.eth_gateway);
    Ethernet.setDNSServerIP(db.conf.eth_dns);
  }
  if (!Ethernet.waitForLocalIP(5000)){
    Serial.println(F("Failed to configure Ethernet"));
    if (!Ethernet.linkStatus()) Serial.println(F("Ethernet cable is not connected."));
    // Stay here forever
    while (true){delay(1000);}
  }
  delay(1000);
  if(!USING_DHCP) delay(1000);
  Serial.printf("Ethernet Initialised on IP %d.%d.%d.%d\n", Ethernet.localIP()[0], Ethernet.localIP()[1], Ethernet.localIP()[2], Ethernet.localIP()[3]);
 #endif

  // init webserver setup mode, only when the two are simultaneously to gnd at the ESP begining
  if((digitalRead(5)==LOW) && (digitalRead(17)==LOW)){
    db.saveConfiguration();
    db.saveSteerSettings();
    db.saveSteerConfiguration();
    setServerMode();
  }

  // Set up main object
  aog.begin(&db, &udpAutosteer, true, true);

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
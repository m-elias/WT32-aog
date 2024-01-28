/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, Jan 21st, 2024. Based on the implementation
  written by Andy (lansalot) located on https://github.com/lansalot/AgOpenGPS_Boards/blob/Keya/TeensyModules/V4.1/Firmware/Autosteer_gps_teensy_v4_1/KeyaCANBUS.ino

  This library handles the driving of Keya motor driver.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef DRIVERKEYA_H
#define DRIVERKEYA_H

#include "Driver.h"
#if MICRO_VERSION == 1
  #include <ACAN_ESP32.h>
#endif
#if MICRO_VERSION == 2
  #include <FlexCAN_T4.h>
#endif
/*
  Enable	0x23 0x0D 0x20 0x01 0x00 0x00 0x00 0x00
  Disable	0x23 0x0C 0x20 0x01 0x00 0x00 0x00 0x00
  Fast clockwise	0x23 0x00 0x20 0x01 0xFC 0x18 0xFF 0xFF (0xfc18 signed dec is - 1000
  Anti - clockwise	0x23 0x00 0x20 0x01 0x03 0xE8 0x00 0x00 (0x03e8 signed dec is 1000
  Slow clockwise	0x23 0x00 0x20 0x01 0xFE 0x0C 0xFF 0xFF (0xfe0c signed dec is - 500)
  Slow anti - clockwise	0x23 0x00 0x20 0x01 0x01 0xf4 0x00 0x00 (0x01f4 signed dec is 500)
*/

class DriverKeya: public Driver {
public:
  DriverKeya() {
      DriverKeya(3);
  }
  DriverKeya(uint8_t _port=3, uint32_t _baudRate=250000) {
   #if MICRO_VERSION == 1
    ACAN_ESP32_Settings settings (_baudRate);
    settings.mRequestedCANMode = ACAN_ESP32_Settings::NormalMode;
    //settings.mRxPin = GPIO_NUM_4 ; // Optional, default Tx pin is GPIO_NUM_4
    //settings.mTxPin = GPIO_NUM_5 ; // Optional, default Rx pin is GPIO_NUM_5
    const ACAN_ESP32_Filter filter = ACAN_ESP32_Filter::singleExtendedFilter (ACAN_ESP32_Filter::data, 0x07000001, 0) ;
    const uint32_t errorCode = ACAN_ESP32::can.begin(settings, filter);
    if (errorCode == 0) {
      Serial.println ("Configuration OK!");
    } else {
      Serial.print ("Configuration error 0x") ;
      Serial.println (errorCode, HEX) ;
    }
   #endif
   #if MICRO_VERSION == 2
    if(_port > 3){//means it is not a Serial but a pin # for defining Teensy RX
      _port = 3;
    }
		if(_port == 1) FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_256> keya;
		else if(_port == 2) FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_256> keya;
		else if(_port == 3) FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> keya;

    keya.begin();
    keya.setBaudRate(_baudRate);
   #endif

    delay(1000);
    Serial.printf("Initialised Keya CANBUS on CAN%d\n", _port);

    value = 0;
  }
	
	void drive(float pwm){
    k = 995;//max range should be [998,-995]
    int actualSpeed = pwm*k;
    if (pwm == 0) {
      disengage();
      return; // don't need to go any further, if we're disabling, we're disabling
    }
    if (debug) Serial.printf("told to steer, with %.3f so....\n",pwm);
    if (debug) Serial.printf("I converted that to speed %d\n",actualSpeed);

   #if MICRO_VERSION == 1
    CANMessage msg;
    msg.id = KeyaPGN;
    msg.ext = true;
    msg.len = 8;
    msg.data[0] = 0x23;
    msg.data[1] = 0x00;
    msg.data[2] = 0x20;
    msg.data[3] = 0x01;
    msg.data[4] = actualSpeed >> 8; // TODO take PWM in instead for speed (this is -1000)
    msg.data[5] = actualSpeed & 0xFF;
    if (pwm < 0) {
      msg.data[6] = 0xff;
      msg.data[7] = 0xff;
      if (debug) Serial.printf("pwm < zero - clockwise - steerSpeed %.3f\n",pwm);
    } else {
      msg.data[6] = 0x00;
      msg.data[7] = 0x00;
      if (debug) Serial.printf("pwm > zero - anticlock-clockwise - steerSpeed %.3f\n",pwm);
    }
    ACAN_ESP32::can.tryToSend(msg);
   #endif
   #if MICRO_VERSION == 2
    CAN_message_t msg;
    msg.id = KeyaPGN;
    msg.flags.extended = true;
    msg.len = 8;
    msg.buf[0] = 0x23;
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x20;
    msg.buf[3] = 0x01;
    msg.buf[4] = actualSpeed >> 8; // TODO take PWM in instead for speed (this is -1000)
    msg.buf[5] = actualSpeed & 0xFF;
    if (pwm < 0) {
      msg.buf[6] = 0xff;
      msg.buf[7] = 0xff;
      if (debug) Serial.printf("pwm < zero - clockwise - steerSpeed %.3f\n",pwm);
    } else {
      msg.buf[6] = 0x00;
      msg.buf[7] = 0x00;
      if (debug) Serial.printf("pwm > zero - anticlock-clockwise - steerSpeed %.3f\n",pwm);
    }
    keya.write(msg);
   #endif
    value = pwm;
    enableSteer();
	}
	
	void disengage(){
   #if MICRO_VERSION == 1
    CANMessage msg;
    msg.id = KeyaPGN;
    msg.ext = true;
    msg.len = 8;
    msg.data[0] = 0x23;
    msg.data[1] = 0x0c;
    msg.data[2] = 0x20;
    msg.data[3] = 0x01;
    msg.data[4] = 0;
    msg.data[5] = 0;
    msg.data[6] = 0;
    msg.data[7] = 0;
    ACAN_ESP32::can.tryToSend(msg);
   #endif
   #if MICRO_VERSION == 2
    CAN_message_t msg;
    msg.id = KeyaPGN;
    msg.flags.extended = true;
    msg.len = 8;
    msg.buf[0] = 0x23;
    msg.buf[1] = 0x0c;
    msg.buf[2] = 0x20;
    msg.buf[3] = 0x01;
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 0;
    msg.buf[7] = 0;
    keya.write(msg);
   #endif
		value = 0;
	}

	void enableSteer(){
   #if MICRO_VERSION == 1
    CANMessage msg;
    msg.id = KeyaPGN;
    msg.ext = true;
    msg.len = 8;
    msg.data[0] = 0x23;
    msg.data[1] = 0x0d;
    msg.data[2] = 0x20;
    msg.data[3] = 0x01;
    msg.data[4] = 0;
    msg.data[5] = 0;
    msg.data[6] = 0;
    msg.data[7] = 0;
    ACAN_ESP32::can.tryToSend(msg);
   #endif
   #if MICRO_VERSION == 2
    CAN_message_t msg;
    msg.id = KeyaPGN;
    msg.flags.extended = true;
    msg.len = 8;
    msg.buf[0] = 0x23;
    msg.buf[1] = 0x0d;
    msg.buf[2] = 0x20;
    msg.buf[3] = 0x01;
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 0;
    msg.buf[7] = 0;
    keya.write(msg);
   #endif
    if (debug) Serial.println("Enabled Keya motor");
	}

  int8_t getCurrent() {
   #if MICRO_VERSION == 1
    CANMessage msg;
    if(ACAN_ESP32::can.receive(msg)){
      if(msg.id == 0x07000001){
        // 0-1 - Cumulative value of angle (360 def / circle)
        // 2-3 - Motor speed, signed int eg -500 or 500
        // 4-5 - Motor current, with "symbol" ? Signed I think that means, but it does appear to be a crap int. 1, 2 for 1, 2 amps etc
        //		is that accurate enough for us?
        // 6-7 - Control_Close (error code)
        if (msg.data[4] == 0xFF) {
          return (256 - msg.data[5]) * 20;
        }else{
          return msg.data[5] * 20;
        }
      }
    }
   #endif
   #if MICRO_VERSION == 2
    CAN_message_t msg;
    if (keya.read(msg)) {
      // parse the different message types
      // heartbeat 0x07000001
      // change heartbeat time in the software, default is 20ms
      if (msg.id == 0x07000001) {
        // 0-1 - Cumulative value of angle (360 def / circle)
        // 2-3 - Motor speed, signed int eg -500 or 500
        // 4-5 - Motor current, with "symbol" ? Signed I think that means, but it does appear to be a crap int. 1, 2 for 1, 2 amps etc
        //		is that accurate enough for us?
        // 6-7 - Control_Close (error code)
        // TODO Yeah, if we ever see something here, fire off a disable, refuse to engage autosteer or..?
        //KeyaCurrentSensorReading = abs((int16_t)((msg.buf[5] << 8) | msg.buf[4]));
        //if (KeyaCurrentSensorReading > 255) KeyaCurrentSensorReading -= 255;
        if (msg.buf[4] == 0xFF) {
          return (256 - msg.buf[5]) * 20;
        }else{
          return msg.buf[5] * 20;
        }
        //if (debugKeya) Serial.println("Heartbeat current is " + String(KeyaCurrentSensorReading));
      }
    }
   #endif 
  }
private:
 #if MICRO_VERSION == 2
  FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> keya;// Seems to work for CAN3, not sure why it didn't for CAN1
 #endif
  uint64_t KeyaPGN = 0x06000001;
  bool debug = false;
};
#endif

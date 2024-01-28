/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, November 30th, 2023.

  This library handles the driving of BNO08X IMU in RVC mode.
  Its tx for serial is SDA (connected to pin 18 in AIO-2.5/4).
  SCL does not require connection.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef IMURVC_H
#define IMURVC_H

#include "Imu.h"

class ImuRvc: public Imu{
public:
	ImuRvc(JsonDB* _db, uint8_t _port=1){
		db = _db;
   #if MICRO_VERSION == 1
    if(_port > 8){//means it is not a Serial but a pin # for defining Teensy RX
      serial = &Serial1;//Rx on pin 15
      //serial->setRX(_port);//IMU SDA (18 in AIO-2.5/4)  SCL not needed. Not possible on 18, not a xbar pin.!!!
    }
		else if(_port == 1) serial = &Serial1;//Rx on pin 0
		else if(_port == 2) serial = &Serial2;//Rx on pin 7
		else if(_port == 3) serial = &Serial;//Rx on pin 15

		serial->begin(115200, SERIAL_8N1, 2, 21);//fix baudrate as per datasheet, RX on pin 2 & TX on 21 of wt32, no need for TX
   #endif
   #if MICRO_VERSION == 2
    if(_port > 8){//means it is not a Serial but a pin # for defining Teensy RX
      serial = &Serial5;//Rx on pin 21
    }
		else if(_port == 1) serial = &Serial1;//Rx on pin 0
		else if(_port == 2) serial = &Serial2;//Rx on pin 7
		else if(_port == 3) serial = &Serial3;//Rx on pin 15
		else if(_port == 4) serial = &Serial4;//Rx on pin 16
		else if(_port == 5) serial = &Serial5;//Rx on pin 21
		else if(_port == 6) serial = &Serial6;//Rx on pin 25
		else if(_port == 7) serial = &Serial7;//Rx on pin 28
		else if(_port == 8) serial = &Serial8;//Rx on pin 34

    serial->begin(115200);
   #endif

    q = Quaternion(0,0,0,0);
    rotation = Vector3(0,0,0);
    acceleration = Vector3(0,0,0);

		getOffset();
		isOn = true;
    delay(100);
    Serial.printf("Imu initialised on p: %d\n", _port);
	}

	bool parse(){
    if(!isBegining()) return false;
    
    uint8_t buffer[19];
		if(!serial->readBytes(buffer, 17)) return false;
		if(!_checkSum(buffer)) return false;

		if(!isOn) return false;
		
		// Adjust the imu value with stored offset
    uint16_t my = (yaw_offset)? yaw_offset*1000 : 0;
    uint16_t mp = (pitch_offset)?pitch_offset*1000 : 0;
    uint16_t mr = (roll_offset)?roll_offset*1000 : 0;
    uint16_t m2pi = 2000*pi;

		// map de array read to the correct numbers (Endian)
    uint16_t y0 = buffer[2]<<8 | buffer[1];
		uint16_t p0 = buffer[4]<<8 | buffer[3];
		uint16_t r0 = buffer[6]<<8 | buffer[5];
		uint16_t ax0= buffer[8]<<8 | buffer[7];
		uint16_t ay0= buffer[10]<<8 | buffer[9];
		uint16_t az0= buffer[12]<<8 | buffer[11];

    // adjust the real values (2nd complement, conversion values...)
    float yaw  =(float)(y0 - ((buffer[2]>127)? 65536 : 0)) * conv - (my % m2pi)/1000;
    float pitch=(float)(p0 - ((buffer[4]>127)? 65536 : 0)) * conv - (mp % m2pi)/1000;
    float roll =(float)(r0 - ((buffer[6]>127)? 65536 : 0)) * conv - (mr % m2pi)/1000;
    float ax   =(float)(ax0- ((buffer[8]>127)? 65536 : 0)) * g;
    float ay   =(float)(ay0- ((buffer[10]>127)? 65536 : 0))* g;
    float az   =(float)(az0- ((buffer[12]>127)? 65536 : 0))* g;

		// map de array read to the correct numbers (2nd complement, conversion values...)
		q.setFromEuler(roll, yaw, pitch, "XYZ");
		rotation = Vector3(roll, yaw, pitch);//in 3js coordenates
		acceleration = Vector3(ax, az, -ay);//in 3js coordenates

		isUsed = false;
    if(serial->available() > 18) parse();

    return true;
	}

	void setOn(bool value=true){
		isOn=value;
	}
	
private:
	HardwareSerial* serial;
	
	bool _checkSum(uint8_t buffer[], uint8_t size=16){
		uint8_t sum = 0;
		for (uint8_t i = 0; i < size; i++) sum += buffer[i];
		if (sum != buffer[size]) return false;
		return true;
	}

  bool isBegining(){
    if (!serial->available()) return false;
    while(serial->read() != 0xAA){}// search for the first byte containing 0xAA
		if(serial->read() != 0xAA) return false;// make sure the next byte is the second 0xAA
    if(serial->available() < 17) return false;//discard not enough data to be right
    return true;
  }
};
#endif
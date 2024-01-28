/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, November 30th, 2023.

  This library handles GNSS reading and writing.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef GNSS_H
#define GNSS_H

#include "GeoMath.h"

class GGA{
public:
    GGA(){}
    GGA(const char *str, bool debug=false){
      parse(str, debug);
    }
    double time = 0;
    double lon = 0;
    double lat = 0;
    uint8_t fixQ = 0;
    uint8_t sat_count = 0;
    double hdop = 0;
    double alt = 0;
    double geoid_sep = 0;
    double dgps_age = 0;
    uint16_t dgps_stn;
    uint8_t faa = '\0';
    bool valid = false;

  void parse(const char *raw, bool debug=false){
    uint8_t i=0;
    uint8_t j=0;
    uint8_t lastIndex=0;
    char str[13][20];
    str[0][0] = '0';

    while(raw[i++]!='*'){
      if(raw[i] != ','){ 
        if(j>0) str[j-1][i-lastIndex] = raw[i];
      }else{
        if(j>0){
          str[j-1][i-lastIndex] = '\0';
          if(debug) Serial.printf("Field: %d value: %s\n", j, str[j-1]);
          if(j==1) time = strtod(str[j-1], NULL);
          if(j==2) lat = strtod(str[j-1], NULL);
          if(j==3) lat *= (str[j-1][0]=='N')? 1 : -1;
          if(j==4) lon = strtod(str[j-1], NULL);
          if(j==5) lon *= (str[j-1][0]=='E')? -1 : 1;
          if(j==6) fixQ = atoi(str[j-1]);
          if(j==7) sat_count = atoi(str[j-1]);
          if(j==8) hdop = strtod(str[j-1], NULL);
          if(j==9) alt = strtod(str[j-1], NULL);
          if(j==11) geoid_sep = strtod(str[j-1], NULL);
          if(j==13) dgps_age = (str[j-1][0]!='\0')? strtod(str[j-1], NULL) : 0.0;
          if(j==14) dgps_stn = (str[j-1][0]!='\0')? atoi(str[j-1]) : 0;
        }

        lastIndex = i+1;
        j++;
      }
    }
    if(j==15) faa = (str[j-1][0]!='\0')? str[j-1][0] : '\0';
    if(j>9) valid = true;
  }
};

class VTG{
public:
    VTG(){}
    VTG(const char *str, bool debug=false){
      parse(str,debug);
    }
    double trackTrue = 0;
    double trackMagnet = 0;
    double speedKnot = 0;
    double speedKmHr = 0;
    uint8_t faa = '\0';
    bool valid = false;

  void parse(const char *raw, bool debug=false){
    int i=0;
    int j=0;
    uint8_t lastIndex=0;
    char str[13][20];
    str[0][0] = '0';

    while(raw[i++]!='*'){
      if(raw[i] != ','){ if(j>0) str[j-1][i-lastIndex] = raw[i];
      }else{
        if(j>0){ 
          str[j-1][i-lastIndex] = '\0';
          if(debug) Serial.printf("Field: %d value: %s\n", j, str[j-1]);
          if(j==1) trackTrue = strtod(str[j-1], NULL);
          if(j==3) trackMagnet = (str[j-1][0]=='\0')?0 : strtod(str[j-1], NULL);
          if(j==5) speedKnot = strtod(str[j-1], NULL);
          if(j==7) speedKmHr = strtod(str[j-1], NULL);
        }
        lastIndex = i+1;
        j++;
      }
    }
    if(j==9){
      str[j-1][1] = '\0';
      faa = (str[j-1][0]!='\0')? str[j-1][0] : '\0';
      if(debug) Serial.printf("Field: %d value: %s\n", j, str[j-1]);
      valid = true;
    }
  }
};

class RMC{
public:
    RMC(){}
    RMC(const char *str, bool debug=false){
      parse(str, debug);
    }
    double time = 0;
    char status = 'A';
    double lon = 0;
    double lat = 0;
    double speed = 0;
    double trackAngle = 0;
    uint16_t date = 0;
    double magneticVariation = 0;
    uint8_t faa = '\0';
    bool valid = false;

  void parse(const char *raw, bool debug=false){
    uint8_t i=0;
    uint8_t j=0;
    uint8_t lastIndex=0;
    char str[10][20];
    str[0][0] = '0';

    while(raw[i++]!='*'){
      if(raw[i] != ','){ 
        if(j>0) str[j-1][i-lastIndex] = raw[i];
      }else{
        if(j>0){
          str[j-1][i-lastIndex] = '\0';
          if(debug) Serial.printf("Field: %d value: %s\n", j, str[j-1]);
          if(j==1) time = strtod(str[j-1], NULL);
          if(j==2) status = str[j-1][0];
          if(j==3) lat = strtod(str[j-1], NULL);
          if(j==4) lat *= (str[j-1][0]=='N')? 1 : -1;
          if(j==5) lon = strtod(str[j-1], NULL);
          if(j==6) lon *= (str[j-1][0]=='E')? -1 : 1;
          if(j==7) speed = strtod(str[j-1], NULL)*0.5144444444;//in m/s
          if(j==8) trackAngle = strtod(str[j-1], NULL);
          if(j==9) date = atoi(str[j-1]);
          if(j==9) magneticVariation = strtod(str[j-1], NULL);
          if(j==10) magneticVariation *= (str[j-1][0]=='E')? -1 : 1;
        }

        lastIndex = i+1;
        j++;
      }
    }
    if(j==11) faa = (str[j-1][0]!='\0')? str[j-1][0] : '\0';
    if(j>9) valid = true;
  }
};

class KSXT{
public:
    KSXT(){}
    KSXT(const char *str, bool debug=false){
      parse(str, debug);
    }
    double time = 0;
    double lon = 0;
    double lat = 0;
    double height = 0;
    double heading = 0;
    double pitch = 0;
    double track = 0;
    double speed = 0;
    double roll = 0;
    uint8_t posQ = 0;
    uint8_t heaQ = 0;
    uint8_t satSlave = 0;
    uint8_t satMaster = 0;
    double east = 0;
    double north = 0;
    double up = 0;
    double eastSpeed = 0;
    double northSpeed = 0;
    double upSpeed = 0;
    uint8_t faa = '\0';
    bool valid = false;

  void parse(const char *raw, bool debug=false){
    uint8_t i=0;
    uint8_t j=0;
    uint8_t lastIndex=0;
    char str[22][20];
    str[0][0] = '0';

    while(raw[i++]!='*'){
      if(raw[i] != ','){ 
        if(j>0) str[j-1][i-lastIndex] = raw[i];
      }else{
        if(j>0){
          str[j-1][i-lastIndex] = '\0';
          if(debug) Serial.printf("Field: %d value: %s\n", j, str[j-1]);
          if(j==1) time = strtod(str[j-1], NULL);
          if(j==2) lon = strtod(str[j-1], NULL);
          if(j==3) lat = strtod(str[j-1], NULL);
          if(j==4) height = strtod(str[j-1], NULL);
          if(j==5) heading = strtod(str[j-1], NULL);
          if(j==6) pitch = strtod(str[j-1], NULL);
          if(j==7) track = strtod(str[j-1], NULL);
          if(j==8) speed = strtod(str[j-1], NULL);
          if(j==9) roll = strtod(str[j-1], NULL);
          if(j==11) posQ = atoi(str[j-1]);
          if(j==13) heaQ = atoi(str[j-1]);
          if(j==14) satSlave = atoi(str[j-1]);
          if(j==15) satMaster = atoi(str[j-1]);
          if(j==16) east = strtod(str[j-1], NULL);
          if(j==17) north = strtod(str[j-1], NULL);
          if(j==18) up = strtod(str[j-1], NULL);
          if(j==19) eastSpeed = strtod(str[j-1], NULL);
          if(j==20) northSpeed = strtod(str[j-1], NULL);
          if(j==21) upSpeed = strtod(str[j-1], NULL);
        }

        lastIndex = i+1;
        j++;
      }
    }
    if(j==22) faa = (str[j-1][0]!='\0')? str[j-1][0] : '\0';
    if(j>20) valid = true;
  }
};

class NMEA{
public:
  NMEA(const char* str, uint8_t _length, bool debug=false){
    strcpy(raw, str);
    if(debug) Serial.printf("Raw message: %s\n", raw);
    length = _length-3;
    if(_checksum(str)){
      valid = true;
      int offset = 3;
      for(int i=0; i<3; i++) type[i]= str[i+offset];
      type[3]='\0';
      if(debug) Serial.printf("NMEA Type: %s\n", type);
      
      if( strcmp(type,"GGA") == 0 )  gga = GGA(str, debug);
      else if(strcmp(type,"VTG")==0) vtg = VTG(str, debug);
      else valid = false;
    }else valid = false;
    if(debug) Serial.print("NMEA parsing done.\n");
  }

  VTG vtg;
  GGA gga;
  bool valid = false;
  char type[4];
  char raw[90];
  uint8_t length;
  
private:
  bool _checksum(const char* str){
    if(str[length]!='*') return false;
    int16_t checksum = 0;
    for (uint8_t i = 1; i < length; i++) checksum ^= str[i];
    const char hexValue[] = {str[length+1], str[length+2]};
    return checksum == strtol(hexValue,0,16);
  }
};

class GNSS{
public:
  GNSS():position(0,0,0){}
	GNSS(uint8_t _port, uint32_t _baudRate=115200):position(0,0,0){
   #if MICRO_VERSION == 1
    uint8_t rxP=5;
    uint8_t txP=17;
    if(_port > 3) _port = 2;

		if(_port == 1) serial = &Serial1;
		else if(_port == 2) {
      serial = &Serial2;
      //pin definition required for esp32
      rxP=5;
      txP=17;
    }
    //else if(_port == 3) serial = &Serial3;
		baudRate = _baudRate;
    serial->begin(baudRate, SERIAL_8N1, rxP, txP);//begin(baudRate); //pin definition required for esp32
   #endif
   #if MICRO_VERSION == 2
    if(_port > 8){//means it is not a Serial but a pin # for defining Teensy RX
      serial = &Serial5;//Rx on pin 21
      //serial->setRX(_port);//IMU SDA (18 in AIO-2.5/4)  SCL not needed. Not possible on 18, not a xbar pin.!!!
    }
		else if(_port == 1) serial = &Serial1;//Rx on pin 0
		else if(_port == 2) serial = &Serial2;//Rx on pin 7
		else if(_port == 3) serial = &Serial3;//Rx on pin 15
		else if(_port == 4) serial = &Serial4;//Rx on pin 16
		else if(_port == 5) serial = &Serial5;//Rx on pin 21
		else if(_port == 6) serial = &Serial6;//Rx on pin 25
		else if(_port == 7) serial = &Serial7;//Rx on pin 28
		else if(_port == 8) serial = &Serial8;//Rx on pin 34

		baudRate = _baudRate;
    serial->begin(baudRate);
   #endif

    //serial->addMemoryForRead(rxBuffer, bufferSize);
    //serial->addMemoryForWrite(txBuffer, bufferSize);
    delay(100);//TODO MCB

    Serial.printf("GNSS initialised on serial: %d\n", _port);
	}
	
	Vector3 position;
	uint32_t time = 0;
	double latitude = 0, longitude = 0, speed = 0, altitude = 0, hdop = 0, dgps_age = 0, speedKnot=0;
  uint8_t fixQuality = 0, sat_count = 0;
  bool isUsed=false;
  
	bool parse(){
    bool isParsed = false;
    while(serial->available() != 0){
      char c = serial->read();
      if(c=='\n'){//reads a 'new line' character
        if(rxBuffer[bufferCounter-3]!='*') return false;//identify that it is completed, the message has a checksum to compare

        rxBuffer[bufferCounter]='\0';//ends the string in the correct position
        NMEA nmea(rxBuffer, bufferCounter);
        if(!nmea.valid) return false;//checks if the message exist
        //updates the gnss variables from the message data
        if(nmea.vtg.valid){
          speed = nmea.vtg.speedKmHr/3.6;
          speedKnot = nmea.vtg.speedKnot;
          //Serial.printf("VTG speed: %.2f\n", speed);
        }else if(nmea.gga.valid){
          //Serial.printf("GGA time: %.2f\n", m->time);
          longitude = nmea.gga.lon;
          latitude = nmea.gga.lat;
          altitude = nmea.gga.alt;
          time = nmea.gga.time;
          fixQuality = nmea.gga.fixQ;
          sat_count = nmea.gga.sat_count;
          hdop = nmea.gga.hdop;
          dgps_age = nmea.gga.dgps_age;

          Vector2 pos2D(0,0);
          pos2D.copy(getPositionMeters());
          position = Vector3(pos2D.x, altitude, pos2D.y);
        }
        isUsed = false;//identify that the object contains new info (that when is used will be marked accordingly)
        isParsed = true;
      }
      
      bufferCounter = (c=='$')? 0 : bufferCounter+1;//updates the count of characters in buffer (rests when is a new line)
      rxBuffer[bufferCounter] = c;//stores the char in the buffer
      //if(bufferCounter==0) Serial.println();
      //Serial.print(c);
    }
    return isParsed;
	}

	Vector2 getPositionMeters(){
		return angleToMeters(latitude, longitude);
	}

  void sendNtrip(uint8_t NTRIPData[], uint16_t size) {
    serial->write(NTRIPData, size);
  }

  static constexpr double EARTH_ORIGIN = 3.14159265 * 6378137;
  static constexpr double pi = 3.14159265;

  static Vector2 angleToMeters(double latitude, double longitude) {
      double x = longitude * GNSS::EARTH_ORIGIN / 180.0;
      double y = std::log(std::tan((90 + latitude) * GNSS::pi / 360.0)) / (GNSS::pi / 180.0);
      y = y * EARTH_ORIGIN / 180.0;
      return Vector2(x, y);
  }

  static Vector2 metersToAngles(double x, double y) {
      double longitude = x / EARTH_ORIGIN * 180.0;
      double latitude = y / EARTH_ORIGIN * 180.0;
      latitude = 180.0 / M_PI * (2 * std::atan(std::exp(latitude * M_PI / 180.0)) - M_PI / 2.0);
      return Vector2(latitude, longitude);
  }

private:
	uint32_t baudRate = 115200;
	uint8_t bufferCounter=0;//NMEA has a maximum of 82 characters, limiterC is to detect the end of the message
	char rxBuffer[512];
	char txBuffer[512];
	HardwareSerial* serial;
};
#endif
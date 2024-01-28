/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, November 30th, 2023.

  This library handles the reading of analog signal on Teensy 
  with ADS1115 card.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef SENSORADS1115READER_H
#define SENSORADS1115READER_H

#include "Wire.h"
#include "Sensor.h"

class SensorADS1115Reader: public Sensor {
public:
    SensorADS1115Reader(JsonDB* _db, uint8_t wireN=1, uint8_t address=0x48, uint8_t _isSingleInput=1) {
    /* Configuration guide for sample address:
      (0x48) address pin low (GND) (default)
      (0x49) address pin high (VCC)
      (0x4A) address pin tied to SDA pin
      (0x4B) address pin tied to SCL pin
    */
		pin = address;
    db = _db;
		isSingleInput = _isSingleInput;

    //set up communication
    if(wireN == 0) i2c = &Wire;
    /*
    else if(wireN == 1) i2c = &Wire1;
    else if(wireN == 2) i2c = &Wire2;
    */

   #if MICRO_VERSION == 1
    if(strcmp(ARDUINO_BOARD,"WT32_ETH01")==0) i2c->begin(15,14,400000);//sda,scl,freq
    else i2c->begin();//21,22 for esp32
   #endif
   #if MICRO_VERSION == 1
    i2c->begin();
   #endif

    // Check ADC
    i2c->beginTransmission(pin);//i2cAddress
    /* Configuration of pointer register
      (0x03) Point mask
      (0x00) Conversion
      (0x01) Configuration
      (0x02) Low threshold
      (0x03) High threshold
    */
    i2c->write(0x00);//Conversion
    i2c->endTransmission();
    i2c->requestFrom(pin, (uint8_t)2);//i2cAddress & # of bytes
    (i2c->available()==0)? Serial.println("ADC Fail!") : Serial.println("ADC Connection OK");

    //Set configuation schema for ADS reading
    //Bits 0 through 4 deal with the comparator function.  The combined default value for these bits is 0x0003
    // OR in the Data rate or Sample Per Seconds bits 5 through 7
    /* Configuration guide for sample rate:
      (0x0000) 8 SPS(Sample per Second), or a sample every 125ms
      (0x0020) 16 SPS, or every 62.5ms
      (0x0040) 32 SPS, or every 31.3ms
      (0x0060) 64 SPS, or every 15.6ms
      (0x0080) 128 SPS, or every 7.8ms  (default)
      (0x00A0) 250 SPS, or every 4ms, note that noise free resolution is reduced to ~14.75-16bits, see table 2 in datasheet
      (0x00C0) 475 SPS, or every 2.1ms, note that noise free resolution is reduced to ~14.3-15.5bits, see table 2 in datasheet
      (0x00E0) 860 SPS, or every 1.16ms, note that noise free resolution is reduced to ~13.8-15bits, see table 2 in datasheet
    */
    config |= 0x0080;
    // OR in the mode bit 8
    config |= 0x0100;//ADS1115_REG_CONFIG_MODE_SINGLE; // Single-shot mode (default)
    // OR in the PGA/voltage range bits 9 through 11
    /* Configuration guide for gain:
      (0x0000) +/-6.144V range = Gain 2/3
      (0x0200) +/-4.096V range = Gain 1
      (0x0400) +/-2.048V range = Gain 2 (default)
      (0x0600) +/-1.024V range = Gain 4
      (0x0800) +/-0.512V range = Gain 8
      (0x0A00) +/-0.256V range = Gain 16
    */
    config |= 0x0000;//gain: ADS1115_REG_CONFIG_PGA_6_144V; /* +/- 6.144V range (limited to VDD +0.3V max!) */
    // OR in the mux channel, bits 12 through 14
    /* Configuration guide for mux channel:
      (0x0000) Differential P = AIN0, N = AIN1 (default)
      (0x1000) Differential P = AIN0, N = AIN3
      (0x2000) Differential P = AIN1, N = AIN3
      (0x3000) Differential P = AIN2, N = AIN3
      (0x4000) Single-ended AIN0
      (0x5000) Single-ended AIN1
      (0x6000) Single-ended AIN2
      (0x7000) Single-ended AIN3
    */
    config |= (isSingleInput)? 0x4000 /*Single-ended AIN0*/: 0x0000/*Differential P = AIN0, N = AIN1 (default)*/;
    // OR in the start conversion bit bit 15
    config |= 0x8000;//ADS1115_REG_CONFIG_OS_SINGLE;// Write: Set to start a single-conversion
	}
	
	void update(){
    // Read the conversion results
    i2c->beginTransmission(pin); //Sets the Address of the ADS1115. 0x48; // address pin low (GND), 0x49; // address pin high (VCC), 0x4A; // address pin tied to SDA pin, 0x4B; // address pin tied to SCL pin
    i2c->write(0x00); //queue the data to be sent, in this case modify the pointer register so that the following RequestFrom reads the conversion register
    i2c->endTransmission(); //Send the petition
    
    i2c->requestFrom(pin, (uint8_t)2); //Request the 2 byte conversion register
    int val = ((i2c->read() << 8) | i2c->read()); //Read each byte.  Shift the first byte read 8 bits to the left and OR it with the second byte.
    value = max(val,0)*6.144/(32768*5);//adjust the range to be 0-1.2288//min(max(val,0)*1023/26000,1023);//adjust the range to be 0-1023
    if(value > 1.0) value = 1.0;

		setAngle();
    triggerConversion();
	}

private:
  uint8_t isSingleInput = 1;
  uint16_t config = 0x0003;// Disable the comparator (default val)
  TwoWire* i2c;

  void triggerConversion(){
    // Write config register to the ADC
    i2c->beginTransmission(pin);//i2cAddress
    i2c->write(0x01);//Configuration
    i2c->write((uint8_t)(config >> 8));
    i2c->write((uint8_t)(config & 0xFF));
    i2c->endTransmission();
  }
};
#endif

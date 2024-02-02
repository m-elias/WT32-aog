/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, November 30th, 2023.

  This library handles the reading of analog signal on ESP32 
  with internal pin.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef SENSORINTERNALREADER_H
#define SENSORINTERNALREADER_H

#include "Sensor.h"
#include <SimpleKalmanFilter.h>
#include <ADC.h>
#include <ADC_util.h>

ADC* adcWAS = new ADC();  // ADC object for setting 16x oversampling medium speed 12 bit A/D object

class SensorInternalReader: public Sensor {
public:
    SensorInternalReader(JsonDB* _db, uint8_t _pin, uint8_t _resolution=12, uint8_t filterConfig=5):filter(filterConfig, filterConfig, 0.01){
		pin = _pin;
		resolution = _resolution;
		db=_db;
    adcWAS->adc1->setAveraging(16);                                     // set number of averages
    adcWAS->adc1->setResolution(12);                                    // set bits of resolution
    adcWAS->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);  // change the conversion speed
    adcWAS->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);      // change the sampling speed
    Serial.printf("Internal sensor reader initialised on p: %d\n", _pin);
	}
	
  uint8_t counter = 0;
  uint8_t resolution = 0;

	void update(){
		value = filter.updateEstimate(adcWAS->adc1->analogRead(pin))/(1 << resolution);// between 0-1.0 //2^resolution
		setAngle();
	}
private:
  SimpleKalmanFilter filter;
};
#endif

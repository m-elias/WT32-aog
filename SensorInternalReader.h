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

class SensorInternalReader: public Sensor {
public:
    SensorInternalReader(JsonDB* _db, uint8_t _pin, uint8_t _resolution=12, uint8_t filterConfig=5):filter(filterConfig, filterConfig/10, 0.01){
		pin = _pin;
		resolution = _resolution;
		db=_db;
    Serial.printf("Internal sensor reader initialised on p: %d\n", _pin);
	}
	
  uint8_t counter = 0;
  uint8_t resolution = 0;

	void update(){
		value = filter.updateEstimate(analogRead(pin))/(1 << resolution);// between 0-1.0 //2^resolution
		setAngle();
	}
private:
  SimpleKalmanFilter filter;
};
#endif

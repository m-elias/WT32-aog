/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, November 30th, 2023.

  This library handles the reading of analog signal on ESP.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef SENSOR_H
#define SENSOR_H

#include "JsonDB.h"

class Sensor {
public:
    Sensor(){}
    Sensor(JsonDB* _db){
      db=_db;
    }

	float value = 0.0;//value:0-1.0
	float angle = 0.0;
  uint8_t counter = 0;
	
	virtual void update()=0;
	
protected:
  uint8_t pin=20;
	JsonDB* db;
	
	void setAngle(){
    uint8_t steerSensorCounts = 150;  // setting db->steerS.steerSensorCounts = 150 or using db->steerS.steerSensorCounts fails for unknown reason
		float a = value*13610 - 6805 - db->steerS.wasOffset*(db->steerC.InvertWAS? 1 :-1);  // *13610 to match "old" ADS1115 CPD & Offset values/scaling, 6805 is center (1/2 of full scale)
    angle = a*(db->steerC.InvertWAS? 1 :-1)/steerSensorCounts;//  ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
    Serial.printf(" value: %.3f a:%.2f CPD:%3i angle:%.2f", value, a, steerSensorCounts, angle);
    if(angle<0) angle *= db->steerS.AckermanFix;
    Serial.printf(" angle:%.2f\n", angle);
	}
};
#endif

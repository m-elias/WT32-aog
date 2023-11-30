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

	float value = 0.0;//value:0-1.0 //1023
	float angle = 0.0;
  uint8_t counter = 0;
	
	virtual void update()=0;
	
protected:
  uint8_t pin=20;
	JsonDB* db;
	
	void setAngle(){
		float a = value*6805 - 6805 - db->steerS.wasOffset*(db->steerC.InvertWAS? 1 :-1);  // 1/2 of full scale
    angle = a*(db->steerC.InvertWAS? 1 :-1)*db->steerS.steerSensorCounts;//  ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
    if(angle<0) angle *= db->steerS.AckermanFix;
	}
};
#endif

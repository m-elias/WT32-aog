/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, November 30th, 2023.

  This library handles the driving of BNO08X IMU.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef IMU_H
#define IMU_H

#include "JsonDB.h"
#include "GeoMath.h"

typedef unsigned char uint8_t;
typedef signed short int int16_t;

class Imu{
public:
	Imu():q(0,0,0,0), rotation(0,0,0), acceleration(0,0,0){}
	
	Quaternion q;
	Vector3 rotation, acceleration;

  virtual bool parse()=0;

	void setOn(bool value=true){
		isOn=value;
	}

	void setOffset(){
		pitch_offset = rotation.z+(pitch_offset?pitch_offset:0);
		yaw_offset = rotation.y+(yaw_offset?yaw_offset:0);
		roll_offset = rotation.x+(roll_offset?roll_offset:0);
		
		//store
    db->get("/ers/static/api/autosteering", [&](JsonDocument& doc){
			doc["pitch_offset"] = pitch_offset;
			doc["yaw_offset"] = yaw_offset;
			doc["roll_offset"] = roll_offset;
		}, 2);
	}
	
	void getOffset(){
		db->get("/ers/static/api/autosteering", [&](JsonDocument& doc){
			pitch_offset = doc["pitch_offset"].as<float>();
			yaw_offset = doc["yaw_offset"].as<float>();
			roll_offset = doc["roll_offset"].as<float>();
		});
	}
	
	Vector3 getOffsetV(){
		return Vector3(pitch_offset, yaw_offset, roll_offset);
	}

  bool used(){
    if(isUsed) return isUsed;
    isUsed = true;
		return !isUsed;
	}

protected:
	double pitch_offset, yaw_offset, roll_offset;
	const double pi = 3.14159265, conv = 3.14159265/18000 /*rad*/, g = 0.00980665/*to transform from mg to m/s2*/;
	bool isOn = true, isUsed = true;
	JsonDB* db;
};
#endif

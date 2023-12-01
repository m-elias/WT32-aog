/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, November 30th, 2023.

  This library takes care of the position related components (GNSS, IMU, WAS).

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef POSITION_H
#define POSITION_H

#include <AsyncUDP_WT32_ETH01.h>
#include "JsonDB.h"
#include "GNSS.h"
#include "IMU.h"
#include "ImuRvc.h"
#include "ImuClassic.h"
#include "Sensor.h"
#include "SensorInternalReader.h"
#include "SensorADS1115Reader.h"

class Position{
public:
  Position():gnss(2,115200){}
	Position(JsonDB* _db, AsyncUDP* udpService):gnss(_db->conf.gnss_port, _db->conf.gnss_baudRate){
		udp = udpService;
    db = _db;

    // Create imu, interact with sensor ######################################################################################################
    (_db->conf.imu_type == 1)? imu = new ImuRvc(_db, _db->conf.imu_port) : imu = new ImuClassic(_db, _db->conf.imu_tickRate);

    // Create and initialize the object to read the WAS sensor ###############################################################################
    (_db->conf.was_type == 1)? was = new SensorInternalReader(_db, _db->conf.was_pin, _db->conf.was_resolution) : was = new SensorADS1115Reader(_db, _db->conf.was_pin);
    delay(100);

    // time configuration variables
    previousTime = millis();
    reportPeriodMs = 1000/_db->conf.reportTickRate;
 }

  GNSS gnss;
  Imu* imu;
  Sensor* was;

	bool report(){
		// set timer to run periodically
    uint32_t now = millis();
		if(now - previousTime < reportPeriodMs) return false;
		previousTime = now;
    
		//actual code to run periodically
		was->update();
		imu->parse();
		gnss.parse();

    // Build the new PANDA sentence ################################################################
    char nmea[100];
    sprintf(nmea, "$PANDA,%lu,%.8f,%c,%.8f,%c,%u,%u,%.2f,%.4f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f*",
                  gnss.time, abs(gnss.latitude), (gnss.latitude < 0)?"S":"N", 
                  abs(gnss.longitude), (gnss.longitude < 0)?"E":"W", gnss.fixQuality, 
                  gnss.sat_count, gnss.hdop, gnss.altitude, gnss.dgps_age, gnss.speedKnot, 
                  imu->rotation.y, imu->rotation.x, imu->rotation.z, imu->acceleration.y);
    // Calculate checksum
    int16_t sum = 0;
    for (uint8_t inx = 1; inx < 110; inx++) {// The checksum calc starts after '$' and ends before '*'
      if (nmea[inx] == '*') break;// '*' Indicates end of data and start of checksum
      sum ^= nmea[inx];  // Build checksum
    }
    sprintf(nmea, "%s%04X\r\n",nmea, sum);//add the checksum in hex

		//send position to udp server #################################################################
    udp->writeTo((uint8_t*)nmea, strlen(nmea), db->conf.server_ip, db->conf.server_destination_port);
		return true;
	}

private:
	AsyncUDP* udp;
 	JsonDB* db;
	uint32_t previousTime;
	uint16_t reportPeriodMs;
};
#endif

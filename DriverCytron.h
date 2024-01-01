/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, November 30th, 2023.

  This library handles the driving of Cytron md13 motor driver.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef DRIVERCYTRON_H
#define DRIVERCYTRON_H

#include "Driver.h"

class DriverCytron: public Driver {
public:
    DriverCytron() {
		DriverCytron(32,31,29);
	}
    DriverCytron(uint8_t pwm, uint8_t nc, uint8_t dir) {
		pin_pwm = pwm;
		pin_nc = nc;
		pin_dir = dir;

		value = 0;
    Serial.printf("Driver cytron initialised on pwm: %d, nc: %d, dir: %d\n", pwm, nc, dir);
    }
	
	void drive(float pwm){
		(pwm == 0) ? digitalWrite(pin_nc, LOW) : digitalWrite(pin_nc, HIGH);
		(pwm < 0) ?  digitalWrite(pin_dir, HIGH) : digitalWrite(pin_dir, LOW);

		value = pwm;
		analogWrite(pin_pwm, pwm*k);
	}
	
	void disengage(){
		digitalWrite(pin_nc, LOW);
		value = 0;
		analogWrite(pin_pwm, value);		
	}
};
#endif

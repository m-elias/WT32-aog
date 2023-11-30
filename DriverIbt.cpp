/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, November 30th, 2023.
  Based on the Procedimental implementation of Brian Tischler (4 Feb 2021).

  This library handles the driving of IBT-2 motor driver.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "DriverIbt.h"

DriverIbt::DriverIbt(uint8_t pin_left, uint8_t pin_enable, uint8_t pin_right){
	pin_pwm = pin_left;
	pin_nc = pin_enable;
	pin_dir = pin_right;
}

void DriverIbt::drive(float pwm){
  value = pwm;
  digitalWrite(pin_nc, 1); // IBT 2 Driver enable connected to BOTH to enable current
  // PWM Left + PWM Right Signal
  if (value > 0){
    analogWrite(pin_dir, 0);//Turn off before other one on
    value = pwm;
    analogWrite(pin_pwm, static_cast<uint8_t>(value*k));
  }else{
    value *= -1;
    analogWrite(pin_pwm, 0);//Turn off before other one on
    analogWrite(pin_dir, static_cast<uint8_t>(value*k));
  }
}

void DriverIbt::disengage(){
  analogWrite(pin_pwm, 0);
  analogWrite(pin_dir, 0);
  digitalWrite(pin_nc, 0);// IBT 2 Driver disable
}

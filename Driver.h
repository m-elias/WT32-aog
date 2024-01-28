/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, November 30th, 2023.

  This library handles the driving of motor as generic that will be
  implemented in other libraries like Ibt, Cytron...

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef DRIVER_H
#define DRIVER_H

#include <Arduino.h>

typedef unsigned char uint8_t;
typedef signed short int int16_t;

class Driver{
public:
  Driver(){}
  float value=0;//value:0-1 ->0-255 with k
  uint8_t pwm(){
    return static_cast<uint8_t>(value*k);
  }
  
  virtual void drive(float pwmDrive)=0;
  virtual void disengage()=0;

  int8_t getCurrent(){
    return 0;
  }

protected:
  uint8_t pin_pwm, pin_nc, pin_dir;
  uint8_t k = 255;
};
#endif

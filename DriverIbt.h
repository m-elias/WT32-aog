/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, November 30th, 2023.

  This library handles the driving of IBT-2 motor driver.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef DRIVERIBT_H
#define DRIVERIBT_H

#include "Driver.h"

class DriverIbt: public Driver{
public:
  DriverIbt(uint8_t pin_left, uint8_t pin_enable, uint8_t pin_right);
  void drive(float pwm);
  void disengage();
};
#endif

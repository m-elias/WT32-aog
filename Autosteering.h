/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, November 30th, 2023.

  This library coordinates all other libraries.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef AUTOSTEERING_H
#define AUTOSTEERING_H

#include "JsonDB.h"
#include <AsyncUDP_WT32_ETH01.h>
#include "ArduinoJson.h"
#include "Position.h"
#include "DriverCytron.h"
#include "DriverIbt.h"
#include "Driver.h"
#include "Sensor.h"
#include "SensorInternalReader.h"

class Autosteering {
public:
  Autosteering() {}

  void begin(JsonDB* _db, AsyncUDP* udpService, bool udpDebug=false, bool sensorsDebug=false){
    db = _db;
    udp = udpService;
    debugUdp = udpDebug;
    position = Position(db, udpService, sensorsDebug);
    delay(1000);//TODO MCB
    //TODO MCB define pwm frequency for esp next line is for teensy
    //analogWriteFrequency(db->conf.driver_pin[0], 490);
    /*
    //keep pulled high and drag low to activate, noise free safe
    pinMode(db->conf.work_pin, INPUT_PULLUP);
    pinMode(db->conf.steer_pin, INPUT_PULLUP);
    pinMode(db->conf.remote_pin, INPUT_PULLUP);
    Serial.printf("Setted work pin: %d, steer pin: %d, remote pin: %d\n",db->conf.work_pin, db->conf.steer_pin, db->conf.remote_pin);
    */

    // Create driver, interact with PWM #######################################################################################################
    (db->conf.driver_type==1)? driver = new DriverCytron(db->conf.driver_pin[0], db->conf.driver_pin[1], db->conf.driver_pin[2]) : driver = new DriverIbt(db->conf.driver_pin[0], db->conf.driver_pin[1], db->conf.driver_pin[2]);
    // Create sensor for automatic stop autosteering (pressure/current)
    if (db->steerC.PressureSensor || db->steerC.CurrentSensor) loadSensor = new SensorInternalReader(_db, db->conf.ls_pin, 12, db->conf.ls_filter);
    // Loop configuration variables
    tickLengthMs = 1000000 / db->conf.globalTickRate;
  }

  void parseUdp(AsyncUDPPacket packet){
    if (packet.length() < 5) return;
    
    if (packet.data()[0] == 0x80 && packet.data()[1] == 0x81 && packet.data()[2] == 0x7F){ //Data
      if(debugUdp) Serial.printf("Udp packet captured frame: %u, length: %zu\n", packet.data()[3], packet.length());
      switch (packet.data()[3]) {
        case 254: // 0xFE Autosteering
          {
            position.gnss.speed = ((float)(packet.data()[5] | packet.data()[6] << 8)) * 0.1;

            guidanceStatusChanged = (guidanceStatus != packet.data()[7]);
            if(guidanceStatusChanged) guidanceStatus = packet.data()[7];

            //Bit 8,9    set point steer angle * 100 is sent
            steerAngleSetPoint = ((float)(packet.data()[8] | ((int8_t)packet.data()[9]) << 8)) * 0.01;  //high low bytes

            if ((bitRead(guidanceStatus, 0) == 0) || (position.gnss.speed < 0.1) || (steerSwitch == 1)) {
              watchdogTimer = WATCHDOG_FORCE_VALUE;  //turn off steering motor
            } else { //valid conditions to turn on autosteer
              watchdogTimer = 0;  //reset watchdog
            }

            //tram = packet.data()[10];
            //relay = packet.data()[11];
            //relayHi = packet.data()[12];

            // Serial Send to agopenGPS ##########################################################################
            uint8_t PGN_253[] = {0x80,0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
            int8_t PGN_253_Size = sizeof(PGN_253) - 1;
            // steering angle
            int16_t sa = (int16_t)(position.was->angle * 100);//TODO: review units, rad or deg?
            PGN_253[5] = (uint8_t)sa;
            PGN_253[6] = sa >> 8;
            // heading
            PGN_253[7] = (uint8_t)9999;
            PGN_253[8] = 9999 >> 8;
            // roll
            PGN_253[9] = (uint8_t)8888;
            PGN_253[10] = 8888 >> 8;
            // switches status
            uint8_t switchByte = 0;
            switchByte |= (digitalRead(db->conf.remote_pin) << 2); //read auto steer enable switch open = 0n closed = Off, put remote in bit 2
            switchByte |= (steerSwitch << 1);                     //put steerswitch status in bit 1 position
            switchByte |= digitalRead(db->conf.work_pin);          //put workswitch status in bit 0 position
            PGN_253[11] = switchByte;
            // PWM value
            PGN_253[12] = driver->pwm();

            //checksum
            int16_t CK_A = 0;
            for (uint8_t i = 2; i < PGN_253_Size; i++) CK_A = (CK_A + PGN_253[i]);
            PGN_253[PGN_253_Size] = CK_A;

            
            udp->writeTo(PGN_253, sizeof(PGN_253), db->conf.server_ip, db->conf.server_destination_port);
           
            //Steer Data 2  ###############################################################################
            if (db->steerC.PressureSensor || db->steerC.CurrentSensor) {
              if (loadSensor->counter++ > 2) {
                uint8_t PGN_250[] = { 0x80,0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
                int8_t PGN_250_Size = sizeof(PGN_250) - 1;

                int sensorReading = 0;
                float sensorSample = loadSensor->value*13610;

                if (db->steerC.PressureSensor){ // Pressure sensor?
                  sensorSample *= 0.25;
                  sensorReading = sensorReading * 0.6 + sensorSample * 0.4;
                }else if (db->steerC.CurrentSensor){ // Current sensor?
                  sensorSample = (abs(775 - sensorSample)) * 0.5;
                  sensorReading = sensorReading * 0.7 + sensorSample * 0.3;    
                  sensorReading = min(sensorReading, 255);
                }

                if (sensorReading >= db->steerC.PulseCountMax) {
                    steerSwitch = 1; // reset values like it turned off
                    currentState = 1;
                    previous = 0;
                }

                PGN_250[5] = (byte)sensorReading;
                
                //checksum
                CK_A = 0;
                for (uint8_t i = 2; i < PGN_250_Size; i++) CK_A = (CK_A + PGN_250[i]);
                PGN_250[PGN_250_Size] = CK_A;

                udp->writeTo(PGN_250, sizeof(PGN_250), db->conf.server_ip, db->conf.server_destination_port);

                loadSensor->counter = 0;
              }
            }
            break;
          }
        case 252: // 0xFC - steer settings
          {
            //PID values
            db->steerS.Kp = ((float)packet.data()[5]);    // read Kp from AgOpenGPS
            db->steerS.highPWM = packet.data()[6];        // read high pwm
            db->steerS.lowPWM = (float)packet.data()[7];  // read lowPWM from AgOpenGPS
            db->steerS.minPWM = packet.data()[8];         //read the minimum amount of PWM for instant on
            float temp = (float)db->steerS.minPWM * 1.2;
            db->steerS.lowPWM = (byte)temp;
            db->steerS.steerSensorCounts = packet.data()[9];   //sent as setting displayed in AOG
            db->steerS.wasOffset = (packet.data()[10]);        //read was zero offset Lo
            db->steerS.wasOffset |= (packet.data()[11] << 8);  //read was zero offset Hi
            db->steerS.AckermanFix = (float)packet.data()[12] * 0.01;

            db->saveSteerSettings();
            break;
          }
        case 251: // 0xFB - SteerConfig
          {
            uint8_t sett = packet.data()[5];  //setting0
            if (bitRead(sett, 0)) db->steerC.InvertWAS = 1;
            else db->steerC.InvertWAS = 0;
            if (bitRead(sett, 1)) db->steerC.IsRelayActiveHigh = 1;
            else db->steerC.IsRelayActiveHigh = 0;
            if (bitRead(sett, 2)) db->steerC.MotorDriveDirection = 1;
            else db->steerC.MotorDriveDirection = 0;
            if (bitRead(sett, 3)) db->steerC.SingleInputWAS = 1;
            else db->steerC.SingleInputWAS = 0;
            if (bitRead(sett, 4)) db->steerC.CytronDriver = 1;
            else db->steerC.CytronDriver = 0;
            if (bitRead(sett, 5)) db->steerC.SteerSwitch = 1;
            else db->steerC.SteerSwitch = 0;
            if (bitRead(sett, 6)) db->steerC.SteerButton = 1;
            else db->steerC.SteerButton = 0;
            if (bitRead(sett, 7)) db->steerC.ShaftEncoder = 1;
            else db->steerC.ShaftEncoder = 0;

            db->steerC.PulseCountMax = packet.data()[6];

            //was speed
            //packet.data()[7];

            sett = packet.data()[8];  //setting1 - Danfoss valve etc
            if (bitRead(sett, 0)) db->steerC.IsDanfoss = 1;
            else db->steerC.IsDanfoss = 0;
            if (bitRead(sett, 1)) db->steerC.PressureSensor = 1;
            else db->steerC.PressureSensor = 0;
            if (bitRead(sett, 2)) db->steerC.CurrentSensor = 1;
            else db->steerC.CurrentSensor = 0;
            if (bitRead(sett, 3)) db->steerC.IsUseY_Axis = 1;
            else db->steerC.IsUseY_Axis = 0;

            //crc
            //packet.data()[13];
            
            db->saveSteerConfiguration();
            break;
          }
        case 200: // Hello from AgIO
          {
            uint8_t helloFromAutoSteer[] = { 0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71 };
            // steering angle
            int16_t sa = (int16_t)(position.was->angle * 100);//TODO: review units, rad or deg?
            helloFromAutoSteer[5] = (uint8_t)sa;
            helloFromAutoSteer[6] = sa >> 8;
            // steering position (without was-offset)
            int16_t sp = (position.was->value - 1)*6805;
            helloFromAutoSteer[7] = (uint8_t)sp;
            helloFromAutoSteer[8] = sp >> 8;
            // switches status
            uint8_t switchByte = 0;
            switchByte |= (digitalRead(db->conf.remote_pin) << 2); //read auto steer enable switch open = 0n closed = Off, put remote in bit 2
            switchByte |= (steerSwitch << 1);                     //put steerswitch status in bit 1 position
            switchByte |= digitalRead(db->conf.work_pin);          //put workswitch status in bit 0 position
            helloFromAutoSteer[9] = switchByte;

            udp->writeTo(helloFromAutoSteer, sizeof(helloFromAutoSteer), db->conf.server_ip, db->conf.server_destination_port);
            break;
          }
        case 201: // change ip
          {
            //make really sure this is the subnet pgn
            if (packet.data()[4] == 5 && packet.data()[5] == 201 && packet.data()[6] == 201){
              db->conf.eth_ip[0] = packet.data()[7];
              db->conf.eth_ip[1] = packet.data()[8];
              db->conf.eth_ip[3] = packet.data()[9];

              db->conf.server_ip[0] = db->conf.eth_ip[0];
              db->conf.server_ip[1] = db->conf.eth_ip[1];
              db->conf.server_ip[2] = db->conf.eth_ip[2];

              db->saveConfiguration();
              ESP.restart();
            }
            break;
          }
        case 202: // whoami
          {
            if (packet.data()[4] == 3 && packet.data()[5] == 202 && packet.data()[6] == 202) { // make really sure this is the reply pgn
              //hello from AgIO
              uint8_t scanReply[] = { 128, 129, 126, 203, 13, 
                                      db->conf.eth_ip[0], db->conf.eth_ip[1], db->conf.eth_ip[2], db->conf.eth_ip[3],
                                      db->conf.eth_ip[0], db->conf.eth_ip[1], db->conf.eth_ip[2], 23 };

              //checksum
              int16_t CK_A = 0;
              for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++) CK_A = (CK_A + scanReply[i]);
              scanReply[sizeof(scanReply) - 1] = CK_A;

              udp->writeTo(scanReply, sizeof(scanReply), db->conf.server_ip, db->conf.server_destination_port);
            }
            break;
          }
      }
    }else{
      if(debugUdp) Serial.println("Unknown packet!!!");
    }
  }

  void udpNtrip(AsyncUDPPacket packet){
    uint16_t size = packet.length();
    uint8_t NTRIPData[size - 4];
    for (int i = 4; i < size; i++) NTRIPData[i - 4] = packet.data()[i];

    position.gnss.sendNtrip(NTRIPData, size - 4);
    if(debugUdp) Serial.print("Udp packet captured for Ntrip\n");
  }

	/*
	update in each loop the data by:
	 - check network status
   - reads Switches and updates from guidanceStatus and previous values
	 - evaluating the maximum steering angle (from speed and geometrical constraints)
	 - command to change the angle of the wheels to the Driver actuator, in the amount required
	*/
	void update() {
    //If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
    if (watchdogTimer++ > 250) watchdogTimer = WATCHDOG_FORCE_VALUE;

    if (db->steerC.SteerSwitch == 1){         //steer switch on - off
      steerSwitch = digitalRead(db->conf.steer_pin); //read auto steer enable switch open = 0n closed = Off
    }else if (db->steerC.SteerButton == 1){   //steer Button momentary
      reading = digitalRead(db->conf.steer_pin); //read auto steer enable switch open = 0n closed = Off
      if (reading == LOW && previous == HIGH){
        if (currentState == 1){
          currentState = 0;
          steerSwitch = 0;
        }else{
          currentState = 1;
          steerSwitch = 1;
        }
      }
      previous = reading;
    }else{                                      // No steer switch and no steer button
      // So set the correct value. When guidanceStatus = 1,
      // it should be on because the button is pressed in the GUI
      // But the guidancestatus should have set it off first
      if (guidanceStatusChanged && guidanceStatus == 1 && steerSwitch == 1 && previous == 0){
        steerSwitch = 0;
        previous = 1;
      }

      // This will set steerswitch off and make the above check wait until the guidanceStatus has gone to 0
      if (guidanceStatusChanged && guidanceStatus == 0 && steerSwitch == 0 && previous == 1){
        steerSwitch = 1;
        previous = 0;
      }
    }

    // Do pid and command angle change
    _changeWheelAngle(); //TODO: review angle unit (steerAngleSetPoint) rad or deg?.
	}

	/*
	  does the timing, to command updating when it is convenient
	*/
	bool run(){
    position.report();//updates the sensors data (gnss, imu, was) using reporting streamRate as internal timer, independently of other timers

    uint32_t now = millis();
 		if(now - lastLoopTime < tickLengthMs) return false;
		lastLoopTime = now;
		
    //actual code to run periodically
		if(guidanceStatus == 1) update();

    return true;
	}

private:
	JsonDB* db;
	AsyncUDP* udp;
  Driver* driver;
  Position position;
  Sensor* loadSensor;
	uint32_t tickLengthMs, lastLoopTime;
  // status
  uint8_t guidanceStatus = 0;
  bool guidanceStatusChanged = false, debugUdp=false;
  // Angle goal
  float steerAngleSetPoint = 0; //the desired angle from AgOpen
  // Networt disconnection check
  const uint16_t WATCHDOG_THRESHOLD = 100;
  const uint16_t WATCHDOG_FORCE_VALUE = 102; // Should be greater than WATCHDOG_THRESHOLD
  uint8_t watchdogTimer = 102;
  //Steer switch button
  uint8_t steerSwitch = 1, currentState = 1, reading = 0 , previous = 0;

	/*
	commands the actuator (motor, valves...) to move to a certain degree
	the pwm value is the intensity of that movement, a real number ranging [-1,1]
	*/
	void _changeWheelAngle() {
		int16_t pwm = (int16_t)(db->steerS.Kp * (position.was->angle - steerAngleSetPoint));//calculate the steering error & set proportional response
    pwm += db->steerS.minPWM*((pwm<0)?-1:1); // adds min throttle factor so no delay from motor resistance.
		// adjust maximum pwm response to pair configuration Maximum and a lower Maximum in case the error is little
		int16_t maxPwm = min((int16_t)db->steerS.highPWM, (int16_t)(abs(pwm) * (db->steerS.highPWM - db->steerS.lowPWM)/3 + db->steerS.lowPWM));
 		if (abs(pwm) > maxPwm) pwm = maxPwm * ((pwm<0)?-1:1); // sets max range
    if (db->steerC.IsDanfoss) {
      // Danfoss: PWM 25% On = Left Position max  (below Valve=Center)
      // Danfoss: PWM 50% On = Center Position
      // Danfoss: PWM 75% On = Right Position max (above Valve=Center)
      pwm = min(max((int)pwm,(int)-250),(int)250);

      // Calculations below make sure pwmDrive values are between 65 and 190
      // This means they are always positive, so in motorDrive, no need to check for
      // db->steerC.isDanfoss anymore
      pwm = pwm >> 2;  // Devide by 4
      pwm += 128;      // add Center Pos.

      // pwmDrive now lies in the range [65 ... 190], which would be great for an ideal opamp
      // However the TLC081IP is not ideal. Approximating from fig 4, 5 TI datasheet, @Vdd=12v, T=@40Celcius, 0 current
      // Voh=11.08 volts, Vol=0.185v
      // (11.08/12)*255=235.45
      // (0.185/12)*255=3.93
      // output now lies in the range [67 ... 205], the center position is now 136
      //pwmDrive = (map(pwmDrive, 4, 235, 0, 255));
    }

    if (watchdogTimer < WATCHDOG_THRESHOLD)	// check if network connection is active
      driver->drive(pwm/maxPwm); // driver needs an input in the range [-1,1]
	}

};
#endif

#include "LineSensor.h"
#include "motors.h"
#include "encoders.h"

#define linesensor_update_time 300
#define motor_update 100
//#define lost_timer 2500

LineSensor Sensors;
Motors Motors;



//void setMotorPower( float left_pwm, float right_pwm );
//void getError();
void lineFollowing(float);
void findLine(unsigned long);


//Encoder Global Variables
int oldLeftCount = 0; // initialise to origin on startup
int oldRightCount = 0;
float distancePerCount = 0.00028;      // distance per count in metres (~ wheel circumference / counts per revolution)
float leftDistanceFromOrigin = 0.0;
float rightDistanceFromOrigin = 0.0;
float wheelDistanceFromOrigin = 0.0425;     // Half the wheel seperation (m)
float XMagnitude = 0.0;
float YMagnitude = 0.0;
float totalAngle = 0;
float turnAngle = 0;        // Variable initialised to zero for the general solution 180 deg turn

void kinematics();

// Frequency variables
unsigned long linesensor_ts = 0;    
unsigned long motor_ts = 0;

float forward_pwm = 14.00;
float lossThreshold = ((Sensors.ls_reading[0] + Sensors.ls_reading[1] + Sensors.ls_reading[2] + Sensors.ls_reading[3] + Sensors.ls_reading[4]) / 5 );        //As it starts on white this will be the value for off the line

// Switch case
int mode = 1;       // controls the operation of the system with a switch case

// gap variables
bool gapPassed = false;    // whether the middle gap has been passed
bool gap = true;          // Will the next loss of line be the middle gap   <-----------CHANGE BACK TO FALSE
bool end = false;          // Will be changed to true when the previous two are true. When this is true and the line has been lost return to start will be initiated.








void setup() {

  Sensors.initialise();
  Motors.initialise();
  // Setup encoders when the robot is 
  // powered on. 
  setupEncoder0();
  setupEncoder1();
  mode = 2;                 //next step on switch statement

}

void loop() {

  unsigned long current_ts = millis();
  unsigned long start_time = micros();
  unsigned long elapsed_time_ls = current_ts - linesensor_ts;
  unsigned long elapsed_time_motor = current_ts - motor_ts;

  unsigned long timer = millis();
  if (timer >= 18000){
    gap = true;           // If more than 18 seconds have passed the next gap will be the centre gap (for the general solution)
  }

  if (gap == true || gapPassed == true){
    end == true;
  }

  Serial.print("gap: ");
  Serial.print(gap);
  Serial.print("\n");

  Serial.print("gapPassed: ");
  Serial.print(gapPassed);
  Serial.print("\n");

  Serial.print("end: ");
  Serial.print(end);
  Serial.print("\n");
  
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------------ENCODER CODE --- TO BE PUT INTO A CLASS!!!------------------------------------------//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Left Wheel Distance
  int currentLeftCount = count_eLeft;                                                   // Variable to store current left wheen count
  int changeLeftCount = currentLeftCount - oldLeftCount;                                // Variable to store the change in left wheel count
  oldLeftCount = currentLeftCount;                                                      // Old left wheen count gets set to current count for the next iteration
  float changeLeftDistance = distancePerCount * changeLeftCount;                        // Change in left wheel distance is calculated from Distance per count (0.28mm) * Change in encode counts
  leftDistanceFromOrigin += changeLeftDistance;                                         // Left wheel distance from origin = old distance + change in distance


//Right Wheel Distance
  int currentRightCount = count_eRight;                                                 // Variable to store current count
  int changeRightCount = currentRightCount - oldRightCount;                             // Variable to store the change in count
  oldRightCount = currentRightCount;                                                    // old count gets reset to the current count for the next iteration
  float changeRightDistance = distancePerCount * changeRightCount;                      // Change in right wheel distance is calculated from Distance per count (0.28mm) * Change in encode counts
  rightDistanceFromOrigin += changeRightDistance;                                       // Right wheel distance from origin = old distance + change in distance

  float ChangeInDistance = (changeLeftDistance + changeRightDistance) / 2;              // The average change from both wheel since last loop
  float DistanceFromOrigin = (leftDistanceFromOrigin + rightDistanceFromOrigin) / 2;  // The total distance from the origin as an average of both wheel

  // Theta R Component (angle)
  // angle = difference in changeInDistance / 2 * length (Initially assuming this means length from the centre of robot)
  float angleChange = changeLeftDistance / (2 * wheelDistanceFromOrigin) - changeRightDistance / (2 * wheelDistanceFromOrigin);           // Angle = the average of the changes in left and right distance
  //Serial.print("AngleChange: ");
  //Serial.print(angleChange * (180/3.142));                    // Displays in degrees for ease of reading
  //Serial.print("\n");
  totalAngle += angleChange;

  float angleFromOrigin = leftDistanceFromOrigin / (2 * wheelDistanceFromOrigin) - rightDistanceFromOrigin / (2 * wheelDistanceFromOrigin);        // A total reading of angle form the origin
  Serial.print("AngleFromChange:  ");
  Serial.print(angleFromOrigin* (180/3.142));
  Serial.print("  | ");
  Serial.print("totalAngle:  ");
  Serial.print(totalAngle * (180/3.14159265));

  XMagnitude = XMagnitude + (ChangeInDistance*cos(angleFromOrigin));
  YMagnitude = YMagnitude + (ChangeInDistance*sin(angleFromOrigin));

  // Serial.print("X Position: ");
  // Serial.print(XMagnitude);  
  // Serial.print("  | ");
  // Serial.print("Y Position: ");
  // Serial.print(YMagnitude);
  // Serial.print("\n");


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//------------------------------------------END OF ENCODER CODE ------------------------------------------------------------///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

switch (mode) {
  case 1:   // Initial state
    break;
  case 2:   // Drive forwards to find line 
      Motors.setMotorPower(14, 13);             // Initial pwm when looking for line
      Sensors.findLine();                       // Function to drive forwards in search of the line
      mode = 3;                                 // Switch to mode 3 - Orientate
  case 3:   // Orientate
      Serial.print("Orientate");
      Motors.setMotorPower(0,14);             // Turns right to initially orientate with the line
      delay(2500);                            // Small 1 second delay to allow enough time for to orientate
      if (abs(Sensors.e_line > 0.1)) {
        lineFollowing(10);                    // starts off slowely to finish orientating
        }
      else {
        mode = 4;                             // Swith to mode for - Follow Line
      }
  case 4:   // Follow Line
      Serial.print("Mode 4");
      if (elapsed_time_ls > linesensor_update_time) {  // Code block that regulates the frequency of the light sensor read.
        Sensors.chargeCapacitors();                    // Charges the capacitors
        Sensors.parallelSensorRead();                  // Reads the sensors
        linesensor_ts = millis();                      // Sets linesensor_ts to current time since program start
      }

      // Normal Line following operation
      if (elapsed_time_motor > motor_update) {          // Non-Blocking Delay
        lineFollowing(14);                              // more time has passed than the time seperation continue line following with a specified forward speed
        motor_ts = millis();                            // time-stamp reset
      }

      if (((Sensors.ls_reading[0] + Sensors.ls_reading[1] + Sensors.ls_reading[2] + Sensors.ls_reading[3] + Sensors.ls_reading[4]) / 5 ) < lossThreshold){          // If mean sensor value is less than line threshold
          // LINE LOST
          Serial.print("Line Lost\n");
          //mode = 5;                                     // Line has been lost so update mode and go to line lost case
      }
  case 5:   // Line Lost
        Serial.print("Mode 5\n");
        if (end == false){               // if not at the end
          if (gap == false){             // and the next loss will not be the gap
            //turn 180 degrees and carry on line following (for the general solution)
          while ((turnAngle * (180/3.142)) < 180 ){ // While angle is less than 180 degrees - keep turning.
            Serial.print("Turn Angle: ");  
            Serial.print(turnAngle);
            delay(10);
            Serial.print("\n");      
            //Motors.setMotorPower(-13,13);  
          }
            if ((turnAngle * (180/3.14159265))  >= 170 ){        // Once the angle is near enough to 180, change back to mode 4 - line following
             mode = 4;
            }
          }
          else if (gap == true) {     // This will be the centre gap
          delay(2000);                // drive forward for enough time to rejoin the line. 
          gapPassed = true;           // Flag to show the middle gap has passed 
          mode = 4;                   // carry on normal operation
          }
        }
        else if (end == true) {
        // Return to start
        Motors.setMotorPower(0, 0);     // stop the motors
        mode = 6;                       // initiate the next mode - return to start (6)
        }
  case 6: ;  // Return To Start
}



}  // end of main loop()


void lineFollowing(float forward_pwm) {

float e_line, w_right, w_left;

//  Serial.print(Sensors.ls_error());
//  Serial.print("\n");
//  Serial.print(Sensors.w_left);
//  Serial.print(" <----- Left \n");
//
//  Serial.print(Sensors.w_right);
//  Serial.print(" <----- Right \n");
//
//  Serial.print(Sensors.e_line);
//  Serial.print(" <----- Error \n");



  float turn_pwm, max_pwm, left, right, neg_turn_pwm;//, forward_pwm;
  max_pwm = 20;  //
  //forward_pwm = 13;
  turn_pwm = Sensors.e_line * max_pwm;  // Turn PWM = error * maximum pwm to be increased outside wheel of a corner
  neg_turn_pwm = -turn_pwm / 2;         // Negative turn PWM to be applied to the inside wheel of a corner
  
  left = forward_pwm - turn_pwm;
  right = forward_pwm + turn_pwm;

//  Serial.print("Left PWM: ");
//  Serial.print(left);
//  Serial.print("\n");
//
//  Serial.print("Right PWM: ");
//  Serial.print(right);
//  Serial.print("\n");

  if (abs(Sensors.e_line)> 0.25 ){
    //Serial.print("RightAngle");
    //Serial.print("here\n");
    if (Sensors.e_line < 0 ){            // If error is negative and line is on the right of the robot
      Motors.setMotorPower( left, 0 );                    // Right PWM = 0, Left PWM = forward + correction turn pwm
    }
    else if (Sensors.e_line > 0){       // If e_line is positive and line is on the left
      Motors.setMotorPower( 0, right );             // Left PWM = 0, Right PWM = forward + correction turn pwm
    }
  }
  
  else {
    if (Sensors.e_line < 0 || abs(Sensors.e_line > 0.1 )){            // If e_line is negative and |e_line| is greater than 0.2
      Motors.setMotorPower( left, (forward_pwm - neg_turn_pwm) );                    // Right PWM = Normal forward speed, Left PWM = forward + correction turn pwm
    }
    else if (Sensors.e_line > 0 || abs(Sensors.e_line > 0.1 )){       // If e_line is positive and |e_line| is greater than 0.2
      Motors.setMotorPower( (forward_pwm - neg_turn_pwm), right );                   // Left PWM = Normal forward speed, Right PWM = forward + correction turn pwm
    }
    else {
      Motors.setMotorPower(left, right);        // e_line is less than |0.1| -- no correction is required at the moment.
    }
  }

}



// A functin to determine the return angle and magnitude when the course has been finished.

void kinematics(){


  float magnitude = sqrt((XMagnitude*XMagnitude) + (YMagnitude*YMagnitude));           // to determine the magnitude of the hypotenuse
  float returnAngle = atan(XMagnitude / YMagnitude)* (180/3.142);                      // the angle from the end point to the origin
  Serial.print("\nMagnitude: ");
  Serial.print(magnitude);
  Serial.print("\n");
  Serial.print("returnAngle: ");
  Serial.print(returnAngle);
  Serial.print("\n");



}

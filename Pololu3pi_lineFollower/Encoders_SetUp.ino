#include "encoders.h"


//encoders encoders;

void updateKinematics();

int oldLeftCount = 0; // initialise to origin on startup
int oldRightCount = 0;
float distancePerCount = 0.00028;      // distance per count in metres (~ wheel circumference / counts per revolution)
float leftDistanceFromOrigin = 0.0;
float rightDistanceFromOrigin = 0.0;
float wheelDistanceFromOrigin = 0.0425;     // Half the wheel seperation (m)
float XMagnitude = 0.0;
float YMagnitude = 0.0;
float totalAngle = 0;           // total angle of the wheel from the normal

void setup() {

  //...
  // Setup encoders when the robot is 
  // powered on. 
  setupEncoder0();
  setupEncoder1();


}

void loop() {
  // put your main code here, to run repeatedly:

  // Serial.print("Left: ");
  // Serial.print(count_eLeft);
  // Serial.print("    |    ");
  // Serial.print("Right: ");
  // Serial.print(count_eRight);
   Serial.print("\n");
  delay(2000);


// LefT Wheel Distance
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
  Serial.print(totalAngle* (180/3.142));

  XMagnitude = XMagnitude + (ChangeInDistance*cos(angleFromOrigin));
  YMagnitude = YMagnitude + (ChangeInDistance*sin(angleFromOrigin));

  // Serial.print("X Position: ");
  // Serial.print(XMagnitude);  
  // Serial.print("  | ");
  // Serial.print("Y Position: ");
  // Serial.print(YMagnitude);
  // Serial.print("\n");

  
kinematics();

}

// A functin to determine the return angle and magnitude when the course has been finished.

void kinematics(){

  //float magnitude = sq(Xmagnitude) + sq(Ymagnitude);           //pythagoras to determine the magnitude of the return length
  float magnitude = sqrt((XMagnitude*XMagnitude) + (YMagnitude*YMagnitude));           //pythagoras to determine the magnitude of the return length
  float returnAngle = atan(XMagnitude / YMagnitude)* (180/3.142);
  Serial.print("\nMagnitude: ");
  Serial.print(magnitude);
  Serial.print("\n");
  Serial.print("returnAngle: ");
  Serial.print(returnAngle);
  Serial.print("\n");



}
 

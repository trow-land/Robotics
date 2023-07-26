// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _LineSensor_H_
#define _LineSensor_H_

#include "motors.h"

# define LeftLeft_LS_Pin A11
# define Left_LS_PIN A0
# define Center_LS_PIN A2
# define Right_LS_PIN A3
# define RightRight_LS_Pin A4
# define LS_EMIT_PIN 11
# define BUZZER_PIN 6

# define NumberOfSensors 5       // Number of sensors in use


// Class to operate the linesensor(s).
class LineSensor {
  public:

    int ls_pin[NumberOfSensors] ;
    float ls_reading[NumberOfSensors];
    float w_left, w_leftleft, w_right, w_rightright, total_w_left, total_w_right, e_line;
    bool linePositionKnown = false;

    //float ls_sum;
    
    LineSensor() { 
    ls_pin[0] = LeftLeft_LS_Pin;
    ls_pin[1] = Left_LS_PIN;
    ls_pin[2] = Center_LS_PIN;
    ls_pin[3] = Right_LS_PIN;
    ls_pin[4] = RightRight_LS_Pin;
    };      // Default constructor, must exist.




void buzzer(int delayDuration, int count){          //Function to control the buzzer for the initial startup beep
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(delayDuration);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(delayDuration);
    count += 1;
} // End of buzzer






void initialise(){
    //Emit pins
    pinMode(LS_EMIT_PIN, OUTPUT); //Should be set to HIGH to enable operation of the line sensors
    digitalWrite(LS_EMIT_PIN, HIGH);

    //Sensor read pins
    pinMode(LeftLeft_LS_Pin, INPUT);
    pinMode(Left_LS_PIN, INPUT);
    pinMode(Center_LS_PIN, INPUT);
    pinMode(Right_LS_PIN, INPUT);
    pinMode(RightRight_LS_Pin, INPUT);

    //Establishing the Serial Connection
    Serial.begin(9600);
    delay(1000);
    Serial.println("*****RESET*****");

    //Reset BEEP
    for (int i = 0; i < 20; i++){
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, HIGH);
    buzzer(1000, 0);
    }

} // End of initialise







void parallelSensorRead(){
  
  int which;                                        //which sensor

  unsigned long start_time;                         //Record the start time.
  start_time = micros();

  int remaining = NumberOfSensors;

  unsigned long end_time_LS[ NumberOfSensors ];       // and empty array to store the sensor readings

  for (int i = 0; i < NumberOfSensors; i++){          // initialises them to zero
    end_time_LS[i] = 0;
  }

  while( remaining > 0 ) {                                   // While we still have sensors to read.

    for( which = 0; which < NumberOfSensors; which++ ) {     // Loops between sensors

        if( digitalRead( ls_pin[ which ] ) == LOW ) {        // If digitalRead() returns LOW, it means this sensor has completed and is below the voltage threshold

          if( end_time_LS[ which ] == 0 ) {                 // Check if this sensor has had a previous value of elapsed time stored

            end_time_LS[which] = micros(); // The time since program started to the sensor to get below threshold
            
            remaining = remaining - 1;
          } // end of if == 0
        } // end of if( digitalRead() )

    } // end of for() looping through each sensor.
  } // end of while( remaining > 0 )
  
    for (which = 0; which < NumberOfSensors; which++){
    unsigned long elapsed_time;
    elapsed_time = end_time_LS[which] - start_time;
    ls_reading[which] = (float)elapsed_time;



//TEMPORARY ------- PRINTS THE OUTPUT OF THE lS_READ ARRAY AFTER THEY HAVE BEEN FILLED 
//------------------------------------------------------------------------------------------------------
  // for (int i = 0; i < NumberOfSensors; i++){        // prints the value followed by a ','
  //   Serial.print(ls_reading[i]);
  //   Serial.print(", ");
  // }

  // //unsigned long elapsed_time;                         
  // unsigned long end_time = micros();
  // elapsed_time = end_time - start_time;
  // Serial.print(elapsed_time);                         // Prints the total elapsed time
  // Serial.print("\n");

//TEMPORARY----------------------------------------------------------------------------------------------

  

    } //end of for(which = 0; which < NumberOfSensors; which++)
} // end of parallelSensorRead;

  





void displayReadings(unsigned long start_time){       // Cycles through the array filled with the sensor readings
    for (int i = 0; i < NumberOfSensors; i++){        // prints the value followed by a ','
    Serial.print(ls_reading[i]);
    Serial.print(", ");
  }

  unsigned long elapsed_time;                         
  unsigned long end_time = micros();
  elapsed_time = end_time - start_time;
  Serial.print(elapsed_time);                         // Prints the total elapsed time
  Serial.print("\n");
}     //End of displayReadings






void chargeCapacitors(){
  
  int which;                                            //Which sensor
  
  for( which = 0; which < NumberOfSensors; which++ ){

    pinMode( ls_pin[which], OUTPUT ); 
    digitalWrite( ls_pin[which], HIGH );                 // temporarily to output and HIGH to charge capacitors
  }
  
  delayMicroseconds(10);                                // Tiny delay for capacitor to charge.
  for( which = 0; which < NumberOfSensors; which++ ){   //  Turn input pin back to an input
    pinMode( ls_pin[which], INPUT );

  }

}

float ls_error(){         // A Function to determine the weighted error from the line from the line sensors
  float sum = 0;
  int i = 0;
  for (i; i < NumberOfSensors; i++){
    sum = sum + float(ls_reading[i]);     // Sums up all of the sensor readings

  }
  w_leftleft = ls_reading[0] / sum;
  w_left = ls_reading[1] / sum;           // Determines the left weight by dividing the reading by the overall sum
  w_right = ls_reading[3] / sum;          // Determines the right weight by dividing the reading by the overall sum
  w_rightright = ls_reading[4] / sum;

  total_w_right = w_right + w_rightright;
  total_w_left = w_left + w_leftleft;

  e_line = total_w_left - total_w_right;              // The error is the given by left - right

  return e_line;
}

void findLine() {

  float lineThreshold = 1500.00;
 
  while (linePositionKnown == false) {     // While loop that executes as long as line is not found
  
    chargeCapacitors();
    parallelSensorRead();  // Read sensors as per normal operation
    //displayReadings(start_time);
    //delay(1000);
    //Serial.print("**************************************\n");
      double sum = ls_reading[0] + ls_reading[1] + ls_reading[2] + ls_reading[3] + ls_reading[4];
      double mean = sum / NumberOfSensors;
      Serial.print(mean);                  // REMOVE
      Serial.print("\n");                  // REMOVE
      //delay(100); //                        REMOVE
      //Serial.print(ls_error());
      //Serial.print("\n");
        if (mean > lineThreshold) {
        Serial.print("Line Found");  // If sensor readings are above threshold then line is found and flag is set to true
        linePositionKnown = true;
      } else {
        linePositionKnown = false;
        //Serial.print("Line NOT found");        
      }
    //}
  } // end of while loop
} // end of void findLine

  
}; //End of class



#endif 

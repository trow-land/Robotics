#ifndef _motors_H_
#define _motors_H_

# define L_PWM_PIN 10
# define L_DIR_PIN 16 
# define R_PWM_PIN 9
# define R_DIR_PIN 15

# define FWD LOW
# define REV HIGH

class Motors {
  public:

  //constructor
  Motors(){};

  void initialise(){

  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);

  //analogWrite(L_PWM_PIN, LOW);
  //analogWrite(R_PWM_PIN, LOW);


  Serial.begin(9600);
  delay(1000);
  Serial.println("*****RESET*****");
  
  }

  void testSequence(){
    for (int i = -100; i <= 100; i++){
    setMotorPower(i, i);
    delay(100);
  }
  }

  void setMotorPower(float left_pwm, float right_pwm){
  if (left_pwm <= -50) {left_pwm = -50;}
  if (left_pwm >= 50) {left_pwm = 50;}
  if (right_pwm <= -50) {right_pwm = -50;}
  if (right_pwm >= 50) {right_pwm = 50;}


  if (right_pwm >= 0)     
  {
  // Right motor is rotating in FORWARD
  digitalWrite(R_DIR_PIN, FWD);
  analogWrite( R_PWM_PIN, right_pwm );
  }
  else {
  // Right motor is rotating in REVERSE
  digitalWrite(R_DIR_PIN, REV);
  analogWrite( R_PWM_PIN, abs(right_pwm) );
  } 

  if (left_pwm >= 0) {
    // Left motor is rotating in FORWARD'
    digitalWrite(L_DIR_PIN, FWD);
    analogWrite( L_PWM_PIN, left_pwm );
  }
  else {
  // Left motor is rotating in REVERSE
  digitalWrite(L_DIR_PIN, REV);
  analogWrite( L_PWM_PIN, abs(left_pwm) );
  }
  }
};

#endif 

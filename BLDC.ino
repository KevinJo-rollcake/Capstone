//NURI ROBOT
//Arduino Uno
//PWM, Direction, Brake
//V3.

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
// 카메라
#define PWM1 2 //PWM
#define Direction1 22 //Direction
#define Brake1 24 //Brake

#define PWM2 4 //PWM
#define Direction2 30 //Direction
#define Brake2 32 //Brake

#define PWM3 6 //PWM
#define Direction3 38 //Direction 
#define Brake3 40 //Brake

#define PWM4 8 //PWM
#define Direction4 46 //Direction
#define Brake4 48 //Brake

#define Duty_MAX 120

volatile float DutyRef1 = 0; //PWM duty reference -1024~1024(-100%~100%)
volatile float Duty1 = 0;

volatile float DutyRef2 = 0; //PWM duty reference -1024~1024(-100%~100%)
volatile float Duty2 = 0;

volatile float DutyRef3 = 0; //PWM duty reference -1024~1024(-100%~100%)
volatile float Duty3 = 0;

volatile float DutyRef4 = 0; //PWM duty reference -1024~1024(-100%~100%)
volatile float Duty4 = 0;

ros::NodeHandle nh;msg_ok
//Sub
void messageCb( const std_msgs::Float32MultiArray& GoTobldc) {
  DutyRef1 = GoTobldc.data[0];
  DutyRef2 = GoTobldc.data[1];
  DutyRef3 = GoTobldc.data[2];
  DutyRef4 = GoTobldc.data[3];
} // callback Function.

ros::Subscriber<std_msgs::Float32MultiArray> BLDC("GoTobldc", messageCb);

void flash() {
  if(Duty1 < DutyRef1) Duty1++;
  else if(Duty1 > DutyRef1) Duty1--;

  if (Duty1 > 0)
  {
    if (Duty1 > Duty_MAX) Duty1 = Duty_MAX;
    analogWrite(PWM1, Duty1); //PWM, Duty
    digitalWrite(Direction1, HIGH);
  }
  else
  {
    if (Duty1 < -Duty_MAX) Duty4 = -Duty_MAX;
    analogWrite(PWM1, -1 * Duty1); //PWM, Duty
    digitalWrite(Direction1, LOW);
  }

  if(Duty2 < DutyRef2) Duty2++;
  else if(Duty2 > DutyRef2) Duty2--;
  if (Duty2 > 0)
  {
    if (Duty2 > Duty_MAX) Duty2 = Duty_MAX;
    analogWrite(PWM2, Duty2); //PWM, Duty
    digitalWrite(Direction2, LOW);
  }
  else
  {
    if (Duty2 < -Duty_MAX) Duty2 = -Duty_MAX;
    analogWrite(PWM2, -1 * Duty2); //PWM, Duty
    digitalWrite(Direction2, HIGH);
  }

  if(Duty3 < DutyRef3) Duty3++;
  else if(Duty3 > DutyRef3) Duty3--;
  if (Duty3 > 0)
  {
    if (Duty3 > Duty_MAX) Duty3 = Duty_MAX;
    analogWrite(PWM3, Duty3); //PWM, Duty
    digitalWrite(Direction3, LOW);
  }
  else
  {
    if (Duty3 < -Duty_MAX) Duty3 = -Duty_MAX;
    analogWrite(PWM3, -1 * Duty3); //PWM, Duty
    digitalWrite(Direction3, HIGH);
  }
  
  if(Duty4 < DutyRef4) Duty4++;
  else if(Duty4 > DutyRef4) Duty4--;
  if (Duty4 > 0)
  {
    if (Duty4 > Duty_MAX) Duty4 = Duty_MAX;
    analogWrite(PWM4, Duty4); //PWM, Duty
    digitalWrite(Direction4, HIGH);
  }
  else
  {
    if (Duty4 < -Duty_MAX) Duty4 = -Duty_MAX;
    analogWrite(PWM4, -1 * Duty4); //PWM, Duty
    digitalWrite(Direction4, LOW);
  }
}

void setup() {
  nh.initNode();
  nh.subscribe(BLDC);
  
  //PWM
  Timer5.initialize();
  Timer5.pwm(PWM1, 0); //PWM, Duty 0%
  Timer5.setPwmDuty(PWM1, 0); //PWM, Duty 0%
  Timer5.pwm(PWM2, 0); //PWM, Duty 0%
  Timer5.setPwmDuty(PWM2, 0); //PWM, Duty 0%
  Timer5.pwm(PWM3, 0); //PWM, Duty 0%
  Timer5.setPwmDuty(PWM3, 0); //PWM, Duty 0%  
  Timer5.pwm(PWM4, 0); //PWM, Duty 0%
  Timer5.setPwmDuty(PWM4, 0); //PWM, Duty 0%
  
  Timer5.setPeriod(50); //PWM period 20kHz
  //Direction
  pinMode(Direction1, OUTPUT); //Direction, Output
  pinMode(Direction2, OUTPUT); //Direction, Output
  pinMode(Direction3, OUTPUT); //Direction, Output
  pinMode(Direction4, OUTPUT); //Direction, Output

  digitalWrite(Direction1, LOW);
  digitalWrite(Direction2, LOW);
  digitalWrite(Direction3, LOW);
  digitalWrite(Direction4, LOW);
  
  //Brake
  pinMode(Brake1, OUTPUT); //Brake, Output
  pinMode(Brake2, OUTPUT); //Brake, Output
  pinMode(Brake3, OUTPUT); //Brake, Output
  pinMode(Brake4, OUTPUT); //Brake, Output

  digitalWrite(Brake1, LOW); //Brake, '0'
  digitalWrite(Brake2, LOW); //Brake, '0'
  digitalWrite(Brake3, LOW); //Brake, '0'
  digitalWrite(Brake4, LOW); //Brake, '0'
  digitalWrite(Brake, LOW); //Brake, '0'

  //Timer interrupt
  MsTimer2::set(10/Slope, flash); //Timer interrupt period  
  MsTimer2::start(); //Timer interrupt start
}

void loop() {
  nh.spinOnce();
}

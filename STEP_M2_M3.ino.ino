#include <TimerOne.h>
#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#define M2_photo_pin 11
#define M3_photo_pin 9

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
// M2이 앞쪽
// M3가 뒤쪽

#define M2dirpin 7 //Motor X direction pin
#define M2steppin 6 //Motor X step pin
#define M2en 8 //Motor X enable pin
#define M3dirpin 4 //Motor Y direction pin
#define M3steppin 5 //Motor Y step pin
#define M3en 12 //Motor Y enable pin


float goal_M2 = 0;
float goal_M3 = 0;
float now_M2 = 0;
float now_M3 = 0;
float goto_M2 = 0;
float goto_M3 = 0;

bool chk = 0;

bool current_M2 = 1;
bool current_M3 = 1;

int startcnt = 1;

int M2_photo = 0;
int M3_photo = 0;
//for Timer5

#define motorInterfaceType 1
// Create a new instance of the AccelStepper class:

AccelStepper stepperM2 = AccelStepper(motorInterfaceType, M2steppin, M2dirpin);
AccelStepper stepperM3 = AccelStepper(motorInterfaceType, M3steppin, M3dirpin);

//------------------------
bool t5_flag = 0;
unsigned int t5_index = 0;
unsigned int send_t_cnt = 0;
//------------------------

ros::NodeHandle_<ArduinoHardware, 2, 1, 250, 150> nh;
//std_msgs::String msg;
std_msgs::String msg_ok;
//ros::Publisher step_pub_right("stepper_position_right", &msg);
ros::Publisher step_OK_right("stepOK_right", &msg_ok);

//Sub
void messageCb( const std_msgs::Float32MultiArray& GoTostep) {
  goal_M2 = GoTostep.data[1];
  goal_M3 = GoTostep.data[2];
} // callback Function.

void messageCbSTART( const std_msgs::Bool& stepstart) {
  chk = stepstart.data;
}

ros::Subscriber<std_msgs::Float32MultiArray> sub_pos_right("GoTostep", messageCb);
ros::Subscriber<std_msgs::Bool> sub_start_right("stepstart", messageCbSTART);


void setup() {

  pinMode(M2dirpin,OUTPUT);
  pinMode(M2steppin,OUTPUT);
  pinMode(M2en,OUTPUT);
  pinMode(M3dirpin,OUTPUT);
  pinMode(M3steppin,OUTPUT);
  pinMode(M3en,OUTPUT);

  digitalWrite(M2en,LOW);// Low Level Enable
  digitalWrite(M3en,LOW);// Low Level Enable
  
  // Set the maximum speed in steps per second:
  nh.initNode();
  //nh.advertise(step_pub_left);
  nh.advertise(step_OK_right);
  nh.subscribe(sub_pos_right);
  nh.subscribe(sub_start_right);
  
  stepperM2.setMaxSpeed(300.0);
  stepperM2.setAcceleration(500.0);
  stepperM3.setMaxSpeed(300.0);
  stepperM3.setAcceleration(500.0);
  Timer1.initialize(10000); //10msec
  Timer1.attachInterrupt(T5_ISR);
  
  t5_flag = 0;
  pinMode(M2_photo_pin, INPUT);
  pinMode(M3_photo_pin, INPUT);
}

 

void loop() {
  nh.spinOnce();
  if(chk && startcnt == 1)
    {
      startcnt = 0;
      while(digitalRead(M2_photo_pin) || digitalRead(M3_photo_pin)){
        if(digitalRead(M2_photo_pin))
          {
            stepperM2.setSpeed(-80);
            stepperM2.runSpeed();
          }
          
        if(digitalRead(M3_photo_pin))
          {
            stepperM3.setSpeed(80);
            stepperM3.runSpeed();
          }
        }

      while(!digitalRead(M2_photo_pin) || !digitalRead(M3_photo_pin)){
        if(!digitalRead(M2_photo_pin))
          {
            stepperM2.setSpeed(-80);
            stepperM2.runSpeed();
          }
          
        if(!digitalRead(M3_photo_pin))
          {
            stepperM3.setSpeed(80);
            stepperM3.runSpeed();
          }
        }
      msg_ok.data = "Ready_right";
      step_OK_right.publish(&msg_ok);
   }
   
  if (t5_flag && startcnt == 0) {
    t5_flag = 0;
    switch (t5_index) {
      case 0:
        t5_index = 1;
        stepperM2.setCurrentPosition(0);
        stepperM3.setCurrentPosition(0);
        current_M2 = 1;
        current_M3 = 1;
        break;
  
      case 1:
        t5_index = 2;
        goto_M2 = goal_M2 - now_M2;
        goto_M3 = goal_M3 - now_M3;
        now_M2 = goal_M2;
        now_M3 = goal_M3;
        break;
  
      case 2:
        t5_index = 3;
        M_control();
        break; 
  
      case 3:
        t5_index = 4;
        break;
  
      case 4:
        t5_index = 5;
        break;
  
      case 5:
        t5_index = 6;
        stepperM2.setCurrentPosition(0);
        stepperM3.setCurrentPosition(0);
        current_M2 = 1;
        current_M3 = 1;
        break;
  
      case 6:
        t5_index = 7;
        goto_M2 = goal_M2 - now_M2;
        goto_M3 = goal_M3 - now_M3;
        now_M2 = goal_M2;
        now_M3 = goal_M3;
        break;
  
      case 7:
        t5_index = 8;
        M_control();
        break;
  
      case 8:
        t5_index = 9;
        break;
  
      case 9:
        t5_index = 0;
        break;
        
      default:
        t5_index = 0;
        break;
      }
    }
}

void M_control(){
  while(current_M2 || current_M3){
  if(goto_M3 >= 0)
    {
      if(stepperM3.currentPosition() != goto_M3)
      {
        stepperM3.setSpeed(150);
        stepperM3.runSpeed();
      }
      else
      {
        current_M3 = 0;
      }
    }
  else
    {
      if(stepperM3.currentPosition() != goto_M3)
      {
        stepperM3.setSpeed(-150);
        stepperM3.runSpeed();
      }
      else
      {
        current_M3 = 0;
      }
    }
    
  if(goto_M2 >= 0)
    {
      if(stepperM2.currentPosition() != goto_M2)
      {
        stepperM2.setSpeed(150);
        stepperM2.runSpeed();
      }
      else
      {
        current_M2 = 0;
      }
    }
  else
    {
      if(stepperM2.currentPosition() != goto_M2)
      {
        stepperM2.setSpeed(-150);
        stepperM2.runSpeed();
      }
      else
      {
        current_M2 = 0;
      }
    }
  }
}


void T5_ISR() {
  t5_flag = 1;
//t5_index++;
}

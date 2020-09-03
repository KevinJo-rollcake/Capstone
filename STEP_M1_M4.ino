#include <TimerOne.h>
#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#define M1_photo_pin 11
#define M4_photo_pin 9

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
// M1이 앞쪽
// M4가 뒤쪽

#define M1dirpin 7 //Motor X direction pin
#define M1steppin 6 //Motor X step pin
#define M1en 8 //Motor X enable pin
#define M4dirpin 4 //Motor Y direction pin
#define M4steppin 5 //Motor Y step pin
#define M4en 12 //Motor Y enable pin


float goal_M1 = 0;
float goal_M4 = 0;
float now_M1 = 0;
float now_M4 = 0;
float goto_M1 = 0;
float goto_M4 = 0;

bool chk = 0;

bool current_M1 = 1;
bool current_M4 = 1;

int startcnt = 1;

int M1_photo = 0;
int M4_photo = 0;
//for Timer5

#define motorInterfaceType 1
// Create a new instance of the AccelStepper class:

AccelStepper stepperM1 = AccelStepper(motorInterfaceType, M1steppin, M1dirpin);
AccelStepper stepperM4 = AccelStepper(motorInterfaceType, M4steppin, M4dirpin);

//------------------------
bool t5_flag = 0;
unsigned int t5_index = 0;
unsigned int send_t_cnt = 0;
//------------------------

ros::NodeHandle_<ArduinoHardware, 2, 1, 250, 150> nh;
//std_msgs::String msg;
std_msgs::String msg_ok;
ros::Publisher step_OK_left("stepOK_left", &msg_ok);

//Sub
void messageCb( const std_msgs::Float32MultiArray& GoTostep) {
  goal_M1 = GoTostep.data[0];
  goal_M4 = GoTostep.data[3];
} // callback Function.

void messageCbSTART( const std_msgs::Bool& stepstart) {
  chk = stepstart.data;
}

ros::Subscriber<std_msgs::Float32MultiArray> sub_pos_left("GoTostep", messageCb);
ros::Subscriber<std_msgs::Bool> sub_start_left("stepstart", messageCbSTART);


void setup() {
  pinMode(M1dirpin,OUTPUT);
  pinMode(M1steppin,OUTPUT);
  pinMode(M1en,OUTPUT);
  pinMode(M4dirpin,OUTPUT);
  pinMode(M4steppin,OUTPUT);
  pinMode(M4en,OUTPUT);

  digitalWrite(M1en,LOW);// Low Level Enable
  digitalWrite(M4en,LOW);// Low Level Enable

  // Set the maximum speed in steps per second:
  nh.initNode();
  //nh.advertise(step_pub_left);
  nh.advertise(step_OK_left);
  nh.subscribe(sub_pos_left);
  nh.subscribe(sub_start_left);
  
  stepperM1.setMaxSpeed(300.0);
  stepperM1.setAcceleration(500.0);
  stepperM4.setMaxSpeed(300.0);
  stepperM4.setAcceleration(500.0);
  Timer1.initialize(10000); //10msec
  Timer1.attachInterrupt(T5_ISR);
  
  t5_flag = 0;
  pinMode(M1_photo_pin, INPUT);
  pinMode(M4_photo_pin, INPUT);
}

 

void loop() {
  
  if(chk && startcnt == 1)
    {
      startcnt = 0;
      while(digitalRead(M1_photo_pin) || digitalRead(M4_photo_pin)){
        if(digitalRead(M1_photo_pin))
          {
            stepperM1.setSpeed(-80);
            stepperM1.runSpeed();
          }
          
        if(digitalRead(M4_photo_pin))
          {
            stepperM4.setSpeed(80);
            stepperM4.runSpeed();
          }
        }

      while(!digitalRead(M1_photo_pin) || !digitalRead(M4_photo_pin)){
        if(!digitalRead(M1_photo_pin))
          {
            stepperM1.setSpeed(-80);
            stepperM1.runSpeed();
          }
          
        if(!digitalRead(M4_photo_pin))
          {
            stepperM4.setSpeed(80);
            stepperM4.runSpeed();
          }
        }
      msg_ok.data = "Ready_left";
      step_OK_left.publish(&msg_ok);
   }
   
  if (t5_flag && startcnt == 0) {
    t5_flag = 0;
    switch (t5_index) {
      case 0:
        t5_index = 1;
        stepperM1.setCurrentPosition(0);
        stepperM4.setCurrentPosition(0);
        current_M1 = 1;
        current_M4 = 1;
        break;
  
      case 1:
        t5_index = 2;
        goto_M1 = goal_M1 - now_M1;
        goto_M4 = goal_M4 - now_M4;
        now_M1 = goal_M1;
        now_M4 = goal_M4;
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
        stepperM1.setCurrentPosition(0);
        stepperM4.setCurrentPosition(0);
        current_M1 = 1;
        current_M4 = 1;
        break;
  
      case 6:
        t5_index = 7;
        goto_M1 = goal_M1 - now_M1;
        goto_M4 = goal_M4 - now_M4;
        now_M1 = goal_M1;
        now_M4 = goal_M4;
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
 nh.spinOnce();
}

void M_control(){
  while(current_M1 || current_M4){
  if(goto_M4 >= 0)
    {
      if(stepperM4.currentPosition() != goto_M4)
      {
        stepperM4.setSpeed(150);
        stepperM4.runSpeed();
      }
      else
      {
        current_M4 = 0;
      }
    }
  else
    {
      if(stepperM4.currentPosition() != goto_M4)
      {
        stepperM4.setSpeed(-150);
        stepperM4.runSpeed();
      }
      else
      {
        current_M4 = 0;
      }
    }
    
  if(goto_M1 >= 0)
    {
      if(stepperM1.currentPosition() != goto_M1)
      {
        stepperM1.setSpeed(150);
        stepperM1.runSpeed();
      }
      else
      {
        current_M1 = 0;
      }
    }
  else
    {
      if(stepperM1.currentPosition() != goto_M1)
      {
        stepperM1.setSpeed(-150);
        stepperM1.runSpeed();
      }
      else
      {
        current_M1 = 0;
      }
    }
  }
}


void T5_ISR() {
  t5_flag = 1;
//t5_index++;
}

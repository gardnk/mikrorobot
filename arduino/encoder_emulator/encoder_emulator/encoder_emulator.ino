#include <ros.h>
#include <sensor_msgs/JointState.h>

const byte pinAforEncoder[] = {2,4,6,8}; //Array of pins set up as A pin -> the interrupt pin
const byte pinBforEncoder[] = {3,5,7,9}; //Array of pins set up as B pin

byte pinAPrev[4]; //previous value of the pins 1-4
int duration[4]; //the number of the pulses from motor 1-4
boolean Direction[4]; //the rotation direction of motor 1-4
float rpm[4]; //rotations per minute of motor 1-4
float speeds[4]; //speed for motor 1-4 in m/s

float ppr; //pulse per rotation for our motor
float wheelradi;
float gearRatio;
float pi;

ros::NodeHandle_<ArduinoHardware, 2, 2, 1000, 1000> nh;

sensor_msgs::JointState msg;

ros::Publisher pub("motor_speeds", &msg);

void setup()
{
  msg.velocity_length = 4;
  
  //Initialize ROS subscription
  nh.initNode();
  nh.advertise(pub);

  //Encoderkode:
  EncoderInit();//Initialize the module
  ppr = 16; //DETTE ER FEIL MEST SANNSYNLIG!!!
  gearRatio = 0.667;
  wheelradi = (0.2032/2); //metres
  pi = 3.14159265;
}

void loop()
{
  nh.spinOnce();
  
  float data[4];

  //start encoderkode:
  for (int i = 0; i<4;i++){
    rpm[i] = duration[i]*gearRatio/ppr;
    speeds[i] = 2*pi*wheelradi*rpm[i]/60; // m/s
  }

  //slutt encoderkode
  
  msg.velocity = rpm;
  
  pub.publish(&msg);
  Serial.flush();
  delay(10);
}

void EncoderInit()
{
  for (int i = 0; i<4;i++){
    Direction[i] = true;//default -> Forward
    pinMode(pinBforEncoder[i],INPUT);
    //attachInterrupt(pinAforEncoder[i], wheelSpeed(i), CHANGE);
  }
  attachInterrupt(pinAforEncoder[0], wheel0, CHANGE);
  attachInterrupt(pinAforEncoder[1], wheel1, CHANGE);
  attachInterrupt(pinAforEncoder[2], wheel2, CHANGE);
  attachInterrupt(pinAforEncoder[3], wheel3, CHANGE);
}

void wheel0(){
  wheelSpeed(0);
}
void wheel1(){
  wheelSpeed(1);
}
void wheel2(){
  wheelSpeed(2);
}
void wheel3(){
  wheelSpeed(3);  
}
  
void wheelSpeed(int n){
  
  int Lstate = digitalRead(pinAforEncoder[n]);
  if((pinAPrev[n] == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(pinBforEncoder[n]);
    if(val == HIGH && Direction[n])
    {
      Direction[n] = false; //Reverse
    }
    else if(val == LOW && !Direction[n])
    {
      Direction[n] = true;  //Forward
    }
  }
  pinAPrev[n] = Lstate;
  
  if(!Direction[n])  duration[n]++;
  else  duration[n]--;
}


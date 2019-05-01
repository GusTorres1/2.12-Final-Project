/*  To use this code, you must first have rosserial_python
 *  and from there you can use rosserial_arduino. Make
 *  sure that rosserial_arduino is running. Ask Jerry the
 *  TA for more clarification.
 */

#include <ros.h>
#include <std_msgs/Int8.h>

// I believe that brown wire goes to 2, white -> 3

int controlInf = 2;
int controlVac = 3;

// led for testing purposes
int ledTest = 13;

// Creates a ROS node
ros::NodeHandle Gripper;

// Subscribes to Int8 message
// 0 - vacuum
// 1 - inflate
// 2 - stop
void messageCb( const std_msgs::Int8& toggle_msg){
  if (toggle_msg.data == 2) {
    stop_all();
  }
  else{
    switchMode(toggle_msg.data);
  }
}

// ROS subscriber listen to: /toggle_vac
ros::Subscriber<std_msgs::Int8> sub("toggle_vac", &messageCb );

void setup() {
  pinMode(controlInf, OUTPUT);
  pinMode(controlVac, OUTPUT);  
  digitalWrite(controlInf, HIGH); // HIGH = off!
  digitalWrite(controlVac, HIGH);

  // TODO: create a publisher

  // Creating a subscriber node
  Gripper.initNode();
  Gripper.subscribe(sub);
}

void loop() {
  Gripper.spinOnce();
  delay(1);
}

// Function for switching between inflate and vacuum
void switchMode(int toggle_msg){
  if (toggle_msg == 1){
    digitalWrite(controlInf, LOW);
    digitalWrite(controlVac, HIGH); // begin to inflate
    
    digitalWrite(ledTest, HIGH); // turn light on!
  }
    
  else if (toggle_msg == 0){
    digitalWrite(controlInf, HIGH);
    digitalWrite(controlVac, LOW); // begin to vacuum

    digitalWrite(ledTest, LOW); // turn light off
  }
}

// Function that stops inflation/vacuuming
void stop_all(){
  digitalWrite(controlInf, HIGH);
  digitalWrite(controlVac, HIGH); // stop both!

  digitalWrite(ledTest, LOW); // turn light off
  }

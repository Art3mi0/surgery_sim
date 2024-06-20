#include <ros/ros.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <iostream>
#include <surgery_sim/PedalEvent.h>


// This node uses the PedalEvent msg type for publishing when a pedal is held. Each pedal has its own variable;
// has a 0 when not pressed and has a 1 when pressed.
// This was created for a savant elite2 foot pedal.
// I do not remember what I went through to get the values I am using. The goal was to have the pedals not programmed
// to a keyboard key, or 1 key that would have zero impact, but I was unable to read from the event files a 
// unique identifier for each pedal press. I may have overlooked something. As a result, I bound the keys shift, ctrl, and alt
// because these seemed the least troubling. I found the key unique value through using a linux command on the event7
// file and using a value that popped up when pressed.
// Requires doing, sudo chmod 777 /dev/input/event7
// if the loop_freq is too low, it will not update correctly if a pedal is held

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pedal_controller");
  ros::NodeHandle node;

  ros::Publisher pub_pedal= node.advertise<surgery_sim::PedalEvent>( "/pedal", 1 );

  struct input_event ev;
  struct input_event input_test;
  surgery_sim::PedalEvent pedal;

  int fd = open("/dev/input/event7", O_RDONLY | O_NONBLOCK);
  
// 458977 left; 458976 middle; 458978 right
// shift, ctrl, alt

  int count = 0;
  int loop_freq = 100;
  ros::Rate loop_rate(loop_freq);
  while (node.ok()){
      read(fd, &ev, sizeof(ev));
      if (ev.value == 458977){
        count ++;
        if (count == 1){
          pedal.left_pedal = 1;
        } else{
          pedal.left_pedal = 0;
          count = 0;
        }
      } else if (ev.value == 458976){
        count ++;
        if (count == 1){
          pedal.middle_pedal = 1;
        } else{
          pedal.middle_pedal = 0;
          count = 0;
        }
      } if (ev.value == 458978){
        count ++;
        if (count == 1){
          pedal.right_pedal = 1;
        } else{
          pedal.right_pedal = 0;
          count = 0;
        }
      }

      pub_pedal.publish(pedal);

      loop_rate.sleep();
      ros::spinOnce();
  }
  close(fd);
  return 0;
};
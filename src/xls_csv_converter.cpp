#include <ros/ros.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <iostream>
#include <surgery_sim/PedalEvent.h>


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "xls_csv_converter");
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
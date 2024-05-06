#include <ros/ros.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <iostream>
#include <surgery_sim/PedalEvent.h>
int test;
int old;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pedal_controller");
  ros::NodeHandle node;

  ros::Publisher pub_point= node.advertise<surgery_sim::PedalEvent>( "/pedal", 1 );

  struct input_event ev;
  struct input_event input_test;
  std::cout<< "About to open\n"<< std::endl;
  int fd = open("/dev/input/event7", O_RDONLY | O_NONBLOCK);
  std::cout<< "Successfully opened\n"<< std::endl;
  
// 458977 left; 458976 middle; 458978 right

  int count = 0;
  int loop_freq = 10;
  ros::Rate loop_rate(loop_freq);
  while (node.ok()){
      read(fd, &ev, sizeof(ev));
      if (ev.value == 458977){
        count ++;
        if (count == 1){
          printf("Left Pedal Pressed!\n");
        } else{
          count = 0;
        }
      } else if (ev.value == 458976){
        count ++;
        if (count == 1){
          printf("Middle Pedal Pressed!\n");
        } else{
          count = 0;
        }
      } if (ev.value == 458978){
        count ++;
        if (count == 1){
          printf("Right Pedal Pressed!\n");
        } else{
          count = 0;
        }
      }
      // test = read(fd, &ev, sizeof(ev));
      // test = ev.value;
      // if (test != old){
      //   printf("New Event!!!");
      //   old = test;
      // }
      // printf ("Event sec: %lu\n", ev.time.tv_sec);
      // printf ("Event usec: %lu\n", ev.time.tv_usec);
      // printf("Event type: %d\n", ev.type);
      // printf("Event code: %d\n", ev.code);
      // printf("Event value: %d\n\n", ev.value);
      // printf("Event value: %s\n", "***************");
      // printf("%d\n", test);
      // sleep(1);
      loop_rate.sleep();
      ros::spinOnce();
  }
  close(fd);
  return 0;
};
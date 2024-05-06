#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <iostream>
int test;
int old;

int main(int argc, char* argv[])
{
    struct input_event ev;
    struct input_event input_test;
    std::cout<< "About to open\n"<< std::endl;
    int fd = open("/dev/input/event7", O_RDONLY | O_NONBLOCK);
    std::cout<< "Successfully opened\n"<< std::endl;
    while(1)
    {
        test = read(fd, &ev, sizeof(ev));
        // test = ev.value;
        // if (test != old){
        //   printf("New Event!!!");
        //   old = test;
        // }
        printf ("Event sec: %lu\n", ev.time.tv_sec);
        printf ("Event usec: %lu\n", ev.time.tv_usec);
        printf("Event type: %d\n", ev.type);
        printf("Event code: %d\n", ev.code);
        printf("Event value: %d\n", ev.value);
        printf("Event value: %s\n", "***************");
        printf("%d\n", test);
        sleep(1);
    }
    close(fd);
    return 0;
}
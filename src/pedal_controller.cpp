#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
int test;
int old;

int main()
{
    struct input_event ev;
    int fd = open("/dev/input/event7", O_RDONLY | O_NONBLOCK);
    while(1)
    {
        read(fd, &ev, sizeof(ev));
        test = ev.value;
        if (test != old){
          printf("New Event!!!");
          old = test;
        }
        printf ("Event sec: %lu\n", ev.time.tv_sec);
        printf ("Event usec: %lu\n", ev.time.tv_usec);
        printf("Event type: %d\n", ev.type);
        printf("Event code: %d\n", ev.code);
        printf("Event value: %d\n", ev.value);
        printf("Event value: %s\n", "***************");
    }
    close(fd);
}
#include "arduino_serial.h"

int Setup() {
  int ret = RS232_OpenComport(cport_nr, bdrate, mode);
  if(ret)
  {
    printf("Can not open comport\n");
    return(-1);
  }
  printf("return=/n",ret);
  usleep(2000000);  /* waits 2000ms for stable condition */
  return(1);
}

void Write(const char* mot_vel) {
    RS232_cputs(cport_nr, mot_vel); // sends string on serial
    printf("Sent to Arduino: '%s'\n", mot_vel);
}

const char* Read() {
    int n = RS232_PollComport(cport_nr, str_recv, (int)BUF_SIZE);
    if(n > 0){
        str_recv[n] = 0;   /* always put a "null" at the end of a string! */
    }
    return str_recv;
}

int main() {
  int n = Setup();
  while(1)
  {
    const char* data = "-99,99,,";
    Write(data);
    usleep(10000);  
    const char* res = Read();
    printf("Received %i bytes: '%s'\n", n, (char *)res);
  }
  return(0);
}

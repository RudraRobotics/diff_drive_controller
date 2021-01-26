extern "C" {
  #include "arduino_serial.h"
}
int main() {
  int n = Setup();
  while(1)
  {
    Write();
    usleep(10000);  
    Read();
  }
  return(0);
}


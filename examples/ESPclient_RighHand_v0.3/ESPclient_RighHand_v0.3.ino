//Left hand code - touch interactions - magnetic sensing and communiaction with Head mounted Device
/* GOALS:
   - Make every Serial call compiled based off a defined symbol
   - "readings" sections of the code are apparently for debugging and should be elimenated
*/

/* NOTES:
   - Using the millis function to time function calls means this code is limited to running
     a max of 49.71 days before needing to be reset to avoid an overflow error
   - "Udp.parsePacket()" slows TimerOne, timing issues for communication dependent on TimerOne
     or other peripherals may occur because of call this function.  The solution is to disable
     the TimerOne interrupt before the function call and re-enabling TimerOne interrupts afterwards.
   - Select/Deselect (15)
   - Object Orientation (14)
   - Pinch Calibration (13)
   - Presses are registered on the edge from NOT touching the button to TOUCHING the button
*/

#include <WiFi.h>
#include <WiFiUdp.h>
WiFiUDP Udp;  // Creation of wifi Udp instance
char packetBuffer[255];
unsigned int localPort = 9999;
const char *ssid = "NETGEAR36-5G"; //seems like personal information
const char *password = "pastelstreet317"; //seems like personal information
IPAddress ipServidor (192, 168, 4, 1);   // Declaration of default IP for server
/*
    The ip address of the client has to be different to the server
    other wise it will conflict because the client tries to connect
    to it self.
*/
IPAddress ipCliente (192, 168, 4, 10);   // Different IP than server
//IPAddress ipCliente3(192, 168, 1, 8);  //UNITY client ip
IPAddress Subnet(255, 255, 255, 0);
const int set_point = 20; //example value based on several online tutorials
int command = 0; //variable that contains the step
char counter[255];
int T9_init = 0;  //3 - Select/Deselect
int T8_init = 0;  //2 - Orientation
int T7_init = 0;  //1 - Pinch

// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Debug Input/Flag Trigger
char input;
bool touch_flag[2][3] = {{false, false, false}, {false, false, false}}; //flags for touch pins, edge detection

//delay timers -
unsigned long touch_timer;
int touch_timer_length = 125; //may cause issues with mismatched data types
int readings_timer_offset = 5; //may cause issues with mismatched data types
bool readings_flag = false; //this is done to avoid any case were the loop takes longer than a millisecond to complete

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  WiFi.mode(WIFI_STA); // ESP-32 as client
  WiFi.config(ipCliente, ipServidor, Subnet);
  Udp.begin(localPort);
  // capacitive touch
  Serial.println("ESP32 Touch Test");
  //initial reading of the sensors
  Serial.print("Initial Values  ");
  T7_init = touchRead(T7);
  T8_init = touchRead(T8);
  T9_init = touchRead(T9);
  // -=-=-=-=-=-=-=-=-
  delay(600);
  //Timer Initializations
  touch_timer = millis();
}

void loop() {
  if (touch_timer <= (millis() - touch_timer_length)) { //checks the touch pin every 120 milliseconds (touch_timer_length)
    if ((T7_init - touchRead(T7) ) > set_point) {
      touch_flag[0][0] = true;   //1 - Pinch
    }
    if ((T8_init - touchRead(T8) ) > set_point) {
      touch_flag[0][1] = true;     //2 - Orientation
    }
    if ((T9_init - touchRead(T9) ) > set_point) {
      touch_flag[0][2] = true;     //3 - Select/Deselect
    }
    if (touch_flag[0][0] != touch_flag[1][0]) {
      if (touch_flag[0][0]) {
        command = 13; //1 - Pinch
      }else{
        command = 113; //1 - Pinch
      }
    }
    if (touch_flag[0][1] != touch_flag[1][1]) {
      if (touch_flag[0][1]) {
        command = 14; //2 - Orientation
      }else{
        command = 114; //2 - Orientation
      }
    }
    if (touch_flag[0][2] != touch_flag[1][2]) {
      if (touch_flag[0][2]) {
        command = 15; //3 - Select/Deselect
      }else{
        command = 115; //3 - Select/Deselect
      }
    }
    for (int x = 0; x < 3; x++) {
      touch_flag[1][x] = touch_flag[0][x];
    }
    touch_timer = millis();
    readings_flag = false;
  }
  //---------------------------------------------------------
  if ((touch_timer <= (millis() - readings_timer_offset)) && (!readings_flag)) {
    getReadings();
    readings_flag = true;
  }
}//end of the Loop

//______________________________________________________________________________________________________________ SEND READINGS
void sendReadings(int testID ) {
  Udp.beginPacket(ipServidor, 9999);  //Initiate transmission of data
  char buf[20];   // buffer to hold the string to append
  sprintf(buf, "%lu", (long unsigned int) testID);  // appending the millis to create a char
  Udp.printf(buf);  // print the char
  //sending words
  Udp.printf("\r\n");   // End segment
  Udp.endPacket();  // Close communication
  Serial.print("enviando: ");   // Serial monitor for user
  Serial.println(buf);
}

//______________________________________________________________________________________________________________ GET READINGS
void getReadings() {
  int packetSize = Udp.parsePacket();   // Size of packet to receive
  if (packetSize) {       // If we received a package
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len - 1] = 0;
    Serial.print("RECIBIDO(IP/Port/Size/Datos11111): ");
    Serial.print(Udp.remoteIP()); Serial.print(" / ");
    Serial.print(Udp.remotePort()); Serial.print(" / ");
    Serial.print(packetSize); Serial.print(" / ");
    Serial.println(packetBuffer);
  }
  Serial.println("");
}

//______________________________________________________________________________________________________________ THETA
float theta(float z, float mag) {
  return ((acos(z / mag)) / 3.14159265); // radians/pi
}

//______________________________________________________________________________________________________________ PHI
float phi(float x, float y) {
  float output = 0;
  if (y == 0) {
    if (x > 0) {
      output = 0;
    } else if (x < 0) {
      output = 1;
    } else {
      output = 0; //if both x and y are 0
    }
  } else if (x == 0) {
    if (y > 0) {
      output = 0.5;
    } else if (y < 0) {
      output = 1.5;
    }
  } else {
    output = (atan(y / x) / 3.14159265);
  }
  return (output);
}

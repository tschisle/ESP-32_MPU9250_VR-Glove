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
   - Presses are registered on the edge (from NOT touching the button to TOUCHING the button)
*/

#include <WiFi.h>
#include <WiFiUdp.h>
WiFiUDP Udp;  // Creation of wifi Udp instance
unsigned int localPort = 9999;
const char *ssid = "Glove";
const char *password = "planningisunderrated";
IPAddress Left_IP (192, 168, 4, 1);   // Declaration of default IP for server
IPAddress Right_IP (192, 168, 4, 10);   // Different IP than server
IPAddress Subnet(255, 255, 255, 0);
const int set_point = 40; //example value based on several online tutorials
int command = 0; //variable that contains the step
int T9_average = 0;  //3 - Select/Deselect
int T8_average = 0;  //2 - Orientation
int T7_average = 0;  //1 - Pinch
const int mtravg = 5;
int mtrollingloc = 0;
int mt_rolling_average[3][mtravg]; //rolling average to smooth pinch gesture (causes some delay so finding a happy medium is necessary) NOTE: rolling average before PLSF is exactly equivalent to a rolling average after PLSF


// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int touchstepper = 0; //alternates reading from each pin to clean up readings

//Debug Input/Flag Trigger
char input;
bool touch_flag[2][3] = {{false, false, false}, {false, false, false}}; //flags for touch pins, edge detection

//delay timers -
unsigned long touch_timer;
int touch_timer_length = 40; //may cause issues with mismatched data types

void setup() {
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  WiFi.mode(WIFI_STA); // ESP-32 as client
  WiFi.config(Right_IP, Left_IP, Subnet);
  Udp.begin(localPort);
  // capacitive touch
  Serial.println("ESP32 Touch Test");
  //initial reading of the sensors
  Serial.print("Initial Values  ");
  T7_average = touchRead(T7);
  T8_average = touchRead(T8);
  T9_average = touchRead(T9);
  for (int x = 0; x < mtravg; x++) {
    mt_rolling_average[0][x] = T7_average;
    mt_rolling_average[1][x] = T8_average;
    mt_rolling_average[2][x] = T9_average;
  }
  // -=-=-=-=-=-=-=-=-
  delay(600);
  //Timer Initializations
  touch_timer = millis();
  digitalWrite(2, LOW);
}

void loop() {
  if (touch_timer <= (millis() - touch_timer_length)) { //checks the touch pin every 120 milliseconds (touch_timer_length)
    mt_rolling_average[0][mtrollingloc] = touchRead(T7);
    mt_rolling_average[1][mtrollingloc] = touchRead(T8);
    mt_rolling_average[2][mtrollingloc] = touchRead(T9);
    T7_average = 0;
    T8_average = 0;
    T9_average = 0;
    for (int x = 0; x < mtravg; x++) {
      T7_average = mt_rolling_average[0][x] + T7_average;
      T8_average = mt_rolling_average[1][x] + T8_average;
      T9_average = mt_rolling_average[2][x] + T9_average;
    }
    T7_average = T7_average / mtravg;
    T8_average = T8_average / mtravg;
    T9_average = T9_average / mtravg;
    mtrollingloc = (mtrollingloc + 1) % mtravg;
    if (touchstepper == 0) {
      if (T7_average < set_point) {
        touch_flag[0][0] = true;   //1 - Pinch
        digitalWrite(2, HIGH);
      } else {
        touch_flag[0][0] = false;   //1 - Pinch
      }
    }
    if (touchstepper == 1) {
      if (T8_average < set_point) {
        touch_flag[0][1] = true;     //2 - Orientation
        digitalWrite(2, HIGH);
      } else {
        touch_flag[0][1] = false;   //1 - Pinch
      }
    }
    if (touchstepper == 2) {
      if (T8_average < set_point) {
        touch_flag[0][2] = true;     //3 - Select/Deselect
        digitalWrite(2, HIGH);
      } else {
        touch_flag[0][2] = false;   //1 - Pinch
      }
      touchstepper = -1;
    }
    touchstepper = touchstepper + 1;
    if (touch_flag[0][0] != touch_flag[1][0]) {
      if (touch_flag[0][0]) {
        command = 13; //1 - Pinch
        sendReadings(command);
        sendReadings(command); //duplicated to ensure delivery
      } else {
        command = 63; //1 - Pinch
        sendReadings(command);
        sendReadings(command); //duplicated to ensure delivery
        digitalWrite(2, LOW);
      }
    }
    if (touch_flag[0][1] != touch_flag[1][1]) {
      if (touch_flag[0][1]) {
        command = 14; //2 - Orientation
        sendReadings(command);
        sendReadings(command); //duplicated to ensure delivery
      } else {
        command = 64; //2 - Orientation
        sendReadings(command);
        sendReadings(command); //duplicated to ensure delivery
        digitalWrite(2, LOW);
      }
    }
    if (touch_flag[0][2] != touch_flag[1][2]) {
      if (touch_flag[0][2]) {
        command = 15; //3 - Select/Deselect
        sendReadings(command);
        sendReadings(command); //duplicated to ensure delivery
      } else {
        command = 65; //3 - Select/Deselect
        sendReadings(command);
        sendReadings(command); //duplicated to ensure delivery
        digitalWrite(2, LOW);
      }
    }
    for (int x = 0; x < 3; x++) {
      touch_flag[1][x] = touch_flag[0][x];
    }
    touch_timer = millis();
  }
}//end of the Loop

//______________________________________________________________________________________________________________ SEND READINGS
void sendReadings(int testID ) {
  Udp.beginPacket(Left_IP, 9999);  //Initiate transmission of data
  Udp.write((byte)testID);
  Udp.endPacket();  // Close communication
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

//Left hand glove control - Server mode
//touch input and magnetic sensing
//touch input always available and magnetic sensing only available after calibration received from *right hand*.

/* GOALS:
   - Make every Serial call compiled based off a defined symbol
   ~ Replace touch multiple tolerances with an autocalibrated value
*/

/* NOTES:
   - THIS HAND DOES HANDLE SENSOR DATA (seemingly optimized for left-handed users?)
   - Using the millis function to time function calls means this code is limited to running 
     a max of 49.71 days before needing to be reset to avoid an overflow error
*/

// Just test touch pin - Touch0 is T0 which is on GPIO 4
/* Press, hold, slide - trans number
   1  - press T7 - B - Right
   2  - press T6 - A - Bottom
   3  - press T8 - Y - Top
   4  - press T9 - X - Left
   5  - swipe from A to B
   6  - swipe from B to A
   7  - swipe from X to Y
   8  - swipe from Y to X
   9  - hold B
   10 - hold A
   11 - hold Y
   12 - hold X
*/
#include <WiFi.h>
#include <WiFiUdp.h>
#include "MPU9250_ESP32.h"
#include <Wire.h>
#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0

// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=- 

//Debug Input/Flag Trigger
char input;

//Timer
unsigned long pinch_tilt_time = 0;
int pinch_tilt_update = 25;

//Magnetometer Variables
float position = 1;
char sinput;
float magavg[3] = {0, 0, 0};
int counter = 0;
const int avgsam = 3; //how many samples to average
const int samples = 10; //how many averaged sample clusters used to find the slope
float least_square_mat[4][samples]; //used to find the slope of the averaged sample clusters   1: sami - samavg 2: magi-magavg 3: 1*2 4: 1*1
float least_square_avg; //used to find the slope of the averaged sample clusters
float least_square_sum_comp[2]; //used to find the slope of the averaged sample clusters
float least_square_slope_inter[2]; //holds slope and intercept values to approximate current value
float approximation; //holds final approximated value
float percentage; //using variable to allow for data validation
float magmag; //holds the magnitude value
float sammat[samples]; //holds the averaged samples in a matrix
float manmagbias[3]; //temporary magnetometer bias to zero each value to simplify gesture analysis
float magmin; //temporarily holds the min reference for the magnetometer
int prevarr[3] = {0, 0, 0};
int samcoun = 0;
float sammin[3] = {0, 0, 0};
float sammax[3] = {0, 0, 0};
const float tolerance = 60; //

//Accelerometer Direction
float acc_theta_cal = 0;
float acc_theta = 0;
float acc_mag = 0;
float acc_phi_cal = 0;
float acc_phi = 0;

//Flags / Conditions
bool tilt_cal = true; //Trigger for calibrating orientation (automatically turned off after calibration)
const float tilt_tolerance = 0.25; //Radians/pi, currently checking delta randians from calibration vector
bool tilt = false; //Flag for reporting when hand is tilted outside of tolerance
bool pinch_max_cal = false; //Trigger for calibrating max pinch locaiton (automatically turned off after calibration)
bool pinch_min_cal = false; //Trigger for calibrating min pinch location (automatically turned off after calibration)
bool pinch_gesture = false; //Trigger for pinch gesture (automatically turned on after min calibration, REQUIRES EXTERNAL STOPPING CONDITION)
bool prev_pinch_max_cal = false; // Edge detection of flag for initializing variables & counters
bool prev_pinch_min_cal = false; // Edge detection of flag for initializing variables & counters

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

//delay timers -
unsigned long touch_timer;
int touch_timer_length = 125; //may cause issues with mismatched data types
int readings_timer_offset = 5; //may cause issues with mismatched data types
bool readings_flag = false; //this is done to avoid any case were the loop takes longer than a millisecond to complete

//_____________________________________________________________________________________ TOUCH PAD OBJECT

class Touchpad
{
    unsigned long tpmat[3][2][2]; //layer, row, column
    /*  touch |top, right|   press |top, right|    time    |top, right|
        flags |left, bott|,  order |left, bott|,   pressed |left, bott|
        Y - Top; B - Right; A - Bottom; X - Left
    */
    const unsigned long inter_button_slide_time = 250; //number of milliseconds between button presses to distinguish a slide from quick button presses
    const unsigned long button_hold_time = 500; //number of milliseconds between distinguishing a press from a hold
    int slide_timer = 0; //counts down time between presses
    int hold_timer = 0; //counts down time holding a press
    const int set_point_h = 45; //value based from trial and error
    const int set_point_l = 10; //value based from trial and error
    int T9_init = 0;  //4 - Y - Top
    int T8_init = 0;  //3 - A - Bottom
    int T7_init = 0;  //1 - B - Right
    int T6_init = 0;  //2 - X - Left
    bool touched = false;//gets set when a touch is detected
    int num_touches = 0; //gets the number of touches within the gesture period
    unsigned long temp_touch_time_ref = 0;//serves as the reference time for the gesture recognition.
    bool slide_detected = false;
    bool hold_detected = false;
    bool press_detected = false;
    bool hold_mask[2][2] = {{false, false}, {false, false}}; //prevents presses from being reported after a hold
    int press_value; //allows the tpmat to be fully cleared
  public:
    void initialize() {
      int i, j, k;
      delay(300); //will likely fix the initial reading issue
      T9_init = touchRead(T9); //this first reading is wrong - taken again at the end of the readings
      T8_init = touchRead(T8); //3 - A - Bottom
      T7_init = touchRead(T7); //1 - B - Right
      T6_init = touchRead(T6); //2 - X - Left
      T9_init = touchRead(T9); //4 - Y - Top
      for (i = 0; i < 3; i++) {
        for (j = 0; j < 2; j++) {
          for (k = 0; k < 2; k++) {
            tpmat[i][j][k] = 0;
          }
        }
      }
      num_touches = 0;
      temp_touch_time_ref = 0;
    }

    int update() {
      int i, j, k;
      slide_detected = false;
      hold_detected = false;
      press_detected = false;
      matrixUpdate();  //also performs touch reads
      /*NOTES
         Since there isn't a command for pressing multiple buttons at once the code assumes this won't happen.
         This can be added in the slide detection.

         Additionally the detection code is written to allow gesture changes with continuous contact to the touchpad

         Possible issue: The hold mask may not be completely robust as it's used now.
      */
      //Slide Detection -- Reports the last slid-over button
      if (num_touches > 1) {
        if (temp_touch_time_ref != 0) {
          press_value = 0;
          for (j = 0; j < 2; j++) {
            for (k = 0; k < 2; k++) {
              if ((num_touches == tpmat[1][j][k]) && (tpmat[1][(j + 1) % 2][(k + 1) % 2] > 0)) {
                //Organized to remove else's
                slide_detected = true;
                press_value = 2; //2 - X - Left
                if (j == 0) {
                  press_value = 1; //1 - B - Right
                }
                if (j == k) {
                  press_value = 4; //4 - Y - Top
                  if (j == 0) {
                    press_value = 3; //3 - A - Bottom
                  }
                }
              }
            }
          }
        }
      }
      //Hold Detection -- Reports the longest held button
      if (hold_timer == 0) {
        press_value = 0;
        for (j = 0; j < 2; j++) {
          for (k = 0; k < 2; k++) {
            if (tpmat[2][j][k] > (button_hold_time - 1)) {
              //Organized to remove else's
              hold_detected = true;
              hold_mask[j][k] = true;
              press_value = 2; //2 - X - Left
              if (j == 0) {
                press_value = 1; //1 - B - Right
              }
              if (j == k) {
                press_value = 4; //4 - Y - Top
                if (j == 0) {
                  press_value = 3; //3 - A - Bottom
                }
              }
            }
          }
        }
      }
      //Press Detection  -- Reports the first pressed button
      if (!touched && (slide_timer == 0)) {
        press_value = 0;
        for (j = 0; j < 2; j++) {
          for (k = 0; k < 2; k++) {
            if ((tpmat[1][j][k] == 1) && (!hold_mask[j][k])) {
              //Organized to remove else's
              if (!hold_mask[j][k]) {
                hold_mask[j][k] = false;
              } else {
                press_detected = true;
                press_value = 2; //2 - X - Left
                if (j == 0) {
                  press_value = 1; //1 - B - Right
                }
                if (j == k) {
                  press_value = 4; //4 - Y - Top
                  if (j == 0) {
                    press_value = 3; //3 - A - Bottom
                  }
                }
              }
            }
          }
        }
      }
      //End Gesture
      if (slide_detected || hold_detected || press_detected) {
        num_touches = 0;
        temp_touch_time_ref = 0;
        for (j = 0; j < 2; j++) {
          for (k = 0; k < 2; k++) {
            tpmat[1][j][k] = 0;
            tpmat[2][j][k] = 0;
          }
        }
        if (slide_detected) {
          return (press_value + 4);
        } else if (hold_detected) {
          return (press_value + 8);
        } else if (press_detected) {
          return (press_value);
        } else {
          return (0);
        }
      }
      return(0);
    }

  private:
    void matrixUpdate(void)
    {
      tpmat[0][0][0] = T8_init - touchRead(T8); //3 - A - Bottom
      tpmat[0][0][1] = T7_init - touchRead(T7); //1 - B - Right
      tpmat[0][1][0] = T6_init - touchRead(T6); //2 - X - Left
      tpmat[0][1][1] = T9_init - touchRead(T9); //4 - Y - Top
      int j, k;
      touched = false;
      if (slide_timer > 0) {
        slide_timer--;
      }
      for (j = 0; j < 2; j++) {
        for (k = 0; k < 2; k++) {
          if (tpmat[0][j][k] > ((set_point_h + set_point_l) / 2)) {
            tpmat[0][j][k] = 1; //boolean TRUE
            touched = true;
            num_touches++;
            slide_timer = (int) inter_button_slide_time;
            if (hold_timer > 0) {
              hold_timer--;
            }
            tpmat[1][j][k] = num_touches; //Touch Order
            if (temp_touch_time_ref == 0) {
              temp_touch_time_ref = millis();
            } else {
              tpmat[2][j][k] = millis() - temp_touch_time_ref; //time held relative to the start of the gesture
            }
          } else {
            tpmat[0][j][k] = 0; //boolean FALSE
          }
        }
      }
      if (!touched) {
        hold_timer = (int) button_hold_time;
      }
    }
};

//WiFi configuration
WiFiUDP Udp; // Creation of wifi Udp instance
Touchpad touch; //Creates touchpad object
char packetBuffer[255];
const int port = 8888;
//const char *ssid =  "NETGEAR36-5G";// "NETGEAR36-5G";
//const char *password = "pastelstreet317";
const char *ssid =  "Apt5";// "NETGEAR36-5G";
const char *password = "123srysry1234";
IPAddress ipCliente (192, 168, 4, 10);   // right hand glove IP address to send messages directly
//IPAddress ipCliente3(192, 168, 4, 12);   // right hand glove IP address to send messages directly
//IPAddress ipCliente3(192, 168, 1, 8);  //UNITY client ip
IPAddress ipCliente3(10, 0, 0, 143);  //UNITY client ip
const char * udpAddress = "10.0.0.43";
//int testID = 0;
byte packet[1];
//Touch configuration
int command = 0;
int prevcommand; //exists since command serves two purposes and holds commands from both left and right gloves
int aux_command = 0;
int touch_reading;

//_____________________________________________________________________________________  SETUP

void setup() {
  //wifi setup
  Serial.begin(115200);
  WiFi.softAP(ssid, password);  // ESP-32 as access point
  Udp.begin(port);
  //touch sensing setup
  Serial.println("ESP32 Touch Test");
  touch.initialize();
  // -=-=-=-=-=-=-=-=-
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  //Magnetometer set up
  Wire.begin();
  delay(1500);
  digitalWrite(13, LOW);
  myIMU.MPU9250SelfTest(myIMU.selfTest);
  // Calibrate gyro and accelerometers, load biases in bias registers
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
  myIMU.initMPU9250();
  // Get magnetometer calibration from AK8963 ROM
  myIMU.initAK8963(myIMU.factoryMagCalibration);
  myIMU.getAres();
  myIMU.getGres();
  myIMU.getMres();
  Serial.println("Started");
  //Precalculating static least square values HAS REDUNDANT ELEMENTS
  for (int x = 0; x < samples; x++) {
    least_square_mat[0][x] = x - ((samples + 1) / 2);
    least_square_mat[3][x] = least_square_mat[0][x] * least_square_mat[0][x];
  }
  least_square_sum_comp[1] = 0;
  for (int x = 0; x < samples; x++) {
    least_square_sum_comp[1] = least_square_sum_comp[1] + least_square_mat[3][x];
  }
  //Timer set up
  pinch_tilt_time = millis() + pinch_tilt_update;
  delay(500); // delay to check the values in the serial monitor
}



//______________________________________________________________________________________________________________ LOOP
void loop() {
  command = touch.update();
  if ((touch_timer <= (millis() - readings_timer_offset)) && (!readings_flag)) {
    prevcommand = command;
    command = getReadings();
    readings_flag = true;
    if(command != -1){ 
      sendReadings(command); //sends a messace every cycle to the client 192, 168, 4, 10
      sendReadingsUnity(command); //sends a messace every cycle to the client 192, 168, 4, 12
    }
    if(prevcommand != 0){
      sendReadings(prevcommand); //sends a messace every cycle to the client 192, 168, 4, 10
      sendReadingsUnity(prevcommand); //sends a messace every cycle to the client 192, 168, 4, 12
    }
  }
  if (command == '1') {
    tilt_cal = true;
    Serial.println("tilt_cal");
  } else if (command == '2') {
    pinch_max_cal = true;
    Serial.println("pinch_max_cal");
  } else if (command == '3') {
    pinch_min_cal = true;
    Serial.println("pinch_min_cal");
  } else if (command == '4') {
    if (!pinch_gesture) {
      pinch_gesture = true;
      Serial.println("pinch_gesture ON");
    } else {
      pinch_gesture = false;
      Serial.println("pinch_gesture OFF");
    }
  }
  if (millis() >= pinch_tilt_time) {
    pinch_tilt_time = millis() + pinch_tilt_update;
    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {
      myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
      // Calculate the accleration value into actual g's, this depends on scale being set
      myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
      myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
      myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];
      myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
      // Calculate the gyro value into actual degrees per second, this depends on scale being set
      myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
      myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
      myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;
      myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
      // Calculate the magnetometer values in milliGauss
      // Include factory calibration per data sheet and user environmental corrections
      // Get actual magnetometer value, this depends on scale being set
      myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
                 * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
      myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
                 * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
      myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
                 * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
    }

    acc_mag = sqrt((myIMU.ax * myIMU.ax) + (myIMU.ay * myIMU.ay) + (myIMU.az * myIMU.az));
    acc_theta = theta(myIMU.az, acc_mag); // radians/pi
    acc_phi = phi(myIMU.ay, myIMU.ax); // radians/pi
    if (tilt_cal) {
      acc_theta_cal = acc_theta;
      acc_phi_cal = acc_phi;
      Serial.println("Tilt calibration complete");
      tilt_cal = false;
    }
    if (abs(acc_theta_cal - acc_theta) > tilt_tolerance) { // you can add an additional condition to fine tine the tilt condition from a cone to cone rotated around an axis
      tilt = true;
      digitalWrite(13, HIGH);
      //Serial.println("+");
    } else {
      tilt = false;
      digitalWrite(13, LOW);
    }
    if (pinch_max_cal || pinch_min_cal || pinch_gesture) { //only reads from magnetometer and performs math if prompted by flags
      if ((prev_pinch_max_cal != pinch_max_cal) || (prev_pinch_min_cal != pinch_min_cal)) { //edge condition - Initialization
        magavg[0] = 0;
        magavg[1] = 0;
        magavg[2] = 0;
        counter = 0;
        do {
          magavg[0] = myIMU.mx + magavg[0];
          magavg[1] = myIMU.my + magavg[1];
          magavg[2] = myIMU.mz + magavg[2];
        } while (counter++ < (avgsam - 1));
        if ((prev_pinch_max_cal != pinch_max_cal)) {//edge condition of first calibration - bias setting
          manmagbias[0] = magavg[0] / avgsam;
          manmagbias[1] = magavg[1] / avgsam;
          manmagbias[2] = magavg[2] / avgsam;
          Serial.println("Pinch MAX calibration complete");
          pinch_max_cal = false;
        } else { //reference setting
          magmin = sqrt(pow(magavg[0] / avgsam, 2) + pow(magavg[1] / avgsam, 2) + pow(magavg[2] / avgsam, 2));
          Serial.println("Pinch MIN calibration complete");
          pinch_min_cal = false;
        }
        magavg[0] = 0;
        magavg[1] = 0;
        magavg[2] = 0;
        counter = 0;
        samcoun = 0;
        pinch_max_cal = false; //clears flag
        pinch_min_cal = false; //clears flag
        prev_pinch_max_cal = pinch_max_cal; //for edge detection
        prev_pinch_min_cal = pinch_min_cal; //for edge detection
      } else {
        myIMU.mx = myIMU.mx - manmagbias[0];
        myIMU.my = myIMU.my - manmagbias[1];
        myIMU.mz = myIMU.mz - manmagbias[2];
        magavg[0] = myIMU.mx + magavg[0];
        magavg[1] = myIMU.my + magavg[1];
        magavg[2] = myIMU.mz + magavg[2];
        if (counter++ == (avgsam - 1)) {
          magavg[0] = magavg[0] / avgsam;
          magavg[1] = magavg[1] / avgsam;
          magavg[2] = magavg[2] / avgsam;
          //converts from 3 coordinates to a magnitude.  This is possible since we're effectively moving the origin to the max position,
          //however we're now operating under the assumption that the curve from the finger movement can be adequately approximated with a line
          magmag = sqrt((magavg[0] * magavg[0]) + (magavg[1] * magavg[1]) + (magavg[2] * magavg[2]));
          //clearing previous values and shifting mag data
          for (int x = 0; x < samples; x++) {
            if (x < (samples - 1)) {
              sammat[x] = sammat[x + 1];
            }
            least_square_avg = 0;
            least_square_sum_comp[0] = 0;
          }
          sammat[samples - 1] = magmag;
          //finding average mag values
          for (int x = 0; x < samples; x++) {
            least_square_avg = least_square_avg + sammat[x];
          }
          least_square_avg = least_square_avg / samples;
          //calculating least square values
          for (int x = 0; x < samples; x++) {
            least_square_mat[1][x] = sammat[x] - least_square_avg;
            least_square_mat[2][x] = least_square_mat[0][x] * least_square_mat[1][x];
          }
          for (int x = 0; x < samples; x++) {
            least_square_sum_comp[0] = least_square_sum_comp[0] + least_square_mat[2][x];
          }
          least_square_slope_inter[0] = least_square_sum_comp[0] / least_square_sum_comp[1]; //slopes
          least_square_slope_inter[1] = least_square_avg - (least_square_slope_inter[0] * ((samples + 1) / 2)); //intercept
          //approximating current mag data from best fit LINE and reporting movement percentage
          //this step provides the unitless value
          approximation = (least_square_slope_inter[0] * samples) + least_square_slope_inter[1];
          percentage = (1 - (approximation / magmin)) * 100; //finds percentage from max to min, inverts the value, then multiples it by 100
          if (percentage < 0) {
            percentage = 0;
          } else if (percentage > 100) {
            percentage = 100;
          }
          percentage = map(percentage, 55, 85, 0, 100); //quick fix
          if (percentage < 0) {
            percentage = 0;
          } else if (percentage > 100) {
            percentage = 100;
          }
          Serial.print("percent = ");
          Serial.println((uint8_t)percentage);
          //-=-=-=-=-=-=-=-=-=-=-=-    OUTPUT
          sendReadings(percentage); //sends a messace every cycle to the client 192, 168, 4, 10
          sendReadingsUnity(percentage); //sends a messace every cycle to the client 192, 168, 4, 12
          //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
          //clearing averaging variables and counter
          magavg[0] = 0;
          magavg[1] = 0;
          magavg[2] = 0;
          counter = 0;
        } else if (counter > avgsam) { //Safety condition to ensure data intigrity under counter fault
          magavg[0] = 0;
          magavg[1] = 0;
          magavg[2] = 0;
          counter = 0;
        }
      }
    }
    prev_pinch_max_cal = pinch_max_cal; //for edge detection
    prev_pinch_min_cal = pinch_min_cal; //for edge detection
  }
}


void sendReadings(int testID) { //sends message to the right hand
  //Send Package to desired Ip
  Udp.beginPacket(ipCliente, 9999); //send package to the desired IP address
  Udp.printf("for right hand: ");
  char buf[20];   // buffer to hold the string to append
  sprintf(buf, "%lu", (long unsigned int) testID);  // appending the testID to create a char
  Udp.printf(buf);  // send the char
  Udp.printf("\r\n");   // End segment
  Udp.endPacket();
}

//______________________________________________________________________________________________________________ SEND READINGS - UNITY
void sendReadingsUnity(int testID) { //sends message to UNITY
  //Send Package to desired Ip
  packet[0] = testID;
  Udp.beginPacket(ipCliente3, port); //send package to the desired IP address
  Udp.write(packet,1);
  Udp.endPacket();
}

//______________________________________________________________________________________________________________ GET READINGS
int getReadings() {
  int packetSize = Udp.parsePacket();   // Size of packet to receive
  int len = -1; //from right hand
  if (packetSize) {       // If we received a package
    len = Udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len - 1] = 0;
    Serial.print("RECIBIDO(IP/Port/Size/Datos11111): ");
    Serial.print(Udp.remoteIP()); Serial.print(" / ");
    Serial.print(Udp.remotePort()); Serial.print(" / ");
    Serial.print(packetSize); Serial.print(" / ");
    Serial.println(packetBuffer);
  }
  Serial.println("");
  return(len); //-1 when nothing is recieved
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
  return(output);
}

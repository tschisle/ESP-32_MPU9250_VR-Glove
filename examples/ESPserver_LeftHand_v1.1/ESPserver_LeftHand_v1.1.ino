//Left hand glove control - Server mode
//touch input and magnetic sensing
//touch input always available and magnetic sensing only available after calibration received from *right hand*.

/* GOALS:
   - Make every Serial call compiled based off a defined symbol
   ~ Replace touch multiple tolerances with an autocalibrated value
   - Store Accelerometer calibration in EEPROM
*/

/* NOTES:
   - THIS HAND HANDLEs SENSOR DATA (seemingly optimized for left-handed users?)
   - Using the millis function to time function calls means this code is limited to running
     a max of 49.71 days before needing to be reset to avoid an overflow error
   - First pinch calibration touch calibrates the accelerometer for the tilt detection
*/

// Just test touch pin - Touch0 is T0 which is on GPIO 4
/* Commands:
    1,    2,   3,      4,          5,             6,                7,               8,              9,                10,                    11,                       12,                                                               13,                            14,              15,               16,            17,      100+###,          51,         52,        53,           54,               55,                  56,                     57,                    58,                   59,                      60,                          61,                             62,                                                                     63,                                  64,                    65,                     66,                  67
  Right, Left, Top, Bottom, Top w/Tilt, Bottom w/Tilt, Right w/Rotation, Left w/Rotation, Top w/Rotation, Bottom w/Rotation, Top w/Tilt w/Rotation, Bottom w/Tilt w/Rotation, Pinch Calibration (1st - Max, 2nd - Min, 3rd - Start, 4th - End), Object Orientation / Rotation, Select/Deselect, Tilt Calibration, Tilt Detected, Pinch Values, End - Right, End - Left, End - Top, End - Bottom, End - Top w/Tilt, End - Bottom w/Tilt, End - Right w/Rotation, End - Left w/Rotation, End - Top w/Rotation, End - Bottom w/Rotation, End - Top w/Tilt w/Rotation, End - Bottom w/Tilt w/Rotation, End - Pinch Calibration (1st - Max, 2nd - Min, 3rd - Start, 4th - End), End - Object Orientation / Rotation, End - Select/Deselect, End - Tilt Calibration, End - Tilt Detected
*/
#include <WiFi.h>
#include <WiFiUdp.h>
#include "MPU9250_ESP32.h"
#include <Wire.h>
#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0
#define SERIAL 1 //toggles communication via USB port

// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Debug Input/Flag Trigger
char input;

//Commands
int right_cmd = 0;
int prev_right_cmd = 0;
int pinch_step = 0;

//Timer
unsigned long pinch_tilt_time = 0;
int pinch_tilt_update = 50;

//Magnetometer Variables
float position = 1;
char sinput;
float magavg[3] = {0, 0, 0};
int counter = 0;
const int avgsam = 3; //how many samples to average
float rolling_average[3][avgsam]; //rolling average to smooth pinch gesture (causes some delay so finding a happy medium is necessary) NOTE: rolling average before PLSF is exactly equivalent to a rolling average after PLSF
int rolling_average_counter = 0;//tracks the location of the rolling average
const int samples = 15; //how many averaged sample clusters used to find the slope
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
int pinch_initialization_buffer = 0; //stops values from being reported until the buffers are filled
float max_phi;
float min_phi;
float current_phi;

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
bool rota = false; //Flag for reporting when rotation commands are requested
bool pinch_flag = false; //for allowing positive edge trigger of pinch calibration steps
bool prev_pinch_flag = false; //for allowing positive edge trigger of pinch calibration steps
bool pinch_max_cal = false; //Trigger for calibrating max pinch locaiton (automatically turned off after calibration)
bool pinch_min_cal = false; //Trigger for calibrating min pinch location (automatically turned off after calibration)
bool pinch_gesture = false; //Trigger for pinch gesture (automatically turned on after min calibration, REQUIRES EXTERNAL STOPPING CONDITION)
bool prev_pinch_max_cal = false; // Edge detection of flag for initializing variables & counters
bool prev_pinch_min_cal = false; // Edge detection of flag for initializing variables & counters


MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);
void sendByteUnity(byte);


//delay timers -
unsigned long touch_timer;
int touch_timer_length = 40; //may cause issues with mismatched data types
bool readings_flag = false; //this is done to avoid any case were the loop takes longer than a millisecond to complete
const int mtravg = 5; //touch reading averages

//_____________________________________________________________________________________ TOUCH PAD OBJECT

class Touchpad
{
    unsigned long tpmat[2][2][2]; //layer, row, column
    /*  touch |top, right|    time    |top, right|
        flags |left, bott|,   pressed |left, bott|
    */
    unsigned long prevtpmat[2][2][2]; //layer, row, column
    /*  touch |top, right|    prev    |top, right|
        flags |left, bott|,  cmd sent |left, bott|
    */
    const int tpcommands[4][2][2] = {//layer, row, column
      {{3, 1}, {2, 4}}, //top, right, left, bottom
      {{5, 1}, {2, 6}},
      {{9, 7}, {8, 10}},
      {{11, 7}, {8, 12}}
    };
    /*  |top, right|
        |left, bott|
        0 - Normal; 1 - Tilt; 2 - Rotation; 3 - Rotation w/ Tilt
    */
    const unsigned long command_report_interval_time = 100; //number of milliseconds between each press command report
    const int set_point_h = 40; //value based from trial and error
    const int set_point_l = 40; //value based from trial and error
    int T4_average = 0;  //4 - Bottom
    int T8_average = 0;  //3 - Top
    int T7_average = 0;  //1 - Right
    int T6_average = 0;  //2 - Left
    int mtrollingloc = 0;
    int mt_rolling_average[4][mtravg]; //rolling average to smooth pinch gesture (causes some delay so finding a happy medium is necessary) NOTE: rolling average before PLSF is exactly equivalent to a rolling average after PLSF

    unsigned long temp_touch_time_ref = 0;//serves as the reference time for the gesture recognition.
  public:
    void initialize() {
      int i, j, k;
      delay(300); //will likely fix the initial reading issue
      T8_average = touchRead(T8); //3 - Y - Top
      T7_average = touchRead(T7); //1 - B - Right
      T6_average = touchRead(T6); //2 - X - Left
      T4_average = touchRead(T4); //4 - A - Bottom
      for (int x = 0; x < mtravg; x++) {
        mt_rolling_average[0][x] = T6_average;
        mt_rolling_average[1][x] = T7_average;
        mt_rolling_average[2][x] = T8_average;
        mt_rolling_average[3][x] = T4_average;
      }
      for (j = 0; j < 2; j++) {
        for (k = 0; k < 2; k++) {
          tpmat[0][j][k] = 0;
          tpmat[1][j][k] = millis();
        }
      }
      temp_touch_time_ref = 0;
    }

    int update() {
      int i, j, k;
      matrixUpdate();  //also performs touch reads
      /*NOTES
         Since multiple buttons can theoritically change states in the same loop cycle, this code instantly
         sends the command.  The update function still returns the command, but this was done as a legacy,
         debug feature and not intented for use.
      */
      i = 0;
      for (j = 0; j < 2; j++) {
        for (k = 0; k < 2; k++) {
          if (tpmat[0][j][k] == 1) {
            if ((millis() - tpmat[1][j][k]) > 99) {
              tpmat[1][j][k] = millis();
              if (!tilt && !rota) {
                if ((prevtpmat[1][j][k] > 0) && (prevtpmat[1][j][k] != tpcommands[0][j][k])) { //checks if tilt or rotation changed while presing a button
                  sendByteUnity(prevtpmat[1][j][k] + 50);
                  sendByteUnity(prevtpmat[1][j][k] + 50);
                }
                sendByteUnity(tpcommands[0][j][k]);
                i = tpcommands[0][j][k];
                prevtpmat[1][j][k] = tpcommands[0][j][k];
              } else if (tilt && !rota) {
                if ((prevtpmat[1][j][k] > 0) && (prevtpmat[1][j][k] != tpcommands[1][j][k])) { //checks if tilt or rotation changed while presing a button
                  sendByteUnity(prevtpmat[1][j][k] + 50);
                  sendByteUnity(prevtpmat[1][j][k] + 50);
                }
                sendByteUnity(tpcommands[1][j][k]);
                i = tpcommands[1][j][k];
                prevtpmat[1][j][k] = tpcommands[1][j][k];
              } else if (!tilt && rota) {
                if ((prevtpmat[1][j][k] > 0) && (prevtpmat[1][j][k] != tpcommands[2][j][k])) { //checks if tilt or rotation changed while presing a button
                  sendByteUnity(prevtpmat[1][j][k] + 50);
                  sendByteUnity(prevtpmat[1][j][k] + 50);
                }
                sendByteUnity(tpcommands[2][j][k]);
                i = tpcommands[2][j][k];
                prevtpmat[1][j][k] = tpcommands[2][j][k];
              } else {
                if ((prevtpmat[1][j][k] > 0) && (prevtpmat[1][j][k] != tpcommands[3][j][k])) { //checks if tilt or rotation changed while presing a button
                  sendByteUnity(prevtpmat[1][j][k] + 50);
                  sendByteUnity(prevtpmat[1][j][k] + 50);
                }
                sendByteUnity(tpcommands[3][j][k]);
                i = tpcommands[3][j][k];
                prevtpmat[1][j][k] = tpcommands[3][j][k];
              }
            }
          }
        }
      }
      //End Gesture
      for (j = 0; j < 2; j++) {
        for (k = 0; k < 2; k++) {
          if ((prevtpmat[0][j][k] > 0) && (prevtpmat[0][j][k] != tpmat[0][j][k])) {
            prevtpmat[0][j][k] = 0; //redunant
            sendByteUnity(prevtpmat[1][j][k] + 50);
            sendByteUnity(prevtpmat[1][j][k] + 50);
            i = tpcommands[3][j][k] + 100;
          }
        }
      }
      return (0);
    }

  private:
    void matrixUpdate(void)
    {
      int j, k;
      for (j = 0; j < 2; j++) {
        for (k = 0; k < 2; k++) {
          prevtpmat[0][j][k] = tpmat[0][j][k];
        }
      }
      mt_rolling_average[0][mtrollingloc] = touchRead(T6);
      mt_rolling_average[1][mtrollingloc] = touchRead(T7);
      mt_rolling_average[2][mtrollingloc] = touchRead(T8);
      mt_rolling_average[3][mtrollingloc] = touchRead(T4);
      T6_average = 0;
      T7_average = 0;
      T8_average = 0;
      T4_average = 0;
      for (int x = 0; x < mtravg; x++) {
        mt_rolling_average[0][x] = T6_average;
        mt_rolling_average[1][x] = T7_average;
        mt_rolling_average[2][x] = T8_average;
        mt_rolling_average[3][x] = T4_average;
      }
      T6_average = T6_average / mtravg;
      T7_average = T7_average / mtravg;
      T8_average = T8_average / mtravg;
      T4_average = T4_average / mtravg;
      mtrollingloc = (mtrollingloc + 1) % mtravg;
      tpmat[0][0][0] = T8_average; //3 - Top
      tpmat[0][0][1] = T7_average; //1 - Right
      tpmat[0][1][0] = T6_average; //2 - Left
      tpmat[0][1][1] = T4_average; //4 - Bottom
      for (j = 0; j < 2; j++) {
        for (k = 0; k < 2; k++) {
          if (tpmat[0][j][k] < ((set_point_h + set_point_l) / 2)) {
            tpmat[0][j][k] = 1; //boolean TRUE
          } else {
            tpmat[0][j][k] = 0; //boolean FALSE
          }
        }
      }
    }
};

//WiFi configuration
WiFiUDP Udp; // Creation of wifi Udp instance
Touchpad touch; //Creates touchpad object
byte packetBuffer[255];
int bufloc = 0;
const int connect_attempts = 4; //each attempt takes 0.5 sec
const int port = 11000;
const char *ssid =  "ZyXEL8C1BF8";// "NETGEAR36-5G";
const char *password = "FFVTWR3NKNP77";
IPAddress Left_IP (192, 168, 4, 1);   // Declaration of default IP for server
IPAddress Right_IP (192, 168, 4, 10);   // Different IP than server
IPAddress Unity_IP(192, 168, 1, 111);  //UNITY client ip
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional
//Touch configuration
int command = 0;
int prevcommand; //exists since command serves two purposes and holds commands from both left and right gloves
int aux_command = 0;
int touch_reading;

//_____________________________________________________________________________________  SETUP

void setup() {
  //wifi setup
  if (SERIAL) {
    Serial.begin(115200);
    //touch sensing setup
    Serial.println("ESP32 Touch Test");
  }
  touch.initialize();
  // -=-=-=-=-=-=-=-=-
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  // WIFI Setup -=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if (!WiFi.config(Left_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    for (int x = 0; x < 500; x ++) {
      if (x < 100) {
        digitalWrite(2, HIGH);
      } else if (x < 200) {
        digitalWrite(2, LOW);
      } else if (x < 300) {
        digitalWrite(2, HIGH);
      } else if (x < 400) {
        digitalWrite(2, LOW);
      } else {
        digitalWrite(2, HIGH);
      }
      delay(1);
    }
  }
  WiFi.begin(ssid, password);
  Udp.begin(port);
  int connect_attempts_counter = 0;
  while ((WiFi.status() != WL_CONNECTED) && (connect_attempts_counter++ < connect_attempts)) {
    //LED Animation
    for (int x = 0; x < 500; x ++) {
      if (x < 100) {
        if (x % 2) {
          digitalWrite(2, HIGH);
        } else {
          digitalWrite(2, LOW);
        }
      } else if (x < 200) {
        if (x % 6) {
          digitalWrite(2, LOW);
        } else {
          digitalWrite(2, HIGH);
        }
      } else if (x < 300) {
        if (x % 12) {
          digitalWrite(2, LOW);
        } else {
          digitalWrite(2, HIGH);
        }
      } else if (x < 400) {
        if (x % 6) {
          digitalWrite(2, LOW);
        } else {
          digitalWrite(2, HIGH);
        }
      } else {
        if (x % 2) {
          digitalWrite(2, HIGH);
        } else {
          digitalWrite(2, LOW);
        }
      }
      if (SERIAL) {
        if (x == 100) {
          Serial.print("Connecting");
        } else if (x == 200) {
          Serial.print(" .");
        } else if (x == 300) {
          Serial.print(" .");
        } else if (x == 400) {
          Serial.println(" .");
        }
      }
      delay(1);
    }
  }

  //Magnetometer set up -=-=-=-=-=-=-=-=-=-=-=-
  Wire.begin();
  delay(1500);
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
  digitalWrite(2, LOW);
}



//______________________________________________________________________________________________________________ LOOP
void loop() {
  if (right_cmd > 0) {
    prev_right_cmd = right_cmd;
  }
  right_cmd = getByte();
  if (SERIAL) {
    if (Serial.read() == '3') {
      right_cmd = 13;
    }
  }
  if ((touch_timer <= (millis() - touch_timer_length))) { //updates touchpad every 120ms
    command = touch.update();
    touch_timer = millis();
  }
  // Select/Deselect
  if (right_cmd == 15) {
    sendByteUnity(15);
  }
  if (right_cmd == 65) {
    sendByteUnity(65);
  }
  // Rotation
  if (right_cmd == 14) {
    rota = true;
  }
  if (right_cmd == 64) {
    rota = false;
  }
  //Pinch
  prev_pinch_flag = pinch_flag;
  if (right_cmd == 13) {
    pinch_flag = true;
  }
  if (right_cmd == 63) {
    pinch_flag = false;
  }
  if (!prev_pinch_flag && pinch_flag) {
    pinch_step++;
    if (pinch_step == 6) {
      pinch_step = 2;
    }
    if (pinch_step == 1) {
      tilt_cal = true;
      pinch_flag = false;
      if (SERIAL) {
        Serial.println("tilt_cal");
      }
    } else if (pinch_step == 2) {
      pinch_max_cal = true;
      pinch_flag = false;
      if (SERIAL) {
        Serial.println("pinch_max_cal");
      }
      counter = 0;
    } else if (pinch_step == 3) {
      pinch_min_cal = true;
      pinch_flag = false;
      if (SERIAL) {
        Serial.println("pinch_min_cal");
      }
      counter = 0;
    } else if (pinch_step == 4) {
      if (!pinch_gesture) {
        pinch_gesture = true;
        pinch_flag = false;
        if (SERIAL) {
          Serial.println("pinch_gesture ON");
        }
      }
    } else {
      pinch_gesture = false;
      pinch_flag = false;
      if (SERIAL) {
        Serial.println("pinch_gesture OFF");
      }
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

      acc_mag = sqrt((myIMU.ax * myIMU.ax) + (myIMU.ay * myIMU.ay) + (myIMU.az * myIMU.az));
      acc_theta = theta(myIMU.az, acc_mag); // radians/pi
      acc_phi = phi(myIMU.ay, myIMU.ax); // radians/pi
      if (tilt_cal) {
        acc_theta_cal = acc_theta;
        acc_phi_cal = acc_phi;
        if (SERIAL) {
          Serial.println("Tilt calibration complete");
        }
        tilt_cal = false;
      }
      if (abs(acc_theta_cal - acc_theta) > tilt_tolerance) { // you can add an additional condition to fine tine the tilt condition from a cone to cone rotated around an axis
        tilt = true;
        if (!pinch_max_cal && !pinch_min_cal && !pinch_gesture) {
          digitalWrite(2, HIGH);
        }
        //Serial.println("+");
      } else {
        tilt = false;
        if (!pinch_max_cal && !pinch_min_cal && !pinch_gesture) {
          digitalWrite(2, LOW);
        }
      }
      rolling_average[0][rolling_average_counter] = myIMU.mx;
      rolling_average[1][rolling_average_counter] = myIMU.my;
      rolling_average[2][rolling_average_counter] = myIMU.mz;
      rolling_average_counter = rolling_average_counter + 1;
      if (rolling_average_counter == avgsam) {
        rolling_average_counter = 0;
      }
      magavg[0] = 0;
      magavg[1] = 0;
      magavg[2] = 0;
      for (int x = 0; x < avgsam; x++) {
        magavg[0] = rolling_average[0][x];
        magavg[1] = rolling_average[1][x];
        magavg[2] = rolling_average[2][x];
      }
      magavg[0] = magavg[0] / avgsam;
      magavg[1] = magavg[1] / avgsam;
      magavg[2] = magavg[2] / avgsam;

      if (pinch_max_cal || pinch_min_cal || pinch_gesture) { //only reads from magnetometer and performs math if prompted by flags
        if ((prev_pinch_max_cal != pinch_max_cal) || (prev_pinch_min_cal != pinch_min_cal)) { //edge condition - Initialization
          digitalWrite(2, HIGH);
        }
        if (pinch_max_cal || pinch_min_cal) {
          if (pinch_max_cal) {//edge condition of first calibration - bias setting
            manmagbias[0] = magavg[0];
            manmagbias[1] = magavg[1];
            manmagbias[2] = magavg[2];
            max_phi = phi(manmagbias[1], manmagbias[2]);
            if (SERIAL) {
              Serial.print("max phi = ");
              Serial.print(max_phi);
            }
            pinch_max_cal = false;
            digitalWrite(2, LOW);
          } else { //reference setting
            min_phi = phi(magavg[1], magavg[2]);
            if (SERIAL) {
              Serial.print("min phi = ");
              Serial.print(min_phi);
            }
            pinch_min_cal = false;
            digitalWrite(2, LOW);
          }
          prev_pinch_max_cal = pinch_max_cal; //for edge detection
          prev_pinch_min_cal = pinch_min_cal; //for edge detection
          digitalWrite(2, LOW);
          pinch_initialization_buffer = 0;
        } else {
          digitalWrite(2, HIGH);
          //converts from 3 coordinates to a magnitude.  This is possible since we're effectively moving the origin to the max position,
          //however we're now operating under the assumption that the curve from the finger movement can be adequately approximated with a line
          magmag = sqrt((magavg[0] * magavg[0]) + (magavg[1] * magavg[1]) + (magavg[2] * magavg[2]));
          current_phi = phi(magavg[1], magavg[2]);
          if (SERIAL) {
            Serial.print("phi = ");
            Serial.print(current_phi);
          }
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
          percentage = map(current_phi * 100, min_phi * 100, max_phi * 100, 0, 100);
          /*
            if (percentage < 0) {
            percentage = 0;
            } else if (percentage > 100) {
            percentage = 100;
            }
            //percentage = map(percentage, 55, 90, 0, 100); //quick fix
            if (percentage < 0) {
            percentage = 0;
            } else if (percentage > 100) {
            percentage = 100;
            }
          */
          if (pinch_initialization_buffer >= (avgsam + samples - 1)) {
            if (SERIAL) {
              Serial.print("percent = ");
              Serial.println(percentage);//(uint8_t)
            }
            //-=-=-=-=-=-=-=-=-=-=-=-    OUTPUT
            sendByteUnity(percentage + 100); //sends a messace every cycle to the client 192, 168, 4, 12
            //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
          } else {
            pinch_initialization_buffer = pinch_initialization_buffer + 1;
          }
          digitalWrite(2, LOW);
        }
      }
      prev_pinch_max_cal = pinch_max_cal; //for edge detection
      prev_pinch_min_cal = pinch_min_cal; //for edge detection
    }
  }
}

//______________________________________________________________________________________________________________ SEND BYTE - UNITY
void sendByteUnity(byte testID) { //sends message to UNITY
  Udp.beginPacket(Unity_IP, port); //send package to the desired IP address
  Udp.write(testID);
  Udp.endPacket();
}

//______________________________________________________________________________________________________________ SEND BUFFER - UNITY NEEDS FIXED
void sendReadingsUnity(int testID) { //sends message to UNITY
  //Send Package to desired Ip
  Udp.beginPacket(Unity_IP, port); //send package to the desired IP address
  Udp.write((byte)testID);
  Udp.endPacket();
}

//______________________________________________________________________________________________________________ GET READINGS
int getByte() {
  int packetSize; //number of bytes received
  int len = -1; //from right hand
  packetSize = Udp.parsePacket();   // Size of packet to receive
  if (packetSize) {
    len = Udp.read();
    Udp.flush();
  }
  return (len); //-1 when nothing is recieved
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

#include <MPU9250_ESP32.h>
#include <PredictiveLeastSquaresFilter.h>
#include <Wire.h>
#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0
#define SAMPLES 10  //how many sample are sent to the filter

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

void setup() {
  Serial.begin(9600);
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
}

void loop() {
  input = Serial.read();
  if (input == '1') {
    tilt_cal = true;
    Serial.println("tilt_cal");
  } else if (input == '2') {
    pinch_max_cal = true;
    Serial.println("pinch_max_cal");
  } else if (input == '3') {
    pinch_min_cal = true;
    Serial.println("pinch_min_cal");
  } else if (input == '4') {
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

float theta(float z, float mag) {
  return ((acos(z / mag)) / 3.14159265); // radians/pi
}

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

//Left hand glove control - Server mode
//touch input and magnetic sensing
//touch input always available and magnetic sensing only available after calibration received from right hand.

// Just test touch pin - Touch0 is T0 which is on GPIO 4
/* Commands:
    1,    2,   3,      4,          5,             6,                7,               8,              9,                10,                    11,                       12,                                                               13,                            14,              15,               16,            17,         1###,         101,        102,       103,          104,              105,                 106,                    107,                   108,                  109,                     110,                         111,                            112,                                                                    113,                                 114,                   115,                    116,                 117
  Right, Left, Top, Bottom, Top w/Tilt, Bottom w/Tilt, Right w/Rotation, Left w/Rotation, Top w/Rotation, Bottom w/Rotation, Top w/Tilt w/Rotation, Bottom w/Tilt w/Rotation, Pinch Calibration (1st - Max, 2nd - Min, 3rd - Start, 4th - End), Object Orientation / Rotation, Select/Deselect, Tilt Calibration, Tilt Detected, Pinch Values, End - Right, End - Left, End - Top, End - Bottom, End - Top w/Tilt, End - Bottom w/Tilt, End - Right w/Rotation, End - Left w/Rotation, End - Top w/Rotation, End - Bottom w/Rotation, End - Top w/Tilt w/Rotation, End - Bottom w/Tilt w/Rotation, End - Pinch Calibration (1st - Max, 2nd - Min, 3rd - Start, 4th - End), End - Object Orientation / Rotation, End - Select/Deselect, End - Tilt Calibration, End - Tilt Detected
*/
//_____________________________________________________________________________________


//_____________________________________________________________________________________
//Touch configuration
int command = 0;
int aux_command = 0;
int set_point = 45;
int T6_average = 0;
int T5_average = 0;
int T7_average = 0;
int touch_limit = 40;
const int mtravg = 5;
int mtrollingloc = 0;
int mt_rolling_average[3][mtravg]; //rolling average to smooth pinch gesture (causes some delay so finding a happy medium is necessary) NOTE: rolling average before PLSF is exactly equivalent to a rolling average after PLSF


//_____________________________________________________________________________________

void setup()
{
  //wifi setup
  Serial.begin(115200);
  delay(500); // delay to chack the values in the serial monitot
}



//______________________________________________________________________________________________________________
void loop()
{
  mt_rolling_average[0][mtrollingloc] = touchRead(T7);
  mt_rolling_average[1][mtrollingloc] = touchRead(T5);
  mt_rolling_average[2][mtrollingloc] = touchRead(T6);
  T7_average = 0;
  T5_average = 0;
  T6_average = 0;
  for (int x = 0; x < mtravg; x++) {
    T7_average = mt_rolling_average[0][x] + T7_average;
    T5_average = mt_rolling_average[1][x] + T5_average;
    T6_average = mt_rolling_average[2][x] + T6_average;
  }
  T7_average = T7_average / mtravg;
  T5_average = T5_average / mtravg;
  T6_average = T6_average / mtravg;
  mtrollingloc = (mtrollingloc + 1)%mtravg;
  for (int x = 1; x < 50; x++) {
    if (T7_average < (2 * x)) {
      Serial.print("7");
      T7_average = 1000;
    } else if (T5_average < (2 * x)) {
      Serial.print("5");
      T5_average = 1000;
    } else if (T6_average < (2 * x)) {
      Serial.print("6");
      T6_average = 1000;
    } else {
      Serial.print(" ");
    }
  }
  if (T7_average < (1000)) {
    Serial.print("7");
    T7_average = 1000;
  }
  if (T5_average < (1000)) {
    Serial.print("5");
    T7_average = 1000;
  }
  if (T6_average < (1000)) {
    Serial.print("6");
    T7_average = 1000;
  }
  Serial.println("");
  /*
    if (touchRead(T7) < touch_limit) {
      Serial.println("7");
    }
    if (touchRead(T5) < touch_limit) {
      Serial.println("8");
    }
    if (touchRead(T6) < touch_limit) {
      Serial.println("9");
    }*/
  delay(50);
}

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
int T9_init = 0;
int T8_init = 0;
int T7_init = 0;
int T6_init = 0;


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
  if(touchRead(T7)<70){
    Serial.println(">");
  }
  delay(50);
}

//Left hand glove control - Server mode
//touch input and magnetic sensing
//touch input always available and magnetic sensing only available after calibration received from right hand.

// Just test touch pin - Touch0 is T0 which is on GPIO 4
/* Commands:
    1,    2,   3,      4,          5,             6,                7,               8,              9,                10,                    11,                       12,                                                               13,                            14,              15,               16,            17,         1###,         101,        102,       103,          104,              105,                 106,                    107,                   108,                  109,                     110,                         111,                            112,                                                                    113,                                 114,                   115,                    116,                 117
  Right, Left, Top, Bottom, Top w/Tilt, Bottom w/Tilt, Right w/Rotation, Left w/Rotation, Top w/Rotation, Bottom w/Rotation, Top w/Tilt w/Rotation, Bottom w/Tilt w/Rotation, Pinch Calibration (1st - Max, 2nd - Min, 3rd - Start, 4th - End), Object Orientation / Rotation, Select/Deselect, Tilt Calibration, Tilt Detected, Pinch Values, End - Right, End - Left, End - Top, End - Bottom, End - Top w/Tilt, End - Bottom w/Tilt, End - Right w/Rotation, End - Left w/Rotation, End - Top w/Rotation, End - Bottom w/Rotation, End - Top w/Tilt w/Rotation, End - Bottom w/Tilt w/Rotation, End - Pinch Calibration (1st - Max, 2nd - Min, 3rd - Start, 4th - End), End - Object Orientation / Rotation, End - Select/Deselect, End - Tilt Calibration, End - Tilt Detected
*/
#include <WiFi.h>
#include <WiFiUdp.h>
#define NOWIFI 1
#define NOSERIAL 0
//_____________________________________________________________________________________

//WiFi configuration
WiFiUDP Udp; // Creation of wifi Udp instance

char packetBuffer[255];

const int port = 8888;

bool seedtouchflag = true;

const char *ssid =  "Apt5";// "NETGEAR36-5G";
const char *password = "123srysry1234";
unsigned long touch_timer;
int touch_timer_length = 120;

IPAddress ipCliente(192, 168, 4, 10);   // right hand glove IP address to send messages directly
//IPAddress ipCliente3(192, 168, 4, 12);   // right hand glove IP address to send messages directly

//IPAddress ipCliente3(192, 168, 1, 8);  //UNITY client ip
IPAddress ipCliente3(10, 0, 0, 143);  //UNITY client ip
const char * udpAddress = "10.0.0.43";

//int testID = 0;
byte packet[1];
//_____________________________________________________________________________________
//Touch configuration
int command = 0;
int aux_command = 0;
int set_point = 45;
int T9_init = 0;
int T8_init = 0;
int T7_init = 0;
int T6_init = 0;
bool touch_flag[2] = {false, false};
int randcommand = 0;
bool pinch_dir = false;
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


//_____________________________________________________________________________________

void setup()
{
  //wifi setup
  Serial.begin(115200);

  if (NOWIFI == 0) {
    WiFi.begin(ssid, password);  // ESP-32 as access point
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.println("Connecting to WiFi..");
    }

    Serial.println("Connected to the WiFi network");
    Udp.begin(port);
  }
  touch_timer = millis();
  delay(500); // delay to chack the values in the serial monitot
}



//______________________________________________________________________________________________________________
void loop()
{
  touch_flag[0] = touch_flag[1];
  if (touchRead(T7) < 40) {
    if(seedtouchflag){
      randomSeed(millis());
      seedtouchflag = false;
    }
    touch_flag[1] = true;
    if (touch_flag[0] != touch_flag[1]) {
      if (rand() % 1000 < 20) {
        randcommand = (rand() % 100) + 100;
      } else {
        randcommand = tpcommands[rand() % 4][rand() % 2][rand() % 2];
      }
    }
    if (randcommand < 100) {
      if (touch_timer <= (millis() - 120)) {
        if (NOSERIAL == 0) {
          Serial.println((byte)randcommand);
        }
        sendReadings(randcommand); //sends a messace every cycle to the client 192, 168, 4, 10
        sendbyteUnity(randcommand); //sends a messace every cycle to the client 192, 168, 4, 12
      }
    } else {
      if (touch_timer <= (millis() - 25)) {
        if (NOSERIAL == 0) {
          Serial.println((byte)randcommand);
        }
        sendReadings(randcommand); //sends a messace every cycle to the client 192, 168, 4, 10
        sendbyteUnity(randcommand); //sends a messace every cycle to the client 192, 168, 4, 12
        if ((randcommand == 200)) {
          pinch_dir = true;
        } else if ((randcommand == 100)) {
          pinch_dir = false;
        } else if ((rand() % 100 < 2)) {
          pinch_dir = !pinch_dir;
        }
        if (!pinch_dir) {
          randcommand = randcommand + 1;
        } else {
          randcommand = randcommand - 1;
        }
      }
    }
  } else {
    touch_flag[1] = false;
    if (touch_flag[0] != touch_flag[1]) {
      touch_timer = millis() - 120;
      if (randcommand < 100) {
        if (NOSERIAL == 0) {
          Serial.println((byte)(randcommand + 50));
        }
        sendReadings(randcommand + 50); //sends a messace every cycle to the client 192, 168, 4, 10
        sendbyteUnity(randcommand + 50); //sends a messace every cycle to the client 192, 168, 4, 12
        if (NOSERIAL == 0) {
          Serial.println((byte)(randcommand + 50));
        }
        sendReadings(randcommand + 50); //sends a messace every cycle to the client 192, 168, 4, 10
        sendbyteUnity(randcommand + 50); //sends a messace every cycle to the client 192, 168, 4, 12
      }
    }
  }
}
//end loop

//-----------------------------------------------------------------
void sendReadings(int testID) //sends message to the right hand
{
  if (NOWIFI == 0) {
    //Send Package to desired Ip
    Udp.beginPacket(ipCliente, 9999); //send package to the desired IP address

    Udp.printf("for right hand: ");
    char buf[20];   // buffer to hold the string to append
    //unsigned long testID = 2015;
    sprintf(buf, "%lu", testID);  // appending the testID to create a char
    Udp.printf(buf);  // send the char
    //sending words
    Udp.printf("\r\n");   // End segment

    Udp.endPacket();
  }
}

//______________________________________________________________________________________________________________ SEND BYTE - UNITY
void sendbyteUnity(byte testID) { //sends message to UNITY
  if (NOWIFI == 0) {
    Udp.beginPacket(ipCliente3, port); //send package to the desired IP address
    Udp.write(testID);
    Udp.endPacket();
  }
}

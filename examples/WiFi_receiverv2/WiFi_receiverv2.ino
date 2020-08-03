#include <WiFi.h>
#include <WiFiUdp.h>

//WiFi configuration
WiFiUDP Udp; // Creation of wifi Udp instance
byte packetBuffer[255];
int bufloc = 0;
const int port = 9999;
const char *ssid =  "Glove";// "NETGEAR36-5G";
const char *password = "planningisunderrated";
IPAddress Right_IP (192, 168, 4, 10);   // right hand glove IP address to send messages directly
IPAddress Unity_IP(10, 0, 0, 143);  //UNITY client ip
//Touch configuration
int command = 0;
int prevcommand; //exists since command serves two purposes and holds commands from both left and right gloves
int aux_command = 0;
int touch_reading;
unsigned long LED_timer;
int LED_on_time = 200;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFi.softAP(ssid, password);  // ESP-32 as access point
  Udp.begin(port);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  delay(1500);
  digitalWrite(2, LOW);

}

void loop() {
  // put your main code here, to run repeatedly:
  command = getReadings();
  if (command > 0) {
    digitalWrite(2, HIGH);
    LED_timer = millis() + 200;
    Serial.println(command);
  }
  if (LED_timer == millis()) {
    digitalWrite(2, LOW);
  }
}


//______________________________________________________________________________________________________________ GET READINGS
int getReadings() {
  int packetSize; //number of bytes received
  int len = -1; //from right hand
  packetSize = Udp.parsePacket();   // Size of packet to receive
  if (packetSize) {
    len = Udp.read();
    Udp.flush();
  }
  return (len); //-1 when nothing is recieved
}

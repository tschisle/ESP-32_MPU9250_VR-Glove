#include <WiFi.h>
#include <WiFiUdp.h>
WiFiUDP Udp;  // Creation of wifi Udp instance
unsigned int localPort = 9999;
const char *ssid = "Glove"; //seems like personal information
const char *password = "planningisunderrated"; //seems like personal information
IPAddress Left_IP (192, 168, 4, 1);   // Declaration of default IP for server
IPAddress Right_IP (192, 168, 4, 10);   // Different IP than server
IPAddress Subnet(255, 255, 255, 0);
const int set_point = 20; //example value based on several online tutorials
int command = 0; //variable that contains the step

void setup() {
  // put your setup code here, to run once:
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  WiFi.begin(ssid, password);
  WiFi.mode(WIFI_STA); // ESP-32 as client
  //WiFi.config(ipCliente, ipServidor, Subnet);
  Udp.begin(localPort);
}

void loop() {
  // put your main code here, to run repeatedly:
  sendReadings(1);
  delay(200);
  digitalWrite(2, HIGH);
  delay(800);
  digitalWrite(2, LOW);
}

//______________________________________________________________________________________________________________ SEND READINGS
void sendReadings(int testID ) {
  Udp.beginPacket(Left_IP, 9999);  //Initiate transmission of data
  Udp.write((byte)testID); 
  Udp.endPacket();  // Close communication
}

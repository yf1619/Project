#include<WiFi.h>



//Everytime run the code, do not forget to push the button//
//Wifi Settings
const char* ssid = "YIWEI_Laptop";        //WiFi Name
const char* password = "1234567890";               //WiFi Password
//Safe pc is host, my pc opeing the APP function works as client.
char* host = "146.169.174.193";             //URL or IP address
long port = 8000;                         //Port to be defined

WiFiClient client;


void setup() {
  Serial.begin(115200); //communicate with PC via usb
  //Serial1.begin(115200, SERIAL_8N1, RXV, TXV);  //communicate with Vision
  //Serial2.begin(9600, SERIAL_8N1, RXD, TXD);  //communicate with Drive via RX0, and TX1
  
  //Wait for my pc connected to wifi,Wifi connection
  WiFi.begin(ssid, password);                 //connect to WiFi
  while (WiFi.status() != WL_CONNECTED){
     delay(500);//Why 500
     Serial.print(".");  
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

//Wifi connect to the host
  delay(500);
  if(client.connect(host, port)){       //connect(ip/url, port)
    Serial.println("Handshake successful");
  }else{
    Serial.println("Handshake Failed"); 
  }
}

void loop() {
 
}

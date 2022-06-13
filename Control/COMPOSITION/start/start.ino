#include<WiFi.h>

//Below is Server-Related variables
//In real situation, my pc providing the hotpot, my pc send the message to the server,
//My pc is client, and becuase server is in safe pc, his pc is host.
const char* ssid = "YIWEI_Laptop";
const char* password = "1234567890";

char* host = "146.169.167.224";             //URL or IP address
long port = 8000;                         //Port to be defined


WiFiClient client;
//Below is radar

const int Radar_Output_Pin = 34;//I should define a ADC1 GPIO. 34 is An ADC1 GPIO//
int ROValue = 0;//I pretend that RO output is a digital value



void setup() {
  
  
  analogRead(GPIO);//https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/ define which GPIO we should use;ADC2 pins cannot be used when Wi-Fi is used.//
  Serial.begin(115200); //communicate with PC via usb
  //Serial1.begin(115200, SERIAL_8N1, RXV, TXV);  //communicate with Vision
  //Serial2.begin(9600, SERIAL_8N1, RXD, TXD);  //communicate with Drive via RX0, and TX1
  
  
  //Serial 1 server connection part
  Serial.begin(115200); 
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid,password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  delay(500);
  //Check whether connection established
  if(client.connect(host, port)){       //connect(ip/url, port)
    Serial.println("Handshake successful");
  }else{
    Serial.println("Handshake Failed"); 
  }
}

void loop() {
  // put your main code here, to run repeatedly:
   // Reading Radar value
  ROValue = analogRead(Radar_Output_Pin);
  Serial.println(ROValue);
  delay(500);
}

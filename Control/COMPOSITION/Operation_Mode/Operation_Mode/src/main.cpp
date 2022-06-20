#include<WiFi.h>
#include<Arduino.h>

//Everytime run the code, do not forget to push the button//
//In this code, there are basic functions to connect the with the server
//Wifi Settings .
const char* ssid = "YIWEI_Laptop";        //WiFi Name
const char* password = "1234567890";               //WiFi Password
//Safe pc is host, my pc opeing the APP function works as client.
char* host = "146.169.174.193";             //URL or IP address
long port = 8000;                         //Port to be defined

WiFiClient client;

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
// By default, driver_mode is 0, when receive message like 1,2,3,4 it is used for different mode
int driver_mode = 0;



// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

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

//Below is the part to build an easy server in the esp32 to act as host, and webpage act as a client.
WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
   
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}

void loop() {
 
}

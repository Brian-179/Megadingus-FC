#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
//#include <SoftwareSerial.h>
#include <SPISlave.h>
#include <SPI.h>
#include "Arduino.h"
#include "HTML_Content.h"

const char* ssid = "FC2.1";
const char* password = "pppppppp";

String hostname = "FC2-1"; //const char*

//const int tx = 12;
//const int rx = 13;

//SoftwareSerial RP2040 (rx,tx);

WiFiServer server(80);

String userInput;
String message;

void setup() {
  //pinMode(rx, INPUT);
  //pinMode(tx, OUTPUT);
  //RP2040.begin(38400);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  SPISlave.begin();
  // Connect to Wi-Fi network
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ssid, password);
  WiFi.setOutputPower(20.5);

  WiFi.hostname(hostname);
  if (!MDNS.begin(hostname)) {
    Serial.println("Error setting up mDNS");
  }

  server.begin();

  Serial.println();
  Serial.print("Access Point Hostname: ");
  Serial.println(hostname);
  Serial.print("Access Point IP address: ");
  Serial.println(WiFi.softAPIP());

}

void loop() {

  if (Serial.available()) {
    userInput = Serial.readString();
    userInput.trim();
  }

  WiFiClient client = server.available();

  if (client) {
    digitalWrite(LED_BUILTIN, LOW);
    String request = client.readStringUntil('\r');
    client.readStringUntil('\n');
    handleRequest(client, request);
    client.stop();
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
  }

  //  if (userInput.length() > 0) {
  //    Serial.println(userInput);
  //    int userInputLen = userInput.length() + 1;
  //    char userInputArray[userInputLen];
  //    userInput.toCharArray(userInputArray, userInputLen);
  //    //SPISlave.setData(userInputArray);
  //    //userInput = "";
  //  }

  SPISlave.onData([](uint8_t *data, size_t len) {
    message = String((char *)data);
    //memcpy(&telem, data, sizeof(telem));
    (void)len;
    //Serial.println(message);
  });

  // The master has read out outgoing data buffer
  // that buffer can be set with SPISlave.setData
  SPISlave.onDataSent([]() {
    //Serial.println("Answer Sent");
    //userInput = "";
    SPISlave.setData(""); //prevents data from being sent more than once
  });
  int messageLen = message.length() + 1;
  char messageArray[messageLen];
  message.toCharArray(messageArray, messageLen);

  //  if (userInput.length() > 0) {
  //    int userInputLen = userInput.length() + 1;
  //    char userInputArray[userInputLen];
  //    userInput.toCharArray(userInputArray, userInputLen);
  //    uint8_t messageArray2[messageLen];
  //    memcpy(messageArray2, messageArray, messageLen);
  //    SPISlave.setData(messageArray2,messageLen);
  //    Serial.println(messageArray);
  //    //userInput = "";
  //  }

  if (userInput.length() > 0) {
    int userInputLen = userInput.length();
    char userInputArray[userInputLen + 1];
    userInput.toCharArray(userInputArray, userInputLen + 1);
    Serial.println(userInputArray);
    SPISlave.setData(userInputArray);
    userInput = "";
  }
  //SPISlave.setData("reboot");
  //RP2040.println("test");
  //  if (RP2040.available()){
  //    Serial.write(RP2040.read());
  //  }
}

void handleRequest(WiFiClient client, const String& request) {
  userInput = "";

  //  // Check if it's a POST request to /process_input
  //  if (request.startsWith("POST /process_input")) {
  //    String line = client.readStringUntil('\n'); // Read the request line
  //    client.readStringUntil('\n'); // Skip the unnecessary header line
  //
  //    // Read the body
  //    String body = client.readString();
  //
  //    // Extract the user input from the body
  //    int userInputStartIndex = body.indexOf("user_input=");
  //    if (userInputStartIndex != -1) {
  //      int userInputEndIndex = body.indexOf('&', userInputStartIndex);
  //      if (userInputEndIndex == -1) {
  //        userInputEndIndex = body.length();
  //      }
  //      userInput = body.substring(userInputStartIndex + 11, userInputEndIndex);
  //      userInput.trim();
  //    }
  //  }


  // Parse the request to get the user input
  int paramIndex = request.indexOf("?user_input=");
  if (paramIndex != -1) {
    int endIndex = request.indexOf(" ", paramIndex);
    if (endIndex != -1) {
      userInput = request.substring(paramIndex + 12, endIndex);
    }
  }

  userInput.trim();

  if (userInput.length() > 0) {
    //Serial.print("User input: ");
    //Serial.println(userInput);
  } else {
    //Serial.println("No user input received");
  }

  if (!client.connected()) {
    return;
  }

  // Send the HTTP response header
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  //client.println("Refresh: 1\r\n"); //refresh every 1 second
  client.println();

  // Send the HTML content with the input form
  //    client.println("<html><body>");
  //    client.println("<h1>Enter your input:</h1>");
  //    client.println("<form method='get'>");
  //    client.println("Input: <input type='text' name='user_input'><br>");
  //    client.println("<input type='submit' value='Submit'>");
  //    client.println("</form>");
  //    client.println("</body></html>");

  client.print(htmlContent);

  if (request.indexOf("/getSerialData") != -1) {
    client.println(userInput);
  }
  if (request.indexOf("/servox") != -1) {
    userInput = "servox";
  }
  if (request.indexOf("/servoy") != -1) {
    userInput = "servoy";
  }
  if (request.indexOf("/parachuteServo180") != -1) {
    userInput = "parachuteServo180";
  }
  if (request.indexOf("/parachuteServo0") != -1) {
    userInput = "parachuteServo0";
  }
  if (request.indexOf("/pyro1") != -1) {
    userInput = "pyro1";
  }
  if (request.indexOf("/pyro2") != -1) {
    userInput = "pyro2";
  }
  if (request.indexOf("/pyro3") != -1) {
    userInput = "pyro3";
  }
  if (request.indexOf("/wipeFlash") != -1) {
    userInput = "wipeFlash";
  }
  if (request.indexOf("/reboot") != -1) {
    userInput = "reboot";
  }
  if (request.indexOf("/rebootESP") != -1) {
    ESP.restart();
  }
  if (request.indexOf("/countdown") != -1) {
    userInput = "countdown";
  }
  if (request.indexOf("/wipeSD") != -1) {
    userInput = "wipeSD";
  }

  delay(10);

  while (client.available()) {
    client.read();
  }
}

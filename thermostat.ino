/*
 author: Rustam Iskenderov
 
*/

#include <ESP8266WiFi.h>

// Enter your WiFi network SSID
const char* ssid = "";
// Enter your WiFi network password
const char* password = "";


/*
 * Go to settings of your router, add port forwarding to the PORT_NETWORK port
 * Find out your public IP, then you can connect to the webserver by simply going to http://my_public_ip:PORT_NETWORK
 */
const int PORT_NETWORK = 8150;

WiFiServer server(PORT_NETWORK);

void setup() {
  Serial.begin(9600);
  delay(10);

  // prepare GPIO2
  pinMode(2, OUTPUT);
  digitalWrite(2, 0);

  // Connect to WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // Start the server
  server.begin();

  // Print the IP address
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void ProcessCheckbox(const WiFiClient& cl)
{

}

void loop() {
  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    delay(1);
    return;
  }

  // Wait until the client sends some data
  Serial.println("new client");
  while (!client.available()) {
    delay(1);
  }


      String http_request = client.readStringUntil('\r');
      Serial.print(http_request);
      client.flush();

      //RESPONSE
      // send a standard http response header
      client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
      // send web page
      client.println("<!DOCTYPE html>");
      client.println("<html>");
      client.println("<head>");
      client.println("<title>Thermostat control</title>");
      client.println("</head>");
      client.println("<body>");
      client.println("<h1>LED</h1>");
      //client.println("<input type=button value=ON onmousedown=location.href='/on' />");

      client.print("<form>");
      client.print("<input name=state type=radio value=cool/> Cool<br>");
      client.print("<input name=state type=radio value=off checked/> Off<br>");
      client.print("<input name=state type=radio value=heat /> Heat<br>");
      client.print("<input name=temperature type=number value=74 /><br>");
      client.print("<input type=submit value=Apply /><br>");
      client.println("</form>");

      client.println("<p>Set temperature in C</p>");
      client.println("<p></p>");
      client.println("<p>Current temperature:</p>");
      client.println("<p>Current state: heating</p>");
      client.println("</body>");
      client.println("</html>");



  ///////////////////////
  return;

  // Read the first line of the request
  String req = client.readStringUntil('\r');
  Serial.println(req);
  client.flush();

  // Match the request
  int val;
  if (req.indexOf("/gpio/0") != -1)
    val = 0;
  else if (req.indexOf("/gpio/1") != -1)
    val = 1;
  else {
    Serial.println("invalid request");
    client.stop();
    return;
  }

  // Set GPIO2 according to the request
  digitalWrite(2, val);

  client.flush();

  // Prepare the response
  String s = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<!DOCTYPE HTML>\r\n<html>\r\nGPIO is now ";
  s += (val) ? "high" : "low";
  s += "</html>\n";

  // Send the response to the client
  client.print(s);
  delay(1);
  Serial.println("Client disonnected");

  // The client will actually be disconnected
  // when the function returns and 'client' object is detroyed
}






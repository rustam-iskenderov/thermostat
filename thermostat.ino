/*
  author: Rustam Iskenderov
  Control your air cooler from anywhere in the world with 3$ NodeMCU (esp8266)
*/

#include <ESP8266WiFi.h>
#include <EEPROM.h>

// Enter your WiFi network SSID
const char* ssid = "";
// Enter your WiFi network password
const char* password = "";

/*
   Go to settings of your router, add port forwarding to the PORT_NETWORK port
   Find out your public IP, then you can connect to the webserver by simply going to http://my_public_ip:PORT_NETWORK
*/
const int PORT_NETWORK = 8150;

WiFiServer server(PORT_NETWORK);

class Thermostat
{
  public:
    enum State { Cool, Off, Heat };

    Thermostat()
    {
      // default values
      _expectedTemperature = 75;
      _actualTemperature = 75;
      _state = State::Off;
    }

    void setup()
    {
      // prepare pins
      pinMode(PIN_HEATER, OUTPUT);
      pinMode(PIN_COOLER, OUTPUT);
      pinMode(PIN_FAN, OUTPUT);

      allLow();
    }

    // applying settings from web page and turning on/off corresponding pins
    void apply()
    {
      allLow();
      
      switch (_state)
      {
        case Thermostat::State::Heat:
          digitalWrite(PIN_HEATER, HIGH);

          if(_actualTemperature < _expectedTemperature)
          {
            digitalWrite(PIN_FAN, HIGH);
          }
          
          break;
        case Thermostat::State::Cool:
          digitalWrite(PIN_COOLER, HIGH);

          if(_actualTemperature > _expectedTemperature)
          {
            digitalWrite(PIN_FAN, HIGH);
          }
          break;
      }
    }

    void load()
    {
      _state = static_cast<State>(EEPROM.read(0));
      _expectedTemperature = EEPROM.read(1);
    }

    void save()
    {
      EEPROM.write(0, _state);
      EEPROM.write(1, _expectedTemperature);
    }

  public:
    State _state;
    int _expectedTemperature;
    int _actualTemperature;

  private:

    void allLow()
    {
      digitalWrite(PIN_COOLER, LOW);
      digitalWrite(PIN_HEATER, LOW);
      digitalWrite(PIN_FAN, LOW);
    }

    const int PIN_HEATER = D0; // shorting R and W wires turns on heater
    const int PIN_COOLER = D1; // shorting R and Y wires turns on cooler
    const int PIN_FAN = D3; // shorting R and G wires turns on fan
    const int TEMPERATURE_THRESHOLD = 3;
};


Thermostat thermostat;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(10); // wait for serial port to connect. Needed for native USB port only
  }

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

  thermostat.setup();
  
  // Print the IP address
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    delay(10);
    return;
  }

  // Wait until the client sends some data
  Serial.println("new client");

  const int timeOut = 3000;
  int time = millis();
  while (!client.available())
  {
    delay(10);
    if (millis() - time > timeOut)
    {
      client.stop();
      return;
    }
  }

  //HTTP REQUEST
  String httpRequest = client.readStringUntil('\r');
  Serial.println(httpRequest);
  client.flush();

  if (httpRequest.indexOf("state=off") != -1)
  {
    thermostat._state = Thermostat::State::Off;
  } else if (httpRequest.indexOf("state=cool") != -1)
  {
    thermostat._state = Thermostat::State::Cool;
  } else if (httpRequest.indexOf("state=heat") != -1)
  {
    thermostat._state = Thermostat::State::Heat;
  }

  int idxTemperature = httpRequest.indexOf("temperature=");
  if (idxTemperature != -1)
  {
    thermostat._expectedTemperature = httpRequest.substring(idxTemperature + 12, idxTemperature + 14).toInt();
  }

  thermostat.apply();

  client.flush();

  //HTTP RESPONSE
  // send a standard http response header
  client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n");
  // send web page
  client.println("<!DOCTYPE html>");
  client.println("<html>");
  client.println("<head>");
  client.println("<title>Thermostat control</title>");
  client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">");
  client.println("</head>");
  client.println("<body>");
  client.println("<h1>Thermostat</h1>");


  // controls
  client.print("<form>");

  const char strChecked[] = "checked";
  char strElement[90];

  sprintf(strElement, "<input name=state type=radio value=cool %s />Cool<br><br>", (thermostat._state == Thermostat::State::Cool) ? strChecked : "");
  client.print(strElement);
  sprintf(strElement, "<input name=state type=radio value=off %s />Off<br><br>", (thermostat._state == Thermostat::State::Off) ? strChecked : "");
  client.print(strElement);
  sprintf(strElement, "<input name=state type=radio value=heat %s />Heat<br><br>", (thermostat._state == Thermostat::State::Heat) ? strChecked : "");
  client.print(strElement);

  client.println("<br>");
  sprintf(strElement, "<input name=temperature type=number value=%d /><br><br>", thermostat._expectedTemperature);
  client.print(strElement);
  client.print("<input type=submit value=Apply /><br>");
  client.println("</form>");

  client.println("<p></p>");
  client.println("<p>Current temperature (F):</p>");
  client.println("</body>");
  client.println("</html>");

  delay(1);
}






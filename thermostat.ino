/**
   The MIT License (MIT)
   Copyright (c) 2017 by Rustam Iskenderov

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:
   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.
   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <EEPROM.h>

#include "NTPClient.h"
#include "ArduinoMedianFilter.h"

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

          if (_actualTemperature < _expectedTemperature)
          {
            digitalWrite(PIN_FAN, HIGH);
          }

          break;
        case Thermostat::State::Cool:
          digitalWrite(PIN_COOLER, HIGH);

          if (_actualTemperature > _expectedTemperature)
          {
            digitalWrite(PIN_FAN, HIGH);
          }
          break;
        case Thermostat::State::Off:
          digitalWrite(PIN_FAN, LOW);
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


namespace Schedule
{
namespace Weekday
{
bool ON = false;
bool OFF = false;
const char* NAME_ON = "wkyon";
const char* NAME_OFF = "wkyoff";

int ON_TIME = -1; // hours
int OFF_TIME = -1;
const char* NAME_ON_TIME = "wkytimeon";
const char* NAME_OFF_TIME = "wkytimeoff";
}

namespace Weekend
{
bool ON = false;
bool OFF = false;
const char* NAME_ON = "wkdon";
const char* NAME_OFF = "wkdoff";

int ON_TIME = -1;
int OFF_TIME = -1;
const char* NAME_ON_TIME = "wkdtimeon";
const char* NAME_OFF_TIME = "wkdtimeoff";
}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
WiFiUDP ntpUDP;

Thermostat thermostat;

NTPClient timeClient(ntpUDP);

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

  timeClient.begin();
  timeClient.setTimeOffset(-6 * 60 * 60);
  timeClient.setUpdateInterval(120000);
  timeClient.update();

  Serial.println(timeClient.getFormattedTime());

  // Print the IP address
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

bool getValue(int &value, const char* name, const String &request)
{
  int idxStart = request.indexOf(name);

  if (idxStart == -1)
  {
    return false;
  }

  idxStart = request.indexOf("=", idxStart + 1);

  if (idxStart == -1)
  {
    return false;
  }

  ++idxStart;

  int idxEnd = request.indexOf("&", idxStart + 1);

  if (idxEnd == -1)
  {
    idxEnd = request.length() - 1;
  }

  value = request.substring(idxStart, idxEnd).toInt();

  return true;
}

const uint16_t SAMPLES_COUNT = 300;
ArduinoMedianFilter<uint16_t, SAMPLES_COUNT> temperatureFilter;


double celciusToFahrenheit(double celsius)
{
  return celsius * 9.0 / 5.0 + 32;
}

double getTemperature()
{
  uint16_t ntcRef; // digital value on NTC thermistor

  for (int i = 0; i < SAMPLES_COUNT; i++)
  {
    temperatureFilter.set(i, analogRead(A0));
    //delay(1);
  }

  temperatureFilter.getAverage(30, ntcRef);

  int gndRef = 5; // value on A0 pin when it's connected to GND
  int vccRef = 985; // value on A0 pin when it's connected to 3.3V
  double Vcc = 3.277; // real voltage between 3.3 and GND pins, use multimeter to measure.

  ntcRef = constrain(ntcRef, gndRef, vccRef);

  // Real voltage on analogue pin
  double Vout = Vcc * (ntcRef - gndRef) / (vccRef - gndRef);

  double R0 = 20210.0; // resistor in voltage divider, use multimeter to measure

  // calculating current resistance of thermistor
  double Rntc = (R0 * Vout) / (Vcc - Vout);

  Rntc = log(Rntc);
  double temperature = 1.0 / (0.001129148 + (0.000234125 + (0.0000000876741 * Rntc * Rntc )) * Rntc );
  temperature -= 273.15;

  return temperature;
}

const char index_html[] = R"=====(
<canvas id="myCanvas" width="200" height="100"
style="border:1px solid #d3d3d3;">
Your browser does not support the canvas element.
</canvas>

<script>

var canvas = document.getElementById("myCanvas");
var ctx = canvas.getContext("2d");
ctx.moveTo(0,0);
ctx.lineTo(200,100);
ctx.stroke();

</script>
)=====";

double gTemperature = 0.0;

void loop() {  
  
  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    gTemperature = celciusToFahrenheit(getTemperature());
    Serial.println(gTemperature);
    timeClient.update();
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

  getValue(thermostat._expectedTemperature, "temperature", httpRequest);

  int tz;
  if (getValue(tz, "timeZone", httpRequest))
  {
    timeClient.setTimeOffset(tz * 3600);
  }

  Schedule::Weekday::ON = (httpRequest.indexOf(Schedule::Weekday::NAME_ON) != -1);
  Schedule::Weekday::OFF = (httpRequest.indexOf(Schedule::Weekday::NAME_OFF) != -1);
  Schedule::Weekend::ON = (httpRequest.indexOf(Schedule::Weekend::NAME_ON) != -1);
  Schedule::Weekend::OFF = (httpRequest.indexOf(Schedule::Weekend::NAME_OFF) != -1);

  getValue(Schedule::Weekday::ON_TIME, Schedule::Weekday::NAME_ON_TIME, httpRequest);
  getValue(Schedule::Weekday::OFF_TIME, Schedule::Weekday::NAME_OFF_TIME, httpRequest);
  getValue(Schedule::Weekend::ON_TIME, Schedule::Weekend::NAME_ON_TIME, httpRequest);
  getValue(Schedule::Weekend::OFF_TIME, Schedule::Weekend::NAME_OFF_TIME, httpRequest);

  thermostat.apply();

  client.flush();

  //HTTP RESPONSE
    
  // send a standard http response header
  client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n");
  // send web page
  const char* httpHeader = 
  R"=====(
  <!DOCTYPE html>
  <html>
  <head>
  <title>Thermostat control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  </head>
  <body>
  <h1>Thermostat</h1>
  )=====";
client.println(httpHeader);

// controls
client.print("<form>");

const char strChecked[] = "checked";
char strElement[90];

sprintf(strElement, "<p>Time: %s</p>", timeClient.getFormattedTime().c_str());
client.print(strElement);

sprintf(strElement, "<p>Temperature: %dF</p><br>", (int)gTemperature);
client.println(strElement);

sprintf(strElement, "<input name=state type=radio value=cool %s />Cool<br><br>", (thermostat._state == Thermostat::State::Cool) ? strChecked : "");
client.print(strElement);
sprintf(strElement, "<input name=state type=radio value=off %s />Off<br><br>", (thermostat._state == Thermostat::State::Off) ? strChecked : "");
client.print(strElement);
sprintf(strElement, "<input name=state type=radio value=heat %s />Heat<br><br>", (thermostat._state == Thermostat::State::Heat) ? strChecked : "");
client.print(strElement);

client.println("<br>");
sprintf(strElement, "<label>Temperature </label><input name=temperature type=number value=%d /><br><br>", thermostat._expectedTemperature);
client.print(strElement);

sprintf(strElement, "<label>Time zone </label><input name=timeZone type=number value=%d /><br><br>", timeClient.getTimeOffset() / 3600);
client.print(strElement);

// Schedule, weekdays

const char* TAG_DIV_CLEAR = "<div style=\"clear:left;\" /><br>";

client.print("<div style=\"float:left;\" >");
sprintf(strElement, "<label>Weekday, ON </label><input name=%s type=checkbox value=true %s />", Schedule::Weekday::NAME_ON, (Schedule::Weekday::ON) ? strChecked : "");
client.print(strElement);

sprintf(strElement, "<label> Time (hours) </label><input name=%s type=number value=%d />", Schedule::Weekday::NAME_ON_TIME, Schedule::Weekday::ON_TIME);
client.print(strElement);

client.print("</div>");
client.print(TAG_DIV_CLEAR);

client.print("<div style=\"float:left;\" >");
sprintf(strElement, "<label>Weekday, OFF </label><input name=%s type=checkbox value=true %s />", Schedule::Weekday::NAME_OFF, (Schedule::Weekday::OFF) ? strChecked : "");
client.print(strElement);

sprintf(strElement, "<label>Time (hours) </label><input name=%s type=number value=%d />", Schedule::Weekday::NAME_OFF_TIME, Schedule::Weekday::OFF_TIME);
client.print(strElement);

client.print("</div>");
client.print(TAG_DIV_CLEAR);

// Schedule, weekends
client.print("<div style=\"float:left;\" >");
sprintf(strElement, "<label>Weekend, ON </label><input name=%s type=checkbox value=true %s />", Schedule::Weekend::NAME_ON, (Schedule::Weekend::ON) ? strChecked : "");
client.print(strElement);

sprintf(strElement, "<label>Time (hours) </label><input name=%s type=number value=%d />", Schedule::Weekend::NAME_ON_TIME, Schedule::Weekend::ON_TIME);
client.print(strElement);

client.print("</div>");
client.print(TAG_DIV_CLEAR);

client.print("<div style=\"float:left;\" >");
sprintf(strElement, "<label>Weekend, OFF </label><input name=%s type=checkbox value=true %s />", Schedule::Weekend::NAME_OFF, (Schedule::Weekend::OFF) ? strChecked : "");
client.print(strElement);

sprintf(strElement, "<label>Time (hours) </label><input name=%s type=number value=%d />", Schedule::Weekend::NAME_OFF_TIME, Schedule::Weekend::OFF_TIME);
client.print(strElement);

client.print("</div>");
client.print(TAG_DIV_CLEAR);

client.print("<input type=submit value=Apply /><br>");
client.println("</form>");

client.println("<p></p>");

client.println(index_html);

client.println("</body>");
client.println("</html>");

timeClient.update();

delay(1);
}






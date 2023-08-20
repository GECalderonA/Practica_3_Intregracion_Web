#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <stdlib.h>
#include <Adafruit_Sensor.h>

#include "data.h"
#include "Settings.h"
#include "UbidotsEsp32Mqtt.h"
#include <UbiConstants.h>
#include <UbiTypes.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include "DHT.h"

#define BUTTON_LEFT 0        // btn activo en bajo
#define LONG_PRESS_TIME 3000 // 3000 milis = 3s
#define DHTPIN 27          // pin 27 lee
#define DHTYPE DHT11

TFT_eSPI tft = TFT_eSPI();
DHT dht(DHTPIN, DHTYPE);

WebServer server(80);

Settings settings;
int lastState = LOW; // para el btn
int currentState;    // the current reading from the input pin
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;

const char *UBIDOTS_TOKEN = "BBFF-wYk29xjJfzN3NobqxDz6gj910JJitp"; // Put here your Ubidots TOKEN
const char *DEVICE_LABEL = "esp32";                                // Put here your Device label to which data  will be published
const char *VARIABLE_LABEL = "temp";                               // Put here your Variable label to which data  will be published
const char *HUMEDAD_VARIABLE_LABEL = "Humedad";                    //  humedad
const char *TEMPERATURA_VARIABLE_LABEL = "Temperatura";            //  temperatura
const char *VARIABLE_LABEL_1 = "sw1";                              // Replace with your variable label to subscribe to
const char *VARIABLE_LABEL_2 = "sw2";                              // Replace with your variable label to subscribe to

int tamano;
int posicion;
char boton = '0';
char val = '0';

const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds
unsigned long timer;

const uint8_t LED1 = 26; // Pin used to write data based on 1's and 0's coming from Ubidots
const uint8_t LED2 = 2; // Pin used to write data based on 1's and 0's coming from Ubidots

Ubidots ubidots(UBIDOTS_TOKEN);

void load404();
void loadIndex();
void loadFunctionsJS();
void restartESP();
void saveSettings();
bool is_STA_mode();
void AP_mode_onRst();
void STA_mode_onRst();
void detect_long_press();
void callback(char *topic, byte *payload, unsigned int length);

// Rutina para iniciar en modo AP (Access Point) "Servidor"
void startAP()
{
  WiFi.disconnect();
  delay(19);
  Serial.println("Starting WiFi Access Point (AP)");
  WiFi.softAP("fabio_AP", "facil123");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  /*ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect(); */

  timer = millis();
}

// Rutina para iniciar en modo STA (Station) "Cliente"
void start_STA_client()
{
  WiFi.softAPdisconnect(true);
  WiFi.disconnect();
  delay(100);
  Serial.println("Starting WiFi Station Mode");
  WiFi.begin((const char *)settings.ssid.c_str(), (const char *)settings.password.c_str());
  WiFi.mode(WIFI_STA);

  int cnt = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    // Serial.print(".");
    if (cnt == 100) // Si después de 100 intentos no se conecta, vuelve a modo AP
      AP_mode_onRst();
    cnt++;
    Serial.println("attempt # " + (String)cnt);
  }

  WiFi.setAutoReconnect(true);
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
  pressedTime = millis();
  // Rutinas de Ubidots

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL_1); // Insert the device and variable's Labels, respectively
  ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL_2); // Insert the device and variable's Labels, respectively

  Serial.println(F("DHTxx test!"));
  dht.begin();
  timer = millis();

  float Humedad = dht.readHumidity();
  float Temperatura = dht.readTemperature();
  tft.init();
  tft.fillScreen(TFT_BLACK); // mirar el color
  tft.drawString("Humedad", 10, 10, 2);
  tft.drawString("Temperatura", 10, 60, 2);
  if (isnan(Humedad) || isnan(Temperatura))
  {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
}

void setup()
{

  Serial.begin(115200);
  delay(2000);

  EEPROM.begin(4096);                 // Se inicializa la EEPROM con su tamaño max 4KB
  pinMode(BUTTON_LEFT, INPUT_PULLUP); // btn activo en bajo

  // settings.reset();
  settings.load(); // se carga SSID y PWD guardados en EEPROM
  settings.info(); // ... y se visualizan

  Serial.println("");
  Serial.println("starting...");

  if (is_STA_mode())
  {
    start_STA_client();
  }
  else // Modo Access Point & WebServer
  {
    startAP();

    /* ========== Modo Web Server ========== */

    /* HTML sites */
    server.onNotFound(load404);

    server.on("/", loadIndex);
    server.on("/index.html", loadIndex);
    server.on("/functions.js", loadFunctionsJS);

    /* JSON */
    server.on("/settingsSave.json", saveSettings);
    server.on("/restartESP.json", restartESP);

    server.begin();
    Serial.println("HTTP server started");
  }
}

void loop()
{
  if (is_STA_mode()) // Rutina para modo Station (cliente Ubidots)
  {
    if (!ubidots.connected())
    {
      ubidots.reconnect();
      ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL_1); // Insert the device and variable's Labels, respectively
      ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL_2); // Insert the device and variable's Labels, respectively
    }
    float Humedad = dht.readHumidity();
    float Temperatura = dht.readTemperature();

    if ((millis() - timer) > PUBLISH_FREQUENCY)
    {
      Serial.print("Humedad: ");
      Serial.print(Humedad);
      Serial.print(" - Temperatura: ");
      Serial.print(Temperatura);
      tft.drawString(String(Humedad), 10, 30, 2);
      tft.drawString(String(Temperatura), 10, 80, 2);
      ubidots.add(HUMEDAD_VARIABLE_LABEL, Humedad);
      ubidots.add(TEMPERATURA_VARIABLE_LABEL, Temperatura);
      ubidots.publish(DEVICE_LABEL);
      timer = millis();
    }
    ubidots.loop();
  }
  else // rutina para AP + WebServer
    server.handleClient();

  delay(10);
  detect_long_press();
}

// funciones para responder al cliente desde el webserver:
// load404(), loadIndex(), loadFunctionsJS(), restartESP(), saveSettings()

void load404()
{
  server.send(200, "text/html", data_get404());
}

void loadIndex()
{
  server.send(200, "text/html", data_getIndexHTML());
}

void loadFunctionsJS()
{
  server.send(200, "text/javascript", data_getFunctionsJS());
}

void restartESP()
{
  server.send(200, "text/json", "true");
  ESP.restart();
}

void saveSettings()
{
  if (server.hasArg("ssid"))
    settings.ssid = server.arg("ssid");
  if (server.hasArg("password"))
    settings.password = server.arg("password");

  settings.save();
  server.send(200, "text/json", "true");
  STA_mode_onRst();
}

// Rutina para verificar si ya se guardó SSID y PWD del cliente
// is_STA_mode retorna true si ya se guardaron
bool is_STA_mode()
{
  if (EEPROM.read(flagAdr))
    return true;
  else
    return false;
}

void AP_mode_onRst()
{
  EEPROM.write(flagAdr, 0);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void STA_mode_onRst()
{
  EEPROM.write(flagAdr, 1);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void detect_long_press()
{
  // read the state of the switch/button:
  currentState = digitalRead(BUTTON_LEFT);

  if (lastState == HIGH && currentState == LOW) // button is pressed
    pressedTime = millis();
  else if (lastState == LOW && currentState == HIGH)
  { // button is released
    releasedTime = millis();

    // Serial.println("releasedtime" + (String)releasedTime);
    // Serial.println("pressedtime" + (String)pressedTime);

    long pressDuration = releasedTime - pressedTime;

    if (pressDuration > LONG_PRESS_TIME)
    {
      Serial.println("(Hard reset) returning to AP mode");
      delay(500);
      AP_mode_onRst();
    }
  }

  // save the the last state
  lastState = currentState;
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  tamano = strlen(topic);
  posicion = tamano - 4;
  printf("switch: %c\n", topic[posicion]);
  boton = topic[posicion];
  val = payload[0];

  if (boton == '1')
  {
    if (val == '1')
    {
      digitalWrite(LED1, HIGH);
    }
    else
    {
      digitalWrite(LED1, LOW);
    }
  }

  if (boton == '2')
  {
    if (val == '1')
    {
      digitalWrite(LED2, HIGH);
    }
    else
    {
      digitalWrite(LED2, LOW);
    }
  }
}
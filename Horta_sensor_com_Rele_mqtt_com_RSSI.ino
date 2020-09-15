/***************************************************
  Adafruit MQTT Library ESP8266 Example

  Must use ESP8266 Arduino from:
    https://github.com/esp8266/Arduino

  Works great with Adafruit's Huzzah ESP board & Feather
  ----> https://www.adafruit.com/product/2471
  ----> https://www.adafruit.com/products/2821

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Tony DiCola for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
int sensor = A0;
int valor = 0.0;
int valor_mapeado;
int rssi = WiFi.RSSI();
#define Lamp D4// pino do rele   //Deve ser alterado conforme o pino utilizado na placa
/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "VIVOFIBRA-9CE6"             // Nome da rede Wifi (2,4ghz)
#define WLAN_PASS       "1879Bage"        // Senha
/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "NGT84"            // Usuário Adafruit
#define AIO_KEY         "87f2d1189fde445bb2c6457bfd530e0f"   // Project Auth Key Ada fruit

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish photocell = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Umidade");
Adafruit_MQTT_Publish sinal = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/RSSINodemcu");
// Setup a feed called 'onoff' for subscribing to changes.
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Lamp");

/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();

void setup() {
  Serial.begin(115200);
  delay(10);

  pinMode(sensor, INPUT);
  pinMode(Lamp, OUTPUT);

  Serial.println(F("||  HORTA CASEIRA  ||"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Conectando à rede... ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connectado");
  Serial.println("Endereço IP: "); Serial.println(WiFi.localIP());
  Serial.print("Sinal(RSSI): ");   //Sinal do Wifi em RSS


  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&onoffbutton);
}


uint32_t x = valor_mapeado;

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &onoffbutton) {
      Serial.print(F("Recebido:"));
      Serial.print((char *)onoffbutton.lastread);
      int onoffbutton_State = atoi((char *)onoffbutton.lastread);
      if (onoffbutton_State == 1 ) {
        Serial.println("  (Ligando Irrigaçao)");
        digitalWrite(Lamp,LOW);
      }
      else {
        Serial.println("   (Desligando irrigador...)");
        digitalWrite(Lamp,HIGH);

      }
    }
  }

  // Now we can publish stuff!
  Serial.print(F("\ UR: "));
  Serial.print(valor_mapeado, DEC);
  Serial.print("%    |");
  Serial.print(F("\RAW (bits): "));
  Serial.print(valor);
  Serial.print(F("     |\RSSI: "));
   Serial.print(rssi);
  int force_value = valor_mapeado;
  valor = analogRead(sensor);
  valor_mapeado = map(valor, 100, 1023, 100, 0);
  if (! photocell.publish(force_value)) {
    Serial.println(F("          ||Falha no MQTT!"));
  } else {
    Serial.println(F("          ||OK"));
    if (! sinal.publish(rssi)) {
      Serial.println(F("          ||not!"));
    }

    delay (6000);
  }


  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  /*
    if(! mqtt.ping()) {
    mqtt.disconnect();
    }
  */
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Conectando ao broker MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Tentando reconectar ao MQTT em 5 segundos...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Conectado!");
}

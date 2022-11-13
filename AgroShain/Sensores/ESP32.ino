/****************************************
 * Incluir Librerias
 ****************************************/
#include "UbidotsEsp32Mqtt.h"
#include <Wire.h>
#include "SHTSensor.h"

/****************************************
   Definir Librerias
 ****************************************/

#define SHT40 SHTSensor::SHT4X
#define LED 12
SHTSensor sht40(SHT40);

/****************************************
 * Definir Constantes
 ****************************************/
const char *UBIDOTS_TOKEN = "BBFF-WmPKRpxkjUIBvzwcDQQi4RBiVTfxZA";  // Poner Ubidots TOKEN

const char *WIFI_SSID = "wifi-Sistemas";      // Poner Wi-Fi SSID
const char *WIFI_PASS = "";      // Poner Wi-Fi password
const char *DEVICE_LABEL = "iotday_demo";   // Reemplace con la etiqueta del dispositivo para suscribirse
const char *VARIABLE_LABEL = "relay"; // Reemplace con su etiqueta variable para suscribirse
const char *PUBLISH_VARIABLE_LABEL1 = "temperatura";                          // Pon aquí tu etiqueta de Variable a la que se publicarán los datos
const char *PUBLISH_VARIABLE_LABEL2 = "humedad";                              // Pon aquí tu etiqueta de Variable a la que se publicarán los datos
const char *PUBLISH_VARIABLE_LABEL3 = "aire"; // Reemplace con su etiqueta variable para suscribirse
const char *PUBLISH_DEVICE_LABEL = "iotday_demo";
const int PUBLISH_FREQUENCY = 1000;                                           // Tasa de actualización en milisegundos
//const uint8_t LED = 12; // Pin utilizado para escribir datos basados ​​en 1 y 0 provenientes de Ubidots


/****************************************
   Variables de propósito general
 ****************************************/

boolean aire;
byte Relay=0x01, val=0x00;
unsigned long timer;
float H, T;
bool L = 0;
Ubidots ubidots(UBIDOTS_TOKEN);

/****************************************
 * Funciones auxiliares
 ****************************************/

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    ///Serial.print(Relay);
    Serial.print((char)payload[i]);
    if ((char)payload[0] == '1')
    {
      Relay=0x01;
      aire=1;
      digitalWrite(LED, HIGH);
      Serial.println("led encendido");
      Serial.println(aire);
    }
    else
    {
      digitalWrite(LED, LOW);
      Relay=0x00;
      Serial.println("led apagado");
      aire=0;
      Serial.println(aire);
    }
    
  }
  Serial.println();
   
}

/****************************************
 * Funciones Main
 ****************************************/

void setup()
{
  // ponga su código de configuración aquí, para ejecutar una vez:
  Wire.begin(); // Join I2C bus
  sht40.init();
  Serial.begin(115200);
  pinMode(LED, OUTPUT);

  // ubidots.setDebug(true);  // descomente esto para que los mensajes de depuración estén disponibles
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL); // Inserte las etiquetas del dispositivo y de la variable, respectivamente
   timer = millis();
}

void loop()
{
  // pon tu código principal aquí, para que se ejecute repetidamente:
  if (!ubidots.connected())
  {
    ubidots.reconnect();
  }

if (millis() - timer >= PUBLISH_FREQUENCY)  // Activa la rutina cada 5 segundos
  {
   ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL); // Inserte las etiquetas del dispositivo y de la variable, respectivamente
   ReadSensor();
   WriteDigitalOutput(); 
   WriteRelay();
   
   ubidots.add(PUBLISH_VARIABLE_LABEL1, T);  // Inserte sus etiquetas variables y el valor a enviar
   ubidots.publish(PUBLISH_DEVICE_LABEL);
   ubidots.add(PUBLISH_VARIABLE_LABEL2, H);  // Inserte sus etiquetas variables y el valor a enviar
   ubidots.publish(PUBLISH_DEVICE_LABEL);
   
   ubidots.add(PUBLISH_VARIABLE_LABEL3, aire);  // Inserte sus etiquetas variables y el valor a enviar
   ubidots.publish(PUBLISH_DEVICE_LABEL);
   timer = millis();
   LED_state();    

  }

 
  ubidots.loop();
}

void ReadSensor(void)
{
  sht40.readSample();
  H = sht40.getHumidity();
  T = sht40.getTemperature();
}


void WriteRelay(void)
{
    
    Wire.beginTransmission(0x0D);  // Transmitir al dispositivo número 44 (0x2C)
    Wire.write(Relay);             // Mandar valor byte
    Wire.endTransmission();      // Deja de trasmitir
    
   }

   void WriteDigitalOutput(void)
{
    Wire.beginTransmission(0x02);  // Transmitir al dispositivo número 44 (0x2C)

    Wire.write(val);             // Mandar valor byte
    Wire.endTransmission();      // Deja de trasmitir
    val++;
    if (val==0xff)
    val=0x00;
   }
   void LED_state(void)
{
  L = !L;
  digitalWrite(LED, L);
}
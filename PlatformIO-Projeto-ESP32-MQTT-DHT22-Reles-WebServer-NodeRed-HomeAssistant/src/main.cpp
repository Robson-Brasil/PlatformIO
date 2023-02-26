#include <Arduino.h>
/**********************************************************************************
IoT - Automação Residencial
Autor : Robson Brasil
Dispositivos : ESP32 WROOM32, DHT22 e Módulo Relé de 8 Canais
Preferences--> URLs adicionais do Gerenciador de placas: 
                                  http://arduino.esp8266.com/stable/package_esp8266com_index.json,https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
Download Board ESP32 (2.0.5):
Broker MQTT
Node-Red / Google Assistant-Nora:  https://smart-nora.eu/  
Para Instalação do Node-Red:       https://nodered.org/docs/getting-started/
Home Assistant
Para Instalação do Home Assistant: https://www.home-assistant.io/installation/
Versão : 20 - Alfa
Última Modificação : 15/22/2023
**********************************************************************************/

// Bibliotecas
#include <WiFi.h>          // Importa a Biblioteca WiFi
#include <PubSubClient.h>  // Importa a Biblioteca PubSubClient
#include <DHT.h>           // Importa a Biblioteca DHT
#include <WiFiUdp.h>       // Importa a Biblioteca WiFiUdp
#include <Arduino.h>       // Biblioteca do ArduinoJson :        https://github.com/bblanchon/ArduinoJson
#include <DNSServer.h>     // Biblioteca do DNSServer :          https://github.com/zhouhan0126/DNSServer---esp32
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

DNSServer dns;

// Tópicos do Subscribe
const char* sub0 = "ESP32/MinhaCasa/QuartoRobson/Ligar-DesligarTudo/Comando";  // Somente por MQTT
const char* sub1 = "ESP32/MinhaCasa/QuartoRobson/Interruptor1/Comando";        // Ligados ao Nora/MQTT
const char* sub2 = "ESP32/MinhaCasa/QuartoRobson/Interruptor2/Comando";        // Ligados ao Nora/MQTT
const char* sub3 = "ESP32/MinhaCasa/QuartoRobson/Interruptor3/Comando";        // Ligados ao Nora/MQTT
const char* sub4 = "ESP32/MinhaCasa/QuartoRobson/Interruptor4/Comando";        // Ligados ao Nora/MQTT
const char* sub5 = "ESP32/MinhaCasa/QuartoRobson/Interruptor5/Comando";        // Ligados ao Nora/MQTT
const char* sub6 = "ESP32/MinhaCasa/QuartoRobson/Interruptor6/Comando";        // Ligados ao MQTT/Alexa
const char* sub7 = "ESP32/MinhaCasa/QuartoRobson/Interruptor7/Comando";        // Ligados ao MQTT/Alexa
const char* sub8 = "ESP32/MinhaCasa/QuartoRobson/Interruptor8/Comando";        // Ligados ao MQTT/Alexa

// Tópicos do Publish
const char* pub0 = "ESP32/MinhaCasa/QuartoRobson/Ligar-DesligarTudo/Estado";  // Somente por MQTT
const char* pub1 = "ESP32/MinhaCasa/QuartoRobson/Interruptor1/Estado";        // Ligados ao Nora/MQTT
const char* pub2 = "ESP32/MinhaCasa/QuartoRobson/Interruptor2/Estado";        // Ligados ao Nora/MQTT
const char* pub3 = "ESP32/MinhaCasa/QuartoRobson/Interruptor3/Estado";        // Ligados ao Nora/MQTT
const char* pub4 = "ESP32/MinhaCasa/QuartoRobson/Interruptor4/Estado";        // Ligados ao Nora/MQTT
const char* pub5 = "ESP32/MinhaCasa/QuartoRobson/Interruptor5/Estado";        // Ligados ao Nora/MQTT
const char* pub6 = "ESP32/MinhaCasa/QuartoRobson/Interruptor6/Estado";        // Ligados ao MQTT/Alexa
const char* pub7 = "ESP32/MinhaCasa/QuartoRobson/Interruptor7/Estado";        // Ligados ao MQTT/Alexa
const char* pub8 = "ESP32/MinhaCasa/QuartoRobson/Interruptor8/Estado";        // Ligados ao MQTT/Alexa
const char* pub9 = "ESP32/MinhaCasa/QuartoRobson/Temperatura";                // Somente por MQTT
const char* pub10 = "ESP32/MinhaCasa/QuartoRobson/Umidade";                   // Somente por MQTT
const char* pub11 = "ESP32/MinhaCasa/QuartoRobson/SensacaoTermica";           // Somente por MQTT

float diff = 1.0;

unsigned long delayTime;

#define ID_MQTT "ESP32-IoT-1" /* ID MQTT (para identificação de seção)           \
                            IMPORTANTE: Este deve ser único no broker (ou seja, \
                            se um client MQTT tentar entrar com o mesmo         \
                            ID de outro já conectado ao broker, o broker        \
                            irá fechar a conexão de um deles).*/

// Defines - Mapeamento de pinos do NodeMCU Relays
#define RelayPin1 23  // D23 Ligados ao Nora/MQTT
#define RelayPin2 22  // D22 Ligados ao Nora/MQTT
#define RelayPin3 21  // D21 Ligados ao Nora/MQTT
#define RelayPin4 19  // D19 Ligados ao Nora/MQTT
#define RelayPin5 18  // D18 Ligados ao Nora/MQTT
#define RelayPin6 5   // D5  Ligados ao MQTT/Alexa
#define RelayPin7 25  // D25 Ligados ao MQTT/Alexa
#define RelayPin8 26  // D26 Ligados ao MQTT/Alexa

// WiFi Status Relé
#define wifiLed 0  // D0
#define DEBOUNCE_TIME 250

int toggleState_0 = 1;  // Definir inteiro para lembrar o estado de alternância para o relé 0
int toggleState_1 = 1;  // Definir inteiro para lembrar o estado de alternância para o relé 1
int toggleState_2 = 1;  // Definir inteiro para lembrar o estado de alternância para o relé 2
int toggleState_3 = 1;  // Definir inteiro para lembrar o estado de alternância para o relé 3
int toggleState_4 = 1;  // Definir inteiro para lembrar o estado de alternância para o relé 4
int toggleState_5 = 1;  // Definir inteiro para lembrar o estado de alternância para o relé 5
int toggleState_6 = 1;  // Definir inteiro para lembrar o estado de alternância para o relé 6
int toggleState_7 = 1;  // Definir inteiro para lembrar o estado de alternância para o relé 7
int toggleState_8 = 1;  // Definir inteiro para lembrar o estado de alternância para o relé 8
int status_todos = 0;   // Definir inteiro para lembrar o estado de alternância para todos

// DHT22 para leitura dos valores  de Temperatura e Umidade
#define DHTPIN 4
#define DHTTYPE DHT22  // DHT 22
DHT dht(DHTPIN, DHTTYPE);

// Configurações do WIFI
const char* SSID = "IoT";                // SSID nome da rede WI-FI que deseja se conectar
const char* PASSWORD = "@IoT@S3nh@S3gur@";  // Senha da rede WI-FI que deseja se conectar

// Configurações do Broker MQTT
const char* BROKER_MQTT = "192.168.15.10";  // URL do broker MQTT que se deseja utilizar
const char* mqttUserName = "RobsonBrasil";  // MQTT UserName
const char* mqttPwd = "loboalfa";           // MQTT Password
int BROKER_PORT = 1883;                     // Porta do Broker MQTT

// IP Estático
IPAddress local_IP(192, 168, 15, 50);
IPAddress gateway(192, 168, 15, 1);
IPAddress subnet(255, 255, 255, 0);
// DNS Estático
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

// Variáveis e objetos globais
WiFiClient espClient;          // Cria o objeto espClient
PubSubClient MQTT(espClient);  // Instância o Cliente MQTT passando o objeto espClient

char str_hum_data[10];
char str_temp_data[10];
char str_tempterm_data[10];
char str_tempF_data[10];

#define MSG_BUFFER_SIZE (1000)
unsigned long lastMsg = 0;
int value = 0;

//WebServer
const char* http_username = "Robson Brasil";
const char* http_password = "@Lobo#Alfa@";
const char* PARAM_INPUT_1 = "output";
const char* PARAM_INPUT_2 = "state";

// Configuração da Porta Usada Pelo AsyncWebServer
AsyncWebServer server(80);

const char login_html[] PROGMEM = R"rawliteral(

<script>function toggleCheckbox(element) {
  var xhr = new XMLHttpRequest();
  if(element.checked){ xhr.open("GET", "/update?output="+element.id+"&state=0", true); }
  else { xhr.open("GET", "/update?output="+element.id+"&state=1", true); }
  xhr.send();
}
</script>
)rawliteral";

// Configuração dos Botões Usados
String processor(const String& var) {
  //Serial.println(var);
  if (var == "BUTTONPLACEHOLDER1") {
    String buttons = "";
    buttons += "<h4>Luz Forte</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"23\" " + outputState(23) + "><span class=\"slider\"></span></label>";
    return buttons;
  }
  if (var == "BUTTONPLACEHOLDER2") {
    String buttons = "";
    buttons += "<h4>Luz Fraca</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"22\" " + outputState(22) + "><span class=\"slider\"></span></label>";
    return buttons;
  }
  if (var == "BUTTONPLACEHOLDER3") {
    String buttons = "";
    buttons += "<h4>Interruptor 3</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"21\" " + outputState(21) + "><span class=\"slider\"></span></label>";
    return buttons;
  }
  if (var == "BUTTONPLACEHOLDER4") {
    String buttons = "";
    buttons += "<h4>Interruptor 4</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"19\" " + outputState(19) + "><span class=\"slider\"></span></label>";
    return buttons;
  }

  if (var == "BUTTONPLACEHOLDER5") {
    String buttons = "";
    buttons += "<h4>Interruptor 5</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"18\" " + outputState(18) + "><span class=\"slider\"></span></label>";
    return buttons;
  }
  if (var == "BUTTONPLACEHOLDER6") {
    String buttons = "";
    buttons += "<h4>Som Bluetooth</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"5\" " + outputState(5) + "><span class=\"slider\"></span></label>";
    return buttons;
  }
  if (var == "BUTTONPLACEHOLDER7") {
    String buttons = "";
    buttons += "<h4>Interruptor 7</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"25\" " + outputState(25) + "><span class=\"slider\"></span></label>";
    return buttons;
  }
  if (var == "BUTTONPLACEHOLDER8") {
    String buttons = "";
    buttons += "<h4>Luz do Quarto</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"26\" " + outputState(26) + "><span class=\"slider\"></span></label>";
    return buttons;
  }

  return String();
}
String outputState(int output) {
  if (digitalRead(output)) {
    return "checked";
  } else {
    return "";
  }
}

// Prototypes
void initSerial();
void initWiFi();
void initMQTT();
void reconectWiFi();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void VerificaConexoesWiFIEMQTT(void);
void initOutput(void);

/*
Implementações das funções
 */
void setup() {
  // Inicializações:
  initOutput();
  initSerial();
  initWiFi();
  initMQTT();
  dht.begin();

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Print do IP Local do ESP32
  Serial.println(WiFi.localIP());
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!request->authenticate(http_username, http_password))
      return request->requestAuthentication();
    request->send(SPIFFS, "/WebServer.html", String(), false, processor);
  });

  server.serveStatic("/", SPIFFS, "/");

  // Send a GET request to <ESP_IP>/update?output=<inputMessage1>&state=<inputMessage2>
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest* request) {
    String inputMessage1;
    String inputMessage2;
    // GET input1 value on <ESP_IP>/update?output=<inputMessage1>&state=<inputMessage2>
    if (request->hasParam(PARAM_INPUT_1) && request->hasParam(PARAM_INPUT_2)) {
      inputMessage1 = request->getParam(PARAM_INPUT_1)->value();
      inputMessage2 = request->getParam(PARAM_INPUT_2)->value();
      digitalWrite(inputMessage1.toInt(), inputMessage2.toInt());
    } else {
      inputMessage1 = "No message sent";
      inputMessage2 = "No message sent";
    }
    Serial.print("GPIO: ");
    Serial.print(inputMessage1);
    Serial.print(" - Set to: ");
    Serial.println(inputMessage2);
    request->send(200, "text/plain", "OK");
  });

  // Start do Servidor
  server.begin();
}

// Função: inicializa comunicação serial com baudrate 115200 (para fins de monitorar no terminal serial
void initSerial() {
  Serial.begin(115200);
}

// Função: inicializa e conecta-se na rede WI-FI desejada
void initWiFi() {
  delay(10);
  Serial.println("------Conexao WI-FI------");
  Serial.print("Conectando-se na rede: ");
  Serial.println(SSID);
  Serial.println("Aguarde");

  reconectWiFi();
}

// Função: inicializa parâmetros de conexão MQTT(endereço do broker, porta e seta função de callback)
void initMQTT() {
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);  // Informa qual broker e porta deve ser conectado
  MQTT.setCallback(mqtt_callback);           // Atribui função de callback (função chamada quando qualquer informação de um dos tópicos subescritos chega)
}

// Função: Função de callback, esta função é chamada toda vez que uma informação de um dos tópicos subescritos chega.
void mqtt_callback(char* topic, byte* payload, unsigned int length) {

  Serial.print("Mensagem enviada ao Broker MQTT no Tópico -> [");
  Serial.print(topic);
  Serial.print("] ");
  payload[length] = '\0';
  String data = "";

  if (strstr(topic, sub0)) {
    for (unsigned int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
      data += (char)payload[i];
    }
    Serial.println();
    if ((char)payload[0] == '0') {
      digitalWrite(RelayPin1, HIGH);  // Ligua o relé. Note que HIGH é o nível de tensão.
      digitalWrite(RelayPin2, HIGH);  // Ligua o relé. Note que HIGH é o nível de tensão.
      digitalWrite(RelayPin3, HIGH);  // Ligua o relé. Note que HIGH é o nível de tensão.
      digitalWrite(RelayPin4, HIGH);  // Ligua o relé. Note que HIGH é o nível de tensão.
      digitalWrite(RelayPin5, HIGH);  // Ligua o relé. Note que HIGH é o nível de tensão.
      digitalWrite(RelayPin6, HIGH);  // Ligua o relé. Note que HIGH é o nível de tensão.
      digitalWrite(RelayPin7, HIGH);  // Ligua o relé. Note que HIGH é o nível de tensão.
      digitalWrite(RelayPin8, HIGH);  // Ligua o relé. Note que HIGH é o nível de tensão.
      status_todos = 0;
      toggleState_0 = 0;
      MQTT.publish(pub0, "0");
    } else {
      digitalWrite(RelayPin1, LOW);  // Desligua o Relé tornando a tensão BAIXA
      digitalWrite(RelayPin2, LOW);  // Desligua o Relé tornando a tensão BAIXA
      digitalWrite(RelayPin3, LOW);  // Desligua o Relé tornando a tensão BAIXA
      digitalWrite(RelayPin4, LOW);  // Desligua o Relé tornando a tensão BAIXA
      digitalWrite(RelayPin5, LOW);  // Desligua o Relé tornando a tensão BAIXA
      digitalWrite(RelayPin6, LOW);  // Desligua o Relé tornando a tensão BAIXA
      digitalWrite(RelayPin7, LOW);  // Desligua o Relé tornando a tensão BAIXA
      digitalWrite(RelayPin8, LOW);  // Desligua o Relé tornando a tensão BAIXA
      status_todos = 1;
      toggleState_0 = 1;
      MQTT.publish(pub0, "1");
    }
  }
  if (strstr(topic, sub1)) {
    for (unsigned int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
      data += (char)payload[i];
    }
    Serial.println();
    if ((char)payload[0] == '0') {
      digitalWrite(RelayPin1, HIGH);  // Ligua o relé. Note que HIGH é o nível de tensão.
      toggleState_1 = 0;
      MQTT.publish(pub1, "0");
    } else {
      digitalWrite(RelayPin1, LOW);  // Desligua o Relé tornando a tensão BAIXA
      toggleState_1 = 1;
      MQTT.publish(pub1, "1");
    }
  }
  if (strstr(topic, sub2)) {
    for (unsigned int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
      data += (char)payload[i];
    }
    Serial.println();

    if ((char)payload[0] == '0') {
      digitalWrite(RelayPin2, HIGH);  // Ligua o relé. Note que HIGH é o nível de tensão.
      toggleState_2 = 0;
      MQTT.publish(pub2, "0");
    } else {
      digitalWrite(RelayPin2, LOW);  // Desligua o Relé tornando a tensão BAIXA
      toggleState_2 = 1;
      MQTT.publish(pub2, "1");
    }
  }
  if (strstr(topic, sub3)) {
    for (unsigned int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
      data += (char)payload[i];
    }
    Serial.println();

    if ((char)payload[0] == '0') {
      digitalWrite(RelayPin3, HIGH);  // Ligua o relé. Note que HIGH é o nível de tensão.
      toggleState_3 = 0;
      MQTT.publish(pub3, "0");
    } else {
      digitalWrite(RelayPin3, LOW);  // Desligua o Relé tornando a tensão BAIXA
      toggleState_3 = 1;
      MQTT.publish(pub3, "1");
    }
  }
  if (strstr(topic, sub4)) {
    for (unsigned int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
      data += (char)payload[i];
    }
    Serial.println();

    if ((char)payload[0] == '0') {
      digitalWrite(RelayPin4, HIGH);  // Ligua o relé. Note que HIGH é o nível de tensão.
      toggleState_4 = 0;
      MQTT.publish(pub4, "0");
    } else {
      digitalWrite(RelayPin4, LOW);  // Desligua o Relé tornando a tensão BAIXA
      toggleState_4 = 1;
      MQTT.publish(pub4, "1");
    }
  }
  if (strstr(topic, sub5)) {
    for (unsigned int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
      data += (char)payload[i];
    }
    Serial.println();

    if ((char)payload[0] == '0') {
      digitalWrite(RelayPin5, HIGH);  // Ligua o relé. Note que HIGH é o nível de tensão.
      toggleState_5 = 0;
      MQTT.publish(pub5, "0");
    } else {
      digitalWrite(RelayPin5, LOW);  // Desligua o Relé tornando a tensão BAIXA
      toggleState_5 = 1;
      MQTT.publish(pub5, "1");
    }
  }
  if (strstr(topic, sub6)) {
    for (unsigned int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
      data += (char)payload[i];
    }
    Serial.println();

    if ((char)payload[0] == '0') {
      digitalWrite(RelayPin6, HIGH);  // Ligua o relé. Note que HIGH é o nível de tensão.
      toggleState_6 = 0;
      MQTT.publish(pub6, "0");
    } else {
      digitalWrite(RelayPin6, LOW);  // Desligua o Relé tornando a tensão BAIXA
      toggleState_6 = 1;
      MQTT.publish(pub6, "1");
    }
  }
  if (strstr(topic, sub7)) {
    for (unsigned int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
      data += (char)payload[i];
    }
    Serial.println();

    if ((char)payload[0] == '0') {
      digitalWrite(RelayPin7, HIGH);  // Ligua o relé. Note que HIGH é o nível de tensão.
      toggleState_7 = 0;
      MQTT.publish(pub7, "0");
    } else {
      digitalWrite(RelayPin7, LOW);  // Desligua o Relé tornando a tensão BAIXA
      toggleState_7 = 1;
      MQTT.publish(pub7, "1");
    }
  }
  if (strstr(topic, sub8)) {
    for (unsigned int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
      data += (char)payload[i];
    }
    Serial.println();

    if ((char)payload[0] == '0') {
      digitalWrite(RelayPin8, HIGH);  // Ligua o relé. Note que HIGH é o nível de tensão.
      toggleState_8 = 0;
      MQTT.publish(pub8, "0");
    } else {
      digitalWrite(RelayPin8, LOW);  // Desligua o Relé tornando a tensão BAIXA
      toggleState_8 = 1;
      MQTT.publish(pub8, "1");
    }
  }
}

/* Função: reconecta-se ao broker MQTT (caso ainda não esteja conectado ou em caso de a conexão cair)
em caso de sucesso na conexão ou reconexão, o subscribe dos tópicos é refeito.*/
void reconnectMQTT() {
  while (!MQTT.connected()) {
    Serial.print("* Tentando se conectar ao Broker MQTT: ");
    Serial.println(BROKER_MQTT);
    if (MQTT.connect(ID_MQTT, mqttUserName, mqttPwd)) {
      Serial.println("Conectado com sucesso ao broker MQTT!");
      MQTT.subscribe(sub0);
      MQTT.subscribe(sub1);
      MQTT.subscribe(sub2);
      MQTT.subscribe(sub3);
      MQTT.subscribe(sub4);
      MQTT.subscribe(sub5);
      MQTT.subscribe(sub6);
      MQTT.subscribe(sub7);
      MQTT.subscribe(sub8);
    } else {
      Serial.println("Falha ao reconectar no broker.");
      Serial.print(MQTT.state());
      Serial.println("Haverá nova tentativa de conexão em 2s");
      delay(2000);
    }
  }
}

// Função: reconecta-se ao WiFi
void reconectWiFi() {

  /* Se já está conectado a rede WI-FI, nada é feito.
Caso contrário, são efetuadas tentativas de conexão*/
  if (WiFi.status() == WL_CONNECTED)
    return;

  WiFi.begin(SSID, PASSWORD);  // Conecta na rede WI-FI
  Serial.println("\nConectando WiFi " + String(SSID));

  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Configuração Falhou");
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Conectado com sucesso na rede ");
  Serial.print(SSID);
  Serial.println("");
  Serial.println("WiFi Conectado");
  Serial.print("Endereço de IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Subnet Mask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("Gateway IP: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("DNS 1: ");
  Serial.println(WiFi.dnsIP(0));
  Serial.print("DNS 2: ");
  Serial.println(WiFi.dnsIP(1));
}

/* Função: verifica o estado das conexões WiFI e ao broker MQTT.
Em caso de desconexão (qualquer uma das duas), a conexão  é refeita.*/
void VerificaConexoesWiFIEMQTT(void) {
  if (!MQTT.connected())
    reconnectMQTT();  // se não há conexão com o Broker, a conexão é refeita

  reconectWiFi();  // se não há conexão com o WiFI, a conexão é refeita "apagar essa linha depois pra testar"
}

// Função: inicializa o output em nível lógico baixo
void initOutput(void) {

  pinMode(RelayPin1, OUTPUT);
  pinMode(RelayPin2, OUTPUT);
  pinMode(RelayPin3, OUTPUT);
  pinMode(RelayPin4, OUTPUT);
  pinMode(RelayPin5, OUTPUT);
  pinMode(RelayPin6, OUTPUT);
  pinMode(RelayPin7, OUTPUT);
  pinMode(RelayPin8, OUTPUT);

  // Durante a partida o LED WiFI, inicia desligado
  pinMode(wifiLed, OUTPUT);
  digitalWrite(wifiLed, HIGH);

  // Durante a partida, todos os Relés iniciam desligados
  digitalWrite(RelayPin1, HIGH);
  digitalWrite(RelayPin2, HIGH);
  digitalWrite(RelayPin3, HIGH);
  digitalWrite(RelayPin4, HIGH);
  digitalWrite(RelayPin5, HIGH);
  digitalWrite(RelayPin6, HIGH);
  digitalWrite(RelayPin7, HIGH);
  digitalWrite(RelayPin8, HIGH);
}
// Programa Principal

void loop() {

  unsigned long now = millis();
  if (now - lastMsg > 5000) {

    float temp_data = dht.readTemperature();  // ou dht.readTemperature(true) para Fahrenheit
    dtostrf(temp_data, 4, 2, str_temp_data);
    /* 4 é largura mínima, 2 é precisão; valor flutuante é copiado para str_sensor*/
    float hum_data = dht.readHumidity();
    dtostrf(hum_data, 4, 2, str_hum_data);
    /* 4 é largura mínima, 2 é precisão; valor flutuante é copiado para str_sensor*/
    float tempF_data = dht.readTemperature(true);
    dtostrf(tempF_data, 4, 2, str_tempF_data);
    /* 4 é largura mínima, 2 é precisão; valor flutuante é copiado para str_sensor*/
    float tempterm_data = 0;
    dtostrf(tempterm_data, 4, 2, str_tempterm_data);
    /* 4 é largura mínima, 2 é precisão; valor flutuante é copiado para str_sensor*/
    tempterm_data = dht.computeHeatIndex(tempF_data, hum_data);
    tempterm_data = dht.convertFtoC(tempterm_data);
    dtostrf(tempterm_data, 4, 2, str_tempterm_data);
    /* 4 é largura mínima, 2 é precisão; valor flutuante é copiado para str_sensor*/

    lastMsg = now;

    MQTT.publish(pub9, str_temp_data);

    MQTT.publish(pub10, str_hum_data);

    MQTT.publish(pub11, str_tempterm_data);

    if (digitalRead(RelayPin1) == HIGH) {
      MQTT.publish(pub1, "0");
    } else {
      MQTT.publish(pub1, "1");
    }
    if (digitalRead(RelayPin2) == HIGH) {
      MQTT.publish(pub2, "0");
    } else {
      MQTT.publish(pub2, "1");
    }
    if (digitalRead(RelayPin3) == HIGH) {
      MQTT.publish(pub3, "0");
    } else {
      MQTT.publish(pub3, "1");
    }
    if (digitalRead(RelayPin4) == HIGH) {
      MQTT.publish(pub4, "0");
    } else {
      MQTT.publish(pub4, "1");
    }
    if (digitalRead(RelayPin5) == HIGH) {
      MQTT.publish(pub5, "0");
    } else {
      MQTT.publish(pub5, "1");
    }
    if (digitalRead(RelayPin6) == HIGH) {
      MQTT.publish(pub6, "0");
    } else {
      MQTT.publish(pub6, "1");
    }
    if (digitalRead(RelayPin7) == HIGH) {
      MQTT.publish(pub7, "0");
    } else {
      MQTT.publish(pub7, "1");
    }
    if (digitalRead(RelayPin8) == HIGH) {
      MQTT.publish(pub8, "0");
    } else {
      MQTT.publish(pub8, "1");
    }
    if (status_todos == 1) {
      MQTT.publish(pub0, "1");
    } else {
      MQTT.publish(pub0, "0");
    }
  }

  // Garante funcionamento das conexões WiFi e ao Broker MQTT
  VerificaConexoesWiFIEMQTT();

  //Keep-Alive da comunicação com Broker MQTT
  MQTT.loop();
}
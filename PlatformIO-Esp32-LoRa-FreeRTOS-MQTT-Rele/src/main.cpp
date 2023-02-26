#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Configuração da rede WiFi
const char* ssid = "IoT";
const char* password = "@IoT@@S3nh@S3gur@";

// Configuração do broker MQTT
const char* mqtt_server = "192.168.15.10";
const int mqtt_port = 1883;
const char* mqtt_user = "RobsonBrasil";
const char* mqtt_password = "loboalfa";
const char* mqtt_topic_state = "casa/relé/estado";
const char* mqtt_topic_cmd = "casa/relé/comando";

// Configuração do relé
const int relay_pin = 17; // Pino digital do relé

// Variáveis globais
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
int relay_state = LOW; // Inicializa o relé desligado

// Protótipos das funções
void task1(void *pvParameters);
void task2(void *pvParameters);
void connectToWiFi();
void connectToMQTT();

void setup() {
  Serial.begin(115200);

  // Configura o pino do relé como saída
  pinMode(relay_pin, OUTPUT);
  digitalWrite(relay_pin, relay_state);

  // Inicializa a conexão com a WiFi
  connectToWiFi();

  // Inicializa a conexão com o broker MQTT
  connectToMQTT();

  // Cria as tarefas
  xTaskCreatePinnedToCore(
    task1,          // Função da tarefa 1
    "Task1",        // Nome da tarefa 1
    10000,          // Tamanho da pilha da tarefa 1
    NULL,           // Parâmetro da tarefa 1
    1,              // Prioridade da tarefa 1
    NULL,           // Handle da tarefa 1
    0               // Núcleo 0
  );
  xTaskCreatePinnedToCore(
    task2,          // Função da tarefa 2
    "Task2",        // Nome da tarefa 2
    10000,          // Tamanho da pilha da tarefa 2
    NULL,           // Parâmetro da tarefa 2
    1,              // Prioridade da tarefa 2
    NULL,           // Handle da tarefa 2
    1               // Núcleo 1
  );
}

void loop() {
  // Vazio, todas as operações são realizadas nas tarefas
}

void task1(void *pvParameters) {
  while(1) {
    // Verifica se a conexão com o broker MQTT está ativa
    if(!mqttClient.connected()) {
      connectToMQTT();
    }

    // Verifica se há mensagens MQTT para serem processadas
    mqttClient.loop();

    // Espera um segundo antes de verificar novamente
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void task2(void *pvParameters) {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // inicialmente, desliga o relé
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) {
    Serial.println("Conectando ao broker MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("Conectado com sucesso!");
      client.subscribe("rele");
    } else {
      Serial.print("Falha na conexao - Estado: ");
      Serial.print(client.state());
      delay(2000);
    }
  }
  Serial.println("Tarefa 2 iniciada.");
  while (1) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      Serial.println("Botao pressionado!");
      digitalWrite(RELAY_PIN, LOW);
      client.publish("rele", "LIGADO");
      vTaskDelay(5000 / portTICK_PERIOD_MS); // aguarda 5 segundos
      digitalWrite(RELAY_PIN, HIGH);
      client.publish("rele", "DESLIGADO");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(9600);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando ao WiFi...");
  }
  Serial.println("Conectado na rede WiFi!");

  xTaskCreatePinnedToCore(
    task1,    // função da tarefa
    "task1",  // nome da tarefa
    10000,    // tamanho da pilha da tarefa
    NULL,     // parâmetro da tarefa
    1,        // prioridade da tarefa
    NULL,     // referência para a task handle
    0);       // núcleo da CPU a ser utilizado

  xTaskCreatePinnedToCore(
    task2,    // função da tarefa
    "task2",  // nome da tarefa
    10000,    // tamanho da pilha da tarefa
    NULL,     // parâmetro da tarefa
    1,        // prioridade da tarefa
    NULL,     // referência para a task handle
    1);       // núcleo da CPU a ser utilizado
}

void loop() {
  vTaskDelete(NULL);
}

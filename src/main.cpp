#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ModbusRTUSlave.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Definições e Variáveis Globais
#define CHANNEL 0 // Canal ESP-NOW
#define NUM_MOTORES 4

//MAC Address do controle remoto
uint8_t remoteMac[] = {0x24, 0x0A, 0xC4, 0x0A, 0x0B, 0x6C};

//MAC Address do leitor de sensores 08:D1:F9:27:B2:54
uint8_t sensorMac[] = {0x81, 0xD1, 0xF9, 0x27, 0xB2, 0x54};


const char *ssid = "Moto G (5) Plus 3746";
const char *senha = "velorio_";
WiFiClient espClient;
boolean conectado = false;
unsigned long tempo = 0;
const int intervalo = 5000; 

//modbus
ModbusRTUSlave modbus(Serial2);
uint16_t holdingRegisters[6] = {0, 0, 0, 0, 0, 0};
bool coils[2];
bool discreteInputs[2];
const uint8_t coilPins[2] = {4, 5};
const uint8_t discreteInputPins[2] = {2, 3};

// Pinos dos motores e freios
const int pinosMotores[NUM_MOTORES] = {23, 22, 21, 19};
const int pinosFreios[NUM_MOTORES] = {2, 4, 5, 18};
const int pinosSentidos[NUM_MOTORES] = {13, 12, 14, 27};
const int manta = 26;

// Variáveis de controle dos motores
bool motor1_ok = false;
bool motor2_ok = false;
bool motor3_ok = false;
bool motor4_ok = false;

// ESP-NOW
typedef struct struct_message {
  bool sensorStatus[18]; // Status ON/OFF de 18 sensores
} struct_message;

struct_message receivedData; // Struct para armazenar os dados recebidos

// Arquivos e JSON
String index_html;
String arquivoConf;
DynamicJsonDocument doc(1024);

// Servidor Web
AsyncWebServer server(80);

// Declaração de Funções
void setupPinos();
void acionaMotor(int motor, int velocidade);
void setupServer();
void handleConfigJson(AsyncWebServerRequest *request);
void saveJson();
void setupWiFi();
void scanWiFiNetworks(AsyncWebServerRequest *request);
void handleWiFiStatus(AsyncWebServerRequest *request);
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len); // ESP-NOW callback

void setup() {
  // Configuração dos pinos dos motores e freios
  setupPinos();
  
  // Inicializa a comunicação serial
  Serial.begin(115200);
  Serial.println("Eclusa Atuadores V2.0");

  // Configura Wi-Fi em modo Station (STA)
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, senha);

  unsigned long startAttemptTime = millis();
  
  // Tentativa de conexão Wi-Fi por 10 segundos
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
    Serial.print(".");
  }

  // Verifica se o Wi-Fi foi conectado ou não
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Conectado ao Wi-Fi");
    int wifiChannel = WiFi.channel();
    Serial.print("Wi-Fi conectado no canal: ");
    Serial.println(wifiChannel);
    
    // Configura o canal do ESP-NOW para o mesmo do Wi-Fi
    esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
    Serial.print("ESP-NOW configurado para o canal: ");
    Serial.println(wifiChannel);
  } else {
    Serial.println("Wi-Fi não conectado. Configurando ESP-NOW sem Wi-Fi.");
    
    // Configura ESP-NOW para funcionar sem Wi-Fi (somente ESP-NOW)
    WiFi.disconnect();  // Desconecta de qualquer tentativa de conexão Wi-Fi
    esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE); // Usa um canal fixo para ESP-NOW
  }

  // Inicializa ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao inicializar ESP-NOW");
    return;
  }

  // Registra callback para quando os dados são recebidos via ESP-NOW
  esp_now_register_recv_cb(OnDataRecv);

  // Inicializa o LittleFS para carregar o arquivo HTML e o arquivo JSON
  if (!LittleFS.begin()) {
    Serial.println("Erro ao montar LittleFS");
    return;
  }

  // Carrega o arquivo index.html
  File file = LittleFS.open("/index.html", "r");
  if (!file) {
    Serial.println("Erro ao abrir index.html");
    return;
  }
  index_html = file.readString();
  file.close();

  // Carrega o arquivo de configuração JSON
  File configFile = LittleFS.open("/config.json", "r");
  arquivoConf = configFile.readString();
  deserializeJson(doc, arquivoConf);
  JsonObject configs = doc.as<JsonObject>();

  if (configs.isNull()) {
    Serial.println("Falha ao carregar configurações");
    return;
  }

  // Atualiza o SSID e a senha de acordo com o arquivo JSON
  ssid = configs["wifiSSID"];
  senha = configs["wifiPassword"];
  configFile.close();

  // Inicializa Modbus
  modbus.configureCoils(coils, 2);
  modbus.configureDiscreteInputs(discreteInputs, 2);
  modbus.configureHoldingRegisters(holdingRegisters, 6);
  modbus.begin(1, 115200);

  // Inicializa o servidor web
  setupServer();
  server.begin();

  Serial.println("Servidor iniciado");
}

void loop() {
  // Se o Wi-Fi for desconectado, tenta reconectar a cada intervalo definido
  if (WiFi.status() != WL_CONNECTED && millis() - tempo > intervalo) {
    setupWiFi();
    tempo = millis();
  }

  // Processo do Modbus
  modbus.poll();
  holdingRegisters[2] = random(0, 100); // Atualiza um valor aleatório para teste
}


// Função para mudar o sentido dos motores 
void mudaSentido(int motor, int sentido) {
  if (motor >= 1 && motor <= NUM_MOTORES) {
    if (motor == 1) {
      digitalWrite(pinosSentidos[0], sentido);
    }
    if (motor == 2) {
      digitalWrite(pinosSentidos[1], sentido);
    }
    if (motor == 3) {
      digitalWrite(pinosSentidos[2], sentido);
    }
    if (motor == 4) {
      digitalWrite(pinosSentidos[3], sentido);
    }
  }
}

// Função para acionar motores e freios
void acionaMotor(int motor, int velocidade) {
  if (motor >= 1 && motor <= NUM_MOTORES) {
    if (motor == 1 && motor1_ok) {
      int idx = motor - 1;
      digitalWrite(pinosFreios[idx], HIGH);
      digitalWrite(pinosSentidos[idx], velocidade < 0 ? HIGH : LOW);
      ledcWrite(idx, velocidade);
    }
    if (motor == 2 && motor2_ok) {
      int idx = motor - 1;
      digitalWrite(pinosFreios[idx], HIGH);
      digitalWrite(pinosSentidos[idx], velocidade < 0 ? HIGH : LOW);
      ledcWrite(idx, velocidade);
    }
    if (motor == 3 && motor3_ok) {
      int idx = motor - 1;
      digitalWrite(pinosFreios[idx], HIGH);
      digitalWrite(pinosSentidos[idx], velocidade < 0 ? HIGH : LOW);
      ledcWrite(idx, velocidade);
    }
    if (motor == 4 && motor4_ok) {
      int idx = motor - 1;
      digitalWrite(pinosFreios[idx], HIGH);
      digitalWrite(pinosSentidos[idx], velocidade < 0 ? HIGH : LOW);
      ledcWrite(idx, velocidade);
    }

  }
}

// Configura todos os pinos
void setupPinos() {
  for (int i = 0; i < NUM_MOTORES; i++) {
    pinMode(pinosMotores[i], OUTPUT);
    pinMode(pinosFreios[i], OUTPUT);
    ledcSetup(i, 300, 8);
    ledcAttachPin(pinosMotores[i], i);
    ledcWrite(i, 0);
  }
  pinMode(manta, OUTPUT);
}

// Função de callback para quando os dados forem recebidos via ESP-NOW
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {

  memcpy(&receivedData, incomingData, sizeof(receivedData)); // Copia os dados recebidos para a struct
  Serial.print("Dados recebidos de: ");
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println(macStr);
  if (receivedData.sensorStatus[17] == false){
  ledcWrite(0, 0);
  digitalWrite(pinosFreios[0], LOW);
  Serial.println("Critério de parada acionado");
  }
// Verifica o status dos sensores e aciona os motores e freios de acordo
  if (memcmp(mac, sensorMac, 6) == 0){
    for ( int i = 0; i < 18; i++)
    {
      switch (receivedData.sensorStatus[i]){
        case true:
          if (i == 1 || i == 2 || i == 3 || i == 4){
            motor1_ok = true;
          }
          if (i == 5 || i == 6 || i == 7 || i == 8){
            motor2_ok = true;
          }
          if (i == 9 || i == 10 || i == 11 || i == 12){
            motor3_ok = true;
          }
          if (i == 13 || i == 14 || i == 15 || i == 16){
            motor4_ok = true;
          }
        case false:
          if (i == 1 || i == 2 || i == 3 || i == 4){
            ledcWrite(0, 0);
            digitalWrite(pinosFreios[0], LOW);
            Serial.println("Critério de parada acionado");
            motor1_ok = false;
          }
          if (i == 5 || i == 6 || i == 7 || i == 8){
            ledcWrite(1, 0);
            digitalWrite(pinosFreios[1], LOW);
            Serial.println("Critério de parada acionado");
          }
          if (i == 9 || i == 10 || i == 11 || i == 12){
            ledcWrite(2, 0);
            digitalWrite(pinosFreios[2], LOW);
            Serial.println("Critério de parada acionado");
          }
          if (i == 13 || i == 14 || i == 15 || i == 16){
            ledcWrite(3, 0);
            digitalWrite(pinosFreios[3], LOW);
            Serial.println("Critério de parada acionado");
          }
        }
      }
    }

  if (memcmp(mac, remoteMac, 6) == 0){

  }
}
// Salva as configurações JSON
void saveJson() {
  File configFile = LittleFS.open("/config.json", "w");
  StaticJsonDocument<200> doc;
  doc["wifiSSID"] = ssid;
  doc["wifiPassword"] = senha;
  String json;
  serializeJson(doc, json);
  configFile.print(json);
  configFile.close();
}

// Manipula a requisição para retornar as configurações em JSON
void handleConfigJson(AsyncWebServerRequest *request) {
  // Cria um objeto JSON
  StaticJsonDocument<200> doc;
  // Define os valores no JSON
  doc["wifiSSID"] = ssid;
  doc["wifiPassword"] = senha;
  // Converte o JSON em uma string
  String json;
  serializeJson(doc, json);
  // Envia a resposta com o tipo de conteúdo JSON
  request->send(200, "application/json", json);
}

// Configuração do servidor web
void setupServer() {
  // Página inicial
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });

  // Configurações de Wi-Fi
  server.on("/WIFI", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/WIFI.html", "text/html");
  });

  // Rota para salvar configurações via JSON
  server.addHandler(new AsyncCallbackJsonWebHandler("/save-config", [](AsyncWebServerRequest *request, JsonVariant &json) {
    JsonObject jsonObj = json.as<JsonObject>();
    ssid = jsonObj["wifiSSID"];
    senha = jsonObj["wifiPassword"];
    saveJson();
    setupWiFi();
    request->send(200, "text/plain", "Configurações salvas");
  }));

  // Controle de motores
  server.on("/control-motor", HTTP_GET, [](AsyncWebServerRequest *request) {
    String motor, pwm;
    if (request->hasParam("motor") && request->hasParam("pwm")) {
      motor = request->getParam("motor")->value();
      pwm = request->getParam("pwm")->value();
      acionaMotor(motor.toInt(), pwm.toInt());
      request->send(200, "text/plain", "Motor " + motor + " acionado com PWM " + pwm);
    } else {
      request->send(400, "text/plain", "Parâmetros inválidos");
    }
  });

  // Controle de relés
  server.on("/relay", HTTP_GET, [](AsyncWebServerRequest *request) {
    String relayNumber, action;
    if (request->hasParam("relay") && request->hasParam("action")) {
      relayNumber = request->getParam("relay")->value();
      action = request->getParam("action")->value();
      int relay = relayNumber.toInt();
      digitalWrite(relay, (action == "ON") ? HIGH : LOW);
      request->send(200, "text/plain", "Relé " + relayNumber + " " + action);
    } else {
      request->send(400, "text/plain", "Parâmetros inválidos");
    }
  });

  // Outras rotas
  server.on("/config.json", HTTP_GET, handleConfigJson);
  server.on("/scan", HTTP_GET, scanWiFiNetworks);
  server.on("/wifi-status", HTTP_GET, handleWiFiStatus);
}

// Configuração do WiFi
void setupWiFi() {
  if (!conectado) {
    WiFi.softAP("PDCA_" + WiFi.macAddress(), "manut_aut");
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, senha);
      // Aguarda a conexão ao Wi-Fi
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("Conectado ao Wi-Fi");

      // Obtenha o canal do Wi-Fi
      int wifiChannel = WiFi.channel();
      Serial.print("Wi-Fi conectado no canal: ");
      Serial.println(wifiChannel);

      // Configura o ESP-NOW para usar o mesmo canal do Wi-Fi
      esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
      Serial.print("ESP-NOW configurado para o canal: ");
      Serial.println(wifiChannel);
    }
  } else {
    WiFi.disconnect();
    WiFi.begin(ssid, senha);
  }
}

// Escaneia as redes WiFi disponíveis
void scanWiFiNetworks(AsyncWebServerRequest *request) {
  int n = WiFi.scanComplete();
  if (n == -2) {
    WiFi.scanNetworks(true);
    request->send(200, "text/plain", "Nenhuma rede encontrada");
  } else if (n) {
    String networks = "";
    for (int i = 0; i < n; ++i) {
      networks += WiFi.SSID(i) + "\n";
    }
    WiFi.scanDelete();
    if(WiFi.scanComplete()==-2){
      WiFi.scanNetworks(true);
      } 
    request->send(200, "text/plain", networks);
  } else {
    request->send(200, "text/plain", "Nenhuma rede encontrada");
  }
}

// Verifica o status do WiFi
void handleWiFiStatus(AsyncWebServerRequest *request) {
  if (WiFi.status() == WL_CONNECTED) {
    request->send(200, "text/html", "<p style='color: green;'>Conectado ao Wi-Fi: " + String(WiFi.SSID()) + "</p>");
  } else {
    request->send(200, "text/html", "<p style='color: red;'>Não conectado ao Wi-Fi.</p>");
  }
}

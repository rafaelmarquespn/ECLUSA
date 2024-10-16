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
#include <string.h> 

// Definições e Variáveis Globais
#define CHANNEL 0 // Canal ESP-NOW
#define NUM_MOTORES 4
//MAC Address do controle remoto
char remoteMac[18] = "E8:6B:EA:D0:C3:88";
//MAC Address do leitor de sensores 08:D1:F9:27:B2:54
char sensorMac[18] = "08:D1:F9:27:B2:54";
// Wi-Fi
const char *ssid = "Moto G (5) Plus 3746";
const char *senha = "velorio_";
WiFiClient espClient;
boolean conectado = false;
unsigned long tempo = 0;
const int intervalo = 1000; 
int attemptCounter = 0;
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
typedef struct struct_remote_control {
  int motorID;      // ID do motor (1-4)
  int velocidade;   // PWM para controle de velocidade (0 a 255)
  int sentido;
  int brake;      // Sentido do motor (0: Horário, 1: Anti-horário)
} struct_remote_control;
struct_remote_control remoteControl; // Struct para armazenar os dados do controle remoto
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
void pararMotorEFreio(int motorIndex);
void mudaSentido(int motor, int sentido);


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
    delay(50);
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
  // Se ainda não atingiu 20 tentativas
  if (attemptCounter < 20) {
    // Se o Wi-Fi estiver desconectado e for o momento de tentar reconectar
    if (WiFi.status() != WL_CONNECTED && millis() - tempo > intervalo) {
      setupWiFi();
      tempo = millis();
      attemptCounter++;  // Incrementa o contador de tentativas
    }
    Serial.println(attemptCounter);
  }

  // Processo do Modbus
  modbus.poll();
  holdingRegisters[2] = random(0, 100); // Atualiza um valor aleatório para teste
}

// Função de callback para quando os dados forem recebidos via ESP-NOW
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Serial.print("Dados recebidos de: ");
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  // Serial.println(macStr);
 // Verifica o status dos sensores apenas se o MAC recebido for de sensor
  if ((strcmp(macStr, sensorMac) == 0)) {
    memcpy(&receivedData, incomingData, sizeof(receivedData)); // Copia os dados recebidos para a struct
    // Verifica se o último sensor aciona o critério de parada
    // Serial.println("Status dos sensores:");
    for (int i = 0; i < 18; i++) {
      bool statusSensor = receivedData.sensorStatus[i];
      // Serial.println("Sensor " + String(i) + ": " + String(statusSensor));
      // Definição de motor e freio associados a cada grupo de sensores
      int motorIndex = (i / 4); // 4 sensores por motor (sensores 1-4 -> motor 0, 5-8 -> motor 1, etc.)

      if (statusSensor == 1) {
        //Ativar motor correspondente
        switch (motorIndex) {
          case 0: motor1_ok = true; break;
          case 1: motor2_ok = true; break;
          case 2: motor3_ok = true; break;
          case 3: motor4_ok = true; break;
        }
      } else {
        // Parar motor e ativar freio correspondente
        pararMotorEFreio(motorIndex);
        Serial.println("Parando motor " + String(motorIndex) + " sensor " + String(i) + ": " + String(statusSensor) );
        switch (motorIndex) {
          case 0: motor1_ok = false; break;
          case 1: motor2_ok = false; break;
          case 2: motor3_ok = false; break;
          case 3: motor4_ok = false; break;
        }
      }
    }
  }

  // Verifique se o MAC é do dispositivo remoto
  if ((strcmp(macStr, remoteMac) == 0)) {
    // Ações específicas para o dispositivo remoto, se necessário
    Serial.println("Dados recebidos do controle remoto");
    memcpy(&remoteControl, incomingData, sizeof(remoteControl)); // Copia os dados recebidos para a struct
    Serial.println(remoteControl.motorID);
    Serial.println(remoteControl.velocidade);
    Serial.println(remoteControl.sentido);
    if (remoteControl.motorID >= 1 && remoteControl.motorID <= NUM_MOTORES) {
      Serial.println("Acionando motor");
      mudaSentido(remoteControl.motorID, remoteControl.sentido);
      acionaMotor(remoteControl.motorID, remoteControl.velocidade);
    if(remoteControl.brake == 1){
      int motorIndex = remoteControl.motorID - 1;
      pararMotorEFreio(motorIndex);

    }
    }
  }
}

// Função para mudar o sentido dos motores
void mudaSentido(int motor, int sentido) {
  if (motor >= 1 && motor <= NUM_MOTORES && sentido == 0) {
    int pino = pinosSentidos[motor - 1]; // Ajusta o índice do array
    digitalWrite(pino, LOW);
    Serial.println("Mudando sentido do motor " + String(motor) + " (pino " + String(pino) + ") para " + String(sentido));  

  } else if (motor >= 1 && motor <= NUM_MOTORES && sentido == 1) {
    int pino = pinosSentidos[motor -1]; // Ajusta o índice do array
    digitalWrite(pino, HIGH);
    Serial.println("Mudando sentido do motor " + String(motor) + " (pino " + String(pino) + ") para " + String(sentido));
  }
}

// Função para acionar motores e freios
void acionaMotor(int motor, int velocidade) {
  if (motor >= 1 && motor <= NUM_MOTORES) {
    if (motor == 1 && motor1_ok) {
      int idx = motor - 1;
      digitalWrite(pinosFreios[idx], HIGH);
      ledcWrite(idx, velocidade);
    }
    if (motor == 2 && motor2_ok) {
      int idx = motor - 1;
      digitalWrite(pinosFreios[idx], HIGH);
      ledcWrite(idx, velocidade);
    }
    if (motor == 3 && motor3_ok) {
      int idx = motor - 1;
      digitalWrite(pinosFreios[idx], HIGH);
      ledcWrite(idx, velocidade);
    }
    if (motor == 4 && motor4_ok) {
      int idx = motor - 1;
      digitalWrite(pinosFreios[idx], HIGH);
      ledcWrite(idx, velocidade);
    }

  }
}

// Função para parar o motor e acionar o freio
void pararMotorEFreio(int motorIndex) {
  ledcWrite(motorIndex, 0);
  digitalWrite(pinosFreios[motorIndex], LOW);
  // Serial.print("Critério de parada acionado para motor ");
  // Serial.println(motorIndex + 1);
}

// Configura todos os pinos
void setupPinos() {
  for (int i = 0; i < NUM_MOTORES; i++) {
    pinMode(pinosMotores[i], OUTPUT);
    pinMode(pinosFreios[i], OUTPUT);
    pinMode(pinosSentidos[i], OUTPUT);
    ledcSetup(i, 300, 8);
    ledcAttachPin(pinosMotores[i], i);
    ledcWrite(i, 0);
  }
  pinMode(manta, OUTPUT);
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
        delay(50);
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


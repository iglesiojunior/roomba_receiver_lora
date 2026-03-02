#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <LoRa.h>
#include "SSD1306Wire.h" 

// ==========================================
// CONFIGURAÇÃO DE HARDWARE (Confirmada)
// ==========================================
// Pinos do Roomba (Ligação Física)
#define ROOMBA_RX_PIN   3  // Fio LARANJA (TXD do Roomba) -> Entra no 23
#define ROOMBA_TX_PIN   1  // Fio AZUL    (RXD do Roomba) -> Sai do 17
#define ROOMBA_WAKE_PIN 12  // Fio VERDE   (DD do Roomba)  -> Pino 12 (Confirmado)

// Pinos do Heltec V2 (Fixos)
#define SCK_PIN     5
#define MISO_PIN    19
#define MOSI_PIN    27
#define SS_PIN      18
#define RST_PIN     14
#define DIO0_PIN    26
#define RST_OLED    16
#define SDA_OLED    4
#define SCL_OLED    15
#define LED_PIN     25

#define BAND 915E6
#define MY_NODE_ID 1

// Objetos
SSD1306Wire display(0x3c, SDA_OLED, SCL_OLED);
HardwareSerial SerialRoomba(1);
QueueHandle_t roombaQueue;

// ==========================================
// ESTRUTURA DE DADOS (PACKED - CRUCIAL)
// ==========================================
#define clamp(value, min, max) (value < min ? min : value > max ? max : value)

struct PayloadCmdVel { float linear_x; float angular_z; };
struct PayloadLed { int32_t color_code; int32_t blink_rate; };
enum MsgType { MSG_CMD_VEL = 0xA0 };

// O atributo "packed" evita o erro de NaN nos floats
typedef struct {
  uint8_t target_id;
  uint8_t msg_type;
  union {
    PayloadCmdVel cmd;
    PayloadLed    led;
  } data;
  uint16_t checksum;
} __attribute__((packed)) LoRaPacket;

// Variáveis de Controle
unsigned long lastCmdTime = 0;

// ==========================================
// COMANDOS ROOMBA
// ==========================================
void wakeUp() {
  pinMode(ROOMBA_WAKE_PIN, OUTPUT);
  digitalWrite(ROOMBA_WAKE_PIN, HIGH); delay(100);
  digitalWrite(ROOMBA_WAKE_PIN, LOW);  delay(500); // Acorda
  digitalWrite(ROOMBA_WAKE_PIN, HIGH); delay(2000);
}

void startSafe() { 
  SerialRoomba.write(128); // START
  delay(50);
  SerialRoomba.write(131); // SAFE MODE (Permite controle mas para se erguer do chão)
  delay(50);
}

void drive(int velocity, int radius) {
  clamp(velocity, -500, 500);
  clamp(radius, -2000, 2000);
  
  SerialRoomba.write(137); // Opcode Drive
  SerialRoomba.write((velocity >> 8) & 0xFF); 
  SerialRoomba.write(velocity & 0xFF);
  SerialRoomba.write((radius >> 8) & 0xFF);   
  SerialRoomba.write(radius & 0xFF);
}

void driveStop() { drive(0, 0); }

// Toca 2 notas para avisar que está pronto
void beepReady() {
  SerialRoomba.write(140); SerialRoomba.write(1); SerialRoomba.write(2);
  SerialRoomba.write(72); SerialRoomba.write(16);
  SerialRoomba.write(76); SerialRoomba.write(16);
  delay(50);
  SerialRoomba.write(141); SerialRoomba.write(1);
}

// ==========================================
// TASK LORA (Recebe Rádio)
// ==========================================
void TaskLoRaReader(void *pvParameters) {
  LoRaPacket packet;
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  if (!LoRa.begin(BAND)) vTaskDelete(NULL);

  for (;;) {
    int packetSize = LoRa.parsePacket();
    // Só aceita pacote do tamanho exato (filtra ruído)
    if (packetSize == sizeof(LoRaPacket)) { 
      LoRa.readBytes((uint8_t*)&packet, sizeof(LoRaPacket));
      
      if ((packet.target_id == MY_NODE_ID || packet.target_id == 255) && 
          packet.msg_type == MSG_CMD_VEL) {
        xQueueSend(roombaQueue, &packet, 0); 
        digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Pisca LED da placa
      }
    } else if (packetSize > 0) {
      while(LoRa.available()) LoRa.read(); // Limpa buffer se for lixo
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ==========================================
// TASK ROOMBA (Move o Robô)
// ==========================================
void TaskRoombaDrive(void *pvParameters) {
  LoRaPacket cmd;
  
  // INICIALIZAÇÃO SERIAL (Baud 115200 CONFIRMADO)
  SerialRoomba.begin(115200, SERIAL_8N1, ROOMBA_RX_PIN, ROOMBA_TX_PIN);
  
  wakeUp();
  startSafe();
  delay(500);
  beepReady(); // Se ouvir isso, o código subiu e o robô acordou!

  for (;;) {
    // Verifica se chegou comando novo na fila
    if (xQueueReceive(roombaQueue, &cmd, 100 / portTICK_PERIOD_MS) == pdTRUE) {
      
      lastCmdTime = millis(); // Atualiza tempo do último comando
      
      float v = cmd.data.cmd.linear_x;
      float w = cmd.data.cmd.angular_z;

      // ATUALIZA DISPLAY
      display.clear();
      display.setFont(ArialMT_Plain_10); display.drawString(0,0, "LORA CONECTADO");
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 20, "V: " + String(v));
      display.drawString(0, 40, "W: " + String(w));
      display.display();

      // LÓGICA DE MOVIMENTO
      // Se apenas Linear (Frente/Trás)
      if (abs(w) < 0.05 && abs(v) > 0.05) {
        drive(v * 500, 32768); // 32768 = Reto (Straight) no protocolo Roomba
      }
      // Se apenas Angular (Girar no próprio eixo)
      else if (abs(v) < 0.05 && abs(w) > 0.05) {
        drive(w * 250, (w > 0) ? 1 : -1); // 1 = Anti-horário, -1 = Horário
      }
      // Movimento Curvo ou Parado
      else {
        // Conversão simples para teste (pode refinar depois)
        int vel_mm = v * 500;
        int rad_mm = (abs(w) > 0.01) ? (500 / w) : 32768; 
        drive(vel_mm, rad_mm);
      }
      
    } else {
      // SEGURANÇA (Watchdog): Se não chegar comando por 1 segundo, PARA!
      if (millis() - lastCmdTime > 1000) {
         driveStop();
         display.clear(); display.drawString(0,0,"SEM SINAL"); display.display();
      }
    }
  }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);

  // Reset OLED
  pinMode(RST_OLED, OUTPUT);
  digitalWrite(RST_OLED, LOW); delay(20);
  digitalWrite(RST_OLED, HIGH); delay(20);
  display.init(); display.flipScreenVertically();
  
  roombaQueue = xQueueCreate(5, sizeof(LoRaPacket));

  xTaskCreatePinnedToCore(TaskLoRaReader, "LoRa", 4096, NULL, 0, NULL, 0); // Core 0
  xTaskCreatePinnedToCore(TaskRoombaDrive, "Roomba", 4096, NULL, 1, NULL, 1); // Core 1
}

void loop() { vTaskDelete(NULL); }
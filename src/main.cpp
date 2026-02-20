#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <LoRa.h>
#include "SSD1306Wire.h" // <--- Biblioteca do Display

// --- HARDWARE ---
#define SCK_PIN 5
#define MISO_PIN 19
#define MOSI_PIN 27
#define SS_PIN 18
#define RST_PIN 14
#define DIO0_PIN 26
#define BAND 915E6
#define MY_NODE_ID 1

// Display OLED (Heltec V2)
#define RST_OLED 16
SSD1306Wire display(0x3c, 4, 15);

// Hardware Roomba
HardwareSerial SerialRoomba(1);
#define LED_PIN 25 // O LED da Heltec V2 geralmente é o 25

// Macros Originais
#define clamp(value, min, max) (value < min ? min : value > max ? max : value)

// Estrutura de Dados
enum MsgType { MSG_CMD_VEL = 0xA0 };
struct PayloadCmdVel { float linear_x; float angular_z; };

typedef struct {
  uint8_t target_id; uint8_t msg_type;
  union { PayloadCmdVel cmd; } data;
  uint16_t checksum; 
} LoRaPacket;

// FILA DO RTOS
QueueHandle_t roombaQueue;

// ============================================================
// SUAS FUNÇÕES ORIGINAIS (SEM ALTERAÇÃO)
// ============================================================
void drive(int velocity, int radius) {
  clamp(velocity, -500, 500);
  clamp(radius, -2000, 2000);
  SerialRoomba.write(137);
  SerialRoomba.write(velocity >> 8); SerialRoomba.write(velocity);
  SerialRoomba.write(radius >> 8);   SerialRoomba.write(radius);
}
void driveStop() { drive(0,0); }

void wakeUp() {
  digitalWrite(5, HIGH); delay(100);
  digitalWrite(5, LOW);  delay(500);
  digitalWrite(5, HIGH); delay(2000);
}
void startSafe() { SerialRoomba.write(128); SerialRoomba.write(131); delay(1000); }


// ============================================================
// TASK 1: RECEPTOR LORA (Alta Prioridade - Núcleo 0)
// ============================================================
void TaskLoRaReader(void *pvParameters) {
  LoRaPacket packet;

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  if (!LoRa.begin(BAND)) vTaskDelete(NULL); 

  for (;;) {
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) { 
      LoRa.readBytes((uint8_t*)&packet, sizeof(LoRaPacket));

      // Filtra ID
      if ((packet.target_id == MY_NODE_ID || packet.target_id == 255) && 
          packet.msg_type == MSG_CMD_VEL) {
        
        // Joga na fila
        xQueueSend(roombaQueue, &packet, 0); 
        digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink LED físico
      }
    }
    vTaskDelay(2); 
  }
}

// ============================================================
// TASK 2: CONTROLADOR ROOMBA + DISPLAY (Prioridade Média - Núcleo 1)
// ============================================================
void TaskRoombaDrive(void *pvParameters) {
  LoRaPacket cmd;
  
  // Setup Roomba
  SerialRoomba.begin(115200, SERIAL_8N1, 16, 17);
  pinMode(5, OUTPUT);
  wakeUp();
  startSafe();

  for (;;) {
    // Espera chegar comando na fila
    if (xQueueReceive(roombaQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      
      float v = cmd.data.cmd.linear_x;
      float w = cmd.data.cmd.angular_z;

      // --- ATUALIZA DISPLAY (Feedback Visual) ---
      display.clear();
      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 0, "RX LORA -> ROOMBA");
      
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 20, "V: " + String(v));
      display.drawString(0, 40, "W: " + String(w));
      display.display();

      // --- EXECUTA LÓGICA DO ROBÔ ---
      drive(v * 100, w * 400);
      
      // Delay não-bloqueante do RTOS
      vTaskDelay(50 / portTICK_PERIOD_MS); 
      
      driveStop();
    }
  }
}

// ============================================================
// SETUP PRINCIPAL
// ============================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);

  // Inicializa Display
  pinMode(RST_OLED, OUTPUT);
  digitalWrite(RST_OLED, LOW); delay(20);
  digitalWrite(RST_OLED, HIGH); delay(20);
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "AGUARDANDO LORA...");
  display.display();

  // Cria Fila e Tasks
  roombaQueue = xQueueCreate(5, sizeof(LoRaPacket));

  xTaskCreatePinnedToCore(TaskLoRaReader, "LoRa", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskRoombaDrive, "Roomba", 4096, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelete(NULL); 
}
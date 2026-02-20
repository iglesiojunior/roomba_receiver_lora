#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <LoRa.h>

// --- HARDWARE E PROTOCOLO ---
#define SCK_PIN 5
#define MISO_PIN 19
#define MOSI_PIN 27
#define SS_PIN 18
#define RST_PIN 14
#define DIO0_PIN 26
#define BAND 915E6
#define MY_NODE_ID 1

// Hardware Roomba
HardwareSerial SerialRoomba(1);
#define LED_PIN 2

// Macros Originais
#define clamp(value, min, max) (value < min ? min : value > max ? max : value)

// Estrutura de Dados
enum MsgType { MSG_CMD_VEL = 0xA0 };
struct PayloadCmdVel { float linear_x; float angular_z; };
typedef struct {
  uint8_t target_id; uint8_t msg_type;
  union { PayloadCmdVel cmd; } data;
  uint16_t checksum; // (Não estamos validando checksum aqui pra simplificar, mas devia)
} LoRaPacket; // Sem packed aqui para facilitar a Fila

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
// Nunca dorme, nunca bloqueia.
// ============================================================
void TaskLoRaReader(void *pvParameters) {
  LoRaPacket packet;

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  if (!LoRa.begin(BAND)) vTaskDelete(NULL); // Mata task se falhar

  for (;;) {
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) { // Chegou algo!
      // Lê para a variável local
      LoRa.readBytes((uint8_t*)&packet, sizeof(LoRaPacket));

      // Filtra ID
      if ((packet.target_id  MY_NODE_ID || packet.target_id  255) && 
          packet.msg_type == MSG_CMD_VEL) {
        
        // JOGA NA FILA E VOLTA A OUVIR IMEDIATAMENTE
        // timeout 0 = se a fila tiver cheia, descarta o pacote velho
        xQueueSend(roombaQueue, &packet, 0); 
        
        digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink rápido
      }
    }
    vTaskDelay(2); // Cede 2ms para o Watchdog do sistema
  }
}

// ============================================================
// TASK 2: CONTROLADOR ROOMBA (Prioridade Média - Núcleo 1)
// Aqui roda sua lógica "lenta" com delays.
// ============================================================
void TaskRoombaDrive(void *pvParameters) {
  LoRaPacket cmd;
  
  // Setup do Roomba fica aqui dentro agora (contexto da task)
  SerialRoomba.begin(115200, SERIAL_8N1, 16, 17);
  pinMode(5, OUTPUT);
  wakeUp();
  startSafe();

  for (;;) {
    // Fica bloqueado esperando chegar algo na fila. Não gasta CPU.
    if (xQueueReceive(roombaQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      
      float v = cmd.data.cmd.linear_x;
      float w = cmd.data.cmd.angular_z;

      // --- SUA LÓGICA ORIGINAL ---
      drive(v  100, w  400);
      
      // Esse delay agora é vTaskDelay (Não-Bloqueante para o sistema)
      // O Rádio continua funcionando enquanto isso aqui espera!
      vTaskDelay(50 / portTICK_PERIOD_MS); 
      
      driveStop();
      // ---------------------------
    }
  }
}

// ============================================================
// SETUP PRINCIPAL
// ============================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);

  // Cria a Fila (Buffer de 5 comandos é suficiente)
  roombaQueue = xQueueCreate(5, sizeof(LoRaPacket));

  // Cria as Tasks em Núcleos Diferentes (Dual Core)
  // LoRa no Core 0 (Rádio)
  xTaskCreatePinnedToCore(TaskLoRaReader, "LoRa", 4096, NULL, 2, NULL, 0);

  // Roomba no Core 1 (Aplicação)
  xTaskCreatePinnedToCore(TaskRoombaDrive, "Roomba", 4096, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelete(NULL); // O loop morre, as tasks vivem
}
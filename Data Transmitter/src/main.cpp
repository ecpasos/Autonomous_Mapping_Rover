/****************************************************************************
 *  Data Transmitter: LiDAR + Encoders → ESP-NOW Transmission to ESP Station
 * 
 *       • Reads and packets LiDAR and Encoder data to transmit
 *       • Receives control commands from ESP Station and relays them
 *         to Motor Controller via serial
 * 
 *  Platform:   ESP32 (Arduino-ESP32 core ≥ 3.0.0)
 * 
 *  Reference: ESP32_RPLIDAR_C1 by guido57
 *  https://github.com/guido57/ESP32_RPLIDAR_C1.git
 * 
 ****************************************************************************/
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "rpLidar.h"
#include "rpLidarTypes.h"
#include "driver/pcnt.h"

/* ----------  USER SETTINGS ----------------------------------- */
static const uint8_t PEER_MAC[6] = { 0x5C,0x01,0x3B,0x96,0x27,0x48 }; // Station ESP32 MAC Addr
#define MSG_TYPE_LIDAR_POINTS  0x10   // followed by N*(angle,dist,qual)
#define MSG_TYPE_ENCODERS      0x11   // followed by 4 × int16_t counts
#define MSG_TYPE_COMMAND       0x12   // Command message
#define MSG_TYPE_CONTROL       0x13
/* -------------------------------------------------------------- */

/* ==========  ENCODERS  ======================================= */
const int ENCODER_A[4]     = {27, 34, 32, 13};
const int ENCODER_B[4]     = {14, 35, 33, 39};
const pcnt_unit_t PCNT_U[4]= {PCNT_UNIT_0,PCNT_UNIT_1,PCNT_UNIT_2,PCNT_UNIT_3};
void   setupEncoder (int idx,int pinA,int pinB,pcnt_unit_t unit);
void   readEncoders(int16_t *dst);
void   encoderTask(void*);

static volatile float g_latestYaw = NAN;      // updated by encoderTask

/* ---------- Encoder Packet -------------- */
#pragma pack(push,1)      // no padding bytes
struct EncoderYawPacket {
    uint8_t  type;        // MSG_TYPE_ENCODERS
    int16_t  counts[4];   // wheel encoder values
    float    yaw;         // most‑recent yaw from serial
};
#pragma pack(pop)


/* ==========  SERIAL  ========================================== */
#define RXD1 26
#define TXD1 25
HardwareSerial motor_serial(1);

/* ==========  LiDAR  ========================================== */
#define RXD2 16
#define TXD2 17
rpLidar *lidar;
void   setupLidar();
int    readLidarOnce();                // returns valid points
void   lidarTask(void*);

/* ==========  ESP-NOW  ======================================== */
void   setupEspNow();
void   onReceive(const uint8_t*,const uint8_t*,int);
void   onSent   (const uint8_t*, esp_now_send_status_t);

/* Helper: build & send 250-byte LiDAR frames */
class LidarPacketBuilder {
  public:
    void begin() { idx = 0; packet[0] = MSG_TYPE_LIDAR_POINTS; }
    void addPoint(float angleDeg, uint16_t dist, uint8_t qual) {
      if (idx >= MAX_POINTS) flush();      // frame full? send it
      uint8_t *p = &packet[2 + idx * BYTES_PER_POINT];
      // angle → 4 bytes (little‑endian)
      memcpy(p, &angleDeg, sizeof(float)); // p[0]..p[3]
      // distance → 2 bytes (little‑endian)
      p[4] = dist & 0xFF;                  // low byte
      p[5] = dist >> 8;                    // high byte
      // quality → 1 byte
      p[6] = qual;                         // p[6]
  
      ++idx;
    }
    void flush() {
      if (!idx) return;
      packet[1] = idx;                                 // point count
      size_t len = 2 + idx * BYTES_PER_POINT;          // payload size
      esp_err_t err = esp_now_send(PEER_MAC, packet, len);
      //if (err != ESP_OK) Serial.printf("ESP‑NOW send error %d\n", err);
      idx = 0;
    }
  private:
    static constexpr uint8_t BYTES_PER_POINT = 7;     
    static constexpr uint8_t MAX_POINTS =
        (250 - 2) / BYTES_PER_POINT;          

    uint8_t packet[250];
    uint8_t idx = 0;
} pkt;





/* ====================  SETUP  ================================ */
void setup() {
  Serial.begin(115200);

  motor_serial.begin(460800, SERIAL_8N1 ,RXD1, TXD1);

  /* encoders -------------------------------------------------- */
  for (int i=0;i<4;i++) setupEncoder(i,ENCODER_A[i],ENCODER_B[i],PCNT_U[i]);
  /* LiDAR ----------------------------------------------------- */
  setupLidar();
  /* ESP-NOW --------------------------------------------------- */
  setupEspNow();
  Serial.println("Initialisation complete.");

  xTaskCreatePinnedToCore(lidarTask,   "LiDAR",   8192, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(encoderTask, "Encoder", 4096, nullptr, 2, nullptr, 1);


  pinMode(2, OUTPUT); // Onboard blue LED
}



/* ====================  MAIN LOOP  ============================ */
void loop() {

}





/* ==================  IMPLEMENTATIONS  ======================== */
void setupEspNow() {
  WiFi.mode(WIFI_STA);                    // no AP beaconing
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed"); for(;;);
  }
  esp_now_register_recv_cb(onReceive);
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.channel = 0; peer.encrypt = false;
  if (!esp_now_is_peer_exist(PEER_MAC)) esp_now_add_peer(&peer);
}

// Callbacks
void onReceive(const uint8_t*, const uint8_t* data, int len) {

  uint8_t msg_type = data[0];
  motor_serial.write(msg_type);
  //Serial.println(msg_type);

  if (msg_type == MSG_TYPE_COMMAND) {
    char command = data[1];
    motor_serial.write(command);
    //Serial.println(command);
  }
  else if (msg_type = MSG_TYPE_CONTROL) {
    if (len >= 1 + sizeof(float)) {
      float value;
      // copy the 4 bytes starting at data[1] into our float
      memcpy(&value, data + 1, sizeof(value));
      motor_serial.write(reinterpret_cast<uint8_t*>(&value), sizeof(value));
    }

  }

}
void onSent(const uint8_t*, esp_now_send_status_t status) {
  //if (status!=ESP_NOW_SEND_SUCCESS) Serial.println("❌ ESP-NOW send failed");
}

/* ----------  LiDAR  ----------------------------------------- */
void setupLidar(){
  Serial2.setRxBufferSize(5000);
  Serial2.begin(460800,SERIAL_8N1,RXD2,TXD2);
  lidar = new rpLidar(&Serial2,460800);
  lidar->resetDevice();
  lidar->setAngleOfInterest(0,360);
  if (lidar->start(standard)) {
      Serial.println("LiDAR ready");
      digitalWrite(2, HIGH); // Onboard LED notifies LiDAR is initalized
    }
  else  Serial.println("LiDAR start failed");
}

int readLidarOnce() {
  int n = lidar->readMeasurePoints();
  if (!n) {                          // restart if scan lost
    lidar->resetDevice();
    lidar->start(standard);
    return 0;
  }
  pkt.begin();                       // start new frame-series
  for (int i=0;i<n;i++) {
    auto &d = lidar->DataBuffer[i];
    // if (d.quality==0 || d.distance_low==0) continue;   // skip bad points

    float angle = (lidar->DataBuffer[i].angle_high * 128 + lidar->DataBuffer[i].angle_low / 2) / 64.0;
    uint16_t dist  = (d.distance_high<<8) | d.distance_low;
    uint8_t  qual  = d.quality>>2;
    if (dist > 0 && qual > 10) 
      pkt.addPoint(angle,dist,qual);
    
  }
  pkt.flush();                       // flush remainder
  return n;
}

/* ----------  Encoders  -------------------------------------- */
void setupEncoder(int idx,int pinA,int pinB,pcnt_unit_t unit){
  pcnt_config_t c{};
  c.pulse_gpio_num  = pinA;
  c.ctrl_gpio_num   = pinB;
  c.channel         = PCNT_CHANNEL_0;
  c.unit            = unit;
  c.pos_mode        = PCNT_COUNT_INC;
  c.neg_mode        = PCNT_COUNT_DEC;
  c.lctrl_mode      = PCNT_MODE_REVERSE;
  c.hctrl_mode      = PCNT_MODE_KEEP;
  c.counter_h_lim   = 32767;
  c.counter_l_lim   = -32768;
  pcnt_unit_config(&c);
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

void readEncoders(int16_t *dst){
  for(int i=0;i<4;i++) pcnt_get_counter_value(PCNT_U[i], &dst[i]);
}

void lidarTask(void*) {
  for (;;) {                       // runs forever
    int n = readLidarOnce();     

    vTaskDelay(pdMS_TO_TICKS(2));

    if (n == 0) vTaskDelay(20 / portTICK_PERIOD_MS);  // idle pause

  }
}

void encoderTask(void*) {

  TickType_t last = xTaskGetTickCount();
  static char line[24];                     // holds ASCII
  uint8_t     len  = 0;

  for (;;) {
    /* ---- 1. Poll UART and build one yaw line ---- */
    while (motor_serial.available()) {
      char c = motor_serial.read();
      if (c == '\n') {                  // end‑of‑line
        line[len] = '\0';
        g_latestYaw = atof(line);     // update mailbox
        len = 0;                      // reset for next line
      }
      else if (c != '\r' && len < sizeof(line)-1) {
        line[len++] = c;              // accumulate
      }
    }

    /* ---- 2. Build the encoder+yaw packet ---- */
    EncoderYawPacket pkt;                 // 13 bytes, packed
    pkt.type = MSG_TYPE_ENCODERS;
    readEncoders(pkt.counts);
    pkt.yaw  = g_latestYaw;               // NAN first few cycles

    esp_now_send(PEER_MAC,
                  reinterpret_cast<uint8_t*>(&pkt),
                  sizeof(pkt));           

    vTaskDelayUntil(&last, pdMS_TO_TICKS(25));  // exactly 25 ms
  }
}



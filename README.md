# Đây là phần code được sử dụng trong phần cứng của hệ thống điều khiển đèn và quạt thông minh của nhóm 9
/*
==================================================
ĐỒ ÁN: ESP32-S3 + DHT11 + LCD + DS3231 + BLYNK + RADAR + GIỌNG NÓI (AI)
- Core 1: Quản lý Cảm biến, Màn hình, Radar & Blynk
- Core 0: Xử lý AI nhận diện giọng nói liên tục
==================================================
*/

#define BLYNK_TEMPLATE_ID "TMPL6_2GbL5Fo"
#define BLYNK_TEMPLATE_NAME "DACS"
#define BLYNK_AUTH_TOKEN "5j9t1-i-VOwuXpiopVXfxvdMLvB6kBFO"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h> 
#include <DHT.h> 

// === THƯ VIỆN AI (NHỚ ĐỔI TÊN NẾU XUẤT PROJECT KHÁC) ===
#define EIDSP_QUANTIZE_FILTERBANK   0
#include <Voice_Controlled_Light_Bulb_inferencing.h>
#include "driver/i2s.h"

// ===============================================
// CẤU HÌNH PHẦN CỨNG (PINS)
// ===============================================
#define I2C_SDA      8
#define I2C_SCL      9
#define DHTPIN       1  
#define DHTTYPE      DHT11 
#define POT_PIN      5
#define BUTTON1_PIN  12 
#define BUTTON2_PIN  10 
#define BUTTON3_PIN  47 
#define PIR_PIN      18  
#define IN1_PIN      4  
#define IN2_PIN      2  
#define ENA_PIN      22  
#define LED_PIN      15  

// CHÂN MIC I2S INMP441
#define I2S_WS 42
#define I2S_SD 40
#define I2S_SCK 41

// ===============================================
// ĐỐI TƯỢNG & BIẾN HỆ THỐNG
// ===============================================
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Tei";
char pass[] = "taigoodkid123";

LiquidCrystal_I2C lcd(0x27, 16, 2);
RTC_DS3231 rtc; 
DHT dht(DHTPIN, DHTTYPE); 

bool autoMode = false;
int fanLevel = 0; 
int ledLevel = 0;
bool ledOverride = false;
int lastExpectedLedLevel = 0; 
bool fanOverride = false;
int lastExpectedFanLevel = 0;
bool isBlynkSynced = false; 

float currentTemp = 0.0;
float currentHum = 0.0;

unsigned long lastButton1 = 0, lastButton2 = 0, lastButton3 = 0;
unsigned long lastLCDUpdate = 0, lastBlynkUpdate = 0, lastDHTRead = 0; 

const int turnOnHour = 18;  
const int turnOffHour = 6;  

// BIẾN RADAR
unsigned long firstDetectedTime = 0;
unsigned long lastMotionTime = 0;
unsigned long currentTimeout = 5000UL; 
bool isPersonPresent = false;

// BIẾN AI & I2S
TaskHandle_t AITaskHandle;
typedef struct {
    int16_t *buffer;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;
static inference_t inference;
static const uint32_t sample_buffer_size = 2048;
static signed short sampleBuffer[sample_buffer_size];
static bool record_status = true;

// KHAI BÁO HÀM AI
static bool microphone_inference_start(uint32_t n_samples);
static bool microphone_inference_record(void);
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr);
static int i2s_init(uint32_t sampling_rate);

// ===============================================
// HÀM BĂM XUNG (PWM)
// ===============================================
void setFanPWM(int level) {
  if (level == 0) { analogWrite(ENA_PIN, 0); digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW); } 
  else if (level == 1) { digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW); analogWrite(ENA_PIN, 127); } 
  else if (level == 2) { digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW); analogWrite(ENA_PIN, 255); }
}

void setLedPWM(int level) {
  if (level == 0) analogWrite(LED_PIN, 0);
  else if (level == 1) analogWrite(LED_PIN, 127); 
  else if (level == 2) analogWrite(LED_PIN, 255); 
}

float nhietdoduocdieuchinh() {
  int potValue = analogRead(POT_PIN);
  float offset = map(potValue, 0, 4095, -100, 100) / 10.0; 
  return currentTemp + offset;
}

// ===============================================
// LUỒNG AI (CHẠY ĐỘC LẬP TRÊN CORE 0)
// ===============================================
void AITask(void * parameter) {
    Serial.printf("\n[AI TASK] Khoi dong tren Core %d\n", xPortGetCoreID());
    
    if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {
        Serial.println("LỖI: Khong the khoi tao Micro I2S");
        vTaskDelete(NULL);
    }

    while(1) {
        // Chờ thu đủ 1 giây âm thanh
        if (!microphone_inference_record()) { vTaskDelay(10 / portTICK_PERIOD_MS); continue; }

        signal_t signal;
        signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
        signal.get_data = &microphone_audio_signal_get_data;
        ei_impulse_result_t result = { 0 };

        // Chạy suy luận mạng Nơ-ron
        if (run_classifier(&signal, &result, false) == EI_IMPULSE_OK) {
            // IN KẾT QUẢ RA SERIAL
            Serial.println("=====================================");
            for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                Serial.printf("%s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
                
                // KIỂM TRA ĐIỀU KIỆN KÍCH HOẠT LỆNH (> 50%)
                if (result.classification[ix].value > 0.5) { 
                    
                    if (strcmp(result.classification[ix].label, "light_on") == 0) {
                        autoMode = false; // Tắt Auto, chuyển sang Thủ công
                        ledLevel = 2;     // Bật đèn Max
                        setLedPWM(ledLevel);
                        Serial.println("\n>>> [AI COMMAND] ĐÃ BẬT ĐÈN!");
                    }
                    else if (strcmp(result.classification[ix].label, "light_off") == 0) {
                        autoMode = false; // Tắt Auto, chuyển sang Thủ công
                        ledLevel = 0;     // Tắt đèn
                        setLedPWM(ledLevel);
                        Serial.println("\n>>> [AI COMMAND] ĐÃ TẮT ĐÈN!");
                    }
                }
            }
            Serial.println("=====================================");
        }
        vTaskDelay(20 / portTICK_PERIOD_MS); // Chống dội Watchdog Timer
    }
}

// ===============================================
// SETUP (Core 1)
// ===============================================
void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL); 
  lcd.init(); lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("System Starting.");

  if (!rtc.begin()) {
    Serial.println("Khong tim thay module RTC!");
    lcd.setCursor(0, 1); lcd.print("RTC Error!      ");
    while (1) delay(10);
  }
  
  rtc.adjust(DateTime(2026, 5, 21, 17, 59, 0)); 
  
  dht.begin();
  
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);
  pinMode(PIR_PIN, INPUT); 

  pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT); pinMode(LED_PIN, OUTPUT);
  setFanPWM(0); setLedPWM(0);

  Serial.println("Đang kích hoạt WiFi ngầm...");
  lcd.setCursor(0, 1); lcd.print("Connecting WiFi ");
  
  WiFi.begin(ssid, pass);
  Blynk.config(auth);

  currentTemp = dht.readTemperature();
  currentHum = dht.readHumidity();
  if (isnan(currentTemp)) currentTemp = 0.0;
  if (isnan(currentHum)) currentHum = 0.0;

  // KÍCH HOẠT LUỒNG AI NGAY SAU KHI SETUP PHẦN CỨNG XONG
  xTaskCreatePinnedToCore(AITask, "AITask", 1024 * 40, NULL, 1, &AITaskHandle, 0);

  Serial.println("================================");
  Serial.println("START SYSTEM: FULL AI + SENSORS");
  Serial.println("MODE: MANUAL");
  Serial.println("================================");
  lcd.clear();
}

BLYNK_CONNECTED() {
  Blynk.syncAll(); isBlynkSynced = true; 
  Serial.println("Blynk Connected successfully!");
}

// ===============================================
// BLYNK APP ĐIỀU KHIỂN
// ===============================================
BLYNK_WRITE(V3) {
  int val = param.asInt(); 
  if (isBlynkSynced && autoMode && (ledLevel != val)) ledOverride = true; 
  ledLevel = val; setLedPWM(ledLevel);
}
BLYNK_WRITE(V4) {
  int val = param.asInt(); 
  if (isBlynkSynced && autoMode && (fanLevel != val)) fanOverride = true;
  fanLevel = val; setFanPWM(fanLevel);
}

// ===============================================
// LOOP CHÍNH (Core 1)
// ===============================================
void loop() {
  if (WiFi.status() == WL_CONNECTED) Blynk.run();

  if (millis() - lastDHTRead > 2000) {
    float h = dht.readHumidity(); float t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) { currentHum = h; currentTemp = t; }
    lastDHTRead = millis();
  }

  handleModeButton();

  if (autoMode) autoControl();
  else manualControl();

  if (millis() - lastLCDUpdate > 500) { updateLCD(); lastLCDUpdate = millis(); }

  if (WiFi.status() == WL_CONNECTED && millis() - lastBlynkUpdate > 2000) {
    updateBlynk(); lastBlynkUpdate = millis();
  }
  vTaskDelay(10 / portTICK_PERIOD_MS); 
}

void updateBlynk() {
  Blynk.virtualWrite(V0, currentTemp); Blynk.virtualWrite(V1, currentHum); 
  Blynk.virtualWrite(V3, ledLevel); Blynk.virtualWrite(V4, fanLevel); 
}

void updateLCD() {
  DateTime now = rtc.now();
  char timeStr[9]; 
  sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  float finalTemp = nhietdoduocdieuchinh();

  lcd.setCursor(0, 0); lcd.print(timeStr);       
  if (fanLevel == 0) lcd.print(" F:OFF "); else if (fanLevel == 1) lcd.print(" F:50% "); else lcd.print(" F:MAX ");

  lcd.setCursor(0, 1); lcd.print(finalTemp, 1); lcd.print("C ");
  if (autoMode) lcd.print("AT "); else lcd.print("MN ");

  if (ledLevel == 0) lcd.print("L:OFF"); else if (ledLevel == 1) lcd.print("L:50%"); else lcd.print("L:MAX");
}

void handleModeButton() {
  if (digitalRead(BUTTON2_PIN) == LOW) {
    if (millis() - lastButton2 > 250) { 
      autoMode = !autoMode;
      if (autoMode) {
        Serial.println("\n========== AUTO MODE ==========");
        ledOverride = false; fanOverride = false;
      } else { Serial.println("\n========= MANUAL MODE ========="); }
      lastButton2 = millis();
    }
  }
}

void manualControl() {
  if (digitalRead(BUTTON1_PIN) == LOW) {
    if (millis() - lastButton1 > 250) {
      fanLevel = (fanLevel + 1) % 3; setFanPWM(fanLevel);
      Serial.print("MANUAL -> FAN Level: "); Serial.println(fanLevel);
      lastButton1 = millis();
    }
  }
  if (digitalRead(BUTTON3_PIN) == LOW) {
    if (millis() - lastButton3 > 250) {
      ledLevel = (ledLevel + 1) % 3; setLedPWM(ledLevel);
      Serial.print("MANUAL -> LED Level: "); Serial.println(ledLevel);
      lastButton3 = millis();
    }
  }
}

void autoControl() {
  // QUẠT
  float finalTemp = nhietdoduocdieuchinh();
  int expectedFanAutoLevel = (finalTemp > 25.0) ? 2 : 0; 
  if (expectedFanAutoLevel != lastExpectedFanLevel) { fanOverride = false; lastExpectedFanLevel = expectedFanAutoLevel; }

  if (digitalRead(BUTTON1_PIN) == LOW) {
    if (millis() - lastButton1 > 250) {
      fanLevel = (fanLevel + 1) % 3; setFanPWM(fanLevel); fanOverride = true; 
      Serial.print("AUTO -> FAN Override Level: "); Serial.println(fanLevel);
      lastButton1 = millis();
    }
  }
  if (!fanOverride) { if (fanLevel != expectedFanAutoLevel) { fanLevel = expectedFanAutoLevel; setFanPWM(fanLevel); } }

  // ĐÈN VÀ RADAR
  DateTime now = rtc.now();
  int currentHour = now.hour();
  bool isNightTime = (currentHour >= turnOnHour || currentHour < turnOffHour);
  bool rawPerson = (digitalRead(PIR_PIN) == HIGH);

  if (rawPerson) {
    if (!isPersonPresent) { firstDetectedTime = millis(); isPersonPresent = true; }
    lastMotionTime = millis();
    unsigned long stayDuration = millis() - firstDetectedTime;

    if (stayDuration > 30UL * 60000UL) currentTimeout = 60UL * 60000UL; 
    else if (stayDuration > 10UL * 60000UL) currentTimeout = 30UL * 60000UL; 
    else if (stayDuration > 5000UL) currentTimeout = 10UL * 60000UL; 
    else currentTimeout = 5000UL; 
  }

  bool isTimerActive = (millis() - lastMotionTime < currentTimeout);
  if (!isTimerActive) { isPersonPresent = false; currentTimeout = 5000UL; }

  int expectedLedAutoLevel = 0;
  if (isNightTime) expectedLedAutoLevel = 2; 
  else if (isTimerActive) expectedLedAutoLevel = 2; 
  else expectedLedAutoLevel = 0; 

  if (expectedLedAutoLevel != lastExpectedLedLevel) { ledOverride = false; lastExpectedLedLevel = expectedLedAutoLevel; }

  if (digitalRead(BUTTON3_PIN) == LOW) {
    if (millis() - lastButton3 > 250) {
      ledLevel = (ledLevel + 1) % 3; setLedPWM(ledLevel); ledOverride = true; 
      Serial.print("AUTO -> LED Override Level: "); Serial.println(ledLevel);
      lastButton3 = millis();
    }
  }

  if (!ledOverride) { if (ledLevel != expectedLedAutoLevel) { ledLevel = expectedLedAutoLevel; setLedPWM(ledLevel); } }
}

// ===============================================
// CÁC HÀM XỬ LÝ I2S MIC (CHO AI)
// ===============================================
static void audio_inference_callback(uint32_t n_bytes) {
    for(int i = 0; i < n_bytes>>1; i++) {
        inference.buffer[inference.buf_count++] = sampleBuffer[i];
        if(inference.buf_count >= inference.n_samples) { inference.buf_count = 0; inference.buf_ready = 1; }
    }
}

static void capture_samples(void* arg) {
  const int32_t i2s_bytes_to_read = (uint32_t)arg;
  size_t bytes_read = i2s_bytes_to_read;
  while (record_status) {
    i2s_read((i2s_port_t)1, (void*)sampleBuffer, i2s_bytes_to_read, &bytes_read, 100);
    if (bytes_read > 0) {
        for (int x = 0; x < i2s_bytes_to_read/2; x++) sampleBuffer[x] = (int16_t)(sampleBuffer[x]) * 8;
        if (record_status) audio_inference_callback(i2s_bytes_to_read); else break;
    }
  }
  vTaskDelete(NULL);
}

static bool microphone_inference_start(uint32_t n_samples) {
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));
    if(inference.buffer == NULL) return false;
    inference.buf_count  = 0; inference.n_samples  = n_samples; inference.buf_ready  = 0;
    i2s_init(EI_CLASSIFIER_FREQUENCY); ei_sleep(100); record_status = true;
    xTaskCreatePinnedToCore(capture_samples, "CaptureSamples", 1024 * 32, (void*)sample_buffer_size, 10, NULL, 0);
    return true;
}

static bool microphone_inference_record(void) {
    while (inference.buf_ready == 0) { vTaskDelay(10 / portTICK_PERIOD_MS); }
    inference.buf_ready = 0; return true;
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length); return 0;
}

static int i2s_init(uint32_t sampling_rate) {
  i2s_config_t i2s_config = { .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX), .sample_rate = sampling_rate, .bits_per_sample = (i2s_bits_per_sample_t)16, .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, .communication_format = I2S_COMM_FORMAT_I2S, .intr_alloc_flags = 0, .dma_buf_count = 8, .dma_buf_len = 512, .use_apll = false, .tx_desc_auto_clear = false, .fixed_mclk = -1, };
  i2s_pin_config_t pin_config = { .bck_io_num = I2S_SCK, .ws_io_num = I2S_WS, .data_out_num = -1, .data_in_num = I2S_SD, };
  i2s_driver_install((i2s_port_t)1, &i2s_config, 0, NULL);
  i2s_set_pin((i2s_port_t)1, &pin_config); i2s_zero_dma_buffer((i2s_port_t)1); return 0;
}
-------------------------------------------------------------------------
# Đây là phần điều khiển bằng giọng nói do AI xử lí thông qua mic INMP441
# Voice Controlled Smart Lighting System

## 1. Giới thiệu Đồ án & Cấu trúc Phần cứng
Dự án này triển khai hệ thống điều khiển đèn thông minh bằng giọng nói ứng dụng công nghệ **TinyML (Machine Learning tại biên)**. Toàn bộ mô hình phân loại hành vi và nhận diện từ khóa (Keyword Spotting) được huấn luyện thông qua nền tảng **Edge Impulse** và xuất bản dưới dạng thư viện C++ tối ưu để nhúng thẳng vào vi điều khiển.

### Cấu trúc Phần cứng của Hệ thống:
* **Khối xử lý trung tâm:** Vi điều khiển hiệu năng cao (ví dụ: ESP32 hoặc tương đương) tích hợp lõi vi xử lý đủ mạnh để chạy thuật toán suy luận toán học từ mô hình AI mà không cần kết nối Internet hoặc Cloud.
* **Khối thu nhận tín hiệu âm thanh:** Sử dụng Microphone kỹ thuật số (như INMP441 giao tiếp qua giao thức I2S) hoặc Microphone Analog (giao tiếp qua bộ ADC) để thu nhận khẩu lệnh của người dùng theo thời gian thực với tần số lấy mẫu tiêu chuẩn 16kHz.
* **Khối chấp hành (Điều khiển công suất):** Đầu ra logic từ vi điều khiển sẽ kích hoạt mạch driver (Relay/Opto/Transistor) để đóng/cắt nguồn điện cấp cho bóng đèn, đảm bảo an toàn điện và cách ly nhiễu giữa khối xử lý tín hiệu nhỏ và khối công suất.

### Mô hình AI core:
Mô hình toán học sau khi nén dữ liệu sẽ phân tích tín hiệu âm thanh liên tục để phân loại chính xác các nhãn trạng thái:
* `light_on`: Nhận diện khẩu lệnh bật đèn.
* `light_off`: Nhận diện khẩu lệnh tắt đèn.
* `noise` / `background`: Lọc bỏ tạp âm môi trường (tiếng quạt, tiếng gió, tiếng xe cộ) để tránh hệ thống phản hồi sai.

---

## 2. Hướng dẫn cài đặt thư viện vào Arduino IDE
Để nạp mã nguồn nhúng này vào phần cứng, bạn cần tích hợp file thư viện Edge Impulse đã được đóng gói:

1. Trên giao diện GitHub này, nhấn vào nút **Code** (màu xanh) -> Chọn **Download ZIP**.
2. Mở phần mềm **Arduino IDE** trên máy tính.
3. Điều hướng trên thanh công cụ: **Sketch** -> **Include Library** -> **Add .ZIP Library...**
4. Trỏ đường dẫn tới file `.zip` vừa tải về để IDE tự động giải nén và cấu hình môi trường build cho compiler.

---

## 3. Hướng dẫn sử dụng & Lệnh lập trình mẫu

### Khai báo cấu trúc thư viện
Ở đầu file mã nguồn chính (`.ino`), bạn bắt buộc phải gọi file header chính xác của mô hình:
```cpp
#include <Voice_Controlled_Light_Bulb_inferencing.h>
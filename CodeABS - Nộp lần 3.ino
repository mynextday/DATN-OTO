#include <WiFiS3.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ═══════════════════════════════════════════════
//  WIFI
// ═══════════════════════════════════════════════
const char* AP_SSID = "ABS_R4_WIFI";
const char* AP_PASS = "66668888";
const int   WS_PORT = 81;

WebSocketsServer wsServer(WS_PORT);

// FIX #6: đếm client thay vì bool
uint8_t clientCount = 0;

// ═══════════════════════════════════════════════
//  LCD
// ═══════════════════════════════════════════════
LiquidCrystal_I2C lcd(0x27, 20, 4);
unsigned long lastLcdMs = 0;
const unsigned long LCD_PERIOD_MS = 500;

// ═══════════════════════════════════════════════
//  CHÂN
// ═══════════════════════════════════════════════
#define WHEEL_SPEED_PIN    2
#define VEHICLE_SPEED_PIN  3

#define VALVE_LPWM   5
#define VALVE_RPWM   6
#define PUMP_LPWM    9
#define PUMP_RPWM   10

#define PRESSURE_PIN A0
// A1 không sử dụng

// ═══════════════════════════════════════════════
//  THÔNG SỐ CẢM BIẾN
// ═══════════════════════════════════════════════
const int   WHEEL_TEETH_COUNT     = 46;
const int   VEHICLE_PULSE_PER_REV = 8;

const float WHEEL_DIAMETER_M   = 0.40f;
const float VEHICLE_DIAMETER_M = 0.08f;

const unsigned long SPEED_PERIOD_MS = 100;

const float PRESSURE_SENSOR_MIN_V   = 0.5f;
const float PRESSURE_SENSOR_MAX_V   = 4.5f;
const float PRESSURE_SENSOR_MAX_PSI = 500.0f;

const float ADC_REF_V = 5.0f;
const float ADC_MAX   = 1023.0f;

// ═══════════════════════════════════════════════
//  THÔNG SỐ ABS
// ═══════════════════════════════════════════════
const int PWM_MAX = 255;

const unsigned long VALVE_ON_TIME_MS = 150;
const unsigned long PUMP_ON_TIME_MS  = 200;
const unsigned long REST_TIME_MS     = 100;

// FIX #5: 1000ms → 200ms
const unsigned long SEND_PERIOD_MS = 200;

// ═══════════════════════════════════════════════
//  NGƯỠNG TỰ ĐỘNG
// ═══════════════════════════════════════════════
const float SPEED_DIFF_THRESHOLD_ON  = 5.0f;
const float SPEED_DIFF_THRESHOLD_OFF = 2.0f;
const float PRESSURE_THRESHOLD_ON    = 50.0f;
const float MIN_VEHICLE_SPEED        = 3.0f;

// ═══════════════════════════════════════════════
//  BIẾN TỐC ĐỘ
// ═══════════════════════════════════════════════
volatile unsigned long wheelPulseCount   = 0;
volatile unsigned long vehiclePulseCount = 0;

unsigned long lastSpeedMs = 0;
unsigned long lastSendMs  = 0;
unsigned long stateStartMs = 0;

float wheel_rpm    = 0.0f;
float wheel_speed  = 0.0f;
float vehicle_rpm  = 0.0f;
float vehicle_speed = 0.0f;

// ═══════════════════════════════════════════════
//  BIẾN CẢM BIẾN
// ═══════════════════════════════════════════════
float pressure_psi = 0.0f;
float pressure_bar = 0.0f;

// ═══════════════════════════════════════════════
//  BIẾN TRẠNG THÁI
// ═══════════════════════════════════════════════
int  cycleIndex  = 0;
int  currentPwm  = 0;
bool done        = false;
bool valveStatus = false;
bool pumpStatus  = false;

// ═══════════════════════════════════════════════
//  STATE MACHINE
// ═══════════════════════════════════════════════
enum AbsState {
  IDLE,
  VALVE_ON,
  PUMP_ON,
  RESTING,
  EMERGENCY_STOP
};

AbsState state = IDLE;

// ═══════════════════════════════════════════════
//  NGẮT
// ═══════════════════════════════════════════════
void wheelSpeedISR() {
  wheelPulseCount++;
}

void vehicleSpeedISR() {
  vehiclePulseCount++;
}

// ═══════════════════════════════════════════════
//  ĐIỀU KHIỂN BTS
// ═══════════════════════════════════════════════
void setValve(int pwm) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(VALVE_RPWM, pwm);
  analogWrite(VALVE_LPWM, 0);
  valveStatus = (pwm > 0);
}

void setPump(int pwm) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(PUMP_RPWM, pwm);
  analogWrite(PUMP_LPWM, 0);
  pumpStatus = (pwm > 0);
}

void allOff() {
  setValve(0);
  setPump(0);
  currentPwm = 0;
}

// ═══════════════════════════════════════════════
//  FORWARD DECLARATION
// ═══════════════════════════════════════════════
void sendStatus();

// ═══════════════════════════════════════════════
//  STATE MACHINE
// ═══════════════════════════════════════════════

// FIX #4: gửi JSON ngay khi state thay đổi
void enterState(AbsState newState) {
  state = newState;
  stateStartMs = millis();
  sendStatus();   // ← gửi ngay, app thấy đúng state
}

void startAbsCycle() {
  if (state != IDLE) return;

  cycleIndex = 1;
  done       = false;

  setValve(PWM_MAX);
  setPump(0);
  currentPwm = PWM_MAX;

  // FIX #10: không giới hạn TOTAL_CYCLES, loop theo checkAutoAbs()
  enterState(VALVE_ON);
  Serial.println(F("ABS: Bat dau chu trinh"));
}

void stopAbsEmergency() {
  allOff();
  enterState(EMERGENCY_STOP);
  Serial.println(F("ABS: DUNG KHAN CAP"));
}

void resetAbs() {
  allOff();
  cycleIndex = 0;
  done       = false;
  enterState(IDLE);
  Serial.println(F("ABS: Da RESET"));
}

void updateAbsStateMachine() {
  unsigned long now     = millis();
  unsigned long elapsed = now - stateStartMs;

  switch (state) {

    case IDLE:
    case EMERGENCY_STOP:
      break;

    case VALVE_ON:
      if (elapsed >= VALVE_ON_TIME_MS) {
        setValve(PWM_MAX);
        setPump(PWM_MAX);
        currentPwm = PWM_MAX;
        enterState(PUMP_ON);
      }
      break;

    case PUMP_ON:
      if (elapsed >= PUMP_ON_TIME_MS) {
        setValve(0);
        setPump(0);
        currentPwm = 0;
        enterState(RESTING);
      }
      break;

    case RESTING:
      if (elapsed >= REST_TIME_MS) {
        // FIX #10: không đếm cycle, cứ lặp — checkAutoAbs() quyết định dừng
        cycleIndex++;
        setValve(PWM_MAX);
        setPump(0);
        currentPwm = PWM_MAX;
        enterState(VALVE_ON);
      }
      break;
  }
}

// ═══════════════════════════════════════════════
//  TỰ ĐỘNG PHÁT HIỆN
// ═══════════════════════════════════════════════
void checkAutoAbs() {
  if (state == EMERGENCY_STOP) return;

  if (vehicle_speed < MIN_VEHICLE_SPEED) {
    if (state != IDLE && state != EMERGENCY_STOP) {
      Serial.println(F("ABS: Toc do qua thap, reset"));
      resetAbs();
    }
    return;
  }

  bool dangPhanh  = (pressure_psi > PRESSURE_THRESHOLD_ON);

  // FIX #9: chỉ tính slip khi wheel_speed có vẻ hợp lệ
  // (không trigger ABS oan khi cảm biến bánh bị lỗi đọc = 0)
  bool wheelValid = (wheel_speed >= 0.0f);
  float speedDiff = wheelValid ? (vehicle_speed - wheel_speed) : 0.0f;

  if (state == IDLE) {
    if (dangPhanh && wheelValid && speedDiff > SPEED_DIFF_THRESHOLD_ON) {
      Serial.print(F("ABS: Kich hoat | P:"));
      Serial.print(pressure_psi, 0);
      Serial.print(F("psi | dV:"));
      Serial.print(speedDiff, 1);
      Serial.println(F("km/h"));
      startAbsCycle();
    }
  } else if (state == VALVE_ON || state == PUMP_ON || state == RESTING) {
    if (!dangPhanh || speedDiff < SPEED_DIFF_THRESHOLD_OFF) {
      Serial.println(F("ABS: Tat - banh on dinh / nha phanh"));
      resetAbs();
    }
  }
}

// ═══════════════════════════════════════════════
//  ĐỌC TỐC ĐỘ
// ═══════════════════════════════════════════════
void updateSpeed() {
  unsigned long now = millis();

  if (now - lastSpeedMs >= SPEED_PERIOD_MS) {
    noInterrupts();
    unsigned long wheelPulses   = wheelPulseCount;
    unsigned long vehiclePulses = vehiclePulseCount;
    wheelPulseCount   = 0;
    vehiclePulseCount = 0;
    interrupts();

    float dt = (now - lastSpeedMs) / 1000.0f;
    lastSpeedMs = now;

    if (dt > 0) {
      float wheelCirc      = 3.14159265f * WHEEL_DIAMETER_M;
      float wheelRev       = wheelPulses / (float)WHEEL_TEETH_COUNT;
      // FIX #7: CHANGE edge → đếm cả rising+falling → chia 2
      wheelRev            /= 2.0f;
      float wheelRevPerSec = wheelRev / dt;
      wheel_rpm   = wheelRevPerSec * 60.0f;
      wheel_speed = wheelRevPerSec * wheelCirc * 3.6f;

      float vehicleCirc      = 3.14159265f * VEHICLE_DIAMETER_M;
      float vehicleRev       = vehiclePulses / (float)VEHICLE_PULSE_PER_REV;
      float vehicleRevPerSec = vehicleRev / dt;
      vehicle_rpm   = vehicleRevPerSec * 60.0f;
      vehicle_speed = vehicleRevPerSec * vehicleCirc * 3.6f;
    }
  }
}

// ═══════════════════════════════════════════════
//  ĐỌC CẢM BIẾN
// ═══════════════════════════════════════════════
void readSensors() {
  int rawP = analogRead(PRESSURE_PIN);
  float vP = rawP * (ADC_REF_V / ADC_MAX);

  pressure_psi = (vP - PRESSURE_SENSOR_MIN_V) *
                 (PRESSURE_SENSOR_MAX_PSI /
                 (PRESSURE_SENSOR_MAX_V - PRESSURE_SENSOR_MIN_V));

  pressure_psi = constrain(pressure_psi, 0.0f, PRESSURE_SENSOR_MAX_PSI);
  pressure_bar = pressure_psi / 14.5038f;
}

// ═══════════════════════════════════════════════
//  JSON  (FIX #1 #2)
// ═══════════════════════════════════════════════

// FIX #2: const char* thay vì String
const char* stateText() {
  switch (state) {
    case IDLE:           return "IDLE";
    case VALVE_ON:       return "VALVE_ON";
    case PUMP_ON:        return "PUMP_ON";
    case RESTING:        return "RESTING";
    case EMERGENCY_STOP: return "EMERGENCY_STOP";
    default:             return "UNKNOWN";
  }
}

// FIX #1: snprintf + char buffer, không dùng String
void buildJson(char* buf, size_t bufSize) {
  bool running = (state == VALVE_ON || state == PUMP_ON || state == RESTING);

  float slipPct = 0.0f;
  if (vehicle_speed > 0.1f) {
    slipPct = (vehicle_speed - wheel_speed) / vehicle_speed * 100.0f;
    slipPct = constrain(slipPct, 0.0f, 100.0f);
  }

  snprintf(buf, bufSize,
    "{"
    "\"wheel_speed\":%.2f,"
    "\"vehicle_speed\":%.2f,"
    "\"wheel_rpm\":%.2f,"
    "\"vehicle_rpm\":%.2f,"
    "\"slip_pct\":%.1f,"
    "\"pressure_bar\":%.2f,"
    "\"pressure_psi\":%.1f,"
    "\"pwm\":%d,"
    "\"valve\":%d,"
    "\"pump\":%d,"
    "\"running\":%d,"
    "\"cycle\":%d,"
    "\"state\":\"%s\""
    "}",
    wheel_speed, vehicle_speed,
    wheel_rpm,   vehicle_rpm,
    slipPct,
    pressure_bar, pressure_psi,
    currentPwm,
    valveStatus ? 1 : 0,
    pumpStatus  ? 1 : 0,
    running     ? 1 : 0,
    cycleIndex,
    stateText()
  );
}

// ═══════════════════════════════════════════════
//  GỬI TRẠNG THÁI  (FIX #3 #4)
// ═══════════════════════════════════════════════
void sendStatus() {
  char json[300];
  buildJson(json, sizeof(json));

  wsServer.broadcastTXT(json);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 2000) {
    lastPrint = millis();
    Serial.print(F("WS clients:"));
    Serial.print(clientCount);
    Serial.print(F(" | State:"));
    Serial.print(stateText());
    Serial.print(F(" | W:"));
    Serial.print(wheel_speed, 1);
    Serial.print(F(" V:"));
    Serial.print(vehicle_speed, 1);
    Serial.print(F(" km/h | P:"));
    Serial.print(pressure_psi, 0);
    Serial.println(F(" psi"));
  }
}

// ═══════════════════════════════════════════════
//  LỆNH
// ═══════════════════════════════════════════════
void handleCommand(const char* cmd) {
  if (strcmp(cmd, "STOP")   == 0) stopAbsEmergency();
  else if (strcmp(cmd, "RESET")  == 0) resetAbs();
  else if (strcmp(cmd, "STATUS") == 0) {
    // Force gửi dù clientCount, dùng cho debug Serial
    char json[300];
    buildJson(json, sizeof(json));
    Serial.println(json);
    if (clientCount > 0) wsServer.broadcastTXT(json);
  }
}

void handleSerialCommand() {
  if (Serial.available()) {
    static char buf[32];
    static uint8_t idx = 0;
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (idx > 0) {
        buf[idx] = '\0';
        // uppercase in-place
        for (uint8_t i = 0; i < idx; i++)
          if (buf[i] >= 'a' && buf[i] <= 'z') buf[i] -= 32;
        handleCommand(buf);
        idx = 0;
      }
    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }
}

// ═══════════════════════════════════════════════
//  WEBSOCKET EVENT
// ═══════════════════════════════════════════════
void webSocketEvent(uint8_t num, WStype_t type,
                    uint8_t* payload, size_t length) {
  switch (type) {

    case WStype_DISCONNECTED:
      // FIX #6: đếm chính xác
      if (clientCount > 0) clientCount--;
      Serial.print(F("WS: Client ngat ket noi id="));
      Serial.print(num);
      Serial.print(F(" | Con lai: "));
      Serial.println(clientCount);
      break;

    case WStype_CONNECTED:
      clientCount++;
      Serial.print(F("WS: Client ket noi id="));
      Serial.print(num);
      Serial.print(F(" | Tong: "));
      Serial.println(clientCount);
      // Gửi state hiện tại ngay cho client mới
      {
        char json[300];
        buildJson(json, sizeof(json));
        wsServer.sendTXT(num, json);
      }
      break;

    case WStype_TEXT: {
      // Uppercase payload rồi xử lý lệnh
      char cmd[32];
      size_t len = min(length, (size_t)31);
      memcpy(cmd, payload, len);
      cmd[len] = '\0';
      for (size_t i = 0; i < len; i++)
        if (cmd[i] >= 'a' && cmd[i] <= 'z') cmd[i] -= 32;
      handleCommand(cmd);
      break;
    }

    default:
      break;
  }
}

// ═══════════════════════════════════════════════
//  LCD
// ═══════════════════════════════════════════════
void lcdPrintFixed(uint8_t col, uint8_t row, const char* text) {
  lcd.setCursor(col, row);
  int maxLen = 20 - col;
  int i = 0;
  while (text[i] && i < maxLen) {
    lcd.print(text[i++]);
  }
  while (i++ < maxLen) lcd.print(' ');
}

void updateLCD() {
  char buf[21];

  // Dòng 0: WS status + cycle
  snprintf(buf, sizeof(buf), "WS:%d Cycle:%d",
           clientCount, cycleIndex);
  lcdPrintFixed(0, 0, buf);

  // Dòng 1: State
  snprintf(buf, sizeof(buf), "State:%s", stateText());
  lcdPrintFixed(0, 1, buf);

  // Dòng 2: Tốc độ
  snprintf(buf, sizeof(buf), "W:%.1f V:%.1fkm/h",
           wheel_speed, vehicle_speed);
  lcdPrintFixed(0, 2, buf);

  // Dòng 3: Áp suất + valve/pump
  snprintf(buf, sizeof(buf), "P:%.0fpsi %s %s",
           pressure_psi,
           valveStatus ? "VC" : "VO",
           pumpStatus  ? "P1" : "P0");
  lcdPrintFixed(0, 3, buf);
}

// ═══════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════
void setup() {
  // FIX #7: INPUT (không pullup) + CHANGE cho MAX9926
  pinMode(WHEEL_SPEED_PIN,   INPUT);
  // FIX #8: RISING cho vehicle (thống nhất với cạnh lên)
  pinMode(VEHICLE_SPEED_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(WHEEL_SPEED_PIN),
                  wheelSpeedISR, CHANGE);   // CHANGE → đếm cả 2 cạnh
  attachInterrupt(digitalPinToInterrupt(VEHICLE_SPEED_PIN),
                  vehicleSpeedISR, RISING); // RISING thống nhất

  pinMode(VALVE_RPWM, OUTPUT);
  pinMode(VALVE_LPWM, OUTPUT);
  pinMode(PUMP_RPWM,  OUTPUT);
  pinMode(PUMP_LPWM,  OUTPUT);

  allOff();

  Serial.begin(115200);
  delay(1500);

  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcdPrintFixed(0, 0, "UNO R4 WiFi ABS");
  lcdPrintFixed(0, 1, "v2 - fixed");
  lcdPrintFixed(0, 2, "Dang tao WiFi AP");

  Serial.println(F("═══ UNO R4 WiFi ABS v2 ═══"));
  Serial.println(F("Van: Thuong Mo (Normally Open)"));
  Serial.println(F("Dang khoi dong WiFi AP..."));

  WiFi.end();
  delay(3000);

  int status = WiFi.beginAP(AP_SSID, AP_PASS);

  if (status != WL_AP_LISTENING) {
    Serial.print(F("LOI WiFi AP, status="));
    Serial.println(status);
    lcdPrintFixed(0, 2, "WiFi AP ERROR");
    lcdPrintFixed(0, 3, "Update WiFi FW!");
  } else {
    IPAddress ip = WiFi.localIP();
    Serial.print(F("WiFi AP: ")); Serial.println(AP_SSID);
    Serial.print(F("IP: "));     Serial.println(ip);
    lcdPrintFixed(0, 2, "WiFi: ABS_R4_WIFI");
    char ipBuf[20];
    snprintf(ipBuf, sizeof(ipBuf), "IP:%s", ip.toString().c_str());
    lcdPrintFixed(0, 3, ipBuf);
  }

  wsServer.begin();
  wsServer.onEvent(webSocketEvent);

  Serial.print(F("WebSocket: ws://"));
  Serial.print(WiFi.localIP());
  Serial.print(F(":"));
  Serial.println(WS_PORT);
  Serial.println(F("Lenh: STOP | RESET | STATUS"));
  Serial.println(F("═══════════════════════════"));

  delay(1500);
  lcd.clear();
}

// ═══════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════
void loop() {
  wsServer.loop();

  handleSerialCommand();
  updateAbsStateMachine();

  wsServer.loop();

  updateSpeed();
  readSensors();
  checkAutoAbs();

  wsServer.loop();

  unsigned long now = millis();


  if (now - lastSendMs >= SEND_PERIOD_MS) {
    lastSendMs = now;
    sendStatus();
  }

  if (now - lastLcdMs >= LCD_PERIOD_MS) {
    lastLcdMs = now;
    updateLCD();
    wsServer.loop();
  }
}

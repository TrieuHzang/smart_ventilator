#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* ================== Pin mapping ================== */
const uint8_t PIN_PWM   = 9;   // OC1A -> PWM 25kHz (ra base NPN qua R 4.7–10k)
const uint8_t PIN_EN    = 8;   // Gate MOSFET low-side (D8) để cắt mass quạt ở Lv0
const uint8_t PIN_BTN   = 3;   // Nút nhấn, INPUT_PULLUP (LOW = nhấn)
const uint8_t PIN_DS    = 4;   // DS18B20 data (kèm 4.7k kéo lên 5V)
const uint8_t PIN_TACH  = 2;   // Tach input (INT0), có pull-up 10k -> 5V

/* ================== LCD I2C ================== */
LiquidCrystal_I2C lcd(0x27, 16, 2); // nếu không hiện, đổi 0x27 ↔ 0x3F

/* ================== DS18B20 ================== */
OneWire oneWire(PIN_DS);
DallasTemperature sensors(&oneWire);

/* ================== Quạt / tham số ================== */
// Fan 4 dây chuẩn PC: PWM "active HIGH" tại dây xanh dương.
// Với mạch NPN kéo xuống, HIGH trên dây PWM = NPN tắt (được kéo lên 5V).
// Ta phát PWM ở D9 rồi ĐẢO duty: OCR1A = (1 - duty_mong_muon) * ICR1.
const float DUTY_OFF = 0.00;
const float DUTY_L1  = 0.30;
const float DUTY_L2  = 0.60;
const float DUTY_L3  = 1.00;

// Ngưỡng Auto (có hysteresis)
const float T_OFF_MAX = 20.0; // <20 -> OFF
const float T_L1_MAX  = 30.0; // 20..30 -> L1
const float T_L2_MAX  = 40.0; // 30..40 -> L2
const float T_HYST    = 0.5;  // chống nhấp nháy quanh ngưỡng

// Thay đổi mượt
const float SLEW_STEP = 0.08;   // thay đổi tối đa mỗi chu kỳ (0..1)
float targetDuty = 0.0, currentDuty = 0.0;

/* ================== Trạng thái =============== */
enum ManualLevel { M_OFF=0, M_L1, M_L2, M_L3 };
ManualLevel manLevel = M_OFF;
bool isAuto = true;

/* Button (debounce + long-press) */
bool btnStable = HIGH;              // INPUT_PULLUP: thả = HIGH
uint32_t btnLastChangeMs = 0;
const uint32_t DEBOUNCE_MS = 30;
bool btnWasPressed = false;
uint32_t btnPressStartMs = 0;
const uint32_t LONG_PRESS_MS = 1200;

/* Tach */
volatile uint32_t tachPulses = 0;   // đếm cạnh FALLING
uint32_t lastRpmMs = 0;
uint16_t rpm = 0;

/* ======= prototypes (do setDuty dùng trong setupPwm25k) ======= */
void setDuty(float d01);
void setupPwm25k();

/* ================== PWM 25kHz trên Timer1 (pin 9) ================== */
void setupPwm25k(){
  pinMode(PIN_PWM, OUTPUT);
  // Timer1 Fast PWM mode 14, TOP = ICR1, no prescale
  // 16MHz / (1 * (1+639)) ≈ 25kHz
  TCCR1A = 0; TCCR1B = 0;
  ICR1 = 639;
  TCCR1A = (1<<COM1A1) | (1<<WGM11);
  TCCR1B = (1<<WGM13)  | (1<<WGM12) | (1<<CS10);
  setDuty(0.0);
}

// ĐẢO duty để ra "thời gian chạy" trên dây PWM quạt
void setDuty(float d01){
  if (d01 < 0) d01 = 0;
  if (d01 > 1) d01 = 1;
  float inv = 1.0 - d01;                // ĐẢO ở đây (active-low tại D9 → active-high ở dây PWM)
  OCR1A = (uint16_t)(inv * ICR1);
}

/* ================== Auto theo nhiệt (hysteresis) ================== */
ManualLevel levelFromTemp(float t){
  static ManualLevel last = M_OFF;
  switch (last){
    case M_OFF:
      if (t >= T_OFF_MAX + T_HYST) last = M_L1;
      break;
    case M_L1:
      if (t <  T_OFF_MAX - T_HYST) last = M_OFF;
      else if (t >= T_L1_MAX + T_HYST) last = M_L2;
      break;
    case M_L2:
      if (t <  T_L1_MAX - T_HYST) last = M_L1;
      else if (t >= T_L2_MAX + T_HYST) last = M_L3;
      break;
    case M_L3:
      if (t <  T_L2_MAX - T_HYST) last = M_L2;
      break;
  }
  return last;
}

/* ================== Nút nhấn ================== */
void handleButton(){
  bool raw = digitalRead(PIN_BTN);
  uint32_t now = millis();
  if (raw != btnStable && (now - btnLastChangeMs) > DEBOUNCE_MS) {
    btnStable = raw; btnLastChangeMs = now;

    if (btnStable == LOW) { // press
      btnWasPressed = true; btnPressStartMs = now;
    } else if (btnWasPressed) { // release
      uint32_t dur = now - btnPressStartMs;
      if (dur >= LONG_PRESS_MS) {
        isAuto = !isAuto;            // đổi chế độ
        if (!isAuto && manLevel==M_OFF) manLevel = M_L1; // vào Manual bật luôn M1
      } else {
        if (!isAuto) {
          // Manual cycle: OFF -> 1 -> 2 -> 3 -> OFF
          switch (manLevel){
            case M_OFF: manLevel = M_L1; break;
            case M_L1:  manLevel = M_L2; break;
            case M_L2:  manLevel = M_L3; break;
            case M_L3:  manLevel = M_OFF; break;
          }
        }
      }
      btnWasPressed = false;
    }
  }
}

/* ================== Tach ISR ================== */
void onTach(){ tachPulses++; } // nhiều quạt: 2 xung/vòng

/* ================== Apply EN + PWM theo level ================== */
void applyLevel(uint8_t level){
  // level: 0..3 tương ứng Lv0..Lv3
  if (level == 0){
    // TẮT HẲN: cắt GND + PWM=0
    setDuty(0.0);
    digitalWrite(PIN_EN, LOW);   // MOSFET OFF → quạt mất GND
  } else {
    // BẬT GND trước, rồi đặt PWM
    digitalWrite(PIN_EN, HIGH);  // MOSFET ON → có GND
    // Kick-start nếu vừa bật và mức thấp
    if (level == 1){
      setDuty(0.60); delay(300);
      setDuty(0.30);
    } else if (level == 2){
      setDuty(0.60);
    } else {
      setDuty(1.00);
    }
  }
}

/* ================== Setup ================== */
void setup() {
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_TACH, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_TACH), onTach, FALLING);

  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, LOW);  // mặc định tắt quạt (cắt mass)

  setupPwm25k();
  sensors.begin();

  lcd.init(); lcd.backlight();
  lcd.clear();

  Serial.begin(115200);
  lastRpmMs = millis();
}

/* ================== Loop ================== */
void loop() {
  handleButton();

  // Đọc nhiệt độ (0.5s/lần)
  static uint32_t lastT = 0;
  static float tempC = 25.0;
  if (millis() - lastT > 500) {
    sensors.requestTemperatures();
    float t = sensors.getTempCByIndex(0);
    if (t > -40 && t < 125) tempC = t;
    lastT = millis();
  }

  // Xác định duty/level theo Auto/Manual
  if (isAuto) {
    ManualLevel lv = levelFromTemp(tempC);
    switch (lv){
      case M_OFF: targetDuty = DUTY_OFF; break;
      case M_L1:  targetDuty = DUTY_L1;  break;
      case M_L2:  targetDuty = DUTY_L2;  break;
      case M_L3:  targetDuty = DUTY_L3;  break;
    }
  } else {
    switch (manLevel){
      case M_OFF: targetDuty = DUTY_OFF; break;
      case M_L1:  targetDuty = DUTY_L1;  break;
      case M_L2:  targetDuty = DUTY_L2;  break;
      case M_L3:  targetDuty = DUTY_L3;  break;
    }
  }

  // Kick-start & Slew mượt (chỉ thay đổi currentDuty; EN xử lý riêng)
  static bool justKick = false;
  static uint32_t kickMs = 0;
  if (currentDuty < 0.2 && targetDuty >= DUTY_L1) {
    currentDuty = 0.6; setDuty(currentDuty);
    justKick = true; kickMs = millis();
  }
  if (justKick && millis() - kickMs > 400) justKick = false;

  if (!justKick){
    if (currentDuty < targetDuty) currentDuty = min(currentDuty + SLEW_STEP, targetDuty);
    else                          currentDuty = max(currentDuty - SLEW_STEP, targetDuty);
    // Lưu ý: setDuty(currentDuty) sẽ được gọi trong applyLevel(...) bên dưới
  }

  // Tính RPM mỗi 1s
  if (millis() - lastRpmMs > 1000) {
    noInterrupts(); uint32_t p = tachPulses; tachPulses = 0; interrupts();
    rpm = (uint16_t)((p / 2.0) * 60.0); // giả sử 2 xung/vòng
    lastRpmMs = millis();
  }

  // Xác định level để điều khiển EN + PWM
  uint8_t levelShown = 0;
  if (targetDuty <= (DUTY_OFF+0.01)) levelShown = 0;
  else if (targetDuty < (DUTY_L2 - 0.05)) levelShown = 1;
  else if (targetDuty < (DUTY_L3 - 0.05)) levelShown = 2;
  else levelShown = 3;

  // Áp dụng EN + PWM (gọi mỗi vòng, idempotent)
  applyLevel(levelShown);
  // Đồng bộ hóa setDuty với currentDuty (slew), trừ khi đang Lv0
  if (levelShown > 0) setDuty(currentDuty);

  // ====== LCD ======
  lcd.setCursor(0,0);
  lcd.print(isAuto ? "AUTO, " : "MANUAL, ");
  lcd.print("T=");
  lcd.print(tempC,1);
  lcd.print((char)223); lcd.print("C");

  lcd.setCursor(0,1);
  lcd.print("Muc: "); lcd.print(levelShown);lcd.print(",");
  lcd.print(" PWM=");
  int dutyPct = (int)(targetDuty * 100.0 + 0.5);
  if (dutyPct < 100) lcd.print(" ");
  if (dutyPct < 10)  lcd.print(" ");
  lcd.print(dutyPct); lcd.print("% ");

  delay(100);
}

#include <Arduino.h>

// TD-8120MG 舵机配置
// PWM范围: 500us~2500us
#define SERVO_PIN 3         // 舵机信号引脚
#define SERVO_CHANNEL 0     // LEDC通道 (0-15)
#define SERVO_FREQ 333      // 舵机频率 333Hz (周期3ms，适合数字舵机)
#define SERVO_RESOLUTION 13 // 分辨率 13位 (0-8191)

// 脉宽定义（单位：ms）
#define PULSE_MIN 0.5       // 最小脉宽 500us
#define PULSE_MAX 2.5       // 最大脉宽 2500us
#define PERIOD_MS (1000.0 / SERVO_FREQ)  // 周期 = 3ms

void servoWrite(int angle) {
  // 限制角度范围
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;

  // 将角度转换为脉宽 (0.5ms - 2.5ms)
  // 0度 = 0.5ms, 180度 = 2.5ms
  float pulseWidth = PULSE_MIN + (angle / 180.0) * (PULSE_MAX - PULSE_MIN);

  // 计算占空比
  // 333Hz = 3ms周期
  // 13位分辨率: 最大值 = 2^13 - 1 = 8191
  uint32_t maxDuty = (1 << SERVO_RESOLUTION) - 1;  // 8191
  uint32_t duty = (uint32_t)((pulseWidth / PERIOD_MS) * maxDuty);

  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print("° -> PulseWidth: ");
  Serial.print(pulseWidth, 3);
  Serial.print("ms -> Duty: ");
  Serial.print(duty);
  Serial.print(" / ");
  Serial.println(maxDuty);

  ledcWrite(SERVO_CHANNEL, duty);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== TD-8120MG Servo Test ===");
  Serial.print("Frequency: ");
  Serial.print(SERVO_FREQ);
  Serial.println(" Hz");
  Serial.print("Period: ");
  Serial.print(PERIOD_MS, 2);
  Serial.println(" ms");
  Serial.print("Resolution: ");
  Serial.print(SERVO_RESOLUTION);
  Serial.println(" bits");
  Serial.print("Pulse Range: ");
  Serial.print(PULSE_MIN * 1000);
  Serial.print("us - ");
  Serial.print(PULSE_MAX * 1000);
  Serial.println("us");

  // 配置LEDC用于舵机控制
  Serial.println("\nConfiguring LEDC...");
  ledcSetup(SERVO_CHANNEL, SERVO_FREQ, SERVO_RESOLUTION);

  Serial.print("Attaching pin GPIO");
  Serial.print(SERVO_PIN);
  Serial.println(" to LEDC channel...");
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);

  Serial.println("\nSetup complete!");
  Serial.println("Testing center position (90°)...");

  // 初始化舵机到90度位置（中间位置）
  servoWrite(0);
  delay(2000);
}
void loop() {
  servoWrite(0);

  delay(5000);
  servoWrite(90);

  delay(5000);
}
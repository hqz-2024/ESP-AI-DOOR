/*
 * ========================================
 * ESP32 NFC门禁系统 - 基于MFRC522和舵机
 * ========================================
 *
 * 功能说明：
 * 1. 使用MFRC522读取MIFARE Classic NFC卡片的UID
 * 2. 验证卡片UID是否匹配预设的授权卡片
 * 3. 验证成功后，控制舵机开门，延迟后自动关门
 *
 * ========================================
 * 硬件接线说明
 * ========================================
 *
 * TD-8120MG舵机接线：
 * ┌─────────────┬──────────┐
 * │ 舵机引脚    │ ESP32    │
 * ├─────────────┼──────────┤
 * │ 黄色(信号)  │ GPIO 1   │
 * │ 红色(电源)  │ 5V       │
 * │ 棕色(地)    │ GND      │
 * └─────────────┴──────────┘
 *
 * MFRC522 NFC读卡器接线：
 * ┌─────────────┬──────────┐
 * │ MFRC522     │ ESP32    │
 * ├─────────────┼──────────┤
 * │ 3.3V        │ 3.3V     │
 * │ RST         │ GPIO 38  │
 * │ GND         │ GND      │
 * │ SDA(SS)     │ GPIO 45  │
 * │ MOSI        │ GPIO 48  │
 * │ MISO        │ GPIO 39  │
 * │ SCK         │ GPIO 47  │
 * └─────────────┴──────────┘
 *
 * 注意：MFRC522必须使用3.3V供电，不能使用5V！
 */

#include <SPI.h>      // SPI通信库
#include <MFRC522.h>  // MFRC522 RFID/NFC读卡器库

// ========================================
// MFRC522 NFC读卡器引脚定义
// ========================================
#define SS_PIN 14       // SPI片选引脚 (Slave Select)
#define RST_PIN 38      // 复位引脚
#define MOSI_PIN 4     // SPI主机输出从机输入引脚
#define MISO_PIN 39     // SPI主机输入从机输出引脚
#define SCK_PIN 5      // SPI时钟引脚

// ========================================
// TD-8120MG 数字舵机配置
// ========================================
#define SERVO_PIN 48         // 舵机PWM信号输出引脚
#define SERVO_CHANNEL 0     // ESP32 LEDC通道 (0-15可选)
#define SERVO_FREQ 333      // PWM频率 333Hz (周期3ms，适合数字舵机)
#define SERVO_RESOLUTION 13 // PWM分辨率 13位 (占空比范围: 0-8191)

// 舵机脉宽定义（单位：毫秒）
#define PULSE_MIN 0.5       // 最小脉宽 0.5ms (对应0度)
#define PULSE_MAX 2.5       // 最大脉宽 2.5ms (对应180度)
#define PERIOD_MS (1000.0 / SERVO_FREQ)  // PWM周期 = 3ms

#define CLOSEDOOR_DELAY 3000
#define OPENDOOR_DEWG  30
#define CLOSEDOOR_DEG  90


// ========================================
// 全局对象和变量
// ========================================
MFRC522 rfid(SS_PIN, RST_PIN);  // 创建MFRC522实例，传入SS和RST引脚
MFRC522::MIFARE_Key key;        // MIFARE卡片密钥结构体
byte nuidPICC[4];               // 存储读取到的卡片UID (4字节)

// ========================================
// 授权卡片UID定义
// ========================================
// 这里定义允许开门的卡片UID
// 十进制: 83 191 16 25
// 十六进制: 0x53 0xBF 0x10 0x19
byte str0 = 0x53;  // UID第1字节
byte str1 = 0xBF;  // UID第2字节
byte str2 = 0x10;  // UID第3字节
byte str3 = 0x19;  // UID第4字节

// ========================================
// 门禁状态控制变量
// ========================================
bool doorOpen = false;                // 门的当前状态 (false=关闭, true=打开)
bool cardPresent = false;             // 卡片是否在读取区 (false=离开, true=在场)
unsigned long cardLeftTime = 0;       // 卡片离开的时间戳
const unsigned long DOOR_DELAY = CLOSEDOOR_DELAY;  // 卡片离开后保持开门的时间 (10秒)

/**
 * ========================================
 * 函数: servoWrite
 * ========================================
 * 功能: 控制舵机转动到指定角度
 *
 * 参数:
 *   @param angle - 目标角度 (0-180度)
 *                  0度   = 关门位置
 *                  90度  = 开门位置
 *                  180度 = 最大角度
 *
 * 工作原理:
 *   1. 将角度转换为对应的PWM脉宽 (0.5ms ~ 2.5ms)
 *   2. 根据PWM频率和分辨率计算占空比数值
 *   3. 通过LEDC输出PWM信号控制舵机
 *
 * 计算公式:
 *   脉宽(ms) = 0.5 + (角度/180) × 2.0
 *   占空比 = (脉宽 / 周期) × (2^分辨率 - 1)
 *
 * 示例:
 *   servoWrite(0);   // 舵机转到0度 (关门)
 *   servoWrite(90);  // 舵机转到90度 (开门)
 */
void servoWrite(int angle) {
  // 限制角度范围在0-180度之间
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;

  // 将角度转换为脉宽 (0.5ms - 2.5ms)
  // 0度 = 0.5ms, 90度 = 1.5ms, 180度 = 2.5ms
  float pulseWidth = PULSE_MIN + (angle / 180.0) * (PULSE_MAX - PULSE_MIN);

  // 计算PWM占空比
  // 333Hz频率 = 3ms周期
  // 13位分辨率: 最大值 = 2^13 - 1 = 8191
  uint32_t maxDuty = (1 << SERVO_RESOLUTION) - 1;  // 8191
  uint32_t duty = (uint32_t)((pulseWidth / PERIOD_MS) * maxDuty);

  // 输出PWM信号到舵机
  ledcWrite(SERVO_CHANNEL, duty);
}

/**
 * ========================================
 * 函数: setup
 * ========================================
 * 功能: 系统初始化函数，在ESP32启动时执行一次
 *
 * 初始化步骤:
 *   1. 初始化串口通信 (115200波特率)
 *   2. 配置LEDC用于舵机PWM控制
 *   3. 初始化舵机到关门位置 (0度)
 *   4. 初始化SPI总线
 *   5. 初始化MFRC522 NFC读卡器
 *   6. 验证MFRC522通信是否正常
 *   7. 设置MIFARE卡片默认密钥
 */
void setup() {
  // 初始化串口通信，波特率115200
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32 NFC门禁系统启动 ===");

  // ========================================
  // 1. 初始化舵机
  // ========================================
  // 配置LEDC (LED Control) 用于生成PWM信号
  ledcSetup(SERVO_CHANNEL, SERVO_FREQ, SERVO_RESOLUTION);
  // 将LEDC通道绑定到舵机引脚
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);
  // 初始化舵机到0度位置（关门状态）
  servoWrite(CLOSEDOOR_DEG);
  Serial.println("舵机初始化完成 (0度关门位置)");

  // ========================================
  // 2. 初始化SPI总线
  // ========================================
  // 使用自定义引脚初始化SPI
  // 参数: (SCK, MISO, MOSI, SS)
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  delay(100);
  Serial.println("SPI总线初始化完成");

  // ========================================
  // 3. 初始化MFRC522读卡器
  // ========================================
  rfid.PCD_Init();
  delay(100);

  // 读取MFRC522固件版本，验证通信是否正常
  byte version = rfid.PCD_ReadRegister(rfid.VersionReg);
  Serial.print(F("MFRC522固件版本: 0x"));
  Serial.print(version, HEX);

  // 检查版本号是否有效
  if (version == 0x91) {
    Serial.println(F(" (v1.0) - 通信正常"));
  } else if (version == 0x92) {
    Serial.println(F(" (v2.0) - 通信正常"));
  } else if (version == 0x00 || version == 0xFF) {
    // 通信失败，版本号为0x00或0xFF
    Serial.println(F(" *** 通信失败! ***"));
    Serial.println(F("请检查:"));
    Serial.println(F("  1. 接线是否正确"));
    Serial.println(F("  2. 3.3V供电是否稳定"));
    Serial.println(F("  3. SPI引脚定义是否正确"));
    Serial.println(F("  4. 尝试交换MISO/MOSI引脚"));
    while(1) { delay(1000); } // 停止程序，进入死循环
  } else {
    Serial.print(F(" (未知版本: 0x"));
    Serial.print(version, HEX);
    Serial.println(F(") - 可能仍可正常工作"));
  }

  // ========================================
  // 4. 设置MIFARE卡片密钥
  // ========================================
  // 设置为默认密钥 FF FF FF FF FF FF
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }

  // 系统就绪提示
  Serial.println(F("\n========================================"));
  Serial.println(F("系统初始化完成！"));
  Serial.println(F("等待刷卡..."));
  Serial.println(F("========================================\n"));
}

/**
 * ========================================
 * 函数: loop
 * ========================================
 * 功能: 主循环函数，持续检测NFC卡片并控制门禁
 *
 * 新的工作逻辑:
 *   1. 持续检测授权卡片是否在读取区
 *   2. 卡片在读取区时，保持门开启 (90度)，不开始倒计时
 *   3. 检测到卡片离开时，记录离开时间，开始倒计时
 *   4. 卡片离开后10秒内如果再次靠近，取消倒计时
 *   5. 卡片离开10秒后，自动关门 (0度)
 */
void loop() {
  bool currentCardDetected = false;  // 本次循环是否检测到授权卡片

  // ========================================
  // 1. 尝试读取卡片 (不使用Halt，保持持续检测)
  // ========================================
  // 关键：不调用PICC_HaltA()，这样卡片会保持激活状态
  // 可以持续被检测到

  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);


  // 尝试唤醒卡片
  if (rfid.PICC_WakeupA(bufferATQA, &bufferSize) == MFRC522::STATUS_OK ||
      rfid.PICC_IsNewCardPresent()) {

    // 尝试读取卡片序列号
    if (rfid.PICC_ReadCardSerial()) {

      // ========================================
      // 2. 获取卡片类型
      // ========================================
      MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);

      // 检查是否为MIFARE Classic类型卡片
      if (piccType == MFRC522::PICC_TYPE_MIFARE_MINI ||
          piccType == MFRC522::PICC_TYPE_MIFARE_1K ||
          piccType == MFRC522::PICC_TYPE_MIFARE_4K) {

        // ========================================
        // 3. 读取卡片UID
        // ========================================
        for (byte i = 0; i < 4; i++) {
          nuidPICC[i] = rfid.uid.uidByte[i];
        }

        // ========================================
        // 4. 验证UID是否匹配授权卡片
        // ========================================
        if (nuidPICC[0] == str0 &&
            nuidPICC[1] == str1 &&
            nuidPICC[2] == str2 &&
            nuidPICC[3] == str3) {

          // 检测到授权卡片
          currentCardDetected = true;

          // 如果门是关闭状态，则开门
          if (!doorOpen) {
            Serial.println("\n========================================");
            Serial.println("检测到授权卡片！");
            Serial.print(F("卡片UID (十进制): "));
            printDec(rfid.uid.uidByte, rfid.uid.size);
            Serial.println();
            Serial.print(F("卡片UID (十六进制): "));
            printHex(rfid.uid.uidByte, rfid.uid.size);
            Serial.println();
            Serial.println(F("\n✓ 授权通过！"));
            Serial.println(F("正在开门..."));

            // 控制舵机开门 (转到90度)
            servoWrite(OPENDOOR_DEWG);
            doorOpen = true;
            Serial.println(F("门已打开"));
            Serial.println(F("卡片保持在读取区时，门将持续开启"));
            Serial.println(F("========================================\n"));
          }

          // 如果卡片之前离开过，现在又回来了
          if (!cardPresent) {
            Serial.println(F("卡片重新进入读取区，取消关门倒计时"));
          }

          // 标记卡片在场
          cardPresent = true;

        } else {
          // UID不匹配 - 拒绝访问
          if (!doorOpen) {  // 只在门关闭时提示，避免重复输出
            Serial.println("\n========================================");
            Serial.println(F("检测到卡片"));
            Serial.print(F("卡片UID (十六进制): "));
            printHex(rfid.uid.uidByte, rfid.uid.size);
            Serial.println();
            Serial.println(F("\n✗ 访问被拒绝！"));
            Serial.println(F("未授权的卡片"));
            Serial.println(F("========================================\n"));
          }
        }
      }
    }
  }

  // ========================================
  // 5. 检测卡片离开事件
  // ========================================
  // 如果上一次循环卡片在场，但本次循环未检测到卡片
  if (cardPresent && !currentCardDetected) {
    // 卡片刚刚离开
    cardPresent = false;
    cardLeftTime = millis();  // 记录卡片离开的时间

    Serial.println("\n========================================");
    Serial.println(F("卡片已离开读取区"));
    Serial.println(F("10秒后自动关门..."));
    Serial.println(F("========================================\n"));
  }

  // ========================================
  // 6. 检查是否需要关门
  // ========================================
  // 条件：门是打开状态 && 卡片已离开 && 离开时间超过10秒
  if (doorOpen && !cardPresent && (millis() - cardLeftTime > DOOR_DELAY)) {
    Serial.println("\n========================================");
    Serial.println(F("卡片离开已超过10秒"));
    Serial.println(F("正在关门..."));

    // 控制舵机关门 (回到0度)
    servoWrite(CLOSEDOOR_DEG);
    doorOpen = false;

    Serial.println(F("门已关闭"));
    Serial.println(F("========================================\n"));
  }

  // 短暂延迟，避免CPU占用过高
  delay(200);
}



/**
 * ========================================
 * 函数: printHex
 * ========================================
 * 功能: 将字节数组以十六进制格式输出到串口
 *
 * 参数:
 *   @param buffer     - 指向字节数组的指针
 *   @param bufferSize - 数组长度 (字节数)
 *
 * 输出格式:
 *   每个字节以2位十六进制显示，不足补0
 *   例如: 53 BF 10 19
 *
 * 示例:
 *   byte data[] = {0x53, 0xBF, 0x10, 0x19};
 *   printHex(data, 4);  // 输出: 53 BF 10 19
 */
void printHex(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    // 如果字节值小于0x10，前面补0
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    // 以十六进制格式输出
    Serial.print(buffer[i], HEX);
  }
}

/**
 * ========================================
 * 函数: printDec
 * ========================================
 * 功能: 将字节数组以十进制格式输出到串口
 *
 * 参数:
 *   @param buffer     - 指向字节数组的指针
 *   @param bufferSize - 数组长度 (字节数)
 *
 * 输出格式:
 *   每个字节以十进制显示，不足2位补0
 *   例如: 83 191 16 25
 *
 * 示例:
 *   byte data[] = {83, 191, 16, 25};
 *   printDec(data, 4);  // 输出: 83 191 16 25
 */
void printDec(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    // 如果字节值小于10，前面补0
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    // 以十进制格式输出
    Serial.print(buffer[i], DEC);
  }
}

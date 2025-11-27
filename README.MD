# ESP32-S3 NFC智能门禁系统

基于ESP32-S3的NFC门禁控制系统，支持MIFARE Classic卡片UID验证和智能门控。

---

## 📋 项目概述

本项目实现了一个基于NFC卡片的智能门禁系统，通过读取MIFARE Classic卡片的UID进行身份验证，验证通过后控制舵机开门。系统支持卡片持续检测和延迟关门功能。

### 核心特性

- ✅ **UID验证**：读取卡片唯一标识符(UID)进行身份验证
- ✅ **智能门控**：卡片在读取区时门保持开启，离开后延迟关门
- ✅ **持续检测**：卡片保持不动时持续检测，避免误关门
- ✅ **可配置参数**：开门角度、关门角度、延迟时间、授权UID均可自定义

---

## 🛠️ 硬件清单

| 组件 | 型号 | 数量 | 说明 |
|------|------|------|------|
| 主控板 | ESP32-S3 | 1 | 主控芯片 |
| NFC读卡器 | MFRC522 | 1 | 13.56MHz RFID模块 |
| 舵机 | TD-8120MG | 1 | 数字舵机，用于门锁控制 |
| NFC卡片 | MIFARE Classic 1K | 1+ | 支持读写的NFC卡片 |

---

## 🔌 硬件接线

### MFRC522 NFC读卡器

| MFRC522引脚 | ESP32-S3引脚 | 说明 |
|-------------|--------------|------|
| 3.3V | 3.3V | 电源 (⚠️ 必须3.3V) |
| RST | GPIO 38 | 复位引脚 |
| GND | GND | 地线 |
| SDA (SS) | GPIO 14 | SPI片选 |
| MOSI | GPIO 4 | SPI数据输出 |
| MISO | GPIO 39 | SPI数据输入 |
| SCK | GPIO 5 | SPI时钟 |

### TD-8120MG 舵机

| 舵机引脚 | ESP32-S3引脚 | 说明 |
|----------|--------------|------|
| 黄色 (信号) | GPIO 48 | PWM控制信号 |
| 红色 (电源) | 5V | 电源 |
| 棕色 (地) | GND | 地线 |

> ⚠️ **重要提示**：MFRC522必须使用3.3V供电，使用5V会损坏模块！

---

## 📦 功能模块

### 1. NFC卡片读取模块

**文件位置**：`main/main.ino` - `loop()` 函数

**功能说明**：
- 使用MFRC522读取MIFARE Classic卡片
- 支持MIFARE Mini/1K/4K类型卡片
- 使用 `PICC_WakeupA()` 持续唤醒卡片
- 读取卡片4字节UID

**工作流程**：
1. 主动唤醒卡片 (`PICC_WakeupA()`)
2. 检测是否有新卡片 (`PICC_IsNewCardPresent()`)
3. 读取卡片序列号 (`PICC_ReadCardSerial()`)
4. 获取卡片类型和UID
5. 进行身份验证

---

### 2. UID验证模块

**文件位置**：`main/main.ino` - `loop()` 函数

**功能说明**：
- 将读取的4字节UID与授权UID进行比对
- 授权UID：`0x53 0xBF 0x10 0x19` (十六进制)
- 对应十进制：`83 191 16 25`

**关键参数**：
```cpp
byte str0 = 0x53;  // UID第1字节
byte str1 = 0xBF;  // UID第2字节
byte str2 = 0x10;  // UID第3字节
byte str3 = 0x19;  // UID第4字节
```

**验证逻辑**：
```
读取卡片UID → 逐字节比对 → 返回验证结果
```

---

### 3. 舵机控制模块

**文件位置**：`main/main.ino` - `servoWrite()` 函数

**功能说明**：
- 使用ESP32 LEDC生成PWM信号
- 控制舵机转动到指定角度
- 支持0-180度范围

**关键参数**：
```cpp
#define SERVO_PIN 48           // 舵机控制引脚
#define SERVO_FREQ 333         // PWM频率 333Hz
#define SERVO_RESOLUTION 13    // 13位分辨率
#define OPENDOOR_DEWG 33       // 开门角度
#define CLOSEDOOR_DEG 90       // 关门角度
```

**PWM计算公式**：
```
脉宽(ms) = 0.5 + (角度/180) × 2.0
占空比 = (脉宽 / 3ms) × 8191
```

---

### 4. 智能门控模块

**文件位置**：`main/main.ino` - `loop()` 函数

**功能说明**：
- 卡片在读取区时门保持开启
- 卡片离开后开始倒计时
- 倒计时结束后自动关门
- 卡片重新靠近可取消倒计时

**关键参数**：
```cpp
#define CLOSEDOOR_DELAY 3000   // 关门延迟时间 (毫秒)
bool doorOpen = false;         // 门状态
bool cardPresent = false;      // 卡片在场状态
```

**状态机逻辑**：
```
[门关闭] → 授权卡片靠近 → [门打开]
[门打开] → 卡片持续在场 → [门保持打开]
[门打开] → 卡片离开 → [倒计时3秒]
[倒计时] → 卡片再次靠近 → [取消倒计时，门保持打开]
[倒计时] → 3秒到期 → [门关闭]
```

---

### 5. 持续检测模块

**文件位置**：`main/main.ino` - `loop()` 函数

**功能说明**：
- 使用 `PICC_WakeupA()` 主动唤醒卡片
- 即使卡片静止不动也能持续检测
- 避免误判卡片离开

**技术细节**：
```cpp
// 双重检测机制
if (rfid.PICC_WakeupA(bufferATQA, &bufferSize) == MFRC522::STATUS_OK ||
    rfid.PICC_IsNewCardPresent()) {
    // 检测到卡片
}
```

---

## 🚀 使用说明

### 1. 环境准备

**所需软件**：
- Arduino IDE 或 PlatformIO
- ESP32开发板支持包
- MFRC522库

**安装依赖**：
```bash
# PlatformIO
pio lib install "MFRC522"

# Arduino IDE
# 库管理器搜索并安装 "MFRC522"
```

### 2. 配置授权卡片UID

在使用门禁系统前，需要先获取你的NFC卡片UID并配置到程序中：

**步骤1：读取卡片UID**
- 上传程序到ESP32
- 将卡片靠近读卡器
- 查看串口输出的UID（十六进制和十进制）

**步骤2：修改授权UID**
在 `main/main.ino` 中修改以下代码：
```cpp
// 将这里的值改为你的卡片UID
byte str0 = 0x53;  // UID第1字节
byte str1 = 0xBF;  // UID第2字节
byte str2 = 0x10;  // UID第3字节
byte str3 = 0x19;  // UID第4字节
```

### 3. 上传程序

1. 连接ESP32-S3到电脑
2. 选择正确的开发板和端口
3. 编译并上传 `main/main.ino`
4. 打开串口监视器 (115200波特率)

### 4. 测试运行

**正常启动输出**：
```
=== NFC DOOR Test ===
ESP32 RFID Door Lock System
MFRC522 Firmware Version: 0x92 (v2.0)
System Ready!
```

**刷卡成功输出**：
```
========================================
检测到授权卡片！
卡片UID (十进制):  83 191 16 25
卡片UID (十六进制):  53 BF 10 19

✓ 授权通过！
正在开门...
门已打开
卡片保持在读取区时，门将持续开启
========================================
```

**刷卡失败输出**：
```
========================================
检测到卡片
卡片UID (十六进制):  AA BB CC DD

✗ 访问被拒绝！
未授权的卡片
========================================
```

---

## ⚙️ 配置参数

所有可配置参数位于 `main/main.ino` 文件顶部：

```cpp
// 舵机角度配置
#define OPENDOOR_DEWG 33       // 开门角度 (0-180度)
#define CLOSEDOOR_DEG 90       // 关门角度 (0-180度)

// 延迟时间配置
#define CLOSEDOOR_DELAY 3000   // 关门延迟 (毫秒)

// 授权卡片UID配置
byte str0 = 0x53;  // UID第1字节
byte str1 = 0xBF;  // UID第2字节
byte str2 = 0x10;  // UID第3字节
byte str3 = 0x19;  // UID第4字节
```

---

## 🔧 故障排除

### 问题1：无法检测到卡片

**可能原因**：
- MFRC522接线错误
- 供电电压不正确 (必须3.3V)
- SPI引脚定义错误

**解决方法**：
1. 检查接线是否正确
2. 确认MFRC522使用3.3V供电
3. 查看串口输出的固件版本号

### 问题2：卡片UID不匹配

**可能原因**：
- 授权UID配置错误
- 使用了未授权的卡片

**解决方法**：
1. 查看串口输出的实际UID
2. 将实际UID配置到程序中
3. 重新上传程序

### 问题3：卡片不动但门自动关闭

**可能原因**：
- 卡片检测逻辑问题
- 延迟时间设置过短

**解决方法**：
1. 检查串口输出是否持续检测到卡片
2. 增加 `CLOSEDOOR_DELAY` 时间
3. 确认使用了 `PICC_WakeupA()` 持续检测

---

## 📁 项目结构

```
ESP-AI-DOOR/
├── main/
│   └── main.ino              # 主程序文件
├── lib/
│   └── rfid-master/          # MFRC522库
├── SERVO-TEST/
│   └── servo.ino             # 舵机测试程序
├── 门锁结构/                  # 3D打印结构文件
│   ├── menjin1-shell.stp     # 外壳模型
│   ├── menjin1-topshell.stp  # 顶盖模型
│   └── ...
├── platformio.ini            # PlatformIO配置
└── README.MD                 # 本文档
```

---

## 📝 技术规格

| 项目 | 规格 |
|------|------|
| 主控芯片 | ESP32-S3 |
| 工作电压 | 3.3V (逻辑) / 5V (舵机) |
| NFC频率 | 13.56 MHz |
| 支持卡片 | MIFARE Classic Mini/1K/4K |
| 舵机PWM | 333Hz, 13位分辨率 |
| 串口波特率 | 115200 |
| 数据块大小 | 16字节 |
| 最大文本长度 | 16字符 |

---

## 📄 许可证

本项目采用开源协议，可自由使用和修改。

---

## 👨‍💻 作者

ESP-AI-DOOR 项目团队

---

## 🔗 相关链接

- [MFRC522库文档](https://github.com/miguelbalboa/rfid)
- [ESP32官方文档](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/)
- [MIFARE Classic规格书](https://www.nxp.com/docs/en/data-sheet/MF1S50YYX_V1.pdf)
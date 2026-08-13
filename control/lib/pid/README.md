# PID 控制器库

## 概述

一个轻量级、零依赖的 PID 控制器实现，同时支持 C++ 面向对象接口和 C 过程式接口。

## 文件

| 文件 | 说明 |
|------|------|
| `pid.h` | 头文件，定义 `Pid` 类、`PidMode_t` 枚举及 C 链接 API |
| `pid.cpp` | 实现文件，包含类成员函数和 C API 包装 |

## 特性

- **双模式支持**：位置式 PID（`PID_MODE_POSITION`）与增量式 PID（`PID_MODE_INCREMENTAL`）
- **抗积分饱和**：位置式 PID 输出饱和时不累积积分项，避免超调
- **C/C++ 兼容**：同一内存布局，C++ 类与 C 结构体可互操作
- **输出限幅**：可设置输出上下限，自动钳位
- **运行时调参**：支持在线修改 `kp`、`ki`、`kd` 及输出范围

## PID 模式说明

| 模式 | 枚举值 | 适用场景 |
|------|--------|----------|
| 位置式 PID | `PID_MODE_POSITION` | 舵机/位置控制，直接输出绝对值 |
| 增量式 PID | `PID_MODE_INCREMENTAL` | 电机速度控制，输出累积值 u(k) |

**位置式 PID** 计算公式：

```
u(k) = Kp * e(k) + Ki * Σe(k) + Kd * (e(k) - e(k-1))
```

**增量式 PID** 计算公式：

```
Δu(k) = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2*e(k-1)+e(k-2))
u(k) = u(k-1) + Δu(k)
```

## C++ API

### 创建与使用

```cpp
#include "pid.h"

// 构造（默认参数：kp=0, ki=0, kd=0, maxOut=100, minOut=-100）
Pid pid(1.0f, 0.5f, 0.1f, 50.0f, -50.0f);

// 设置目标值
pid.setSetpoint(10.0f);

// 更新并获取输出（在控制循环中调用）
float output = pid.update(currentValue);
```

### 运行时调参

```cpp
// 调整 PID 参数
pid.setTunings(2.0f, 1.0f, 0.2f);

// 修改输出限制
pid.setLimits(100.0f, -100.0f);

// 切换模式（会重置内部状态）
pid.setMode(PID_MODE_INCREMENTAL);

// 重置所有历史状态
pid.reset();
```

### 公共数据成员

可直接访问底层参数，便于调试或批量赋值：

```cpp
pid.kp = 2.5f;
pid.ki = 0.8f;
pid.setpoint = 42.0f;
```

### 完整示例：电机速度控制（增量式 PID）

```cpp
#include "pid.h"

int main() {
    Pid speedPid(0.8f, 0.3f, 0.05f, 1000.0f, -1000.0f);
    speedPid.setMode(PID_MODE_INCREMENTAL);
    speedPid.setSetpoint(1800.0f);  // 目标转速 1800 RPM

    while (true) {
        float currentRpm = read_encoder();
        float pwm = speedPid.update(currentRpm);
        set_motor_pwm(pwm);
        // 调用周期：10ms
    }
}
```

## C API

C 语言（或需要 C 链接的场景）通过以下函数操作：

```c
#include "pid.h"

// 创建实例
Pid_t* pid = Pid_Create();
Pid_Init(pid, 1.0f, 0.5f, 0.1f, 50.0f, -50.0f);

// 控制循环
while (1) {
    float current = read_sensor();
    float output = Pid_Update(pid, current);
    apply_output(output);
}

// 清理
Pid_Destroy(pid);
```

### C API 函数一览

| 函数 | 说明 |
|------|------|
| `Pid_Create()` | 堆上创建 PID 实例 |
| `Pid_Destroy(self)` | 销毁实例并释放内存 |
| `Pid_Init(self, kp, ki, kd, maxOut, minOut)` | 初始化参数 |
| `Pid_Update(self, currentValue)` | 更新计算并返回输出 |
| `Pid_SetSetpoint(self, setpoint)` | 设置目标值 |
| `Pid_SetTunings(self, kp, ki, kd)` | 调整 PID 参数 |
| `Pid_SetLimits(self, maxOut, minOut)` | 设置输出范围 |
| `Pid_Reset(self)` | 重置所有历史状态 |
| `Pid_SetMode(self, mode)` | 切换模式（重置状态） |
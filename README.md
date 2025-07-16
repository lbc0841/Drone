# Drone

## 相關 repo

- [Drone](https://github.com/lbc0841/Drone)
- [Drone PCB](https://github.com/lbc0841/DronePCB)
- [Drone Controller](https://github.com/lbc0841/DroneController)
- [傳輸協議](PROTOCOL.md)

## 說明

此無人機主要為研究用途<br>
偏向玩具類無人機

### 整個項目包含三部分

- STM32 內部程式 (本repo): [Drone](https://github.com/lbc0841/Drone)
- 對應的 PCB 版: [Drone PCB (v2)](https://github.com/lbc0841/DronePCB)
- 遙控 APP: [Drone Controller (Android)](https://github.com/lbc0841/DroneController)

(不要問我為甚麼 PCB 是 v2，我是絕對不會說我 v1 板子的藍芽模塊沒接 VDD 的)

### 開發軟體分別為

- STM32CubeIDE
- EasyEDA
- AndroidStudio

## 待完成

- 調整 PID
- 無人機外殼

## 使用這個項目

### Step1. 複製項目

```cmd
git clone https://github.com/lbc0841/Drone.git
git clone https://github.com/lbc0841/DronePCB.git
git clone https://github.com/lbc0841/DroneController.git
```

### Step2. 購買元件/組裝

跟據 [Drone PCB](https://github.com/lbc0841/DronePCB) README.md 的元件表採購

另外你還需要

- 電池 (選擇放電倍率高的)
- DAP-Link
- PID 調適架 (手搓也行)
- 空心杯馬達 (716)

接著將元件焊上 PCB 版

### Step3. 燒入

使用 DAP-Link 燒入 STM32

### Step4. 下載 APP

項目不提供 .apk 文件<br>
用 Android Studio 安裝

### Step5. 調整 PID 增益

由 APP 調整 PID 增益

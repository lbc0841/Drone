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

(不要問我為甚麼 PCB 是 v2，v1 板子洗完發現藍芽模塊沒接 VDD)

### 開發軟體分別為

- STM32CubeIDE
- EasyEDA
- AndroidStudio

## 待完成

- 調整 PID
- 無人機外殼

## 使用這個項目

### 1️⃣ 複製項目

```cmd
git clone https://github.com/lbc0841/Drone.git
git clone https://github.com/lbc0841/DronePCB.git
git clone https://github.com/lbc0841/DroneController.git
```

### 2️⃣ 購買元件/組裝

跟據 [Drone PCB](https://github.com/lbc0841/DronePCB) `README.md` 的元件表採購

另外你還需要

- 電池 (選擇放電倍率高的)
- DAP-Link
- PID 調適架 (手搓也行)
- 空心杯馬達 (716)
- 葉片 (4個)

接著將元件焊上 PCB 版<br>
並組裝馬達

### 3️⃣ 燒入

設置 Run Configuration (為了用 DAP-Link 燒入)

1. 在調適器選擇 `ST-LINK(OpenOCD)`
2. Configuration Script 選擇本項目提供的 `DapLinkDebug.cfg`

使用 DAP-Link 燒入 STM32

### 4️⃣ 下載 APP

用 Android Studio 開啟 [DroneController](https://github.com/lbc0841/DroneController)<br>

在手機開啟 開發人員選項 > 使用USB偵錯<br>
用 USB 線連接手機與電腦

或者直接輸出 .apk

### 5️⃣ 調整 PID 增益

由 APP 調整 PID 增益

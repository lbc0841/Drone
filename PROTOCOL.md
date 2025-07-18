# Protocol

| Header | Command | 數值 * N | Checksum |
| ---- | ---- | -------- | -----|
| 0xAA | 0x?? | 0x?? * N | 0x?? |

(Checksum = Header xor Command xor Val1 xor Val2 ...)

## 設置馬達速度

方向: 手機 -> 裝置

7 bytes

| Header | Command | Speed FL | Speed FR | Speed BL | Speed BR | Checksum |
| ---- | ---- | --------- | --------- | --------- | --------- | ---- |
| 0xAA | 0x01 | 0x00~0xFF | 0x00~0xFF | 0x00~0xFF | 0x00~0xFF | 0x?? |

Speed FL: 左前方馬達速度<br>
Speed FR: 右前方馬達速度<br>
Speed BL: 左後方馬達速度<br>
Speed BR: 右後方馬達速度

## 接收 QMI8658 數值

方向: 裝置 -> 手機

19 bytes

| Header | Command | ax I | ax F | ay I | ay F | az I | az F | gx I | gx F | gy I | gy F| gz I | gz F | Pitch I | Pitch F | Roll I | Roll F| Checksum |
| ---- | ---- | --------- | --------- | --------- | --------- | --------- | --------- | --------- | --------- | --------- | --------- | --------- | --------- | --------- | --------- | --------- | --------- | ---- |
| 0xAA | 0x02 | 0x00~0xFF | 0x00~0x63 | 0x00~0xFF | 0x00~0x63 | 0x00~0xFF | 0x00~0x63 | 0x00~0xFF | 0x00~0xFF | 0x00~0xFF | 0x00~0xFF | 0x00~0xFF | 0x00~0xFF | 0x00~0xFF | 0x00~0xFF | 0x00~0xFF | 0x00~0xFF | 0x?? |

a? I/F: ？軸加速度 (整數/小數 部分)<br>
g? I/F: ？軸角速度 (整數/小數 部分)

Pitch I/F: Pitch (整數/小數 部分)<br>
Roll I/F: Roll (整數/小數 部分)

> e.g.<br>
ax I = X軸加速度 (整數部分)

## 接收 LPS22HB 數值

LPS22HB 目前僅觀賞用途

## 設置 PID 增益

方向: 手機 -> 裝置

6 bytes

| Header | Command | Kp | Ki | Kd | Checksum |
| ---- | ---- | --------- | --------- | --------- | ---- |
| 0xAA | 0x01 | 0x00~0xFF | 0x00~0xFF | 0x00~0xFF | 0x?? |

Kp: 比例項增益<br>
Ki: 積分項增益<br>
Kd: 微分項增益

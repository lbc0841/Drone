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

| Header | Command | ax I | ax F | ay I | ay F | az I | az F |
| ---- | ---- | --------- | --------- | --------- | --------- | --------- | --------- |
| 0xAA | 0x02 | 0x00~0xFF | 0x00~0xFF | 0x00~0xFF | 0x00~0xFF | 0x00~0xFF | 0x00~0xFF |

| gx I | gx F | gy I | gy F| gz I | gz F |
| --------- | --------- | --------- | --------- | --------- | --------- |
| 0x00~0xFF | 0x00~0xFF | 0x00~0xFF | 0x00~0xFF | 0x00~0xFF | 0x00~0xFF |

| Pitch I | Pitch F | Roll I | Roll F| Checksum |
| --------- | --------- | --------- | --------- | ---- |
| 0x00~0xFF | 0x00~0xFF | 0x00~0xFF | 0x00~0xFF | 0x?? |

ax I: X 軸加速度 (整數部分)<br>
ax F: X 軸加速度 (小數部分)<br>
ay I: Y 軸加速度 (整數部分)<br>
ay F: Y 軸加速度 (小數部分)<br>
az I: Z 軸加速度 (整數部分)<br>
az F: Z 軸加速度 (小數部分)

gx I: X 軸角速度 (整數部分)<br>
gx F: X 軸角速度 (小數部分)<br>
gy I: Y 軸角速度 (整數部分)<br>
gy F: Y 軸角速度 (小數部分)<br>
gz I: Z 軸角速度 (整數部分)<br>
gz F: Z 軸角速度 (小數部分)

Pitch I: Pitch (整數部分)<br>
Pitch F: Pitch (小數部分)<br>
Roll I: Roll (整數部分)<br>
Roll F: Roll (小數部分)

## 接收 LPS22HB 數值

LPS22HB 目前僅裝飾用途

## 設置 PID 增益

方向: 手機 -> 裝置

6 bytes

| Header | Command | Kp | Ki | Kd | Checksum |
| ---- | ---- | --------- | --------- | --------- | ---- |
| 0xAA | 0x01 | 0x00~0xFF | 0x00~0xFF | 0x00~0xFF | 0x?? |

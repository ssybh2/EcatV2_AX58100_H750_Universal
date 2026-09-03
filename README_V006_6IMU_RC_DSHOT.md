# ProductCode 0x06 — 6IMU + DJI RC + DShot

当前分支：`feature/6imu-rc-dshot-pdo-v006`

## Profile

- ProductCode `0x00000006`
- application data：M→S `80 B`，S→M `192 B`
- EtherCAT：Outputs `81 B`，Inputs `193 B`
- S→M：6 × IMU `0..125`，diagnostics `126..159`，DJI RC `160..178`，reserved `179..191`
- M→S：DShot `0..7`，reserved `8..79`

## SOES mapping

S→M application data 为 24 × uint64_t。加上 1 个 `slave_status` mapping，SM3 总数为 25：

```c
#define MAX_MAPPINGS_SM3 25
```

旧值 21 会在 PREOP→SAFEOP 报 `0x001E Invalid input configuration`。

链接脚本必须将 DMA buffer 标记为：

```ld
.dma_buffer (NOLOAD) :
```

## EEPROM：已验证方法

当前已验证 2048-byte AX58100 SII **没有静态 RxPDO / TxPDO category**；实际 PDO/SM mapping 由 H750 上 SOES CoE object dictionary 动态提供。

0x06 镜像由已知可工作的 ProductCode 0x05 EEPROM 复制，只修改：

```text
byte offset 0x14: 0x05 → 0x06
```

其余 2047 bytes 不变。实际烧写后已验证：

```text
Product Code : 00000006
Checksum     : 009C
calculated   : 009C
Output       : 81 B
Input        : 193 B
SAFE_OP      : State 4
```

仓库镜像：`ecat/device/eeprom_0x06_6imu_rc_dshot.bin`

不要使用旧 ProductCode 0x03 的 `slave.bin` 作为 0x06 烧写基线。

Master 对应分支：
<https://github.com/ssybh2/EcatV2_Master/tree/feature/6imu-rc-dshot-pdo-v006>

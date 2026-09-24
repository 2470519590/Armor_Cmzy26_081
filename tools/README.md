# 装甲板 CAN 调参与报文工具

`armor_can_tool.py` 是 Python 3.10 命令行工具，直接通过 `ctypes` 调用 CAN 分析仪厂商的 64 位 `ControlCAN.dll`。结构体和 API 调用方式来自本机示例目录：

`C:\Users\24705\Desktop\文档\CAN分析仪二次开发示例源代码20250326\python(x64)\python3.8.0推荐`

## 安装与 DLL

在 conda 环境中运行：

```powershell
conda create -n py310 python=3.10
conda activate py310
python --version
```

默认 DLL 路径是 `tools/vendor/ControlCAN.dll`。该文件是示例目录中的 64 位 DLL；必须使用 64 位 Python。也可以用 `--dll` 指向其他同型号 DLL。

示例使用的 `VCI_USBCAN2` 设备类型是 `4`，默认设备序号是 `0`。命令行中的 `--device 4` 表示设备类型，`--device-index 0` 表示同型号设备的序号。

## CAN 配置

默认配置为标准 11 位帧、500 kbps、正常模式：

- `Timing0=0x00`
- `Timing1=0x1C`
- TX 默认通道 `1`
- RX 默认通道 `1`

本项目当前 CAN 总线接在分析仪通道 1。供应商示例使用通道 0/1 只是示例接法，不能替代实际接线配置。发送和接收通道可以分别指定；不要求必须相同。`--channel N` 可同时设置两个通道，单独使用 `--tx-channel` 或 `--rx-channel` 时会覆盖它：

```powershell
python tools\armor_can_tool.py listen --device 4 --tx-channel 1 --rx-channel 1
```

若设备使用其他时序，可传入整数或十六进制值，例如 `--timing0 0x03 --timing1 0x1C`。

## 监听报文

持续打印时间戳、CAN ID、DLC、数据和识别出的 P04 ACK：

```powershell
python tools\armor_can_tool.py listen --device 4 --channel 1
```

按 Ctrl+C 停止。使用 `--csv` 追加保存报文：

```powershell
python tools\armor_can_tool.py listen --csv capture.csv
```

CSV 包含主机 UTC 时间、ControlCAN 时间戳、CAN ID、DLC、十六进制数据和 ACK 解码结果。`*.csv`、`*.log`、`__pycache__` 和 `vendor` 已加入 Git 忽略规则。

## 设置单个装甲板 thr_hit

只向指定 NodeID 发送一次 P04 参数写入，并等待匹配 ACK：

```powershell
python tools\armor_can_tool.py set-thr `
  --device 4 `
  --tx-channel 1 `
  --rx-channel 1 `
  --node 1 `
  --value 250000 `
  --timeout 5
```

也兼容别名 `set-threshold`。NodeID 合法范围为 `1..4`，阈值合法范围为 `1000..67108864`。NodeID=2、阈值=250000 时，发送：

```text
CAN ID = 0x14B
DATA   = 04 90 D0 03 00 00 sequence 00
```

`sequence` 每次命令随机生成。Byte 5 固定为 `0x00`，因此只写 RAM，不发送保存 Flash 的操作；装甲板重启后会恢复默认值。工具不会广播，也不会修改其他 NodeID。

## 监控指定 NodeID

只突出显示指定 NodeID 的 P04 ACK，其他帧仍被接收但不打印：

```powershell
python tools\armor_can_tool.py monitor --node 2
```

也可加 `--csv monitor.csv` 保存全部接收帧。

## ACK 格式与错误

参数写入 ID 为：

```text
NodeBase = 0x130 + (NodeID - 1) * 0x10
SET ID   = NodeBase + 0x0B
ACK ID   = SET ID + 1
```

写入帧 DLC 为 8：`04`、小端 `uint32 threshold`、`00`、`sequence`、`00`。

ACK 必须是 DLC=8，Byte 0=`0x04`，Byte 1 为结果，Byte 2-5 为实际应用值，Byte 6 回显 sequence，Byte 7=`0x00`。工具会校验 ACK CAN ID、P04 编号、sequence、result 和实际应用值：

- `result=0` 且实际值等于请求值：成功
- `result!=0`：设备拒绝或参数无效
- 实际值不等于请求值：报告应用值不匹配
- 超时：报告等待的 ACK ID 和 sequence
- DLL 不存在、设备打开/初始化/启动失败：报告对应 ControlCAN API 和返回值

## 无硬件时的限制

没有实体 CAN 分析仪、正确的 64 位 DLL 和总线连接时，只能进行 Python 语法检查、帮助命令和协议编码/解码测试，不能声称完成硬件联调。工具不会在无设备时伪造发送成功或 ACK。

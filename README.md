# iCar Autopilot 2026

## 编译

```bash
mkdir -p build && cd build
cmake ..
make -j4
```

## API 密钥

密钥只从环境变量读取，禁止提交到 Git：

```bash
export QIANFAN_API_KEY='新的文本大模型密钥'
export VISUAL_API_KEY='新的视觉大模型密钥'
export ICAR_MANUAL_TOKEN='<与电脑端一致的强随机令牌>'
# 可选：只允许指定电脑连接 8080
export ICAR_MANUAL_ALLOWED_IP='192.168.5.101'
# 可选：export QIANFAN_MODEL='账号有权限的模型名'
```

旧密钥已经出现在 Git 历史中，必须在服务商控制台撤销；只修改最新源码不能让旧密钥失效。

## 完整启动

先启动串口/TCP桥接：

```bash
cd build
./boot
```

另开终端，从自然语言指令配置开始：

```bash
cd ~/workspace/icar_autopilot_2026th
python3 src/start.py
```

脚本询问是否启动 boot 时输入 `n`。电脑端可在进入施工区前启动持续监看：

```bash
python tools/manual_control_client.py <小车IP> --token '<强随机令牌>'
```

AUTO 模式仅监看；施工区状态机报警停车并发布 MANUAL 后才开放 WASD。空格急停，`R` 确认后返回自动。

## 安全要求

- 每次控制或协议修改后先架空驱动轮测试。
- 必须保留独立物理急停。
- 人工接管心跳丢失 500 ms 后停车。
- 相机故障、持续丢线和任务完成均保持发送停车命令，不能带着最后速度退出。
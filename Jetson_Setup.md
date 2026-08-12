# Jetson Orin 环境搭建傻瓜说明书

> 目标：真机到手后半天内刷好 Ubuntu + ROS2 + 固定IP + SSH免密  
> 适用：NVIDIA Jetson Orin NX / AGX（JetPack 6.0, Ubuntu 22.04）

---

## 一、刷机（SDK Manager）

### 1.1 准备

| 物品 | 说明 |
|------|------|
| 宿主机 | Ubuntu 22.04 x86_64（虚拟机也可以） |
| USB-C 数据线 | Jetson 进入 Recovery 模式 |
| 键盘+鼠标+显示器 | 第一次启动配置用 |

### 1.2 步骤

```bash
# 1. 在宿主机下载 SDK Manager
wget https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v3.0/sdkm-jetson_2024.2.1_amd64.deb
sudo dpkg -i sdkm-jetson_2024.2.1_amd64.deb

# 2. 启动 SDK Manager
sdkmanager --cli

# 3. 选择:
#    - Jetson Orin → JetPack 6.0 (L4T R36.3.0)
#    - 勾选: Host Machine = NO（我们是 Ubuntu 宿主机, 不是 Jetson 自己）
#    - 勾选: Jetson Linux + CUDA + cuDNN + TensorRT + OpenCV
#    - Storage: NVMe SSD (建议 256GB+)
```

### 1.3 Recovery 模式

```
1. 断开 Jetson 电源
2. 用跳线短接 FC_REC 和 GND（Orin NX: Pin 9-10）
3. 接上 USB-C 到宿主机
4. 重新上电
5. SDK Manager 检测到 Jetson 后自动刷写
6. 刷完拔掉短接线，重启
```

---

## 二、首次开机配置

### 2.1 创建用户

```
用户名: haijing
密码:   haijing2026
hostname: haijing-jetson
```

### 2.2 换国内源（清华源）

```bash
sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak
sudo sed -i 's/ports.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list
sudo apt update
```

---

## 三、ROS 2 Humble 安装

```bash
# 1. 添加 ROS 2 源
sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 2. 安装 ROS 2 Humble (Base 版, 够用)
sudo apt update
sudo apt install -y ros-humble-ros-base

# 3. 安装常用工具
sudo apt install -y python3-rosdep python3-colcon-common-extensions \
  ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-tf2-* ros-humble-vision-msgs

# 4. rosdep 初始化
sudo rosdep init
rosdep update

# 5. 加入 .bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 6. 验证
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener
# 看到 "Hello World" 就是装好了
```

---

## 四、固定 WiFi IP

### 4.1 查看本机网络接口

```bash
nmcli device status
# 找到 WiFi 接口名，一般是 wlan0 或 wlp1s0
```

### 4.2 固定 IP（假设路由器网段 192.168.1.x）

```bash
# 变量替换成你的实际接口名和网段
WIFI_IFACE=wlan0          # 改成你查到的接口名
JETSON_IP=192.168.1.100   # Jetson 的固定 IP
ROUTER_IP=192.168.1.1     # 路由器 IP
DNS_IP=192.168.1.1        # DNS（跟路由器一致）

# 创建连接配置
sudo nmcli connection add \
  type wifi \
  con-name "FixedWiFi" \
  ifname $WIFI_IFACE \
  ipv4.method manual \
  ipv4.addresses $JETSON_IP/24 \
  ipv4.gateway $ROUTER_IP \
  ipv4.dns $DNS_IP \
  wifi.mode infrastructure

# 连接 WiFi（替换成你的 SSID 和密码）
sudo nmcli connection modify FixedWiFi wifi.ssid "你的WiFi名"
sudo nmcli connection modify FixedWiFi wifi-sec.key-mgmt wpa-psk
sudo nmcli connection modify FixedWiFi wifi-sec.psk "你的WiFi密码"

# 激活连接
sudo nmcli connection up FixedWiFi

# 验证
ip addr show $WIFI_IFACE | grep inet
# 输出应包含: inet 192.168.1.100/24
```

### 4.3 备用：命令行连 WiFi

如果上面出问题，用这招：

```bash
sudo nmcli device wifi connect "WiFi名" password "密码"
sudo nmcli connection modify "$(nmcli -t -f NAME c show --active | head -1)" \
  ipv4.method manual ipv4.addresses 192.168.1.100/24 ipv4.gateway 192.168.1.1
```

---

## 五、SSH 免密登录

### 5.1 在开发电脑上生成密钥（Windows 已做可跳过）

```bash
ssh-keygen -t ed25519 -C "your_email@example.com" -f ~/.ssh/id_ed25519
```

### 5.2 把公钥拷到 Jetson

```bash
# 在开发电脑上执行（把 IP 换成 Jetson 的固定 IP）
ssh-copy-id haijing@192.168.1.100
# 输入 Jetson 密码: haijing2026
```

### 5.3 验证

```bash
ssh haijing@192.168.1.100
# 应该直接登录，不提示密码
```

### 5.4 配置 SSH 别名（可选）

在开发电脑的 `~/.ssh/config` 加：

```
Host jetson
    HostName 192.168.1.100
    User haijing
    IdentityFile ~/.ssh/id_ed25519
```

之后直接 `ssh jetson` 秒连。

---

## 六、装 CUDA 版 PyTorch + YOLOv5

```bash
# 1. JetPack 自带 CUDA, 验证一下
nvcc --version  # 应显示 CUDA 12.x
python3 -c "import torch; print(torch.cuda.is_available())"
# 如果报错说明 torch 还没装

# 2. 装 PyTorch (JetPack 6.0 对应 CUDA 12.2)
wget https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/torch-2.3.0-cp310-cp310-linux_aarch64.whl
pip3 install torch-2.3.0-cp310-cp310-linux_aarch64.whl

# 3. 装 torchvision
sudo apt install -y libjpeg-dev zlib1g-dev
git clone --branch v0.18.0 https://github.com/pytorch/vision torchvision
cd torchvision && python3 setup.py install --user

# 4. 验证
python3 -c "import torch; print(torch.cuda.is_available())"  # 应输出 True
```

---

## 七、开机自启 ROS2 节点

```bash
# 创建 systemd 服务
sudo tee /etc/systemd/system/vision-node.service << 'EOF'
[Unit]
Description=HaiYing Vision Node
After=network.target

[Service]
Type=simple
User=haijing
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && python3 /home/haijing/scripts/yolo_detector.py'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# 启用
sudo systemctl enable vision-node
sudo systemctl start vision-node
sudo systemctl status vision-node
```

---

## 八、环境检查清单

刷好后逐条确认：

| # | 检查项 | 命令 | 期望结果 |
|---|--------|------|---------|
| 1 | Ubuntu 版本 | `lsb_release -a` | Ubuntu 22.04 |
| 2 | ROS2 版本 | `ros2 --version` | humble |
| 3 | 固定IP | `hostname -I` | 192.168.1.100 |
| 4 | SSH免密 | `ssh haijing@192.168.1.100` | 不输密码直接进 |
| 5 | CUDA | `nvcc --version` | CUDA 12.x |
| 6 | PyTorch GPU | `python3 -c "import torch; print(torch.cuda.is_available())"` | True |
| 7 | YOLOv5 | `python3 -c "import sys;sys.path.insert(0,'yolov5');from models.common import DetectMultiBackend;print('OK')"` | OK |
| 8 | 自启服务 | `systemctl status vision-node` | active |
| 9 | 磁盘空间 | `df -h /` | >100GB 可用 |
| 10 | ROS2测试 | `ros2 topic list` | /parameter_events, /rosout |

---

## 九、常见问题

**Q: SDK Manager 刷机卡住不动？**  
A: 拔掉 Jetson 电源和 USB，重新进 Recovery 模式再试。确认短接线接触良好。

**Q: nmcli 连不上 WiFi？**  
A: `sudo nmcli radio wifi on` 开启无线，然后 `nmcli device wifi list` 扫描网络。

**Q: SSH 连不上？**  
A: `sudo systemctl status ssh` 确认 SSH 服务在跑。`sudo ufw status` 确认 22 端口没被防火墙挡。

**Q: pip3 install 太慢？**  
A: `pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple`

**Q: 忘记 Jetson IP 了？**  
A: 接显示器+键盘，登录后 `hostname -I` 查看。

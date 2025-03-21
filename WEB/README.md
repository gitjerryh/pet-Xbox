# 机械狗STA控制端树莓派摄像头集成

本项目将ESP32控制的机械狗STA与树莓派摄像头集成，实现更高质量的实时视频流。

## 系统架构

- **ESP32**: 负责机械狗的控制和Web界面提供
- **树莓派**: 运行摄像头服务，提供高质量视频流

## 安装步骤

### 在树莓派上:

1. 安装必要的依赖:
```bash
sudo apt update
sudo apt install -y python3-pip python3-opencv libcamera-dev
pip3 install flask picamera2 numpy opencv-python
```

2. 将`camera_server.py`文件复制到树莓派上，例如放在用户主目录下:
```bash
# 假设文件已通过某种方式传输到树莓派
# 例如使用SCP: scp camera_server.py pi@树莓派IP:/home/pi/
```

3. 设置服务自启动(推荐方法):
```bash
# 将服务文件复制到系统目录
sudo cp camera-service.service /etc/systemd/system/

# 启用服务
sudo systemctl enable camera-service.service

# 启动服务
sudo systemctl start camera-service.service

# 检查服务状态
sudo systemctl status camera-service.service
```

4. 另一种方法 - 使用rc.local设置开机自启动:
```bash
# 编辑rc.local文件
sudo nano /etc/rc.local

# 在exit 0前添加以下行
python3 /home/pi/camera_server.py &

# 保存并退出
```

5. 手动启动摄像头服务:
```bash
python3 /home/pi/camera_server.py
```

### 在ESP32上:

1. 将修改后的代码上传到ESP32
2. ESP32将通过网页界面显示树莓派摄像头的实时画面

## 使用说明

1. 树莓派和ESP32必须在同一个局域网内
2. 访问ESP32的IP地址，网页将自动显示树莓派摄像头的实时画面
3. 如果摄像头画面未显示，请检查:
   - 树莓派摄像头服务是否正常运行
   - 树莓派的IP地址是否正确
   - 树莓派的8000端口是否可访问

## 故障排除

1. 如果摄像头画面无法显示:
   - 检查服务状态：`sudo systemctl status camera-service.service`
   - 查看服务日志：`sudo journalctl -u camera-service.service`
   - 确认树莓派摄像头服务正在运行：`ps aux | grep camera_server.py`
   - 检查树莓派防火墙是否允许8000端口：`sudo ufw status`
   - 直接访问树莓派摄像头服务测试：`http://树莓派IP:8000/`

2. 如果摄像头画面延迟严重:
   - 确保使用5GHz Wi-Fi网络而不是2.4GHz
   - 确保树莓派和ESP32距离无线路由器较近
   - 调整camera_server.py中的帧率和分辨率参数
   - 可以降低JPEG质量(修改`encode_params`中的值)

3. 如果树莓派重启后服务未自动启动:
   - 检查systemd服务：`sudo systemctl status camera-service.service`
   - 重新启用服务：`sudo systemctl enable camera-service.service`
   - 手动启动服务：`sudo systemctl start camera-service.service`

## 技术说明

- 网页界面(ESP32): 端口80
- 摄像头流服务(树莓派): 端口8000
- 摄像头分辨率: 640x480
- 帧率: 30fps
- JPEG压缩质量: 90%

## 自定义选项

如果需要修改摄像头参数，可以编辑camera_server.py文件中的以下部分:

1. 调整分辨率:
```python
config = picam2.create_video_configuration(
    main={
        "size": (640, 480),  # 修改为所需分辨率，例如(800, 600)
        "format": "RGB888"
    },
    ...
)
```

2. 调整帧率:
```python
controls={
    "FrameDurationLimits": (33333, 33333),  # 30fps，修改为所需帧率
    ...
}
```

3. 调整图像质量:
```python
encode_params = [cv2.IMWRITE_JPEG_QUALITY, 90]  # 修改90为所需质量(1-100)
```

修改后需要重启服务:
```bash
sudo systemctl restart camera-service.service
``` 
# test_csi.py
import cv2

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=640,
    display_height=360,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

print("尝试通过 GStreamer 打开 CSI 摄像头...")
pipeline = gstreamer_pipeline(sensor_id=0, flip_method=0)
print(f"使用的 Pipeline: {pipeline}")

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if cap.isOpened():
    print("摄像头打开成功！按 'q' 退出")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("读取失败")
            break
        cv2.imshow("CSI Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
else:
    print("摄像头打开失败！")
    print("1. 请检查 sudo systemctl restart nvargus-daemon")
    print("2. 请检查排线方向")
    print("3. 请运行 sudo /opt/nvidia/jetson-io/jetson-io.py 配置驱动")
import cv2
import socket
import threading
import time
import numpy as np
from ultralytics import YOLO
# uv run AI_Controller_stable.py
# ================= CẤU HÌNH HỆ THỐNG =================
ROBOT_IP = "192.168.100.20"
ROBOT_PORT = 6868 
CAM_URL = "http://192.168.137.70:81/stream"
MODEL_PATH = "FinalModel.pt"
CONF_THRESHOLD = 0.8

CLASS_MAP = {0: "left", 1: "obstacle", 2: "right", 3: "stop"}

# --- CẤU HÌNH VÙNG NGUY HIỂM ---
DANGER_ZONE_HEIGHT = 0.5 # Tăng lên 0.4 để đường trung bình nằm ở vị trí hợp lý
DANGER_ZONE_WIDTH = 0.8  

# --- BIẾN ĐIỀU KHIỂN ---
current_command = None
last_sent_time = 0
SEND_INTERVAL = 4.0  # Cooldown cho biển báo rẽ

is_running = True
is_connected = False
tcp_sock = None
frame_buffer = None

# ================= KẾT NỐI TCP =================
def connect_worker():
    global tcp_sock, is_connected, is_running
    while is_running:
        if not is_connected:
            try:
                tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                tcp_sock.settimeout(2)
                tcp_sock.connect((ROBOT_IP, ROBOT_PORT))
                is_connected = True
                print("✅ Connected to Robot!")
            except:
                is_connected = False
                time.sleep(2)
        time.sleep(1)

def send_tcp_cmd(cmd):
    global is_connected
    if not is_connected or tcp_sock is None: return
    try:
        tcp_sock.sendall((cmd + "\n").encode())
        print(f"📡 TCP Sent: {cmd}")
    except:
        is_connected = False

# ================= LOGIC KIỂM TRA VÙNG (MIDLINE TRIGGER) =================
def check_obstacle_in_zone(box, frame_w, frame_h):
    obj_x1, obj_y1, obj_x2, obj_y2 = box # obj_y2 là Đáy của vật
    
    # 1. Xác định ranh giới Danger Zone
    zone_h_pixel = int(frame_h * DANGER_ZONE_HEIGHT)
    zone_y_start = frame_h - zone_h_pixel
    
    # 2. Tính ĐƯỜNG TRUNG BÌNH của Danger Zone (Trigger Line)
    # y_trigger = (Đỉnh Danger + Đáy Danger) / 2
    y_trigger = (zone_y_start + frame_h) / 2
    
    # 3. Ranh giới chiều ngang
    zone_w_pixel = int(frame_w * DANGER_ZONE_WIDTH)
    zone_x_start = (frame_w - zone_w_pixel) // 2
    zone_x_end = zone_x_start + zone_w_pixel
    obj_center_x = (obj_x1 + obj_x2) // 2

    # ĐIỀU KIỆN: Đáy vật (y2) vượt qua đường trung bình (y_trigger)
    cond_y = obj_y2 >= y_trigger
    cond_x = zone_x_start <= obj_center_x <= zone_x_end
    
    return (cond_y and cond_x), (zone_x_start, zone_y_start, zone_x_end, frame_h, int(y_trigger))

# ================= LOGIC GỬI LỆNH THÔNG MINH =================
def controller_logic(detected_label):
    global current_command, last_sent_time
    if not detected_label: return current_command
    now = time.time()
    should_send = False

    if detected_label != current_command:
        should_send = True
    else:
        if detected_label in ["obstacle", "stop"]:
            if (now - last_sent_time) > 0.5: should_send = True
        elif (now - last_sent_time) > SEND_INTERVAL:
            should_send = True

    if should_send:
        current_command = detected_label
        last_sent_time = now
        send_tcp_cmd(detected_label)
    return current_command

# ================= LUỒNG CAMERA =================
def fetch_frames():
    global frame_buffer, is_running
    cap = cv2.VideoCapture(CAM_URL)
    while is_running:
        ret, frame = cap.read()
        if ret: frame_buffer = frame
        else: time.sleep(1); cap.open(CAM_URL)

# ================= MAIN LOOP =================
def main():
    global frame_buffer, is_running
    model = YOLO(MODEL_PATH)
    threading.Thread(target=fetch_frames, daemon=True).start()
    threading.Thread(target=connect_worker, daemon=True).start()

    cv2.namedWindow("AI Controller", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("AI Controller", 640, 480)

    while is_running:
        if frame_buffer is None: continue
        frame = frame_buffer.copy()
        h, w = frame.shape[:2]
        
        # Mặc định để vẽ danger zone
        zx1, zy1, zx2, zy2, z_mid = (int(w*(1-DANGER_ZONE_WIDTH)/2), h-int(h*DANGER_ZONE_HEIGHT), int(w*(1+DANGER_ZONE_WIDTH)/2), h, h-int(h*DANGER_ZONE_HEIGHT/2))

        results = model(frame, imgsz=320, conf=CONF_THRESHOLD, verbose=False)
        best_label = None
        detected_box = None

        for r in results:
            for box in r.boxes:
                label = CLASS_MAP.get(int(box.cls[0]))
                coords = box.xyxy[0].cpu().numpy().astype(int)

                if label == "obstacle":
                    is_danger, zone_info = check_obstacle_in_zone(coords, w, h)
                    zx1, zy1, zx2, zy2, z_mid = zone_info
                    if is_danger:
                        best_label, detected_box = label, coords
                    else:
                        # Vẽ Safe (Vàng)
                        cv2.rectangle(frame, (coords[0], coords[1]), (coords[2], coords[3]), (0, 255, 255), 1)
                elif label and best_label != "obstacle":
                    best_label, detected_box = label, coords

        sent_cmd = controller_logic(best_label)

        # --- GIAO DIỆN HIỂN THỊ ---
        # Vẽ Danger Zone (Đỏ nhạt)
        cv2.rectangle(frame, (zx1, zy1), (zx2, zy2), (0, 0, 180), 1)
        # Vẽ TRIGGER LINE (Vạch trắng - Đường trung bình)
        cv2.line(frame, (zx1, z_mid), (zx2, z_mid), (255, 255, 255), 2)
        cv2.putText(frame, "TRIGGER LINE (1/2)", (zx1, z_mid - 5), 1, 0.8, (255, 255, 255), 1)

        if detected_box is not None:
            x1, y1, x2, y2 = detected_box
            clr = (0, 0, 255) if best_label in ["obstacle", "stop"] else (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), clr, 3)
            # Vẽ chấm nhỏ tại ĐÁY vật để dễ quan sát
            cv2.circle(frame, ((x1+x2)//2, y2), 5, (255, 255, 0), -1)
            cv2.putText(frame, f"{best_label.upper()}", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, clr, 2)

        # Status CMD
        cv2.rectangle(frame, (0,0), (220, 40), (0,0,0), -1)
        txt = sent_cmd.upper() if sent_cmd else "SEARCHING"
        cv2.putText(frame, f"CMD: {txt}", (10, 30), 1, 1.5, (0, 255, 0), 2)

        cv2.imshow("AI Controller", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): is_running = False

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
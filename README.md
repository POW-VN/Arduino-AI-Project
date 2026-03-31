# Arduino-AI-Project
AI-powered line follower with sign recognition and obstacle avoidance.

## 📌 Giới thiệu
Dự án này kết hợp **Arduino** và **AI (YOLO model)** để xây dựng xe dò line thông minh:
- Nhận diện biển báo giao thông.
- Tránh chướng ngại vật.
- Điều khiển qua ESP32-CAM.

## ⚙️ Thành phần chính
- **AI_Controller_stable.py**: Script Python điều khiển, xử lý hình ảnh và inference từ YOLO.
- **FinalModel.pt**: Mô hình YOLO đã huấn luyện.
- **ESP32CAM_CONNECT/**: Code kết nối camera ESP32.
- **.venv/**: Môi trường ảo Python.

## 🚀 Cách chạy
1. Clone repo:
   ```bash
   git clone https://github.com/POW-VN/Arduino-AI-Project.git
   cd Arduino-AI-Project
   
2. Tạo môi trường ảo và cài đặt thư viện:
   python -m venv .venv
source .venv/bin/activate   # Linux/Mac
.venv\Scripts\activate      # Windows
pip install -r requirements.txt

4. Chạy controller:
   python AI_Controller_stable.py
   
🛠️ Yêu cầu
Python 3.9+
OpenCV
Ultralytics YOLO
Arduino IDE
ESP32-CAM module

📂 Cấu trúc thư mục
Arduino-AI-Project/
│
├── AI_Controller_stable.py   # Code chính
├── FinalModel.pt             # Mô hình YOLO đã huấn luyện
├── ESP32CAM_CONNECT/         # Code kết nối camera
├── .venv/                    # Môi trường ảo Python
└── README.md                 # Tài liệu dự án

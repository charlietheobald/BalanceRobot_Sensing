import socket
import threading
import cv2
import numpy as np
from flask import Flask, Response, render_template_string

app = Flask(__name__)

UDP_IP = "0.0.0.0"
UDP_PORT = 5001

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))

print("Loading custom Roboflow ONNX model on laptop...")
net = cv2.dnn.readNetFromONNX("best.onnx")

latest_frame = None
lock = threading.Lock()

HTML_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Mission Control</title>
    <style>
        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background-color: #0d1117;
            color: #c9d1d9;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            max-width: 800px;
            width: 100%;
            text-align: center;
        }
        header {
            margin-bottom: 30px;
        }
        h1 {
            color: #58a6ff;
            font-size: 2.5rem;
            margin-bottom: 10px;
            font-weight: 600;
            letter-spacing: -0.5px;
        }
        p {
            color: #8b949e;
            font-size: 1.1rem;
        }
        .stream-card {
            background-color: #161b22;
            border: 1px solid #30363d;
            border-radius: 12px;
            padding: 16px;
            box-shadow: 0 8px 24px rgba(0, 0, 0, 0.3);
            display: inline-block;
            width: 100%;
            max-width: 680px;
        }
        .video-wrapper {
            position: relative;
            background-color: #010409;
            border-radius: 8px;
            overflow: hidden;
            aspect-ratio: 4/3;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        img {
            width: 100%;
            height: 100%;
            object-fit: contain;
        }
        .status-badge {
            display: inline-flex;
            align-items: center;
            gap: 8px;
            margin-top: 15px;
            background: #21262d;
            padding: 6px 12px;
            border-radius: 20px;
            font-size: 0.85rem;
            border: 1px solid #30363d;
        }
        .pulse {
            width: 8px;
            height: 8px;
            background-color: #238636;
            border-radius: 50%;
            box-shadow: 0 0 0 0 rgba(35, 134, 54, 0.7);
            animation: pulsing 1.5s infinite;
        }
        @keyframes pulsing {
            0% { transform: scale(0.95); box-shadow: 0 0 0 0 rgba(35, 134, 54, 0.7); }
            70% { transform: scale(1); box-shadow: 0 0 0 8px rgba(35, 134, 54, 0); }
            100% { transform: scale(0.95); box-shadow: 0 0 0 0 rgba(35, 134, 54, 0); }
        }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1>Robot Mission Control</h1>
            <p>Live Object Tracking Feed</p>
        </header>
        
        <div class="stream-card">
            <div class="video-wrapper">
                <img src="{{ url_for('video_feed') }}" alt="Awaiting video stream packets from Raspberry Pi...">
            </div>
            <div class="status-badge">
                <div class="pulse"></div>
                <span>Receiver Link Active (Port 5001)</span>
            </div>
        </div>
    </div>
</body>
</html>
"""

def receive_udp_packets():
    global latest_frame
    while True:
        try:
            packet, _ = sock.recvfrom(65535)
            np_arr = np.frombuffer(packet, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                with lock:
                    latest_frame = frame
        except Exception as e:
            print(f"Network packet extraction failed: {e}")

def run_onnx_inference(frame):
    (h, w) = frame.shape[:2]
    
    blob = cv2.dnn.blobFromImage(frame, 1.0/255.0, (640, 640), swapRB=True, crop=False)
    net.setInput(blob)
    outputs = net.forward()
    
    predictions = np.squeeze(outputs)
    if predictions.shape[0] < predictions.shape[1]:
        predictions = predictions.T
        
    boxes = []
    confidences = []
    
    for row in predictions:
        classes_scores = row[4:]
        class_id = np.argmax(classes_scores)
        confidence = classes_scores[class_id]
        
        if confidence > 0.40:
            cx, cy, width, height = row[0], row[1], row[2], row[3]
            
            x_factor = w / 640.0
            y_factor = h / 640.0
            
            left = int((cx - width / 2.0) * x_factor)
            top = int((cy - height / 2.0) * y_factor)
            width = int(width * x_factor)
            height = int(height * y_factor)
            
            boxes.append([left, top, width, height])
            confidences.append(float(confidence))
            
    indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.40, 0.45)
    
    for i in indices:
        if isinstance(i, np.ndarray):
            i = i[0]
        box = boxes[i]
        left, top, width, height = box[0], box[1], box[2], box[3]
        
        cv2.rectangle(frame, (left, top), (left + width, top + height), (0, 0, 255), 3)
        label = f"Robot Found: {confidences[i] * 100:.1f}%"
        cv2.putText(frame, label, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
    return frame

def generate_frames():
    global latest_frame
    while True:
        with lock:
            if latest_frame is None:
                continue
            frame_copy = latest_frame.copy()
            
        processed_frame = run_onnx_inference(frame_copy)
        
        ret, encoded_image = cv2.imencode(".jpg", processed_frame)
        if not ret:
            continue
            
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + encoded_image.tobytes() + b'\r\n')

@app.route("/")
def index():
    return render_template_string(HTML_PAGE)

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    t = threading.Thread(target=receive_udp_packets)
    t.daemon = True
    t.start()
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
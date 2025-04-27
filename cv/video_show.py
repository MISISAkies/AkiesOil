import cv2
from flask import Flask, Response
import numpy as np

app = Flask(__name__)

# Инициализация камеры
cap = cv2.VideoCapture('/dev/video0')
if not cap.isOpened():
    raise RuntimeError("Не удалось открыть камеру!")

def gen_frames():
    """Генератор кадров для потока."""
    while True:
        success, frame = cap.read()
        if not success:
            break
        
        # Опциональная обработка кадра (например, изменение размера)
        frame = cv2.resize(frame, (640, 480))
        
        # Кодирование кадра в JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        
        # Формирование кадра для HTTP-потока
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    """Главная страница с видео."""
    return """
    <html>
      <head>
        <title>Камера</title>
      </head>
      <body>
        <h1>Поток с камеры</h1>
        <img src="/video_feed" width="640" height="480">
      </body>
    </html>
    """

@app.route('/video_feed')
def video_feed():
    """Роут для видео-потока."""
    return Response(gen_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

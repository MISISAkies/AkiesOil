import cv2
import os
from datetime import datetime

cap = cv2.VideoCapture('/dev/video1')
if not cap.isOpened():
    print("Ошибка: не удалось открыть камеру.")
    exit()

frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv2.CAP_PROP_FPS)) or 30

fourcc = cv2.VideoWriter_fourcc(*'XVID')
os.makedirs('recordings', exist_ok=True)
output_filename = f'recordings/video_{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}.avi'

out = cv2.VideoWriter(output_filename, fourcc, fps, (frame_width, frame_height))
print(f"Запись видео начата: {output_filename}")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Ошибка: не удалось получить кадр.")
            break
        
        out.write(frame)  # запись кадра
        
        # Выход по Ctrl+C (так как cv2.waitKey() не используется)
except KeyboardInterrupt:
    print("Запись остановлена пользователем.")
finally:
    cap.release()
    out.release()
    print("Видео сохранено.")

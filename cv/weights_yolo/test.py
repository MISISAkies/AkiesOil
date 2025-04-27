import os
import numpy as np
import cv2
from ultralytics import YOLO
from flask import Flask, Response
import time
from datetime import datetime

# Параметры
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)
detector_qr = cv2.QRCodeDetector()

threshold = 0.6  # Порог для модели YOLO
class_names_dict = {0: 'green', 1: 'red', 2: 'two', 3: 'three', 4: 'zero', 5: 'five', 6: 'cow', 7: 'one', 8: 'four'}

# Путь к лог файлу + очистка
log_file_path = "recognized_log.txt"
open(log_file_path, "w").close()  # Очистить лог при старте

def write_log(message):
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    with open(log_file_path, "a") as f:
        f.write(f"[{timestamp}] {message}\n")

# Загрузка модели YOLO
model = YOLO('/home/ubuntu/robots_monorepo/mcu/weights_yolo/misis_hack_yolo_ver2.pt')

# Инициализация захвата видео
cap = cv2.VideoCapture('/dev/video0')
if not cap.isOpened():
    print("Ошибка: не удалось открыть видео.")
    exit()

# Инициализация Flask
app = Flask(__name__)

# Функция для захвата кадров и распознавания
def gen_frames():
    while True:
        ret, frame = cap.read()
        if not ret:
            break  # Если кадры закончились, выходим из цикла

        frame = cv2.resize(frame, (640, 360))
        flipped_frame = cv2.flip(frame, 0)

        # Обнаружение ArUco маркеров
        corners, ids, _ = detector.detectMarkers(frame)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                y_max = max(corners[i][0][:, 1])
                x_center = int(np.mean(corners[i][0][:, 0]))
                y_center = int(np.mean(corners[i][0][:, 1]))
            frame_gr = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            mask = np.zeros(frame.shape[:2], np.uint8)

            frame_g = cv2.GaussianBlur(frame_gr, (7, 7), 0)
            binr = cv2.threshold(frame_g, 90, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

            kernel = np.ones((35, 35), np.uint8)
            closing = cv2.morphologyEx(binr, cv2.MORPH_CLOSE, kernel, iterations=1)
            closing = cv2.bitwise_not(closing)

            frame_mask_on = cv2.bitwise_and(frame, frame, mask=closing)
            final_before_contours = cv2.threshold(closing, 0, 255, cv2.THRESH_BINARY)[1]

            contours, hierarchy = cv2.findContours(image=final_before_contours,
                                                   mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
            for cnt in contours:
                mask = np.zeros(frame.shape[:2], np.uint8)
                cv2.drawContours(mask, [cnt], 0, 255, cv2.FILLED)
                mean_color = cv2.mean(frame, mask=mask)
                mean_color_sorted = sorted(mean_color)[::-1]

                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                if 1500 < cv2.contourArea(cnt) < 3000 and mean_color_sorted[0] < 50:
                    if (mean_color_sorted[0] < mean_color_sorted[1] * 1.75):
                        if not (int(x_center) - 75 < cx < int(x_center) + 75 and int(y_center) - 75 < cy < int(
                                y_center) + 75):
                            cv2.fillPoly(frame, pts=[cnt], color=(0, 255, 0))
                            leak_message = f'Leakage detected at the oil pump number {ids.flatten().tolist()}'
                            print(leak_message)
                            write_log(leak_message)
                            print(cv2.contourArea(cnt))

        # Проверка QR кода на текущем кадре
        qr_data, qr_bbox, qr_straight = detector_qr.detectAndDecode(frame)
        if qr_bbox is not None and qr_data:
            print(f"[QR] Найден QR код: {qr_data}")
            write_log(f"QR код: {qr_data}")
        else:
            print("[QR] QR код не найден на кадре.")

        # Детекция объектов с помощью YOLO
        results = model(flipped_frame)[0]

        for result in results.boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = result

            if score > threshold:
                object_name = class_names_dict.get(int(class_id))

                if object_name is not None:  # Только если объект известен
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.putText(frame, object_name,
                                (int((x1 + x2) / 2) + 15, int((y1 + y2) / 2) + 15),
                                cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                    cv2.circle(frame, (int((x1 + x2) / 2), int((y1 + y2) / 2)), 4, (255, 0, 0), 2)

                    # Проверка, если объект — это one, zero или green
                    if object_name in ['one', 'zero', 'green']:
                        # Отображаем слово "leakage" слева от рамки
                        text_x = max(10, int(x1) - 10)  # Не даём тексту выйти за левую границу
                        cv2.putText(frame, "leakage",
                                    (text_x, int(y1) - 10),
                                    cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
                        # Записываем в лог
                        write_log(f"Leakage detected for object: {object_name} with confidence {score:.2f}")

                    write_log(f"Обнаружен объект: {object_name} с точностью {score:.2f}")

        # Кодируем изображение с аннотациями в формат JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        # Возвращаем кадр как поток данных
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# Роут Flask для потока видео
@app.route('/video')
def video():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)
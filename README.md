# Кубок РТК: Высшая Лига. Хакатон – Добыча нефти

## Описание

**Задача:**  
Разработать автономного робота для обследования устьевых арматур нефтяных скважин на смоделированном полигоне.

**Возможности робота:**
- Перемещение по линии навигации между объектами полигона, включая пересечения и "обманные" повороты.
- Сбор данных с манометров: определение положения стрелки (зелёная/красная зона) и считывание показаний.
- Поиск возможных мест "протечек" на устьевой арматуре.
- Распознавание ArUco-маркеров для идентификации датчиков и объектов.
- Корректный старт и завершение маршрута по обозначенным ячейкам полигона.

**Цель:**  
Максимальная автоматизация и точность обследования в условиях макета нефтяного месторождения.

---

## Основной функционал проекта

- **Автоматическое следование по линии** — благодаря датчику Amperka Octoliner робот распознаёт и точно следует за дорожной разметкой.
- **Построение карты окружающей среды** — с помощью лидара RPLIDAR робот сканирует пространство, выявляет препятствия и формирует карту.
- **Распознавание объектов окружающей среды** — веб-камера реализует передачу видеопотока в реальном времени и распознавание объектов для реагирования на изменения.
- **Управление моторами** — Arduino Uno с Amperka Motor Shield обеспечивает надёжное и плавное управление движением колёс.
- **Взаимодействие между модулями** — ROS2 объединяет все сенсоры и исполнительные устройства в единую систему обмена данными.
- **Гибкость архитектуры** — возможно добавлять или заменять сенсоры и модули без изменений базовой логики работы.

---

## Технологии и инструменты

[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu)](https://ubuntu.com/)
[![ROS2](https://img.shields.io/badge/ROS2-Iron-22314E?logo=ros&logoColor=white)](https://ros.org/)
[![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-5-C51A4A?logo=raspberrypi&logoColor=white)](https://www.raspberrypi.com/)
[![Arduino](https://img.shields.io/badge/Arduino-Uno-008184?logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C++-17-00599C?logo=c%2B%2B&logoColor=white)](https://isocpp.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?logo=opencv&logoColor=white)](https://opencv.org/)
[![YOLO](https://img.shields.io/badge/YOLO-v8-00BCD4?logo=yolo&logoColor=white)](https://github.com/ultralytics/ultralytics)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-IDE-FFA500?logo=platformio&logoColor=white)](https://platformio.org/)
[![Flask](https://img.shields.io/badge/Flask-2.3-000000?logo=flask&logoColor=white)](https://flask.palletsprojects.com/)

**Аппаратные модули:**  
- Amperka Octoliner (датчик линии)  
- RPLIDAR (лидар)  
- Amperka Motor Shield  
- Веб-камера (USB)

**ПО и библиотеки:**  
- ROS2 для коммуникации между модулями  
- Arduino Framework и PlatformIO для микроконтроллера  
- Python и OpenCV для обработки изображений  
- YOLO (Ultralytics) для детекции объектов  
- Flask для web-интерфейса видеопотока  

---

## Запуск проекта

### 1. Сборка робота

Соберите робота согласно схеме, приведённой в документации.

---

### 2. Прошивка микроконтроллера

```bash
# Клонирование репозитория
git clone https://github.com/MISISAkies/AkiesOil.git
cd AkiesOil/mcu/arduino_robot_neft_simply

# Прошивка через PlatformIO
pio run -t upload
```

---

### 3. Настройка системы компьютерного зрения

```bash
# Установка зависимостей
pip install ultralytics opencv-python flask

# Загрузка модели YOLO
wget https://github.com/MISISAkies/AkiesOil/releases/download/0.0.1/misis_hack_yolo_ver2.pt -P weights_yolo/
```

Измените путь к модели в файле `test.py`:

```python
model = YOLO('weights_yolo/misis_hack_yolo_ver2.pt')
```

---

### 4. Запуск системы

```bash
# Запуск системы компьютерного зрения
cd AkiesOil/cv
python test.py
```

Откройте в браузере:
```
http://127.0.0.1:5001/video
```

---

## Команда проекта

- **Саргин Ярослав Сергеевич** — разработка прошивки для калибровки, ориентирования и движения по трассе, разработка и отладка системы компьютерного зрения.
- **Епифанцев Тарас Сергеевич** — инженер, сборка базы робота, разработка прошивки, отладка PID-регуляторов, реализация дистанционной отладки.
- **Плешевич Милена Юрьевна** — сборка базы робота, разработка и отладка системы компьютерного зрения, обучение YOLO, сбор и разметка датасета, интеграция с ROS2.

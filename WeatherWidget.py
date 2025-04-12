from PyQt5.QtWidgets import QApplication, QMainWindow, QMenuBar, QAction, QCheckBox, QVBoxLayout, QWidget, QLabel, \
    QComboBox, \
    QPushButton, QDialog, QRadioButton, QFormLayout, QGroupBox, QHBoxLayout, QGridLayout, QLineEdit, QMessageBox, \
    QFileDialog, QFrame, QTextEdit, QDesktopWidget
from PyQt5.QtGui import QPalette, QColor, QFont, QIcon, QPixmap, QImage
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5 import QtCore, QtGui
import sys
import os
import lxml
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Wedge
from ctypes import POINTER, cast
from comtypes import CLSCTX_ALL
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
import requests
import pygame
from copy import deepcopy
from random import choice, randrange, randint
import time
from datetime import datetime
from bs4 import BeautifulSoup as bs

class WeatherWorker(QThread):
    weather_data_signal = pyqtSignal(str, str)

    def run(self):
        api_key = "74365752f8a119614ab2da11017688d0"
        city = "Moscow"
        url = f"http://api.openweathermap.org/data/2.5/weather?q={city}&appid={api_key}&units=metric&lang=ru"

        while True:
            weather_text = ""
            weather_icon_code = ""
            attempt_count = 0

            while not weather_text:
                try:
                    attempt_count += 1
                    response = requests.get(url, timeout=10)

                    if response.status_code != 200:
                        print(f"Попытка {attempt_count}: Ошибка HTTP {response.status_code} - {response.reason}")
                        time.sleep(5)
                        continue

                    data = response.json()

                    if data.get("cod") == 200:
                        city_name = data["name"]
                        temperature = round(data["main"]["temp"])
                        description = data["weather"][0]["description"]
                        weather_icon_code = data["weather"][0]["icon"]

                        weather_text = f"Город: {city_name}\nТемпература: {temperature}°C\nПогода: {description}"
                    else:
                        error_message = data.get("message", "Город не найден")
                        print(f"Попытка {attempt_count}: Ошибка: {error_message} (Код: {data.get('cod')})")

                except requests.exceptions.RequestException as e:
                    print(f"Попытка {attempt_count}: Ошибка подключения: {e}")
                    time.sleep(5)

            self.weather_data_signal.emit(weather_text, weather_icon_code)

            wait_time_minutes = randint(30, 40)
            print(f"Следующее обновление данных через {wait_time_minutes} минут.")
            time.sleep(wait_time_minutes * 60)

class WeatherWidget(QWidget):
    def __init__(self):
        super().__init__()

        # Сначала виджет будет скрыт
        self.setVisible(False)

        # Создаем элементы интерфейса для виджета
        self.initUI()

        # Создаем поток для получения данных о погоде
        self.weather_thread = WeatherWorker()
        self.weather_thread.weather_data_signal.connect(self.update_weather_data)
        self.weather_thread.start()

    def initUI(self):
        layout = QGridLayout()

        self.weather_info_label = QLabel("Загрузка данных о погоде...")
        self.weather_info_label.setFont(QFont("Arial", 18))

        self.weather_icon_label = QLabel()
        # self.weather_icon_label.setFixedSize(100, 100)

        layout.addWidget(self.weather_info_label, 0, 0, alignment=Qt.AlignLeft | Qt.AlignTop)
        layout.addWidget(self.weather_icon_label, 0, 1, alignment=Qt.AlignLeft | Qt.AlignTop)

        self.setLayout(layout)
        self.weather_info_label.setStyleSheet("""
            background-color: #ffffff; 
            border: 3px solid blue; 
            border-radius: 15px; 
            padding: 10px;
        """)
        self.weather_icon_label.setStyleSheet("""
            background-color: #ffffff; 
            border: 3px solid blue; 
            border-radius: 20px; 
        """)

        # self.setFixedHeight(250)
        # self.setFixedWidth(730)

    def update_weather_data(self, weather_text, weather_icon_code):
        self.weather_info_label.setText(weather_text)

        if weather_icon_code:
            icon_url = f"http://openweathermap.org/img/wn/{weather_icon_code}@2x.png"
            try:
                icon_response = requests.get(icon_url, timeout=10)
                if icon_response.status_code == 200:
                    image = QImage()
                    image.loadFromData(icon_response.content)
                    pixmap = QPixmap(image)
                    self.weather_icon_label.setPixmap(pixmap)
                else:
                    print(f"Ошибка загрузки иконки: HTTP {icon_response.status_code} - {icon_response.reason}")
            except requests.exceptions.RequestException as e:
                print(f"Ошибка при загрузке иконки погоды: {e}")

        if "Город:" in weather_text:
            self.setVisible(True)
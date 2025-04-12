from PyQt5.QtWidgets import QApplication, QMainWindow, QMenuBar, QAction, QCheckBox, QVBoxLayout, QWidget, QLabel, \
    QComboBox, \
    QPushButton, QDialog, QRadioButton, QFormLayout, QGroupBox, QHBoxLayout, QGridLayout, QLineEdit, QMessageBox, \
    QFileDialog, QFrame, QTextEdit, QDesktopWidget
from PyQt5.QtGui import QPalette, QColor, QFont, QIcon, QPixmap, QImage
from PyQt5.QtCore import Qt,QTimer
from datetime import datetime


class DateTimeWidget(QWidget):
    def __init__(self):
        super().__init__()

        self.initUI()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_datetime)
        self.timer.start(1000)

    def initUI(self):
        layout = QGridLayout()

        # Создаем метку для отображения времени и даты
        self.datetime_label = QLabel()
        self.datetime_label.setFont(QFont("Arial", 24))
        self.datetime_label.setAlignment(Qt.AlignCenter)
        self.datetime_label.setStyleSheet("""
            background-color: #3b84e3; 
            border: 3px solid white; 
            border-radius: 10px; 
            padding: 10px;
        """)
        layout.addWidget(self.datetime_label, 0, 0, alignment=Qt.AlignLeft | Qt.AlignTop)
        self.setLayout(layout)

        self.update_datetime()

    def update_datetime(self):
        # Устанавливаем текущие время и дату
        current_datetime = datetime.now()
        self.datetime_label.setText(
            f"{current_datetime.strftime('%H:%M')}\n{current_datetime.strftime('%d.%m.%Y')}"
        )

    '''def resizeEvent(self, event):
        # Получаем размеры экрана
        screen_geometry = QGuiApplication.primaryScreen().geometry()
        screen_width = screen_geometry.width()
        screen_height = screen_geometry.height()

        # Настраиваем размер шрифта пропорционально экрану
        font_size = int(min(screen_width, screen_height) * 0.023)
        font = QFont("Arial", font_size)
        self.datetime_label.setFont(font)

        # Устанавливаем фиксированные размеры виджета
        self.setFixedWidth(int(screen_width * 0.2))
        self.setFixedHeight(int(screen_height * 0.14))'''
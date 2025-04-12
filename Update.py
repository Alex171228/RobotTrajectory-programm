import requests
import os
from dotenv import load_dotenv
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import Qt

# Загружаем токены из .env файла
load_dotenv()

token_main = os.getenv("GITHUB_MAIN_TOKEN")
token_second = os.getenv("GITHUB_SECOND_TOKEN")

# Первый приватный репозиторий
owner = "MarishaBag"
repo = "Autumn-tears"
url = f"https://api.github.com/repos/{owner}/{repo}/releases/latest"
headers_main = {"Authorization": f"token {token_main}"}

# Второй приватный репозиторий (обновление)
second_owner = "Alex171228"
second_repo = "RobotTrajectory-programm"
second_url = f"https://api.github.com/repos/{second_owner}/{second_repo}/releases/latest"
headers_second = {"Authorization": f"token {token_second}"}

# Путь к файлу версии
base_dir = os.path.dirname(os.path.abspath(__file__))
version_file = os.path.join(base_dir, "release_version.txt")


def check_internet_connection():
    try:
        response = requests.get("https://api.github.com", timeout=5)
        return response.status_code == 200
    except requests.RequestException:
        return False


def get_current_version():
    if os.path.exists(version_file):
        with open(version_file, "r") as file:
            return file.read().strip()
    return None


def get_latest_release_info(repo_url, headers):
    response = requests.get(repo_url, headers=headers)
    if response.status_code == 200:
        release_data = response.json()
        return release_data["tag_name"], release_data["html_url"]
    else:
        QMessageBox.information(None, "Обновление", f"Ошибка при получении данных о релизе: {response.status_code}")
        return None, None


def check_for_update():
    if not check_internet_connection():
        QMessageBox.information(None, "Обновление", "Вы не подключены к интернету.")
        return

    current_version = get_current_version()
    QMessageBox.information(None, "Обновление", f"Текущая версия программы: {current_version}")

    latest_version, release_link = get_latest_release_info(second_url, headers_second)

    if latest_version is None or release_link is None:
        QMessageBox.information(None, "Обновление", "Не удалось получить данные о последнем релизе.")
        return

    if current_version == latest_version:
        QMessageBox.information(None, "Обновление", f"У вас уже установлена последняя версия: {latest_version}. Обновление не требуется.")
    else:
        msg_box = QMessageBox()
        msg_box.setWindowTitle("Обновление")
        msg_box.setText(
            f"Доступна новая версия: {latest_version}.<br>"
            f"Текущая версия: {current_version}.<br>"
            f"Для обновления скачайте новый exe файл.<br><br>"
            f"<a href='{release_link}'>Скачать новую версию</a>"
        )
        msg_box.setTextFormat(1)
        msg_box.setTextInteractionFlags(msg_box.textInteractionFlags() | Qt.TextBrowserInteraction)
        msg_box.exec_()

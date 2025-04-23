from PyQt5.QtWidgets import QApplication, QMainWindow, QMenuBar, QAction, QCheckBox, QVBoxLayout, QWidget, QLabel, \
    QComboBox, \
    QPushButton, QDialog, QRadioButton, QFormLayout, QGroupBox, QHBoxLayout, QGridLayout, QLineEdit, QMessageBox, \
    QFileDialog, QScrollArea, QFrame, QTextEdit, QDesktopWidget,QAbstractButton,QTextBrowser, QButtonGroup
from PyQt5.QtGui import QPalette, QColor, QFont, QIcon, QPixmap, QImage
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QEvent
from PyQt5 import QtCore, QtGui
import sys
import os
import zipfile
import shutil
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
from DateTimeWidget import DateTimeWidget
from WeatherWidget import WeatherWidget
from HoroscopeWidget import HoroscopeWidget
from Update import check_for_update
from scipy.interpolate import UnivariateSpline
import scipy as sp
import scipy.interpolate

class TrajectoryCalculator:
    def __init__(self, app_instance):
        # Получаем ссылку на экземпляр RobotTrajectoryApp
        self.app_instance = app_instance
        self.robot_type = None
        self.spline = False



        self.PID_differential = 0.0  # Коэффиценты ПИДа
        self.PID_proportional = 0.0
        self.PID_integral = 0.0

        # Параметры регуляторов
        # self.Kp = [1.0, 1.0, 0, 0]
        # self.Ki = [0.0, 0.0, 0, 0]
        # self.Kd = [0.0, 0.0, 0, 0]



        self.Kp = [0, 0, 0, 0]
        self.Ki = [0, 0, 0, 0]
        self.Kd = [0, 0, 0, 0]
        # Параметры Циклограммы
        # self.t = [10, 20, 30, 40, 50, 60, 70, 80, 90]
        # self.q1 = [0.1, 0.1, 0.1, 0.45, 0.9, 0.9, 0.9, 0.45, 0.1]
        # self.q2 = [0.1, 0.45, 0.9, 0.9, 0.9, 0.45, 0.1, 0.1, 0.1]
        # self.q3 = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        # self.q4 = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.t = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.q1 = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.q2 = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.q3 = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.q4 = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.Integral_channel = 0  # Служебная переменная для работы интегратора (оставить)
        self.Ra = 1  # Сопротивление двигателя (оставить)
        self.T_e = 0.002  # Электрическая постоянная
        self.Cm = 1  # Ещё один коэффцент двигателя
        self.Ce = 1  # Ещё один коэффцент двигателя
        self.Fi = 1  # Поток через двигатель (оставить)

        # self.J = 1.1  # Момент инерции двигателя

        # Параметры двигателей
        # self.J = [1.1, 0, 0, 0] # Момент инерции двигателя
        self.J = [0, 0, 0, 0]  # Момент инерции двигателя
        self.n = [0, 0, 0, 0]
        self.Umax = [0, 0, 0, 0]
        self.Ku = [0, 0, 0, 0]
        self.Kq = [0, 0, 0, 0]

        # self.I_1 = 0.1  # Моменты инерции первого и второго звена
        # self.I_2 = 0.1
        self.I_1 = 0.0  # Моменты инерции первого и второго звена
        self.I_2 = 0.0

        # Переменные для сообщений об ошибках
        self.error_1 = []
        self.avg_error_1 = 0
        self.median_error_1 = 0
        self.reg_time_1 = []
        self.avg_reg_time_1 = 0
        self.median_reg_time_1 = 0

        self.error_2 = []
        self.avg_error_2 = 0
        self.median_error_2 = 0
        self.reg_time_2 = []
        self.avg_reg_time_2 = 0
        self.median_reg_time_2 = 0

        self.spline_error_1 = []
        self.spline_avg_error_1 = 0
        self.spline_median_error_1 = 0
        self.spline_reg_time_1 = []
        self.spline_avg_reg_time_1 = 0
        self.spline_median_reg_time_1 = 0

        self.spline_error_2 = []
        self.spline_avg_error_2 = 0
        self.spline_median_error_2 = 0
        self.spline_reg_time_2 = []
        self.spline_avg_reg_time_2 = 0
        self.spline_median_reg_time_2 = 0


        # self.momentd_1 = 0 #декарт

        # # Параметры SCARA
        # self.moment_1 = 0
        # self.moment_2 = 0
        # self.moment_3 = 0
        #
        # # Параметры Цилиндр
        # self.momentc_1 = 0
        # self.momentc_2 = 0
        # self.momentc_3 = 0



        # self.Processor_frequency = 8 * 1e-3  # Это чатстота вычислителя из параметров вычислителя. В общем отвечает за быстродействие системы
        # # Это внутренние служебные переменные
        # self.K_U = 1  # Управляем мы напряжением и эта штука "переводит" координату в напряжение
        # # self.Ku = [0, 0, 0, 0]
        # self.Integral_channel = 0  # Служебная переменная для работы интегратора
        # self.Second_integral_block = 0  # Служебная переменная для работы интегратора
        # self.Third_integral_block = 0  # Служебная переменная для работы интегратор
        # self.time_prev = -1e-8  # Ещё две служебные переменные, для функционирования cистемы
        # self.U_prev = 0
        # self.last_counted_moment = 0
        # # self.accuracy = 1e-4  # Чем число меньше, тем мы ближе к реальности

        self.q1s_min = 0
        self.q1s_max = 0
        self.q2s_min = 0
        self.q2s_max = 0
        self.q3s_min = 0
        self.q3s_max = 0
        self.zs_min = 0
        self.zs_max = 0
        self.q1c_min = 0
        self.q1c_max = 0
        self.a2c_min = 0
        self.a2c_max = 0
        self.q3c_min = 0
        self.q3c_max = 0
        self.zc_min = 0
        self.zc_max = 0
        self.momentc_1 = 0
        self.momentc_2 = 0
        self.momentc_3 = 0
        self.lengthc_1 = 0
        self.lengthc_2 = 0
        self.distancec = 0
        self.massc_2 = 0
        self.massc_3 = 0
        self.moment_1 = 0
        self.moment_2 = 0
        self.moment_3 = 0
        self.length_1 = 0
        self.length_2 = 0
        self.distance = 0
        self.masss_2 = 0
        self.masss_3 = 0
        self.x_min, self.x_max = 0, 0
        self.y_min, self.y_max = 0, 0
        self.z_min, self.z_max = 0, 0
        self.q_min, self.q_max = 0, 0
        self.massd_1 = 0
        self.massd_2 = 0
        self.massd_3 = 0
        self.momentd_1 = 0
        # Параметры Колер
        self.momentcol_1 = 0
        self.momentcol_2 = 0
        self.momentcol_3 = 0
        self.lengthcol_1 = 0
        self.lengthcol_2 = 0
        self.distancecol = 0
        self.masscol_2 = 0
        self.masscol_3 = 0

        # Ограничения по координатам Колер
        self.q1col_min = 0
        self.q1col_max = 0
        self.a2col_min = 0
        self.a2col_max = 0
        self.q3col_min = 0
        self.q3col_max = 0
        self.zcol_min = 0
        self.zcol_max = 0

        #Переменные под контурное управление
        self.type_of_control = "Позиционное"
        self.line_x1 = 0
        self.line_x2 = 0
        self.line_y1 = 0
        self.line_y2 = 0
        self.line_speed = 0
        self.circle_x = 0
        self.circle_y = 0
        self.circle_radius = 0
        self.circle_speed = 0

        self.t_contur = []
        self.x_contur = []
        self.y_contur = []

        self.t_contur_control = []
        self.q1_contur_control = []
        self.q2_contur_control = []

        #Переменные для обобщённый ускорений и скоростей
        # self.speed_1 = []
        # self.speed_2 = []
        # self.acceleration_1 = []
        # self.acceleration_2 = []
        #
        # self.speed_1_spline = []
        # self.speed_2_spline = []
        # self.acceleration_1_spline = []
        # self.acceleration_2_spline = []

        # Звено 1
        self.q_error_array_1 = []
        self.SAU_SUM_array_1 = []
        self.U_array_1 = []
        self.Ustar_array_1 = []
        self.I_array_1 = []
        self.M_ed_array_1 = []
        self.M1_array = []
        self.M_ed_corrected_array_1 = []
        self.acceleration_array_1 = []
        self.speed_array_1 = []
        self.output_q_array_1 = []

        # Звено 2
        self.q_error_array_2 = []
        self.SAU_SUM_array_2 = []
        self.U_array_2 = []
        self.Ustar_array_2 = []
        self.I_array_2 = []
        self.M_ed_array_2 = []
        self.M2_array = []
        self.M_ed_corrected_array_2 = []
        self.acceleration_array_2 = []
        self.speed_array_2 = []
        self.output_q_array_2 = []

        # self.output_q_array = []
        # self.output_time_array = []
        # self.trajectory_q_1 = []
        # self.trajectory_q_2 = []
        # self.output_time_array = []

        # Реальные координаты
        self.real_trajectory_x = []
        self.real_trajectory_y = []
        self.nothing = []
        self.cyclogram_real_x = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.cyclogram_real_y = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        # Переменные связанные со сплайном
        self.num_splain_dots = 100
        self.q_1_spline = []
        self.q_2_spline = []
        self.t_spline = []
        self.trajectory_q_1_spline = []
        self.trajectory_q_2_spline = []
        self.output_time_array_spline = []
        self.real_trajectory_x_spline = []
        self.real_trajectory_y_spline = []

        # Скопированные переменные
        self.q_1_spline_copied = []
        self.q_2_spline_copied = []
        self.t_spline_copied = []

        self.trajectory_q_1 = []
        self.trajectory_q_2 = []
        self.output_time_array = []

        self.trajectory_q_1_spline = []
        self.trajectory_q_2_spline = []
        self.output_time_array_spline = []




    def set_cyclogram(self, t, q1, q2, q3, q4,type_of_control = "Позиционное"):
        self.t = t
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3
        self.q4 = q4
        self.type_of_control = type_of_control


    def set_cartesian_limits(self, x_min, x_max, y_min, y_max, z_min, z_max, q_min, q_max):
        self.x_min, self.x_max = x_min, x_max
        self.y_min, self.y_max = y_min, y_max
        self.z_min, self.z_max = z_min, z_max
        self.q_min, self.q_max = q_min, q_max
        # print("set_cartesian_limits", "2Д вызвано")

    def set_cartesian_params(self, massd_1, massd_2, massd_3, momentd_1):
        self.massd_1 = massd_1
        self.massd_2 = massd_2
        self.massd_3 = massd_3
        self.momentd_1 = momentd_1
        # print("set_cartesian_params", "3Д вызвано")

    def set_scara_limits(self, q1s_min, q1s_max, q2s_min, q2s_max, q3s_min, q3s_max, zs_min, zs_max):
        self.q1s_min = q1s_min
        self.q1s_max = q1s_max
        self.q2s_min = q2s_min
        self.q2s_max = q2s_max
        self.q3s_min = q3s_min
        self.q3s_max = q3s_max
        self.zs_min = zs_min
        self.zs_max = zs_max
        # print("set_scara_limits", "2C вызвано")
    def set_scara_params(self, moment_1, moment_2, moment_3, length_1, length_2, distance, masss_2, masss_3):
        self.moment_1 = moment_1
        self.moment_2 = moment_2
        self.moment_3 = moment_3
        self.length_1 = length_1
        self.length_2 = length_2
        self.distance = distance
        self.masss_2 = masss_2
        self.masss_3 = masss_3
        # print("set_scara_params", "3C вызвано")

    def set_cylindrical_limits(self, q1c_min, q1c_max, a2c_min, a2c_max, q3c_min, q3c_max, zc_min, zc_max):
        self.q1c_min = q1c_min
        self.q1c_max = q1c_max
        self.a2c_min = a2c_min
        self.a2c_max = a2c_max
        self.q3c_min = q3c_min
        self.q3c_max = q3c_max
        self.zc_min = zc_min
        self.zc_max = zc_max
        # print("Я вызвался!")
        # print("set_cylindrical_limits", "2Ц вызвано")

    def set_cylindrical_params(self, momentc_1, momentc_2, momentc_3, lengthc_1, lengthc_2, distancec, massc_2,
                               massc_3):
        self.momentc_1 = momentc_1
        self.momentc_2 = momentc_2
        self.momentc_3 = momentc_3
        self.lengthc_1 = lengthc_1
        self.lengthc_2 = lengthc_2
        self.distancec = distancec
        self.massc_2 = massc_2
        self.massc_3 = massc_3
        # print("Я долбаёб!")
        # print("set_cylindrical_limits", "3Ц вызвано")

    def set_coler_limits(self, q1col_min, q1col_max, a2col_min, a2col_max, q3col_min, q3col_max, zcol_min, zcol_max):
        self.q1col_min = q1col_min
        self.q1col_max = q1col_max
        self.a2col_min = a2col_min
        self.a2col_max = a2col_max
        self.q3col_min = q3col_min
        self.q3col_max = q3col_max
        self.zcol_min = zcol_min
        self.zcol_max = zcol_max

    def set_coler_params(self, momentcol_1, momentcol_2, momentcol_3, lengthcol_1, lengthcol_2, distancecol, masscol_2,
                               masscol_3):
        self.momentcol_1 = momentcol_1
        self.momentcol_2 = momentcol_2
        self.momentcol_3 = momentcol_3
        self.lengthcol_1 = lengthcol_1
        self.lengthcol_2 = lengthcol_2
        self.distancecol = distancecol
        self.masscol_2 = masscol_2
        self.masscol_3 = masscol_3

    def set_robot_type(self, robot_type):
        self.robot_type = robot_type
        # print("set_robot_type", "4 вызвано")

    def set_spline(self, spline):
        self.spline = spline

    # def cyclogramma(self, t, q1, q2, q3, q4):
    #     self.t = t
    #     self.q1 = q1
    #     self.q2 = q2
    #     self.q3 = q3
    #     self.q4 = q4


    def PID_regulator(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        # print("PID_regulator", "5 вызвано")

    def get_motor_params(self,J, n, Umax, Ku, Kq):
        self.J = J
        self.n = n
        self.Umax = Umax
        self.Ku = Ku
        self.Kq = Kq
        # print("get_motor_params", "6 вызвано")

    def update_trajectory(self, selected_coords, all_coords):
        # Установка невыбранных координат в ноль в зависимости от типа робота
        if self.robot_type == "Декартовый":
            coords_to_zero = {"1-я координата", "2-я координата", "3-я координата", "4-я координата"}
        elif self.robot_type == "Скара":
            coords_to_zero = {"1-я координата", "2-я координата", "3-я координата", "4-я координата"}
        elif self.robot_type == "Цилиндрический":
            coords_to_zero = {"1-я координата", "2-я координата", "3-я координата", "4-я координата"}
        else:
            print("Неизвестный тип робота.")
            return

        for coord in all_coords.keys():
            if coord in coords_to_zero and coord not in selected_coords:
                all_coords[coord] = (0, 0)

    def True_I(self, robot_type):
        if self.robot_type == "Декартовый":
            self.I_1 = self.momentd_1
            self.I_2 = self.momentd_1
        elif self.robot_type == "Скара":
            self.I_1 = self.moment_1
            self.I_2 = self.moment_2
        elif self.robot_type == "Цилиндрический":
            self.I_1 = self.momentc_1
            self.I_2 = self.momentc_2
        elif self.robot_type == "Колер":
            self.I_1 = self.momentcol_1
            self.I_2 = self.momentcol_2
        return self.I_1, self.I_2


    def True_q_min_max(self, robot_type, num_zvena):
        if num_zvena == 1:
            if self.robot_type == "Декартовый":
                self.q_min = self.x_min
                self.q_max = self.x_max
            elif self.robot_type == "Скара":
                self.q_min = self.q1s_min
                self.q_max = self.q1s_max
            elif self.robot_type == "Цилиндрический":
                self.q_min = self.q1c_min
                self.q_max = self.q1c_max
            elif self.robot_type == "Колер":
                self.q_min = self.a2col_min
                self.q_max = self.a2col_max
                # print(self.q1s_min, self.q1s_max, "self.q1s_min, self.q1s_max")
        elif num_zvena == 2:
            if self.robot_type == "Декартовый":
                self.q_min = self.y_min
                self.q_max = self.y_max
            elif self.robot_type == "Скара":
                self.q_min = self.q2s_min
                self.q_max = self.q2s_max
            elif self.robot_type == "Цилиндрический":
                self.q_min = self.a2c_min
                self.q_max = self.a2c_max
            elif self.robot_type == "Колер":
                self.q_min = self.q1col_min
                self.q_max = self.q1col_max
                # print(self.q2s_min, self.q2s_max, "self.q2s_min, self.q2s_max")
        return self.q_min, self.q_max

    def True_a_1_a_2(self, robot_type):
        if self.robot_type == "Декартовый":
            self.a_1 = self.x_max
            self.a_2 = self.y_max
        elif self.robot_type == "Скара":
            self.a_1 = self.length_1
            self.a_2 = self.length_2
        elif self.robot_type == "Цилиндрический":
            self.a_1 = self.lengthc_1
            self.a_2 = self.lengthc_2
        elif self.robot_type == "Колер":
            self.a_1 = self.lengthcol_1
            self.a_2 = self.lengthcol_2
        return self.a_1, self.a_2

    def set_line_params(self, x1, x2, y1, y2, speed,type_of_control):
        self.line_x1 = x1
        self.line_x2 = x2
        self.line_y1 = y1
        self.line_y2 = y2
        self.line_speed = speed
        self.type_of_control = type_of_control

        self.contur_line_creation(x1, x2, y1, y2)
        self.reverse_coordinate_transform()

    def set_circle_params(self, x,y,radius, speed, type_of_control):
        self.circle_x = x
        self.circle_y = y
        self.circle_radius = radius
        self.circle_speed = speed

        self.type_of_control = type_of_control

        self.contur_circle_creation(x, y, radius)
        self.reverse_coordinate_transform()

    def contur_line_creation(self, x1, x2, y1, y2):
        t_linspace = np.linspace(10,110,1000)
        x_contur = []
        y_contur = []
        dx = (x2-x1)/1000
        dy = (y2-y1)/1000
        for i in range (1000):
            x_contur.append(x1+dx*i)
            y_contur.append(y1+dy*i)
        self.t_contur = t_linspace
        self.x_contur = x_contur
        self.y_contur = y_contur

    def contur_circle_creation(self, x,y,radius):
        t_linspace = np.linspace(10, 110, 1000)
        fi_linspace = np.linspace(0,2*np.pi,1000)
        x_contur = []
        y_contur = []
        for i in range(len(fi_linspace)):
            x_contur.append(x+radius*np.cos(fi_linspace[i]))
            y_contur.append(y+radius*np.sin(fi_linspace[i]))
        self.t_contur = t_linspace
        self.x_contur = x_contur
        self.y_contur = y_contur


    def reverse_coordinate_transform(self):
        self.t_contur_control = []
        self.q1_contur_control = []
        self.q2_contur_control = []
        a_1, a_2 = self.True_a_1_a_2(self.robot_type)

        if self.robot_type == "Декартовый":
            self.t_contur_control = self.t_contur
            self.q1_contur_control = self.x_contur
            self.q2_contur_control = self.y_contur

        if self.robot_type == "Цилиндрический":
            self.t_contur_control = self.t_contur

            for i in range (len(self.t_contur)):
                if self.y_contur[i] == 0:
                    if self.x_contur[i] == 0:
                        tan = 1
                    else:
                        tan = 10**6
                else:
                    tan = self.x_contur[i]/self.y_contur[i]

                q1 = -1* np.arctan(tan)
                if (np.isnan(q1)):
                    q1 = 0
                q2 = np.sqrt((self.x_contur[i])**2+(self.y_contur[i])**2) - a_1

                self.q1_contur_control.append(q1)
                self.q2_contur_control.append(q2)
        if self.robot_type == "Скара":
            self.t_contur_control = self.t_contur
            for i in range(len(self.t_contur)):
                if self.x_contur[i] == 0:
                    if self.y_contur[i] == 0:
                        a = 0.25*np.pi
                    else:
                        a = np.pi/2
                else:
                    a = np.arctan2(self.y_contur[i],self.x_contur[i])
                if (np.isnan(a)):
                    a = 0
                r = np.sqrt((self.x_contur[i])**2+(self.y_contur[i])**2)
                g1 = np.arccos(((a_1**2) - (a_2**2) + (r**2))/(2*a_1*r))
                if (np.isnan(g1)):
                    g1 = 0
                g2 = np.arcsin((a_1/a_2)*np.sin(g1))
                if (np.isnan(g2)):
                    g2 = 0
                ARM = -1

                q1 = -np.pi/2 + a - g1*np.sign(ARM)
                q2 = (g1+g2)*np.sign(ARM)
                self.q1_contur_control.append(q1)
                self.q2_contur_control.append(q2)


        if self.robot_type == "Колер":
            self.t_contur_control = self.t_contur
            for i in range(len(self.t_contur)):
                square = 1 - (self.x_contur[i]/a_2)**2
                if square < 0:
                    square =0
                q1 = self.y_contur[i] - a_2*np.sqrt(square)

                q2 = -np.arcsin(self.x_contur[i]/a_2)
                if (np.isnan(q2)):
                    q2 = 0

                self.q1_contur_control.append(q1)
                self.q2_contur_control.append(q2)

    def generate_derivative_array(self, q1_array, q2_array,t_array):
        dq1_array = []
        dq2_array = []
        for i in range(len(t_array)):
            if i == 0:
                dq1_array.append(0)
                dq2_array.append(0)
            else:
                dq1_array.append(min(1,((q1_array[i]-q1_array[i-1])/(t_array[i]-t_array[i-1]))))
                dq2_array.append(min(1,((q2_array[i]-q2_array[i-1])/(t_array[i]-t_array[i-1]))))
        return dq1_array, dq2_array, t_array







    def spline_creation(self,q1, q2, t):
        # q1 = self.q1
        # q2 = self.q2
        # t = self.t
        # self.num_splain_dots
        num_splain_dots = self.num_splain_dots

        q1_spline = sp.interpolate.CubicSpline(t, q1, bc_type=((2, 0), (2, 0)))
        q2_spline = sp.interpolate.CubicSpline(t, q2, bc_type=((2, 0), (2, 0)))
        t_spline = np.arange(t[0], t[len(t)-1] + 1 / num_splain_dots, 1 / num_splain_dots)
        self.q_1_spline = q1_spline(t_spline)
        self.q_2_spline = q2_spline(t_spline)
        self.t_spline = t_spline

        #Копирование
        self.q_1_spline_copied = self.q_1_spline.copy()
        self.q_2_spline_copied = self.q_2_spline.copy()
        self.t_spline_copied = self.t_spline.copy()
        return self.q_1_spline, self.q_2_spline,  self.t_spline


    def quality_of_regulation(self,q, t, trajectory_q,  output_time_array):
        j = 0
        dist_prev = 3600
        real_time_array = []
        erorr_stable_array = []
        regulation_time_array = []
        for i in range(len(output_time_array)):
            dist = abs(output_time_array[i]-t[j])
            if (dist < dist_prev):
                dist_prev = dist
            else:
                dist_prev = 3600
                real_time_array.append(i-1)
                j = j + 1
            if (j == len(t)-1) and (i == len(output_time_array)-1):
                real_time_array.append(i)

        for i in range(len(real_time_array)):
            erorr_stable_array.append(q[i]-trajectory_q[real_time_array[i]])

        flag_point_notfound = True
        point_number = 0
        t_now = 0
        t_reg = 3600
        for i in range(len(output_time_array)):
            if (output_time_array[i]>=t[point_number]):
                point_number = point_number + 1
                flag_point_notfound = True
                t_now = output_time_array[i]

            if (flag_point_notfound) and ((abs(trajectory_q[i]-trajectory_q[real_time_array[point_number]])) <= 0.05*abs(trajectory_q[real_time_array[point_number]])):
                t_reg = output_time_array[i] - t_now
                regulation_time_array.append(t_reg)
                flag_point_notfound = False
                # print("Point added", i)

            if (flag_point_notfound == False) and ((abs(trajectory_q[i]-trajectory_q[real_time_array[point_number]])) > 0.05*abs(trajectory_q[real_time_array[point_number]])):
                flag_point_notfound = True
                del regulation_time_array[-1]
                # print("Point deleted", i)

        return erorr_stable_array, regulation_time_array

    def beautiful_print(self, erorr_stable_array, regulation_time_array, mode):
        avg_error = np.average(erorr_stable_array)
        median_error = np.median(erorr_stable_array)
        avg_reg_time = np.average(regulation_time_array)
        median_reg_time = np.median(regulation_time_array)

        # Сохранение данных в переменные
        if mode == "звено_1":
            self.error_1 = erorr_stable_array
            self.avg_error_1 = avg_error
            self.median_error_1 = median_error
            self.reg_time_1 = regulation_time_array
            self.avg_reg_time_1 = avg_reg_time
            self.median_reg_time_1 = median_reg_time
        elif mode == "звено_2":
            self.error_2 = erorr_stable_array
            self.avg_error_2 = avg_error
            self.median_error_2 = median_error
            self.reg_time_2 = regulation_time_array
            self.avg_reg_time_2 = avg_reg_time
            self.median_reg_time_2 = median_reg_time
        elif mode == "сплайн_1":
            self.spline_error_1 = erorr_stable_array
            self.spline_avg_error_1 = avg_error
            self.spline_median_error_1 = median_error
            self.spline_reg_time_1 = regulation_time_array
            self.spline_avg_reg_time_1 = avg_reg_time
            self.spline_median_reg_time_1 = median_reg_time
        elif mode == "сплайн_2":
            self.spline_error_2 = erorr_stable_array
            self.spline_avg_error_2 = avg_error
            self.spline_median_error_2 = median_error
            self.spline_reg_time_2 = regulation_time_array
            self.spline_avg_reg_time_2 = avg_reg_time
            self.spline_median_reg_time_2 = median_reg_time

        # Вывод данных
        # print(f"Ошибки для каждой точки: {erorr_stable_array}")
        # print(f"Среднее значение ошибок: {avg_error}")
        # print(f"Медианное значение ошибок: {median_error}")
        # print("—————")
        # print(f"Время регулирования для каждой точки: {regulation_time_array}")
        # print(f"Среднее значение времени регулирования: {avg_reg_time}")
        # print(f"Медианное значение времени регулирования: {median_reg_time}")
        # print("——————————————————————————————————————————————————————————————————————————————————————————")

    def checking_for_computational_errors(self, Med_1, Med_2, M1, M2):
        if self.robot_type == "Скара":
            if np.sign(Med_1) != np.sign(M1):
                M1 = 0.5 * Med_1
            if np.sign(Med_2) != np.sign(M2):
                M2 = 0.5 * Med_2
            M1 = Med_1 * ((2 * M1 / Med_1) / (1 + (2 * M1 / Med_1)))
            M2 = Med_2 * ((2 * M2 / Med_2) / (1 + (2 * M2 / Med_2)))
        else:
            pass
        return M1, M2

    def excess_fluctuation_filter(self, Med_1, Med_2, M1, M2):
        M1, M2 = self.checking_for_computational_errors(Med_1, Med_2, M1, M2)
        if np.sign(Med_1) != np.sign(M1):
            pass

        else:
            if M1 >= 0:
                if M1 <= 0.5 * Med_1:

                    pass
                if M1 > 0.5 * Med_1:
                    if Med_1 == 0:
                        M1 = 0
                    else:
                        M1 = Med_1 * ((2 * M1 / Med_1) / (1 + (2 * M1 / Med_1)))

            else:
                if M1 >= 0.5 * Med_1:

                    pass
                if M1 < 0.5 * Med_1:
                    if Med_1 == 0:
                        M1 = 0
                    else:

                        M1 = Med_1 * ((2 * M1 / Med_1) / (1 + (2 * M1 / Med_1)))

        if np.sign(Med_2) != np.sign(M2):
            pass

        else:
            if M2 >= 0:
                if M2 <= 0.5 * Med_2:

                    pass
                if M2 > 0.5 * Med_2:
                    if Med_2 == 0:
                        M2 = 0
                    else:
                        M2 = Med_2 * ((2 * M2 / Med_2) / (1 + (2 * M2 / Med_2)))

            else:
                if M2 >= 0.5 * Med_2:

                    pass
                if M2 < 0.5 * Med_2:
                    if Med_2 == 0:
                        M2 = 0
                    else:
                        M2 = Med_2 * ((2 * M2 / Med_2) / (1 + (2 * M2 / Med_2)))
        return M1, M2

    def ROBOT_function(self, q1, q2, t):

        # Переменные

        # Служебные
        test_flag = -1
        accuracy = 1e-3
        time_prev = -1e-8
        test_counter = 0

        Integral_channel_1 = 0
        Integral_channel_2 = 0

        # Механические
        K_U = 1
        Ce = 1
        Fi = 1
        Ra = 1
        Cm = 1
        T_I = 0.21
        T_U = 0.07
        flag_1 = True

        # Получаемые
        Kp = self.Kp
        Ki = self.Ki
        Kd = self.Kd
        J = self.J
        robot_type = self.robot_type
        I_1, I_2 = self.True_I(robot_type)
        q_min_1, q_max_1 = self.True_q_min_max(robot_type, 1)
        q_min_2, q_max_2 = self.True_q_min_max(robot_type, 2)

        # Рассчитываемые параметры
        # Звено 1
        q_input_1 = 1
        q_output_1 = 0
        U_1 = 0
        I_a_1 = 0
        a_w_1 = 0
        W_1 = 0
        q_error_prev_1 = 0

        # Звено 2

        q_input_2 = 1
        q_output_2 = 0
        U_2 = 0
        I_a_2 = 0
        a_w_2 = 0
        W_2 = 0
        q_error_prev_2 = 0

        # Выходные Массивы
        # Время
        output_time_array = []

        # Звено 1
        q_error_array_1 = []
        SAU_SUM_array_1 = []
        U_array_1 = []
        Ustar_array_1 = []
        I_array_1 = []
        M_ed_array_1 = []
        M1_array = []
        M_ed_corrected_array_1 = []
        acceleration_array_1 = []
        speed_array_1 = []
        output_q_array_1 = []

        # Звено 2
        q_error_array_2 = []
        SAU_SUM_array_2 = []
        U_array_2 = []
        Ustar_array_2 = []
        I_array_2 = []
        M_ed_array_2 = []
        M2_array = []
        M_ed_corrected_array_2 = []
        acceleration_array_2 = []
        speed_array_2 = []
        output_q_array_2 = []

        # Динамический расчёт траектории движения
        for i in range(len(t)):
            flag_2 = True

            if flag_1 == True:
                time_start = 0
                time_stop = t[0]
                flag_1 = False
            else:
                time_start = t[i - 1]
                time_stop = t[i]

            steps = []
            time_now = time_start
            while time_now <= time_stop:
                steps.append(time_now)
                time_now = time_now + accuracy

            # Задание на координаты
            q_input_1 = q1[i]
            q_input_2 = q2[i]

            for j in steps:
                time = j


                q_error_1 = q_input_1 - q_output_1
                q_error_2 = q_input_2 - q_output_2

                # Выход ПИДа
                # Звено 1
                Proportional_channel_1 = Kp[0] * q_error_1
                Integral_channel_1 = Integral_channel_1 + Ki[0] * q_error_1 * accuracy
                Differential_channel_1 = Kd[0] * ((q_error_1 - q_error_prev_1) / accuracy)

                if (abs(Differential_channel_1) > 10):
                    if Differential_channel_1 > 0:
                        Differential_channel_1 = 10
                    else:
                        Differential_channel_1 = -10

                SAU_SUM_1 = Proportional_channel_1 + Integral_channel_1 + Differential_channel_1
                q_error_prev_1 = q_error_1

                # Звено 2
                Proportional_channel_2 = Kp[1] * q_error_2
                Integral_channel_2 = Integral_channel_2 + Ki[1] * q_error_2 * accuracy
                Differential_channel_2 = Kd[1] * ((q_error_2 - q_error_prev_2) / accuracy)

                if (abs(Differential_channel_2) > 10):
                    if Differential_channel_2 > 0:
                        Differential_channel_2 = 10
                    else:
                        Differential_channel_2 = -10

                SAU_SUM_2 = Proportional_channel_2 + Integral_channel_2 + Differential_channel_2
                q_error_prev_2 = q_error_2

                # Расчёт напряжения
                # Звено 1
                U_need_1 = SAU_SUM_1 * K_U
                U_1 = U_1 + (U_need_1 * (1 / T_U) - U_1 * (1 / T_U)) * (accuracy)
                U_changed_1 = U_1 - W_1 * Ce * Fi
                # Звено 2
                U_need_2 = SAU_SUM_2 * K_U
                U_2 = U_2 + (U_need_2 * (1 / T_U) - U_2 * (1 / T_U)) * (accuracy)
                U_changed_2 = U_2 - W_2 * Ce * Fi

                # Расчёт тока
                # Звено 1
                if Ra != 0:
                    nI_Ra_1 = U_changed_1 / Ra
                else:
                    nI_Ra_1 = 0

                I_a_1 = I_a_1 + (nI_Ra_1 * (1 / T_I) - I_a_1 * (1 / T_I)) * (accuracy)
                I_a__Cm_1 = I_a_1 * Cm

                # Звено 2
                if Ra != 0:
                    nI_Ra_2 = U_changed_2 / Ra
                else:
                    nI_Ra_2 = 0

                I_a_2 = I_a_2 + (nI_Ra_2 * (1 / T_I) - I_a_2 * (1 / T_I)) * (accuracy)
                I_a__Cm_2 = I_a_2 * Cm

                # Расчёт момента
                # Звено 1
                M_ed_1 = I_a__Cm_1 * Fi
                # Звено 2
                M_ed_2 = I_a__Cm_2 * Fi

                # Расчёт моментов звеньев (Тут оба момента считаются сразу)
                if self.robot_type == "Декартовый":
                    d1 = (self.massd_1 + self.massd_2) / 2
                    d3 = self.massd_2 / 2
                    M1 = 2 * d1 * a_w_1
                    M2 = 2 * d3 * a_w_2
                    M1, M2 = self.excess_fluctuation_filter(M_ed_1, M_ed_2, M1, M2)

                elif self.robot_type == "Скара":
                    d1 = (self.moment_1 + self.masss_2 * self.length_1 ** 2 + 2 * self.masss_2 * self.length_2 * self.length_2 * self.length_1 * np.cos(
                                 q_output_1) + self.masss_2 * self.length_2 ** 2 + self.moment_2 / 2) / 2
                    d2 = (2 * self.masss_2 * self.length_2 * self.length_2 * self.length_1 + self.masss_2 * self.length_2 ** 2 + self.moment_2 / 2) / 2
                    d3 = (self.masss_2 * self.length_2 ** 2 + self.moment_2 / 2) / 2
                    M1 = 2 * d1 * a_w_1 + 2 * d2 * a_w_2 - 2 * self.masss_2 * self.length_2 * self.length_1 * np.sin(
                        q_output_2) * W_1 * W_2 - self.masss_2 * self.length_2 * self.length_1 * np.sin(
                        q_output_2) * W_2 ** 2
                    M2 = 2 * d1 * a_w_1 + 2 * d3 * a_w_2 + self.masss_2 * self.length_2 * self.length_1 * np.sin(
                        q_output_2) * W_1 ** 2
                    M1, M2 = self.excess_fluctuation_filter(M_ed_1, M_ed_2, M1, M2)

                elif self.robot_type == "Цилиндрический":
                    d1 = (1 / 2) * (self.momentc_1 + ((self.momentc_2 / 2) + self.massc_2 * (
                                self.lengthc_1 - 0.5 * self.lengthc_2 + q_output_2) ** 2))
                    d3 = self.massc_2 / 2
                    M1 = 2 * d1 * a_w_1 + 2 * self.massc_2 * (
                                self.lengthc_1 - 0.5 * self.lengthc_2 + q_output_2) * W_1 * W_2
                    M2 = 2 * d3 * a_w_1 - self.massc_2 * (
                                self.lengthc_1 - 0.5 * self.lengthc_2 + q_output_2) * W_1 * W_1
                    M1, M2 = self.excess_fluctuation_filter(M_ed_1, M_ed_2, M1, M2)

                elif self.robot_type == "Колер":
                    d1 = (1 + self.masscol_2) / 2
                    d2 = self.masscol_2 * self.lengthcol_2 * np.sin(q_output_2) / 2
                    d3 = (self.momentcol_2 + self.masscol_2 * self.lengthcol_2 ** 2) / 2
                    M1 = 2 * d1 * a_w_1 + 2 * d1 * a_w_1 + self.masscol_2 * self.lengthcol_2 * np.cos(
                        q_output_2) * W_2 ** 2
                    M2 = 2 * d2 * a_w_1 + 2 * d3 * a_w_1
                    # M1, M2, test_counter_1_1, test_counter_1_2,test_counter_1_3,test_counter_2_1,test_counter_2_2, test_counter_2_3 = self.excess_fluctuation_filter(M_ed_1, M_ed_2, M1, M2, test_counter_1_1, test_counter_1_2,test_counter_1_3,test_counter_2_1,test_counter_2_2, test_counter_2_3)
                    M1, M2 = self.excess_fluctuation_filter(M_ed_1, M_ed_2, M1, M2)

                # Звено 1
                M_ed_corrected_1 = M_ed_1 - M1
                M_ed_corrected_2 = M_ed_2 - M2


                # Расчёт углового ускорения
                # Звено 1
                if J[0] != 0:
                    a_w_1 = M_ed_corrected_1 / J[0]
                else:
                    a_w_1 = 0

                # Звено 2
                if J[1] != 0:
                    a_w_2 = M_ed_corrected_2 / J[0]
                else:
                    a_w_2 = 0

                # Расчёт скорости
                # Звено 1
                W_1 = W_1 + a_w_1 * (accuracy)
                # Звено2
                W_2 = W_2 + a_w_2 * (accuracy)

                # Расчёт координаты
                # Звено 1
                q_output_1 = q_output_1 + W_1 * (accuracy)
                # Звено 2
                q_output_2 = q_output_2 + W_2 * (accuracy)

                # Проверка на выход за пределы возможных координат
                # Звено 1
                if q_output_1 < q_min_1:
                    q_output_1 = q_min_1
                if q_output_1 > q_max_1:
                    q_output_1 = q_max_1

                # Звено 2
                if q_output_2 < q_min_2:
                    q_output_2 = q_min_2
                if q_output_2 > q_max_2:
                    q_output_2 = q_max_2

                time_prev = time

                output_time_array.append(time)
                # Звено 1
                q_error_array_1.append(q_error_1)
                SAU_SUM_array_1.append(SAU_SUM_1)
                U_array_1.append(U_1)
                Ustar_array_1.append(U_changed_1)
                I_array_1.append(I_a_1)
                M_ed_array_1.append(M_ed_1)
                M1_array.append(M1)
                M_ed_corrected_array_1.append(M_ed_corrected_1)
                acceleration_array_1.append(a_w_1)
                speed_array_1.append(W_1)
                output_q_array_1.append(q_output_1)

                # Звено 2
                q_error_array_2.append(q_error_2)
                SAU_SUM_array_2.append(SAU_SUM_2)
                U_array_2.append(U_2)
                Ustar_array_2.append(U_changed_2)
                I_array_2.append(I_a_2)
                M_ed_array_2.append(M_ed_2)
                M2_array.append(M2)
                M_ed_corrected_array_2.append(M_ed_corrected_2)
                acceleration_array_2.append(a_w_2)
                speed_array_2.append(W_2)
                output_q_array_2.append(q_output_2)

        # print(np.median(output_q_array_1), "output_q_array_1")
        # print(np.median(output_q_array_2), "output_q_array_2")
        # print(np.median(speed_array_2), "W")
        # print(np.median(acceleration_array_2), "1")
        # print(np.median(M_ed_corrected_array_2), "2")
        # print(np.median(M2_array), "3")
        # print(np.median(M_ed_array_2), "4")
        # print(np.median(I_array_2), "5")
        # print(np.median(Ustar_array_2), "6")
        # print(np.median(U_array_2), "7")
        # print(np.median(SAU_SUM_array_2), "8")
        # print(np.median(q_error_array_2), "9")
        return (
            output_time_array,
            q_error_array_1,
            SAU_SUM_array_1,
            U_array_1,
            Ustar_array_1,
            I_array_1,
            M_ed_array_1,
            M1_array,
            M_ed_corrected_array_1,
            acceleration_array_1,
            speed_array_1,
            output_q_array_1,
            q_error_array_2,
            SAU_SUM_array_2,
            U_array_2,
            Ustar_array_2,
            I_array_2,
            M_ed_array_2,
            M2_array,
            M_ed_corrected_array_2,
            acceleration_array_2,
            speed_array_2,
            output_q_array_2
        )

    def get_obobshennie_coordinates(self):

        if self.type_of_control == "Позиционное":
            if self.spline == False:
                (
                    self.output_time_array,
                    self.q_error_array_1,
                    self.SAU_SUM_array_1,
                    self.U_array_1,
                    self.Ustar_array_1,
                    self.I_array_1,
                    self.M_ed_array_1,
                    self.M1_array,
                    self.M_ed_corrected_array_1,
                    self.acceleration_array_1,
                    self.speed_array_1,
                    self.trajectory_q_1,
                    self.q_error_array_2,
                    self.SAU_SUM_array_2,
                    self.U_array_2,
                    self.Ustar_array_2,
                    self.I_array_2,
                    self.M_ed_array_2,
                    self.M2_array,
                    self.M_ed_corrected_array_2,
                    self.acceleration_array_2,
                    self.speed_array_2,
                    self.trajectory_q_2
                ) = self.ROBOT_function(self.q1, self.q2, self.t)

                erorr_stable_array, regulation_time_array = self.quality_of_regulation(self.q1, self.t,
                                                                                       self.trajectory_q_1,
                                                                                       self.output_time_array)
                self.beautiful_print(erorr_stable_array, regulation_time_array, "звено_1")

                erorr_stable_array, regulation_time_array = self.quality_of_regulation(self.q2, self.t,
                                                                                       self.trajectory_q_2,
                                                                                       self.output_time_array)
                self.beautiful_print(erorr_stable_array, regulation_time_array, "звено_2")


                # print(np.median(self.trajectory_q_1),"self.trajectory_q_1")
                # print(np.median(self.trajectory_q_2), "self.trajectory_q_2")

            else:
                q_1_spline, q_2_spline, t_spline = self.spline_creation(self.q1, self.q2, self.t)
                (
                    self.output_time_array,
                    self.q_error_array_1,
                    self.SAU_SUM_array_1,
                    self.U_array_1,
                    self.Ustar_array_1,
                    self.I_array_1,
                    self.M_ed_array_1,
                    self.M1_array,
                    self.M_ed_corrected_array_1,
                    self.acceleration_array_1,
                    self.speed_array_1,
                    self.trajectory_q_1,

                    self.q_error_array_2,
                    self.SAU_SUM_array_2,
                    self.U_array_2,
                    self.Ustar_array_2,
                    self.I_array_2,
                    self.M_ed_array_2,
                    self.M2_array,
                    self.M_ed_corrected_array_2,
                    self.acceleration_array_2,
                    self.speed_array_2,
                    self.trajectory_q_2
                ) = self.ROBOT_function( q_1_spline, q_2_spline, t_spline)
                erorr_stable_array, regulation_time_array = self.quality_of_regulation(self.q1, self.t,
                                                                                       self.trajectory_q_1,
                                                                                       self.output_time_array)
                self.beautiful_print(erorr_stable_array, regulation_time_array, "звено_1")
                erorr_stable_array, regulation_time_array = self.quality_of_regulation(self.q2, self.t,
                                                                                       self.trajectory_q_2,
                                                                                       self.output_time_array)
                self.beautiful_print(erorr_stable_array, regulation_time_array, "звено_2")

            # # Первое звено (сплайн)
            # num_zvena = 1
            # self.trajectory_q_1_spline, self.output_time_array_spline,self.speed_1_spline, self.acceleration_1_spline = self.ROBOT_function(num_zvena, q_1_spline, q_2_spline, t_spline)
            # erorr_stable_array, regulation_time_array = self.quality_of_regulation(self.q1, self.t, self.trajectory_q_1_spline, self.output_time_array_spline)
            # print("Первое звено робота — сплайн")
            # self.beautiful_print(erorr_stable_array, regulation_time_array, "сплайн_1")
            #
            # # Второе звено (сплайн)
            # num_zvena = 2
            # self.trajectory_q_2_spline, self.output_time_array_spline,self.speed_2_spline, self.acceleration_2_spline = self.ROBOT_function(num_zvena, q_1_spline, q_2_spline, t_spline)
            # erorr_stable_array, regulation_time_array = self.quality_of_regulation(self.q2, self.t, self.trajectory_q_2_spline, self.output_time_array_spline)
            # print("Второе звено робота — сплайн")
            # self.beautiful_print(erorr_stable_array, regulation_time_array, "сплайн_2")
            # return (
            #     self.trajectory_q_1, self.trajectory_q_2, self.output_time_array,
            #     self.trajectory_q_1_spline, self.trajectory_q_2_spline, self.output_time_array_spline
            # )

        if self.type_of_control == "Контурное":
            (
                self.output_time_array,
                self.q_error_array_1,
                self.SAU_SUM_array_1,
                self.U_array_1,
                self.Ustar_array_1,
                self.I_array_1,
                self.M_ed_array_1,
                self.M1_array,
                self.M_ed_corrected_array_1,
                self.acceleration_array_1,
                self.speed_array_1,
                self.trajectory_q_1,
                self.q_error_array_2,
                self.SAU_SUM_array_2,
                self.U_array_2,
                self.Ustar_array_2,
                self.I_array_2,
                self.M_ed_array_2,
                self.M2_array,
                self.M_ed_corrected_array_2,
                self.acceleration_array_2,
                self.speed_array_2,
                self.trajectory_q_2
            ) = self.ROBOT_function(self.q1_contur_control, self.q2_contur_control, self.t_contur_control)

            erorr_stable_array, regulation_time_array = self.quality_of_regulation(self.q1_contur_control, self.t_contur_control,
                                                                                   self.trajectory_q_1,
                                                                                   self.output_time_array)
            self.beautiful_print(erorr_stable_array, regulation_time_array, "звено_1")

            erorr_stable_array, regulation_time_array = self.quality_of_regulation(self.q2_contur_control, self.t_contur_control,
                                                                                   self.trajectory_q_2,
                                                                                   self.output_time_array)
            self.beautiful_print(erorr_stable_array, regulation_time_array, "звено_2")


            # # Первое звено
            # num_zvena = 1
            # self.trajectory_q_1, self.output_time_array, self.speed_1, self.acceleration_1 = self.ROBOT_function(num_zvena, self.q1_contur_control, self.q2_contur_control, self.t_contur_control)
            # erorr_stable_array, regulation_time_array = self.quality_of_regulation(self.q1_contur_control, self.t_contur_control, self.trajectory_q_1,
            #                                                                        self.output_time_array)
            # self.beautiful_print(erorr_stable_array, regulation_time_array, "звено_1")
            #
            # # Второе звено
            # num_zvena = 2
            # self.trajectory_q_2, self.output_time_array, self.speed_2, self.acceleration_2 = self.ROBOT_function(num_zvena, self.q1_contur_control, self.q2_contur_control, self.t_contur_control)
            # erorr_stable_array, regulation_time_array = self.quality_of_regulation(self.q2_contur_control, self.t_contur_control, self.trajectory_q_2,
            #                                                                        self.output_time_array)
            #
            # self.beautiful_print(erorr_stable_array, regulation_time_array, "звено_2")
            #
            # return (
            #     self.trajectory_q_1, self.trajectory_q_2, self.output_time_array,
            # )



    def coordinate_transform(self):
        if self.type_of_control == "Позиционное":
            trajectory_q_1 = self.trajectory_q_1
            trajectory_q_2 = self.trajectory_q_2
            # trajectory_q_1_spline = self.trajectory_q_1_spline
            # trajectory_q_2_spline = self.trajectory_q_2_spline

            cyclogramm_q_1 = self.q1
            cyclogramm_q_2 = self.q2
            robot_type = self.robot_type

            real_trajectory_x = self.real_trajectory_x
            real_trajectory_y = self.real_trajectory_y
            # real_trajectory_x_spline = self.real_trajectory_x_spline
            # real_trajectory_y_spline = self.real_trajectory_y_spline

            cyclogram_real_x = self.cyclogram_real_x
            cyclogram_real_y = self.cyclogram_real_y
            a_1 , a_2 = self.True_a_1_a_2(robot_type)
            if robot_type == "Декартовый":
                real_trajectory_x = trajectory_q_1
                real_trajectory_y = trajectory_q_2
                # real_trajectory_x_spline = trajectory_q_1_spline
                # real_trajectory_y_spline = trajectory_q_2_spline

                cyclogram_real_x = cyclogramm_q_1
                cyclogram_real_y = cyclogramm_q_2

            if robot_type == "Цилиндрический":
                for i in range(len(trajectory_q_1)):
                    real_trajectory_x.append(-(a_1 + trajectory_q_2[i]) * np.sin(trajectory_q_1[i]))
                    real_trajectory_y.append((a_1 + trajectory_q_2[i]) * np.cos(trajectory_q_1[i]))
                # for i in range(len(trajectory_q_1_spline)):
                #     real_trajectory_x_spline.append(-(a_1 + trajectory_q_2_spline[i]) * np.sin(trajectory_q_1_spline[i]))
                #     real_trajectory_y_spline.append((a_1 + trajectory_q_2_spline[i]) * np.cos(trajectory_q_1_spline[i]))

                for i in range(len(cyclogramm_q_1)):
                    cyclogram_real_x[i] = (-(a_1 + cyclogramm_q_2[i]) * np.sin(cyclogramm_q_1[i]))
                    cyclogram_real_y[i] = ((a_1 + cyclogramm_q_2[i]) * np.cos(cyclogramm_q_1[i]))


            if robot_type == "Скара":
                for i in range(len(trajectory_q_1)):
                    real_trajectory_x.append(
                        -a_1 * np.sin(trajectory_q_1[i]) - a_2 * np.sin(trajectory_q_1[i] + trajectory_q_2[i]))
                    real_trajectory_y.append(
                        a_1 * np.cos(trajectory_q_1[i]) + a_2 * np.cos(trajectory_q_1[i] + trajectory_q_2[i]))
                # for i in range(len(trajectory_q_1_spline)):
                #     real_trajectory_x_spline.append(
                #         -a_1 * np.sin(trajectory_q_1_spline[i]) - a_2 * np.sin(
                #             trajectory_q_1_spline[i] + trajectory_q_2_spline[i]))
                #     real_trajectory_y_spline.append(
                #         a_1 * np.cos(trajectory_q_1_spline[i]) + a_2 * np.cos(
                #             trajectory_q_1_spline[i] + trajectory_q_2_spline[i]))
                for i in range(len(cyclogramm_q_1)):
                    cyclogram_real_x[i] = -a_1 * np.sin(cyclogramm_q_1[i]) - a_2 * np.sin(
                        cyclogramm_q_1[i] + cyclogramm_q_2[i])
                    cyclogram_real_y[i] = a_1 * np.cos(cyclogramm_q_1[i]) + a_2 * np.cos(
                        cyclogramm_q_1[i] + cyclogramm_q_2[i])

            if robot_type == "Колер":
                for i in range(len(trajectory_q_1)):
                    real_trajectory_x.append(-a_2 * np.sin(trajectory_q_2[i]))
                    real_trajectory_y.append(trajectory_q_1[i] + a_2 * np.cos(trajectory_q_2[i]))
                # for i in range(len(trajectory_q_1_spline)):
                #     real_trajectory_x_spline.append(-a_2 * np.sin(trajectory_q_2_spline[i]))
                #     real_trajectory_y_spline.append(trajectory_q_1_spline[i] + a_2 * np.cos(trajectory_q_2_spline[i]))

                for i in range(len(cyclogramm_q_1)):
                    cyclogram_real_x[i] = (-a_2 * np.sin(cyclogramm_q_2[i]))
                    cyclogram_real_y[i] = (cyclogramm_q_1[i] + a_2 * np.cos(cyclogramm_q_2[i]))

            self.real_trajectory_x = real_trajectory_x
            self.real_trajectory_y = real_trajectory_y
            # self.real_trajectory_x_spline = real_trajectory_x_spline
            # self.real_trajectory_y_spline = real_trajectory_y_spline
            self.cyclogram_real_x = cyclogram_real_x
            self.cyclogram_real_y = cyclogram_real_y
            # return self.real_trajectory_x, self.real_trajectory_y, self.real_trajectory_x_spline, self.real_trajectory_y_spline, self.cyclogram_real_x, self.cyclogram_real_y
            return self.real_trajectory_x, self.real_trajectory_y, self.cyclogram_real_x, self.cyclogram_real_y

        if self.type_of_control == "Контурное":

            robot_type = self.robot_type

            trajectory_q_1 = self.trajectory_q_1
            trajectory_q_2 = self.trajectory_q_2

            real_trajectory_x = self.real_trajectory_x
            real_trajectory_y = self.real_trajectory_y

            a_1, a_2 = self.True_a_1_a_2(robot_type)
            if robot_type == "Декартовый":
                real_trajectory_x = trajectory_q_1
                real_trajectory_y = trajectory_q_2

            if robot_type == "Цилиндрический":
                for i in range(len(trajectory_q_1)):
                    real_trajectory_x.append(-(a_1 + trajectory_q_2[i]) * np.sin(trajectory_q_1[i]))
                    real_trajectory_y.append((a_1 + trajectory_q_2[i]) * np.cos(trajectory_q_1[i]))


            if robot_type == "Скара":
                for i in range(len(trajectory_q_1)):
                    real_trajectory_x.append(
                        -a_1 * np.sin(trajectory_q_1[i]) - a_2 * np.sin(trajectory_q_1[i] + trajectory_q_2[i]))
                    real_trajectory_y.append(
                        a_1 * np.cos(trajectory_q_1[i]) + a_2 * np.cos(trajectory_q_1[i] + trajectory_q_2[i]))

            if robot_type == "Колер":
                for i in range(len(trajectory_q_1)):
                    real_trajectory_x.append(-a_2 * np.sin(trajectory_q_2[i]))
                    real_trajectory_y.append(trajectory_q_1[i] + a_2 * np.cos(trajectory_q_2[i]))

            self.real_trajectory_x = real_trajectory_x
            self.real_trajectory_y = real_trajectory_y

            return self.real_trajectory_x, self.real_trajectory_y

    #Функции отвечающие за рисование графиков
    def decart_plane(self):

        fig = plt.figure(figsize=(11, 11))
        ax = fig.add_subplot()
        # if self.type_of_control == "Позиционное":
        #     if (self.spline == False):
        #         plt.plot(self.real_trajectory_x,  self.real_trajectory_y, color='red', label = 'Траектория')
        #     else:
        #         plt.plot(self.real_trajectory_x_spline, self.real_trajectory_y_spline, color='cyan', label='Траектория по сплайну')
        #     plt.scatter(self.cyclogram_real_x, self.cyclogram_real_y, c="blue",
        #                 linewidths=1,
        #                 marker="^",
        #                 edgecolor="black",
        #                 s=50, alpha=0.6, label = 'Заданные циклограммой точки')
        # if self.type_of_control == "Контурное":
        #     plt.plot(self.real_trajectory_x, self.real_trajectory_y, color='red', label='Траектория')
        #     plt.scatter(self.x_contur, self.y_contur, s=5, color='midnightblue', label='Контур')
        # plt.grid(True)
        robot_type = self.robot_type
        #Добавляет рабочую область для Декарта
        if robot_type == "Декартовый":
            a_1, a_2 = self.True_a_1_a_2(robot_type)
            rect = Rectangle((0, 0), a_1, a_2, linewidth=1, facecolor="palegreen",alpha=0.4, label = 'Рабочая область')
            plt.gca().set_aspect('equal', adjustable='box')
            ax.add_patch(rect)
            ax.set_facecolor('azure')

        # Добавляет рабочую область для Цилиндра
        if robot_type == "Цилиндрический":

            a_1 = self.lengthc_1
            a2_min = self.a2c_min  # Из текущего класса, а не self.app
            a2_max = self.a2c_max  # Из текущего класса, а не self.app

            # Рассчитываем минимальный и максимальный радиус
            rad_min = a_1 + a2_min
            rad_max = a_1 + a2_max

            # Создаем сектора рабочей области с заданным минимальным и максимальным радиусом
            small = Wedge((0, 0), rad_min, np.rad2deg(self.q1c_min) + 90, np.rad2deg(self.q1c_max) + 90, color="white",
                          alpha=1)
            big = Wedge((0, 0), rad_max, np.rad2deg(self.q1c_min) + 90, np.rad2deg(self.q1c_max) + 90,
                        color="darkgreen",
                        alpha=0.5, label = 'Рабочая область')

            # Настраиваем отображение и добавляем фигуры
            plt.gca().set_aspect('equal', adjustable='box')
            ax.add_artist(big)
            ax.add_artist(small)

        #Добавляют рабочую область для Скары
        if robot_type == "Скара":

            a_1, a_2 = self.True_a_1_a_2(robot_type)
            rad_min = a_1 - a_2
            rad_max = a_1 + a_2

            helping = (rad_max - rad_min) / 2  # !!!
            hypotenuse = helping + rad_min  # !!!

            small = Wedge((0, 0), rad_min, np.rad2deg(self.q1s_min) + 90, np.rad2deg(self.q1s_max) + 90, color="white",
                          alpha=1)
            big = Wedge((0, 0), rad_max, np.rad2deg(self.q1s_min) + 90, np.rad2deg(self.q1s_max) + 90,
                        color="darkgreen",
                        alpha=0.5, label = 'Рабочая область')
            right = Wedge(
                (hypotenuse * np.cos(self.q1s_max + np.pi / 2), hypotenuse * np.sin(self.q1s_max + np.pi / 2)), helping,
                np.rad2deg(self.q1s_max) + 90,
                np.rad2deg(self.q1s_max) + 270, color="darkgreen", alpha=0.5)  # !!!
            left = Wedge((hypotenuse * np.cos(self.q1s_min + np.pi / 2), hypotenuse * np.sin(self.q1s_min + np.pi / 2)),
                         helping,
                         np.rad2deg(self.q1s_min) + 270,
                         np.rad2deg(self.q1s_min) + 90, color="darkgreen", alpha=0.5)  # !!!

            plt.gca().set_aspect('equal', adjustable='box')
            ax.add_artist(big)
            ax.add_artist(small)
            ax.add_artist(right)  # !!!
            ax.add_artist(left)  # !!!
            plt.grid(True)

        # Добавляют рабочую область для Колера
        if robot_type == "Колер":
            # Получаем параметры из экземпляра приложения
            a1_min = self.a2col_min
            a1_max = self.a2col_max
            a1 = self.lengthcol_1
            a2 = self.lengthcol_2
            q1_min = self.q1col_min
            q1_max = self.q1col_max

            q1_linspace = np.linspace(q1_min, q1_max, 100)
            a1_linspace = np.linspace(a1_min, a1_max, 50)
            x_linspace = []
            y_linspace = []
            for i in range(len(a1_linspace)):
                for j in range(len(q1_linspace)):
                    y_linspace.append(a1_linspace[i] + a2 * np.cos(q1_linspace[j]))
                    x_linspace.append(-1 * a2 * np.sin(q1_linspace[j]))

            plt.scatter(x_linspace, y_linspace, c="palegreen",
                        s=80, alpha=0.5, label = 'Рабочая область')


        if self.type_of_control == "Позиционное":
            if (self.spline == False):
                plt.plot(self.real_trajectory_x,  self.real_trajectory_y, color='red', label = 'Траектория')
            else:
                plt.plot(self.real_trajectory_x, self.real_trajectory_y, color='cyan', label='Траектория по сплайну')
            plt.scatter(self.cyclogram_real_x, self.cyclogram_real_y, c="blue",
                        linewidths=1,
                        marker="^",
                        edgecolor="black",
                        s=50, alpha=0.6, label = 'Заданные циклограммой точки')
        if self.type_of_control == "Контурное":
            plt.plot(self.real_trajectory_x, self.real_trajectory_y, color='red', label='Траектория')
            plt.scatter(self.x_contur, self.y_contur, s=5, color='midnightblue', label='Контур')

        plt.grid(True)
        plt.xlim([-1.1, 1.1])  # Подкорректировано для более гибкого отображения
        plt.ylim([-1.1, 1.1])
        plt.legend()
        plt.title('Траектория движения робота')
        plt.axhline(y=0, color='steelblue', lw=1)
        plt.axvline(x=0, color='steelblue', lw=1)
        plt.grid(True, color='lightskyblue')
        if self.robot_type == "Декартовый":
            plt.title('Траектория робота Декарта')
        if self.robot_type == "Цилиндрический":
            plt.title('Траектория робота Цилиндр')
        if self.robot_type == "Скара":
            plt.title('Траектория робота Скара')
        if self.robot_type == "Колер":
            plt.title('Траектория робота Колер')
        plt.show()




    def obobshennie_coordinates_from_time(self):


        fig = plt.figure(figsize=(11, 11))
        ax = fig.add_subplot()

        if self.type_of_control == "Позиционное":
            if (self.spline == False):
                plt.plot(self.output_time_array, self.trajectory_q_1, color='red', label = 'Обощённые координаты первого звена')
                plt.plot(self.output_time_array, self.trajectory_q_2, color='blue', label = 'Обощённые координаты второго звена')
            else:
                plt.plot(self.output_time_array, self.trajectory_q_1, color='maroon', label='Обощённые координаты первого звена - сплайн')
                plt.plot(self.output_time_array, self.trajectory_q_2, color='cyan', label='Обощённые координаты второго звена- сплайн')

            plt.scatter(self.t, self.q1, c="red",
                        linewidths=1,
                        marker="s",
                        edgecolor="black",
                        s=50, alpha=0.5,  label = 'Циклограмма координат первого звена')
            plt.scatter(self.t, self.q2, c="blue",
                        linewidths=1,
                        marker="^",
                        edgecolor="black",
                        s=50, alpha=0.5,  label = 'Циклограмма координат второго звена')

            plt.grid(True)
            plt.legend(bbox_to_anchor=[1,1], loc ='lower center')

            plt.xlim([-0.1, 1.1*max(self.output_time_array)])  # Подкорректировано для более гибкого отображения
            plt.ylim([(min((1.1*min(self.trajectory_q_1)),(1.1*min(self.trajectory_q_2)))), 0.15+max((1.1*max(self.trajectory_q_1)),(1.1*max(self.trajectory_q_2)))])

        if self.type_of_control == "Контурное":
            plt.plot(self.output_time_array, self.trajectory_q_1, color='red', label = 'Обощённые координаты первого звена')
            plt.plot(self.output_time_array, self.trajectory_q_2, color='blue', label = 'Обощённые координаты второго звена')
            plt.scatter(self.t_contur_control, self.q1_contur_control, c="darkorange",
                         label='Контур движения первого звена')
            plt.scatter(self.t_contur_control, self.q2_contur_control, c="darkgreen",
                         label='Контур движения второго звена')
            plt.grid(True)
            plt.legend(bbox_to_anchor=[1, 1], loc='lower center')

            plt.xlim([-0.1, 1.1 * max(self.t_contur_control)])  # Подкорректировано для более гибкого отображения
            plt.ylim([(min((1.1 * min(self.trajectory_q_1)), (1.1 * min(self.trajectory_q_2)))),
                      0.15 + max((1.1 * max(self.trajectory_q_1)), (1.1 * max(self.trajectory_q_2)))])

        plt.title('Траектория движения робота')
        plt.axhline(y=0, color='steelblue', lw=1)
        plt.axvline(x=0, color='steelblue', lw=1)
        plt.grid(True, color='lightskyblue')
        if self.robot_type == "Декартовый":
            plt.title('Обобщённые координаты робота Декарта')
        if self.robot_type == "Цилиндрический":
            plt.title('Обобщённые координаты робота Цилиндр')
        if self.robot_type == "Скара":
            plt.title('Обобщённые координаты робота Скара')
        if self.robot_type == "Колер":
            plt.title('Обобщённые координаты робота Колер')
        plt.show()

    # Дополнительные

    def decart_coordinates_from_time(self):
        fig = plt.figure(figsize=(11, 11))
        ax = fig.add_subplot()

        if self.type_of_control == "Позиционное":
            if (self.spline == False):
                plt.plot(self.output_time_array, self.real_trajectory_x, color='red',
                         label='Декартовы координаты первого звена')
                plt.plot(self.output_time_array, self.real_trajectory_y, color='blue',
                         label='Декартовы координаты второго звена')
            else:
                plt.plot(self.output_time_array, self.real_trajectory_x, color='maroon',
                         label='Декартовы координаты первого звена - сплайн')
                plt.plot(self.output_time_array, self.real_trajectory_y, color='cyan',
                         label='Декартовы координаты второго звена- сплайн')

            plt.scatter(self.t, self.cyclogram_real_x, c="red",
                        linewidths=1,
                        marker="s",
                        edgecolor="black",
                        s=50, alpha=0.5, label='Циклограмма декартовых координат первого звена')
            plt.scatter(self.t, self.cyclogram_real_y, c="blue",
                        linewidths=1,
                        marker="^",
                        edgecolor="black",
                        s=50, alpha=0.5, label='Циклограмма координат второго звена')

            plt.grid(True)
            plt.legend(bbox_to_anchor=[1, 1], loc='lower center')

            plt.xlim([-0.1, 1.1 * max(self.output_time_array)])  # Подкорректировано для более гибкого отображения
            plt.ylim([(min((1.1 * min(self.real_trajectory_x)), (1.1 * min(self.real_trajectory_y)))),
                      0.15 + max((1.1 * max(self.real_trajectory_x)), (1.1 * max(self.real_trajectory_y)))])
            # plt.xlim([-0.1, 1.1 * max(self.output_time_array)])  # Подкорректировано для более гибкого отображения
            # plt.ylim([(min((1.1 * min(self.real_trajectory_x)), (1.1 * min(self.real_trajectory_y))),
            #           0.15 + max((1.1 * max(self.real_trajectory_x)), (1.1 * max(self.real_trajectory_y))))])

        if self.type_of_control == "Контурное":
            plt.plot(self.output_time_array, self.real_trajectory_x, color='red',
                     label='Декартовы координаты первого звена')
            plt.plot(self.output_time_array, self.real_trajectory_y, color='blue',
                     label= 'Декартовы координаты второго звена')
            plt.scatter(self.t_contur_control, self.x_contur, c="darkorange",
                        label='Контур движения первого звена')
            plt.scatter(self.t_contur_control, self.y_contur, c="darkgreen",
                        label='Контур движения второго звена')
            plt.grid(True)
            plt.legend(bbox_to_anchor=[1, 1], loc='lower center')

            plt.xlim([-0.1, 1.1 * max(self.t_contur_control)])  # Подкорректировано для более гибкого отображения
            plt.ylim([(min((1.1 * min(self.real_trajectory_x)), (1.1 * min(self.real_trajectory_y)))),
                      0.15 + max((1.1 * max(self.real_trajectory_x)), (1.1 * max(self.real_trajectory_y)))])
            # plt.xlim([-0.1, 1.1 * max(self.t_contur_control)])  # Подкорректировано для более гибкого отображения
            # plt.ylim([(min((1.1 * min(self.real_trajectory_x)), (1.1 * min(self.real_trajectory_y))),
            #            0.15 + max((1.1 * max(self.real_trajectory_x)), (1.1 * max(self.real_trajectory_y))))])

        plt.title('Траектория движения робота')
        plt.axhline(y=0, color='steelblue', lw=1)
        plt.axvline(x=0, color='steelblue', lw=1)
        plt.grid(True, color='lightskyblue')
        if self.robot_type == "Декартовый":
            plt.title('Декартовы координаты робота Декарта')
        if self.robot_type == "Цилиндрический":
            plt.title('Декартовы координаты робота Цилиндр')
        if self.robot_type == "Скара":
            plt.title('Декартовы координаты робота Скара')
        if self.robot_type == "Колер":
            plt.title('Декартовы координаты робота Колер')
        plt.show()

    def voltage_from_time(self):
        fig = plt.figure(figsize=(11, 11))
        ax = fig.add_subplot()

        if self.type_of_control == "Позиционное":
            if (self.spline == False):
                plt.plot(self.output_time_array, self.U_array_1, color='red',
                         label='Напряжение —  первое звено')
                plt.plot(self.output_time_array, self.U_array_2, color='blue',
                         label='Напряжение — второе звено')
            else:
                plt.plot(self.output_time_array, self.U_array_1, color='maroon',
                         label='Напряжение —  первое звено - сплайн')
                plt.plot(self.output_time_array, self.U_array_2, color='cyan',
                         label='Напряжение —  первое звено - сплайн')

            plt.grid(True)
            plt.legend(bbox_to_anchor=[1, 1], loc='lower center')

            plt.xlim([-0.1, 1.1 * max(self.output_time_array)])  # Подкорректировано для более гибкого отображения
            plt.ylim([(min((1.1 * min(self.U_array_1)), (1.1 * min(self.U_array_2)))),
                      0.15 + max((1.1 * max(self.U_array_1)), (1.1 * max(self.U_array_2)))])

        if self.type_of_control == "Контурное":
            plt.plot(self.output_time_array, self.U_array_1, color='red',
                     label='Напряжение —  первое звено')
            plt.plot(self.output_time_array, self.U_array_2 , color='blue',
                     label='Напряжение —  второе звено')

            plt.grid(True)
            plt.legend(bbox_to_anchor=[1, 1], loc='lower center')

            plt.xlim([-0.1, 1.1 * max(self.output_time_array)])  # Подкорректировано для более гибкого отображения
            plt.ylim([(min((1.1 * min(self.U_array_1)), (1.1 * min(self.U_array_2)))),
                      0.15 + max((1.1 * max(self.U_array_1)), (1.1 * max(self.U_array_2)))])

        plt.title('Напряжения')
        plt.axhline(y=0, color='steelblue', lw=1)
        plt.axvline(x=0, color='steelblue', lw=1)
        plt.grid(True, color='lightskyblue')
        if self.robot_type == "Декартовый":
            plt.title('Напряжения робот Декарт')
        if self.robot_type == "Цилиндрический":
            plt.title('Напряжения робот Цилиндр')
        if self.robot_type == "Скара":
            plt.title('Напряжения робот Скара')
        if self.robot_type == "Колер":
            plt.title('Напряжения робот Колер')
        plt.show()

    def voltage_star_from_time(self):
        fig = plt.figure(figsize=(11, 11))
        ax = fig.add_subplot()

        if self.type_of_control == "Позиционное":
            if (self.spline == False):
                plt.plot(self.output_time_array,  self.Ustar_array_1, color='red',
                         label='Напряжение* (Напряжение - ЭДС) первое звено')
                plt.plot(self.output_time_array,  self.Ustar_array_2, color='blue',
                         label='Напряжение* (Напряжение - ЭДС) второе звено')
            else:
                plt.plot(self.output_time_array,  self.Ustar_array_1, color='maroon',
                         label='Напряжение* (Напряжение - ЭДС) первое звено - сплайн')
                plt.plot(self.output_time_array,  self.Ustar_array_2, color='cyan',
                         label='Напряжение* (Напряжение - ЭДС) второе звено- сплайн')

            plt.grid(True)
            plt.legend(bbox_to_anchor=[1, 1], loc='lower center')

            plt.xlim([-0.1, 1.1 * max(self.output_time_array)])  # Подкорректировано для более гибкого отображения
            plt.ylim([(min((1.1 * min( self.Ustar_array_1)), (1.1 * min( self.Ustar_array_1)))),
                      0.15 + max((1.1 * max(self.Ustar_array_2)), (1.1 * max(self.Ustar_array_2)))])

        if self.type_of_control == "Контурное":
            plt.plot(self.output_time_array,  self.Ustar_array_1, color='red',
                     label='Напряжение* (Напряжение - ЭДС) первое звено')
            plt.plot(self.output_time_array,  self.Ustar_array_2 , color='blue',
                     label='Напряжение* (Напряжение - ЭДС) второе звено')

            plt.grid(True)
            plt.legend(bbox_to_anchor=[1, 1], loc='lower center')

            plt.xlim([-0.1, 1.1 * max(self.output_time_array)])  # Подкорректировано для более гибкого отображения
            plt.ylim([(min((1.1 * min(self.Ustar_array_1)), (1.1 * min( self.Ustar_array_2)))),
                      0.15 + max((1.1 * max(self.Ustar_array_1)), (1.1 * max(self.Ustar_array_2)))])

        plt.title('Напряжение* (Напряжение - ЭДС)')
        plt.axhline(y=0, color='steelblue', lw=1)
        plt.axvline(x=0, color='steelblue', lw=1)
        plt.grid(True, color='lightskyblue')
        if self.robot_type == "Декартовый":
            plt.title('Напряжение* (Напряжение - ЭДС) робот Декарт')
        if self.robot_type == "Цилиндрический":
            plt.title('Напряжение* (Напряжение - ЭДС) робот Цилиндр')
        if self.robot_type == "Скара":
            plt.title('Напряжение* (Напряжение - ЭДС) робот Скара')
        if self.robot_type == "Колер":
            plt.title('Напряжение* (Напряжение - ЭДС) робот Колер')
        plt.show()

    def current_from_time(self):
        fig = plt.figure(figsize=(11, 11))
        ax = fig.add_subplot()

        if self.type_of_control == "Позиционное":
            if (self.spline == False):
                plt.plot(self.output_time_array, self.I_array_1, color='red',
                         label='Ток первое звено')
                plt.plot(self.output_time_array, self.I_array_2, color='blue',
                         label='Ток второе звено')
            else:
                plt.plot(self.output_time_array_spline, self.I_array_1, color='maroon',
                         label='Ток первое звено - сплайн')
                plt.plot(self.output_time_array_spline, self.I_array_2, color='cyan',
                         label='Ток второе звено- сплайн')

            plt.grid(True)
            plt.legend(bbox_to_anchor=[1, 1], loc='lower center')

            plt.xlim([-0.1, 1.1 * max(self.output_time_array)])  # Подкорректировано для более гибкого отображения
            plt.ylim([(min((1.1 * min(self.I_array_1)), (1.1 * min(self.I_array_2)))),
                      0.15 + max((1.1 * max(self.I_array_1)), (1.1 * max(self.I_array_2)))])

        if self.type_of_control == "Контурное":
            plt.plot(self.output_time_array, self.I_array_1, color='red',
                     label='Ток первое звено')
            plt.plot(self.output_time_array, self.I_array_2, color='blue',
                     label='Ток второе звено')

            plt.grid(True)
            plt.legend(bbox_to_anchor=[1, 1], loc='lower center')

            plt.xlim([-0.1, 1.1 * max(self.output_time_array)])  # Подкорректировано для более гибкого отображения
            plt.ylim([(min((1.1 * min(self.I_array_1)), (1.1 * min(self.I_array_2)))),
                      0.15 + max((1.1 * max(self.I_array_1)), (1.1 * max(self.I_array_2)))])

        plt.title('Ток')
        plt.axhline(y=0, color='steelblue', lw=1)
        plt.axvline(x=0, color='steelblue', lw=1)
        plt.grid(True, color='lightskyblue')
        if self.robot_type == "Декартовый":
            plt.title('Ток робот Декарт')
        if self.robot_type == "Цилиндрический":
            plt.title('Ток робот Цилиндр')
        if self.robot_type == "Скара":
            plt.title('Ток робот Скара')
        if self.robot_type == "Колер":
            plt.title('Ток робот Колер')
        plt.show()

    def motor_moment_from_time(self):
        fig = plt.figure(figsize=(11, 11))
        ax = fig.add_subplot()

        if self.type_of_control == "Позиционное":
            if (self.spline == False):
                plt.plot(self.output_time_array, self.M_ed_array_1, color='red',
                         label='Момент электродвижущий первое звено')
                plt.plot(self.output_time_array, self.M_ed_array_2, color='blue',
                         label='Момент электродвижущий второе звено')
            else:
                plt.plot(self.output_time_array, self.M_ed_array_1, color='maroon',
                         label='Момент электродвижущий первое звено - сплайн')
                plt.plot(self.output_time_array, self.M_ed_array_2, color='cyan',
                         label='Момент электродвижущий второе звено- сплайн')

            plt.grid(True)
            plt.legend(bbox_to_anchor=[1, 1], loc='lower center')

            plt.xlim([-0.1, 1.1 * max(self.output_time_array)])  # Подкорректировано для более гибкого отображения
            plt.ylim([(min((1.1 * min(self.M_ed_array_1)), (1.1 * min(self.M_ed_array_2)))),
                      0.15 + max((1.1 * max(self.M_ed_array_1)), (1.1 * max(self.M_ed_array_2)))])

        if self.type_of_control == "Контурное":
            plt.plot(self.output_time_array, self.M_ed_array_1, color='red',
                     label='Момент электродвижущий первое звено')
            plt.plot(self.output_time_array,self.M_ed_array_2, color='blue',
                     label='Момент электродвижущий второе звено')

            plt.grid(True)
            plt.legend(bbox_to_anchor=[1, 1], loc='lower center')

            plt.xlim([-0.1, 1.1 * max(self.output_time_array)])  # Подкорректировано для более гибкого отображения
            plt.ylim([(min((1.1 * min(self.M_ed_array_1)), (1.1 * min(self.M_ed_array_2)))),
                      0.15 + max((1.1 * max(self.M_ed_array_1)), (1.1 * max(self.M_ed_array_2)))])

        plt.title('Момент электродвижущий')
        plt.axhline(y=0, color='steelblue', lw=1)
        plt.axvline(x=0, color='steelblue', lw=1)
        plt.grid(True, color='lightskyblue')
        if self.robot_type == "Декартовый":
            plt.title('Момент электродвижущий робот Декарт')
        if self.robot_type == "Цилиндрический":
            plt.title('Момент электродвижущий робот Цилиндр')
        if self.robot_type == "Скара":
            plt.title('Момент электродвижущий робот Скара')
        if self.robot_type == "Колер":
            plt.title('Момент электродвижущий робот Колер')
        plt.show()

    def load_moment_from_time(self):
        fig = plt.figure(figsize=(11, 11))
        ax = fig.add_subplot()

        if self.type_of_control == "Позиционное":
            if (self.spline == False):
                plt.plot(self.output_time_array, self.M1_array, color='red',
                         label='Момент нагрузки первое звено')
                plt.plot(self.output_time_array, self.M2_array, color='blue',
                         label='Момент нагрузки второе звено')
            else:
                plt.plot(self.output_time_array, self.M1_array, color='maroon',
                         label='Момент нагрузки первое звено - сплайн')
                plt.plot(self.output_time_array, self.M2_array, color='cyan',
                         label='Момент нагрузки второе звено- сплайн')

            plt.grid(True)
            plt.legend(bbox_to_anchor=[1, 1], loc='lower center')

            plt.xlim([-0.1, 1.1 * max(self.output_time_array)])  # Подкорректировано для более гибкого отображения
            plt.ylim([(min((1.1 * min(self.M1_array)), (1.1 * min(self.M2_array)))),
                      0.15 + max((1.1 * max(self.M1_array)), (1.1 * max(self.M2_array)))])

        if self.type_of_control == "Контурное":
            plt.plot(self.output_time_array, self.M1_array, color='red',
                     label='Момент нагрузки первое звено')
            plt.plot(self.output_time_array, self.M2_array, color='blue',
                     label='Момент нагрузки второе звено')

            plt.grid(True)
            plt.legend(bbox_to_anchor=[1, 1], loc='lower center')

            plt.xlim([-0.1, 1.1 * max(self.output_time_array)])  # Подкорректировано для более гибкого отображения
            plt.ylim([(min((1.1 * min(self.M1_array)), (1.1 * min(self.M2_array)))),
                      0.15 + max((1.1 * max(self.M1_array)), (1.1 * max(self.M2_array)))])

        plt.title('Момент нагрузки')
        plt.axhline(y=0, color='steelblue', lw=1)
        plt.axvline(x=0, color='steelblue', lw=1)
        plt.grid(True, color='lightskyblue')
        if self.robot_type == "Декартовый":
            plt.title('Момент нагрузки робот Декарт')
        if self.robot_type == "Цилиндрический":
            plt.title('Момент нагрузки робот Цилиндр')
        if self.robot_type == "Скара":
            plt.title('Момент нагрузки робот Скара')
        if self.robot_type == "Колер":
            plt.title('Момент нагрузки робот Колер')
        plt.show()

    def moment_star_from_time(self):
        fig = plt.figure(figsize=(11, 11))
        ax = fig.add_subplot()

        if self.type_of_control == "Позиционное":
            if (self.spline == False):
                plt.plot(self.output_time_array, self.M_ed_corrected_array_1, color='red',
                         label='Момент* (МЭ - М нагрузки) первое звено')
                plt.plot(self.output_time_array, self.M_ed_corrected_array_2, color='blue',
                         label='Момент* (МЭ - М нагрузки) второе звено')
            else:
                plt.plot(self.output_time_array, self.M_ed_corrected_array_1, color='maroon',
                         label='Момент* (МЭ - М нагрузки) первое звено - сплайн')
                plt.plot(self.output_time_array, self.M_ed_corrected_array_2, color='cyan',
                         label='Момент* (МЭ - М нагрузки) второе звено- сплайн')

            plt.grid(True)
            plt.legend(bbox_to_anchor=[1, 1], loc='lower center')

            plt.xlim([-0.1, 1.1 * max(self.output_time_array)])  # Подкорректировано для более гибкого отображения
            plt.ylim([(min((1.1 * min(self.M_ed_corrected_array_1)), (1.1 * min(self.M_ed_corrected_array_2)))),
                      0.15 + max((1.1 * max(self.M_ed_corrected_array_1)), (1.1 * max(self.M_ed_corrected_array_2)))])

        if self.type_of_control == "Контурное":
            plt.plot(self.output_time_array, self.M_ed_corrected_array_1, color='red',
                     label='Момент* (МЭ - М нагрузки) первое звено')
            plt.plot(self.output_time_array, self.M_ed_corrected_array_2, color='blue',
                     label='Момент* (МЭ - М нагрузки) второе звено')

            plt.grid(True)
            plt.legend(bbox_to_anchor=[1, 1], loc='lower center')

            plt.xlim([-0.1, 1.1 * max(self.output_time_array)])  # Подкорректировано для более гибкого отображения
            plt.ylim([(min((1.1 * min(self.M_ed_corrected_array_1)), (1.1 * min(self.M_ed_corrected_array_2)))),
                      0.15 + max((1.1 * max(self.M_ed_corrected_array_1)), (1.1 * max(self.M_ed_corrected_array_2)))])

        plt.title('Момент* (МЭ - М нагрузки)')
        plt.axhline(y=0, color='steelblue', lw=1)
        plt.axvline(x=0, color='steelblue', lw=1)
        plt.grid(True, color='lightskyblue')
        if self.robot_type == "Декартовый":
            plt.title('Момент* (МЭ - М нагрузки) робот Декарт')
        if self.robot_type == "Цилиндрический":
            plt.title('Момент* (МЭ - М нагрузки) робот Цилиндр')
        if self.robot_type == "Скара":
            plt.title('Момент* (МЭ - М нагрузки) робот Скара')
        if self.robot_type == "Колер":
            plt.title('Момент* (МЭ - М нагрузки) робот Колер')
        plt.show()

    def obobshennie_speed_from_time(self):
        fig = plt.figure(figsize=(11, 11))
        ax = fig.add_subplot()

        if self.type_of_control == "Позиционное":
            if (self.spline == False):
                plt.plot(self.output_time_array, self.speed_array_1, color='red',
                         label='Обощённые скорости первого звена')
                plt.plot(self.output_time_array,  self.speed_array_2, color='blue',
                         label='Обощённые скорости второго звена')
            else:
                plt.plot(self.output_time_array,  self.speed_array_1, color='maroon',
                         label='Обощённые скорости первого звена - сплайн')
                plt.plot(self.output_time_array, self.speed_array_2, color='cyan',
                         label='Обощённые скорости второго звена- сплайн')


            plt.grid(True)
            plt.legend(bbox_to_anchor=[1, 1], loc='lower center')

            plt.xlim([-0.1, 1.1 * max(self.output_time_array)])  # Подкорректировано для более гибкого отображения
            plt.ylim([(min((1.1 * min(self.speed_array_1)), (1.1 * min(self.speed_array_2)))),
                      0.15 + max((1.1 * max(self.speed_array_1)), (1.1 * max(self.speed_array_2)))])

        if self.type_of_control == "Контурное":
            plt.plot(self.output_time_array, self.speed_array_1, color='red',
                     label='Обощённые скорости первого звена')
            plt.plot(self.output_time_array, self.speed_array_2, color='blue',
                     label='Обощённые скорости второго звена')

            plt.grid(True)
            plt.legend(bbox_to_anchor=[1, 1], loc='lower center')

            plt.xlim([-0.1, 1.1 * max(self.output_time_array)])  # Подкорректировано для более гибкого отображения
            plt.ylim([(min((1.1 * min(self.speed_array_1)), (1.1 * min(self.speed_array_2)))),
                      0.15 + max((1.1 * max(self.speed_array_1)), (1.1 * max(self.speed_array_2)))])

        plt.title('Обобщённые скорости движения робота')
        plt.axhline(y=0, color='steelblue', lw=1)
        plt.axvline(x=0, color='steelblue', lw=1)
        plt.grid(True, color='lightskyblue')
        if self.robot_type == "Декартовый":
            plt.title('Обобщённые скорости робота Декарта')
        if self.robot_type == "Цилиндрический":
            plt.title('Обобщённые скорости робота Цилиндр')
        if self.robot_type == "Скара":
            plt.title('Обобщённые скорости робота Скара')
        if self.robot_type == "Колер":
            plt.title('Обобщённые скорости робота Колер')
        plt.show()

    def obobshennie_acceleration_from_time(self):
        fig = plt.figure(figsize=(11, 11))
        ax = fig.add_subplot()

        if self.type_of_control == "Позиционное":
            if (self.spline == False):
                plt.plot(self.output_time_array, self.acceleration_array_1, color='red',
                         label='Обощённые ускорения первого звена')
                plt.plot(self.output_time_array, self.acceleration_array_2, color='blue',
                         label='Обощённые ускорения второго звена')
            else:
                plt.plot(self.output_time_array, self.acceleration_array_1, color='maroon',
                         label='Обощённые ускорения первого звена - сплайн')
                plt.plot(self.output_time_array, self.acceleration_array_2, color='cyan',
                         label='Обощённые ускорения второго звена- сплайн')

            plt.grid(True)
            plt.legend(bbox_to_anchor=[1, 1], loc='lower center')

            plt.xlim([-0.1, 1.1 * max(self.output_time_array)])  # Подкорректировано для более гибкого отображения
            plt.ylim([(min((1.1 * min(self.acceleration_array_1)), (1.1 * min(self.acceleration_array_2)))),
                      0.15 + max((1.1 * max(self.acceleration_array_1)), (1.1 * max(self.acceleration_array_2)))])

        if self.type_of_control == "Контурное":
            plt.plot(self.output_time_array, self.acceleration_array_1, color='red',
                     label='Обощённые ускорения первого звена')
            plt.plot(self.output_time_array, self.acceleration_array_2, color='blue',
                     label='Обощённые ускорения второго звена')

            plt.grid(True)
            plt.legend(bbox_to_anchor=[1, 1], loc='lower center')

            plt.xlim([-0.1, 1.1 * max(self.output_time_array)])  # Подкорректировано для более гибкого отображения
            plt.ylim([(min((1.1 * min(self.acceleration_array_1)), (1.1 * min(self.acceleration_array_2)))),
                      0.15 + max((1.1 * max(self.acceleration_array_1)), (1.1 * max(self.acceleration_array_2)))])

        plt.title('Обобщённые ускорения движения робота')
        plt.axhline(y=0, color='steelblue', lw=1)
        plt.axvline(x=0, color='steelblue', lw=1)
        plt.grid(True, color='lightskyblue')
        if self.robot_type == "Декартовый":
            plt.title('Обобщённые ускорения робота Декарта')
        if self.robot_type == "Цилиндрический":
            plt.title('Обобщённые ускорения робота Цилиндр')
        if self.robot_type == "Скара":
            plt.title('Обобщённые ускорения робота Скара')
        if self.robot_type == "Колер":
            plt.title('Обобщённые ускорения робота Колер')
        plt.show()

    # Функции которые вызываются при закрытии диалогового окна с выводом графика
    def draw_on_decart_plane(self):

        self.output_time_array.clear()
        self.q_error_array_1.clear()
        self.SAU_SUM_array_1.clear()
        self.U_array_1.clear()
        self.Ustar_array_1.clear()
        self.I_array_1.clear()
        self.M_ed_array_1.clear()
        self.M1_array.clear()
        self.M_ed_corrected_array_1.clear()
        self.acceleration_array_1.clear()
        self.speed_array_1.clear()
        self.trajectory_q_1.clear()
        self.q_error_array_2.clear()
        self.SAU_SUM_array_2.clear()
        self.U_array_2.clear()
        self.Ustar_array_2.clear()
        self.I_array_2.clear()
        self.M_ed_array_2.clear()
        self.M2_array.clear()
        self.M_ed_corrected_array_2.clear()
        self.acceleration_array_2.clear()
        self.speed_array_2.clear()
        self.trajectory_q_2.clear()

        # self.trajectory_q_1.clear()
        # self.trajectory_q_2.clear()
        # self.output_time_array.clear()
        # self.trajectory_q_1_spline.clear()
        # self.trajectory_q_2_spline.clear()
        # self.output_time_array_spline.clear()

        self.real_trajectory_x.clear()
        self.real_trajectory_y.clear()
        # self.real_trajectory_x_spline.clear()
        # self.real_trajectory_y_spline.clear()

        # self.speed_1.clear()
        # self.speed_2.clear()
        # self.acceleration_1.clear()
        # self.acceleration_2.clear()

        # if self.type_of_control == "Позиционное":
        #     self.trajectory_q_1, self.trajectory_q_2, self.output_time_array, self.trajectory_q_1_spline, self.trajectory_q_2_spline, self.output_time_array_spline = self.get_obobshennie_coordinates()
        # if self.type_of_control == "Контурное":
        #     self.trajectory_q_1, self.trajectory_q_2, self.output_time_array = self.get_obobshennie_coordinates()
        self.get_obobshennie_coordinates()

        if self.type_of_control == "Позиционное":
            self.real_trajectory_x, self.real_trajectory_y, self.cyclogram_real_x, self.cyclogram_real_y = self.coordinate_transform()
        if self.type_of_control == "Контурное":
            self.real_trajectory_x, self.real_trajectory_y = self.coordinate_transform()

        self.decart_plane()



    def draw_obobshennie_coordinates_from_time(self):
        self.output_time_array.clear()
        self.q_error_array_1.clear()
        self.SAU_SUM_array_1.clear()
        self.U_array_1.clear()
        self.Ustar_array_1.clear()
        self.I_array_1.clear()
        self.M_ed_array_1.clear()
        self.M1_array.clear()
        self.M_ed_corrected_array_1.clear()
        self.acceleration_array_1.clear()
        self.speed_array_1.clear()
        self.trajectory_q_1.clear()
        self.q_error_array_2.clear()
        self.SAU_SUM_array_2.clear()
        self.U_array_2.clear()
        self.Ustar_array_2.clear()
        self.I_array_2.clear()
        self.M_ed_array_2.clear()
        self.M2_array.clear()
        self.M_ed_corrected_array_2.clear()
        self.acceleration_array_2.clear()
        self.speed_array_2.clear()
        self.trajectory_q_2.clear()

        # self.trajectory_q_1.clear()
        # self.trajectory_q_2.clear()
        # self.output_time_array.clear()
        # self.trajectory_q_1_spline.clear()
        # self.trajectory_q_2_spline.clear()
        # self.output_time_array_spline.clear()
        # self.speed_1.clear()
        # self.speed_2.clear()
        # self.acceleration_1.clear()
        # self.acceleration_2.clear()

        # if self.type_of_control == "Позиционное":
        #     self.trajectory_q_1, self.trajectory_q_2, self.output_time_array, self.trajectory_q_1_spline, self.trajectory_q_2_spline, self.output_time_array_spline = self.get_obobshennie_coordinates()
        # if self.type_of_control == "Контурное":
        #     self.trajectory_q_1, self.trajectory_q_2, self.output_time_array = self.get_obobshennie_coordinates()
        self.get_obobshennie_coordinates()
        self.obobshennie_coordinates_from_time()

    def draw_decart_coordinates_from_time(self):

        self.output_time_array.clear()
        self.q_error_array_1.clear()
        self.SAU_SUM_array_1.clear()
        self.U_array_1.clear()
        self.Ustar_array_1.clear()
        self.I_array_1.clear()
        self.M_ed_array_1.clear()
        self.M1_array.clear()
        self.M_ed_corrected_array_1.clear()
        self.acceleration_array_1.clear()
        self.speed_array_1.clear()
        self.trajectory_q_1.clear()
        self.q_error_array_2.clear()
        self.SAU_SUM_array_2.clear()
        self.U_array_2.clear()
        self.Ustar_array_2.clear()
        self.I_array_2.clear()
        self.M_ed_array_2.clear()
        self.M2_array.clear()
        self.M_ed_corrected_array_2.clear()
        self.acceleration_array_2.clear()
        self.speed_array_2.clear()
        self.trajectory_q_2.clear()

        # self.trajectory_q_1.clear()
        # self.trajectory_q_2.clear()
        # self.output_time_array.clear()
        # self.trajectory_q_1_spline.clear()
        # self.trajectory_q_2_spline.clear()
        # self.output_time_array_spline.clear()

        self.real_trajectory_x.clear()
        self.real_trajectory_y.clear()
        # self.real_trajectory_x_spline.clear()
        # self.real_trajectory_y_spline.clear()

        # self.speed_1.clear()
        # self.speed_2.clear()
        # self.acceleration_1.clear()
        # self.acceleration_2.clear()

        # if self.type_of_control == "Позиционное":
        #     self.trajectory_q_1, self.trajectory_q_2, self.output_time_array, self.trajectory_q_1_spline, self.trajectory_q_2_spline, self.output_time_array_spline = self.get_obobshennie_coordinates()
        # if self.type_of_control == "Контурное":
        #     self.trajectory_q_1, self.trajectory_q_2, self.output_time_array = self.get_obobshennie_coordinates()

        # self.get_obobshennie_coordinates()
        #
        # if self.type_of_control == "Позиционное":
        #     self.real_trajectory_x, self.real_trajectory_y, self.cyclogram_real_x, self.cyclogram_real_y = self.coordinate_transform()
        # if self.type_of_control == "Контурное":
        #     self.real_trajectory_x, self.real_trajectory_y = self.coordinate_transform()

        self.get_obobshennie_coordinates()

        if self.type_of_control == "Позиционное":
            self.real_trajectory_x, self.real_trajectory_y, self.cyclogram_real_x, self.cyclogram_real_y = self.coordinate_transform()
        if self.type_of_control == "Контурное":
            self.real_trajectory_x, self.real_trajectory_y = self.coordinate_transform()
        print("Мы здесь")
        self.decart_coordinates_from_time()

    def draw_voltage_from_time(self):
        self.output_time_array.clear()
        self.q_error_array_1.clear()
        self.SAU_SUM_array_1.clear()
        self.U_array_1.clear()
        self.Ustar_array_1.clear()
        self.I_array_1.clear()
        self.M_ed_array_1.clear()
        self.M1_array.clear()
        self.M_ed_corrected_array_1.clear()
        self.acceleration_array_1.clear()
        self.speed_array_1.clear()
        self.trajectory_q_1.clear()
        self.q_error_array_2.clear()
        self.SAU_SUM_array_2.clear()
        self.U_array_2.clear()
        self.Ustar_array_2.clear()
        self.I_array_2.clear()
        self.M_ed_array_2.clear()
        self.M2_array.clear()
        self.M_ed_corrected_array_2.clear()
        self.acceleration_array_2.clear()
        self.speed_array_2.clear()
        self.trajectory_q_2.clear()

        self.get_obobshennie_coordinates()
        self.voltage_from_time()

    def draw_voltage_star_from_time(self):
        self.output_time_array.clear()
        self.q_error_array_1.clear()
        self.SAU_SUM_array_1.clear()
        self.U_array_1.clear()
        self.Ustar_array_1.clear()
        self.I_array_1.clear()
        self.M_ed_array_1.clear()
        self.M1_array.clear()
        self.M_ed_corrected_array_1.clear()
        self.acceleration_array_1.clear()
        self.speed_array_1.clear()
        self.trajectory_q_1.clear()
        self.q_error_array_2.clear()
        self.SAU_SUM_array_2.clear()
        self.U_array_2.clear()
        self.Ustar_array_2.clear()
        self.I_array_2.clear()
        self.M_ed_array_2.clear()
        self.M2_array.clear()
        self.M_ed_corrected_array_2.clear()
        self.acceleration_array_2.clear()
        self.speed_array_2.clear()
        self.trajectory_q_2.clear()

        self.get_obobshennie_coordinates()
        self.voltage_star_from_time()

    def draw_current_from_time(self):
        self.output_time_array.clear()
        self.q_error_array_1.clear()
        self.SAU_SUM_array_1.clear()
        self.U_array_1.clear()
        self.Ustar_array_1.clear()
        self.I_array_1.clear()
        self.M_ed_array_1.clear()
        self.M1_array.clear()
        self.M_ed_corrected_array_1.clear()
        self.acceleration_array_1.clear()
        self.speed_array_1.clear()
        self.trajectory_q_1.clear()
        self.q_error_array_2.clear()
        self.SAU_SUM_array_2.clear()
        self.U_array_2.clear()
        self.Ustar_array_2.clear()
        self.I_array_2.clear()
        self.M_ed_array_2.clear()
        self.M2_array.clear()
        self.M_ed_corrected_array_2.clear()
        self.acceleration_array_2.clear()
        self.speed_array_2.clear()
        self.trajectory_q_2.clear()

        self.get_obobshennie_coordinates()
        self.current_from_time()

    def draw_motor_moment_from_time(self):
        self.output_time_array.clear()
        self.q_error_array_1.clear()
        self.SAU_SUM_array_1.clear()
        self.U_array_1.clear()
        self.Ustar_array_1.clear()
        self.I_array_1.clear()
        self.M_ed_array_1.clear()
        self.M1_array.clear()
        self.M_ed_corrected_array_1.clear()
        self.acceleration_array_1.clear()
        self.speed_array_1.clear()
        self.trajectory_q_1.clear()
        self.q_error_array_2.clear()
        self.SAU_SUM_array_2.clear()
        self.U_array_2.clear()
        self.Ustar_array_2.clear()
        self.I_array_2.clear()
        self.M_ed_array_2.clear()
        self.M2_array.clear()
        self.M_ed_corrected_array_2.clear()
        self.acceleration_array_2.clear()
        self.speed_array_2.clear()
        self.trajectory_q_2.clear()

        self.get_obobshennie_coordinates()
        self.motor_moment_from_time()

    def draw_load_moment_from_time(self):
        self.output_time_array.clear()
        self.q_error_array_1.clear()
        self.SAU_SUM_array_1.clear()
        self.U_array_1.clear()
        self.Ustar_array_1.clear()
        self.I_array_1.clear()
        self.M_ed_array_1.clear()
        self.M1_array.clear()
        self.M_ed_corrected_array_1.clear()
        self.acceleration_array_1.clear()
        self.speed_array_1.clear()
        self.trajectory_q_1.clear()
        self.q_error_array_2.clear()
        self.SAU_SUM_array_2.clear()
        self.U_array_2.clear()
        self.Ustar_array_2.clear()
        self.I_array_2.clear()
        self.M_ed_array_2.clear()
        self.M2_array.clear()
        self.M_ed_corrected_array_2.clear()
        self.acceleration_array_2.clear()
        self.speed_array_2.clear()
        self.trajectory_q_2.clear()

        self.get_obobshennie_coordinates()
        self.load_moment_from_time()

    def draw_moment_star_from_time(self):
        self.output_time_array.clear()
        self.q_error_array_1.clear()
        self.SAU_SUM_array_1.clear()
        self.U_array_1.clear()
        self.Ustar_array_1.clear()
        self.I_array_1.clear()
        self.M_ed_array_1.clear()
        self.M1_array.clear()
        self.M_ed_corrected_array_1.clear()
        self.acceleration_array_1.clear()
        self.speed_array_1.clear()
        self.trajectory_q_1.clear()
        self.q_error_array_2.clear()
        self.SAU_SUM_array_2.clear()
        self.U_array_2.clear()
        self.Ustar_array_2.clear()
        self.I_array_2.clear()
        self.M_ed_array_2.clear()
        self.M2_array.clear()
        self.M_ed_corrected_array_2.clear()
        self.acceleration_array_2.clear()
        self.speed_array_2.clear()
        self.trajectory_q_2.clear()

        self.get_obobshennie_coordinates()
        self.moment_star_from_time()

    def draw_obobshennie_speed_from_time(self):
        self.output_time_array.clear()
        self.q_error_array_1.clear()
        self.SAU_SUM_array_1.clear()
        self.U_array_1.clear()
        self.Ustar_array_1.clear()
        self.I_array_1.clear()
        self.M_ed_array_1.clear()
        self.M1_array.clear()
        self.M_ed_corrected_array_1.clear()
        self.acceleration_array_1.clear()
        self.speed_array_1.clear()
        self.trajectory_q_1.clear()
        self.q_error_array_2.clear()
        self.SAU_SUM_array_2.clear()
        self.U_array_2.clear()
        self.Ustar_array_2.clear()
        self.I_array_2.clear()
        self.M_ed_array_2.clear()
        self.M2_array.clear()
        self.M_ed_corrected_array_2.clear()
        self.acceleration_array_2.clear()
        self.speed_array_2.clear()
        self.trajectory_q_2.clear()
        # self.speed_1.clear()
        # self.speed_2.clear()
        # self.acceleration_1.clear()
        # self.acceleration_2.clear()
        # self.trajectory_q_1.clear()
        # self.trajectory_q_2.clear()
        # self.output_time_array.clear()
        # self.trajectory_q_1_spline.clear()
        # self.trajectory_q_2_spline.clear()
        # self.output_time_array_spline.clear()


        # if self.type_of_control == "Позиционное":
        #     self.trajectory_q_1, self.trajectory_q_2, self.output_time_array, self.trajectory_q_1_spline, self.trajectory_q_2_spline, self.output_time_array_spline = self.get_obobshennie_coordinates()
        # if self.type_of_control == "Контурное":
        #     self.trajectory_q_1, self.trajectory_q_2, self.output_time_array = self.get_obobshennie_coordinates()

        # if (self.spline == False):
        #     trajectory_q_1_speed, trajectory_q_2_speed, output_time_array_speed = self.generate_derivative_array(self.trajectory_q_1, self.trajectory_q_2, self.output_time_array)
        # else:
        #     trajectory_q_1_speed, trajectory_q_2_speed, output_time_array_speed = self.generate_derivative_array(self.trajectory_q_1_spline, self.trajectory_q_2_spline, self.output_time_array_spline)

        self.get_obobshennie_coordinates()
        self.obobshennie_speed_from_time()

    def draw_obobshennie_acceleration_from_time(self):
        self.output_time_array.clear()
        self.q_error_array_1.clear()
        self.SAU_SUM_array_1.clear()
        self.U_array_1.clear()
        self.Ustar_array_1.clear()
        self.I_array_1.clear()
        self.M_ed_array_1.clear()
        self.M1_array.clear()
        self.M_ed_corrected_array_1.clear()
        self.acceleration_array_1.clear()
        self.speed_array_1.clear()
        self.trajectory_q_1.clear()
        self.q_error_array_2.clear()
        self.SAU_SUM_array_2.clear()
        self.U_array_2.clear()
        self.Ustar_array_2.clear()
        self.I_array_2.clear()
        self.M_ed_array_2.clear()
        self.M2_array.clear()
        self.M_ed_corrected_array_2.clear()
        self.acceleration_array_2.clear()
        self.speed_array_2.clear()
        self.trajectory_q_2.clear()
        # self.speed_1.clear()
        # self.speed_2.clear()
        # self.acceleration_1.clear()
        # self.acceleration_2.clear()
        # self.trajectory_q_1.clear()
        # self.trajectory_q_2.clear()
        # self.output_time_array.clear()
        # self.trajectory_q_1_spline.clear()
        # self.trajectory_q_2_spline.clear()
        # self.output_time_array_spline.clear()
        #
        # if self.type_of_control == "Позиционное":
        #     self.trajectory_q_1, self.trajectory_q_2, self.output_time_array, self.trajectory_q_1_spline, self.trajectory_q_2_spline, self.output_time_array_spline = self.get_obobshennie_coordinates()
        # if self.type_of_control == "Контурное":
        #     self.trajectory_q_1, self.trajectory_q_2, self.output_time_array = self.get_obobshennie_coordinates()
        self.get_obobshennie_coordinates()
        self.obobshennie_acceleration_from_time()


class WorkspaceCalculator:
    def __init__(self, app_instance):
        # Получаем ссылку на экземпляр RobotTrajectoryApp
        self.app = app_instance

        self.t = 4
        self.Cyclogramma = [[1 * self.t, 0.1, 0.1, 0, 0],
                            [2 * self.t, 0.1, 0.5, 0, 0],
                            [3 * self.t, 0.3, 0.7, 0, 0],
                            [4 * self.t, 0.5, 0.9, 0, 0],
                            [5 * self.t, 0.7, 0.7, 0, 0],
                            [6 * self.t, 0.9, 0.5, 0, 0],
                            [7 * self.t, 0.9, 0.1, 0, 0],
                            [8 * self.t, 0.5, 0.1, 0, 0],
                            [9 * self.t, 0.1, 0.1, 0, 0]]

        self.PID_differential = 0.3  # Коэффиценты ПИДа
        self.PID_proportional = 1
        self.PID_integral = 0.0

        self.Integral_channel = 0  # Служебная переменная для работы интегратора
        self.Ra = 1  # Сопротивление двигателя
        self.T_e = 0.002  # Электрическая постоянная
        self.Cm = 1  # Ещё один коэффцент двигателя
        self.Ce = 1  # Ещё один коэффцент двигателя
        self.Fi = 1  # Поток через двигатель

        self.I_1 = 0.1  # Моменты инерции первого и второго звена
        self.I_2 = 0.1

        self.J = 1  # Момент инерции двигателя

        self.Processor_frequency = 8 * 1e-3  # Это чатстота вычислителя из параметров вычислителя. В общем отвечает за быстродействие системы
        # Это внутренние служебные переменные
        self.K_U = 1  # Управляем мы напряжением и эта штука "переводит" координату в напряжение
        self.Integral_channel = 0  # Служебная переменная для работы интегратора
        self.Ra = 1  # Сопротивление двигателя
        self.T_e = 0.002  # Электрическая постоянная
        self.Cm = 1  # Ещё один коэффцент двигателя
        self.Ce = 1  # Ещё один коэффцент двигателя
        self.Fi = 1  # Поток через двигатель
        self.Second_integral_block = 0  # Служебная переменная для работы интегратора
        self.Third_integral_block = 0  # Служебная переменная для работы интегратор
        self.time_prev = -1e-8  # Ещё две служебные переменные, для функционирования cистемы
        self.U_prev = 0
        self.last_counted_moment = 0
        self.accuracy = 1e-4  # Чем число меньше, тем мы ближе к реальности

        self.q1s_min = 0
        self.q1s_max = 0
        self.q2s_min = 0
        self.q2s_max = 0
        self.q3s_min = 0
        self.q3s_max = 0
        self.zs_min = 0
        self.zs_max = 0
        self.q1c_min = 0
        self.q1c_max = 0
        self.a2c_min = 0
        self.a2c_max = 0
        self.q3c_min = 0
        self.q3c_max = 0
        self.zc_min = 0
        self.zc_max = 0
        self.momentc_1 = 0
        self.momentc_2 = 0
        self.momentc_3 = 0
        self.lengthc_1 = 0
        self.lengthc_2 = 0
        self.distancec = 0
        self.massc_2 = 0
        self.massc_3 = 0
        self.q1col_min = 0
        self.q1col_max = 0
        self.a2col_min = 0
        self.a2col_max = 0
        self.q3col_min = 0
        self.q3col_max = 0
        self.zcol_min = 0
        self.zcol_max = 0
        self.momentcol_1 = 0
        self.momentcol_2 = 0
        self.momentcol_3 = 0
        self.lengthcol_1 = 0
        self.lengthcol_2 = 0
        self.distancecol = 0
        self.masscol_2 = 0
        self.masscol_3 = 0

    def set_scara_limits(self, q1s_min, q1s_max, q2s_min, q2s_max, q3s_min, q3s_max, zs_min, zs_max):
        self.q1s_min = q1s_min
        self.q1s_max = q1s_max
        self.q2s_min = q2s_min
        self.q2s_max = q2s_max
        self.q3s_min = q3s_min
        self.q3s_max = q3s_max
        self.zs_min = zs_min
        self.zs_max = zs_max

    def set_cylindrical_limits(self, q1c_min, q1c_max, a2c_min, a2c_max, q3c_min, q3c_max, zc_min, zc_max):
        self.q1c_min = q1c_min
        self.q1c_max = q1c_max
        self.a2c_min = a2c_min
        self.a2c_max = a2c_max
        self.q3c_min = q3c_min
        self.q3c_max = q3c_max
        self.zc_min = zc_min
        self.zc_max = zc_max

    def set_cylindrical_params(self, momentc_1, momentc_2, momentc_3, lengthc_1, lengthc_2, distancec, massc_2,
                               massc_3):
        self.momentc_1 = momentc_1
        self.momentc_2 = momentc_2
        self.momentc_3 = momentc_3
        self.lengthc_1 = lengthc_1
        self.lengthc_2 = lengthc_2
        self.distancec = distancec
        self.massc_2 = massc_2
        self.massc_3 = massc_3

    def set_coler_limits(self, q1col_min, q1col_max, a2col_min, a2col_max, q3col_min, q3col_max, zcol_min, zcol_max):
        self.q1col_min = q1col_min
        self.q1col_max = q1col_max
        self.a2col_min = a2col_min
        self.a2col_max = a2col_max
        self.q3col_min = q3col_min
        self.q3col_max = q3col_max
        self.zcol_min = zcol_min
        self.zcol_max = zcol_max

    def set_coler_params(self, momentcol_1, momentcol_2, momentcol_3, lengthcol_1, lengthcol_2, distancecol, masscol_2,
                               masscol_3):
        self.momentcol_1 = momentcol_1
        self.momentcol_2 = momentcol_2
        self.momentcol_3 = momentcol_3
        self.lengthcol_1 = lengthcol_1
        self.lengthcol_2 = lengthcol_2
        self.distancecol = distancecol
        self.masscol_2 = masscol_2
        self.masscol_3 = masscol_3

    def set_robot_type(self, robot_type):
        self.robot_type = robot_type

    def draw_workspace(self):
        # Метод для рисования рабочей области робота в зависимости от типа.
        if self.robot_type == "Декартовый":
            self._draw_cartesian_workspace()
        elif self.robot_type == "Скара":
            self._draw_scara_workspace()
        elif self.robot_type == "Цилиндрический":
            self._draw_cylindrical_workspace()
        elif self.robot_type == "Колер":
            self._draw_coler_workspace()

    def _draw_cartesian_workspace(self):
        # print("Мы тут на Бахмуте ебашимся")
        # Рисование рабочей области для декартового робота.
        fig = plt.figure(figsize=(7, 7))
        fig.patch.set_facecolor('#00f0d4')
        ax = fig.add_subplot()

        a_1 = self.app.x_max
        a_2 = self.app.y_max

        rect = Rectangle((0, 0), a_1, a_2, linewidth=1, facecolor="palegreen")
        plt.gca().set_aspect('equal', adjustable='box')
        ax.add_patch(rect)
        ax.set_facecolor('azure')
        plt.xlim([-0.3, a_1 + 0.3])
        plt.ylim([-0.3, a_2 + 0.3])

        plt.axhline(y=0, color='steelblue', lw=1)
        plt.axvline(x=0, color='steelblue', lw=1)
        plt.grid(True, color='lightskyblue')
        plt.title('Рабочая область робота Декарта')
        plt.show()

    def _draw_scara_workspace(self):
        # Рисование рабочей области для SCARA робота.
        fig = plt.figure(figsize=(7, 7))
        fig.patch.set_facecolor('#00f0d4')
        ax = fig.add_subplot()

        a_1 = self.app.length_1
        a_2 = self.app.length_2


        # q1_linspace = np.linspace(self.q1s_min, self.q1s_max, 100)
        # q2_linspace = np.linspace(self.q2s_min, self.q2s_max, 100)
        # x_linspace = []
        # y_linspace = []
        # for i in range(len(q1_linspace)):
        #     for j in range(len(q2_linspace)):
        #         y_linspace.append(a_1*np.cos(q1_linspace[i])+a_2*np.cos(q1_linspace[i]+q2_linspace[j]))
        #         x_linspace.append(-a_1*np.sin(q1_linspace[i])-a_2*np.sin(q1_linspace[i]+q2_linspace[j]))
        #
        # plt.scatter(x_linspace, y_linspace, c="palegreen",
        #             s=80, alpha=0.5)

        rad_min = a_1 - a_2
        rad_max = a_1 + a_2


        helping = (rad_max - rad_min) / 2  # !!!
        hypotenuse = helping + rad_min  # !!!

        small = Wedge((0, 0), rad_min, np.rad2deg(self.q1s_min) + 90, np.rad2deg(self.q1s_max) + 90, color="white",
                      alpha=1)
        big = Wedge((0, 0), rad_max, np.rad2deg(self.q1s_min) + 90, np.rad2deg(self.q1s_max) + 90, color="darkgreen",
                    alpha=0.5)
        right = Wedge((hypotenuse * np.cos(self.q1s_max + np.pi/2), hypotenuse * np.sin(self.q1s_max + np.pi/2)), helping,
                      np.rad2deg(self.q1s_max) + 90,
                      np.rad2deg(self.q1s_max) + 270, color="darkgreen", alpha=0.5)  # !!!
        left = Wedge((hypotenuse * np.cos(self.q1s_min + np.pi/2), hypotenuse * np.sin(self.q1s_min + np.pi/2)), helping,
                     np.rad2deg(self.q1s_min) + 270,
                     np.rad2deg(self.q1s_min) + 90, color="darkgreen", alpha=0.5)  # !!!

        plt.gca().set_aspect('equal', adjustable='box')
        ax.add_artist(big)
        ax.add_artist(small)
        ax.add_artist(right)  # !!!
        ax.add_artist(left)  # !!!
        plt.grid(True)
        plt.xlim([-2, 2])
        plt.ylim([-2, 2])
        plt.title('Рабочая область робота Скара')
        plt.axhline(y=0, color='steelblue', lw=1)
        plt.axvline(x=0, color='steelblue', lw=1)
        plt.grid(True, color='lightskyblue')
        plt.show()

    def _draw_cylindrical_workspace(self):
        # Рисование рабочей области для цилиндрического робота
        fig = plt.figure(figsize=(7, 7))
        fig.patch.set_facecolor('#00f0d4')
        ax = fig.add_subplot()

        # Получаем параметры из экземпляра приложения
        a_1 = self.app.lengthc_1
        a2_min = self.a2c_min  # Из текущего класса, а не self.app
        a2_max = self.a2c_max  # Из текущего класса, а не self.app

        # Рассчитываем минимальный и максимальный радиус
        rad_min = a_1 + a2_min
        rad_max = a_1 + a2_max

        # Создаем сектора рабочей области с заданным минимальным и максимальным радиусом
        small = Wedge((0, 0), rad_min, np.rad2deg(self.q1c_min) + 90, np.rad2deg(self.q1c_max) + 90, color="white",
                      alpha=1)
        big = Wedge((0, 0), rad_max, np.rad2deg(self.q1c_min) + 90, np.rad2deg(self.q1c_max) + 90, color="darkgreen",
                    alpha=0.5)

        # Настраиваем отображение и добавляем фигуры
        plt.gca().set_aspect('equal', adjustable='box')
        ax.add_artist(big)
        ax.add_artist(small)

        # Настройка отображения осей и сетки
        plt.grid(True)
        plt.xlim([-1.5, 1.5])  # Подкорректировано для более гибкого отображения
        plt.ylim([-1.5, 1.5])
        plt.title('Рабочая область робота Цилиндра')
        plt.axhline(y=0, color='steelblue', lw=1)
        plt.axvline(x=0, color='steelblue', lw=1)
        plt.grid(True, color='lightskyblue')
        plt.show()

    def _draw_coler_workspace(self):
        # Рисование рабочей области для цилиндрического робота
        fig = plt.figure(figsize=(7, 7))
        fig.patch.set_facecolor('#00f0d4')
        ax = fig.add_subplot()

        # Получаем параметры из экземпляра приложения
        a1_min = self.a2col_min
        a1_max = self.a2col_max
        a1 = self.lengthcol_1
        a2 = self.lengthcol_2
        q1_min = self.q1col_min
        q1_max = self.q1col_max



        # # height = a1-(np.cos(q1_max)*a2)
        # height = a1
        # x_min = 0 - a2
        # y_min = 0
        # width = 2*a2
        #
        #
        # # Создаем сектора рабочей области с заданным минимальным и максимальным радиусом
        # rect_white = Rectangle((x_min, y_min),width, height, linewidth=1, facecolor="white",alpha=1)
        #
        # # rect_1 = Wedge((0, 0), a2, 0, 180, color="white",
        # #               alpha=1)
        # top = Wedge((0, a1), a2, 0, 360, color="darkgreen",
        #               alpha=0.5)
        # bottom = Wedge((0, 0), a2, 0, 360, color="darkgreen",
        #             alpha=0.5)
        #
        # bottom_white = Wedge((0, 0), 5*a2, np.rad2deg(q1_max) + 90, np.rad2deg(q1_min) + 90, color="white",
        #             alpha=1)
        # rect_green = Rectangle((x_min, y_min), width, height, linewidth=1, facecolor="darkgreen", alpha=0.5)
        #
        # # Настраиваем отображение и добавляем фигуры
        # plt.gca().set_aspect('equal', adjustable='box')
        #
        # ax.add_artist(top)
        # ax.add_artist(bottom)
        # ax.add_patch(rect_white)
        # ax.add_patch(rect_green)
        # ax.add_artist(bottom_white)

        q1_linspace = np.linspace(q1_min, q1_max, 100)
        a1_linspace = np.linspace(a1_min, a1_max, 50)
        x_linspace = []
        y_linspace = []
        for i in range (len(a1_linspace)):
            for j in range (len(q1_linspace)):
                y_linspace.append(a1_linspace[i]+a2*np.cos(q1_linspace[j]))
                x_linspace.append(-1*a2*np.sin(q1_linspace[j]))

        plt.scatter(x_linspace, y_linspace, c="palegreen",
                        s=80, alpha=0.5)


        # Настройка отображения осей и сетки
        plt.grid(True)
        plt.xlim([-1.5, 1.5])  # Подкорректировано для более гибкого отображения
        plt.ylim([-1.5, 1.5])
        plt.title('Рабочая область робота Цилиндра')
        plt.axhline(y=0, color='steelblue', lw=1)
        plt.axvline(x=0, color='steelblue', lw=1)
        plt.grid(True, color='lightskyblue')
        plt.show()


class RobotTrajectoryApp(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        # Словари для хранения состояний
        self.calculator_values = {'bit_depth': '', 'exchange_cycle': '', 'control_cycle': '', 'filter_constant': ''}
        self.robot_type = "Декартовый"  # Для сохранения выбранного типа робота
        self.movement_type = None  # Для сохранения выбранного типа движения
        self.current_file_path = None  # Для хранения пути сохраненного файла
        self.update_checked = False
        self.spline = False

        # Инициализация класса для расчета рабочей области
        self.workspace_calculator = WorkspaceCalculator(self)
        # Инициализация класса для расчета графика
        self.trajectory_calculator = TrajectoryCalculator(self)

        # Параметры декартового робота
        self.massd_1 = 0
        self.massd_2 = 0
        self.massd_3 = 0
        self.momentd_1 = 0

        # Ограничения по координатам
        self.x_min = 0
        self.y_min = 0
        self.q_min = 0
        self.z_min = 0
        self.x_max = 0
        self.y_max = 0
        self.q_max = 0
        self.z_max = 0

        # Параметры SCARA
        self.moment_1 = 0
        self.moment_2 = 0
        self.moment_3 = 0
        self.length_1 = 0
        self.length_2 = 0
        self.distance = 0
        self.masss_2 = 0
        self.masss_3 = 0

        # Ограничения по координатам скара
        self.q1s_min = 0
        self.q1s_max = 0
        self.q2s_min = 0
        self.q2s_max = 0
        self.q3s_min = 0
        self.q3s_max = 0
        self.zs_min = 0
        self.zs_max = 0

        # Параметры Цилиндр
        self.momentc_1 = 0
        self.momentc_2 = 0
        self.momentc_3 = 0
        self.lengthc_1 = 0
        self.lengthc_2 = 0
        self.distancec = 0
        self.massc_2 = 0
        self.massc_3 = 0

        # Ограничения по координатам Цилиндр
        self.q1c_min = 0
        self.q1c_max = 0
        self.a2c_min = 0
        self.a2c_max = 0
        self.q3c_min = 0
        self.q3c_max = 0
        self.zc_min = 0
        self.zc_max = 0

        # Параметры Колер
        self.momentcol_1 = 0
        self.momentcol_2 = 0
        self.momentcol_3 = 0
        self.lengthcol_1 = 0
        self.lengthcol_2 = 0
        self.distancecol = 0
        self.masscol_2 = 0
        self.masscol_3 = 0

        # Ограничения по координатам Цилиндр
        self.q1col_min = 0
        self.q1col_max = 0
        self.a2col_min = 0
        self.a2col_max = 0
        self.q3col_min = 0
        self.q3col_max = 0
        self.zcol_min = 0
        self.zcol_max = 0

        # Параметры двигателей
        self.J = [0, 0, 0, 0]
        self.n = [0, 0, 0, 0]
        self.Umax = [0, 0, 0, 0]
        self.Ku = [0, 0, 0, 0]
        self.Kq = [0, 0, 0, 0]

        # Параметры регуляторов
        self.Kp = [0, 0, 0, 0]
        self.Ki = [0, 0, 0, 0]
        self.Kd = [0, 0, 0, 0]

        # Циклограмма
        self.t = [0] * 9
        self.q1 = [0] * 9
        self.q2 = [0] * 9
        self.q3 = [0] * 9
        self.q4 = [0] * 9

        self.trajectory_type = None  # Изначально тип траектории не установлен

        # Параметры прямой
        self.line_params = [0, 0, 0, 0, 0]  # [x1, x2, y1, y2, Скорость]
        # Параметры окружности
        self.circle_params = [0, 0, 0, 0]  # [x, y, радиус, Скорость]

        # Переменные для графика
        # Инициализация радио-кнопок
        self.cartesian_radio = QRadioButton("Декартовый")
        self.scara_radio = QRadioButton("Скара")
        self.cylindrical_radio = QRadioButton("Цилиндрический")
        self.coler_radio = QRadioButton("Цилиндрический")
        self.selected_coordinate_type=None

        # Установить начальное значение для типа робота
        self.cartesian_radio.setChecked(True)

        self.horoscope_view = True

        # Создание меню-бара
        self.create_menu_bar()
        self.prompt_load_last_file()
        self.init_ui()

    def init_ui(self):

        # Палитра тёмнозелёного цвета
        palette = self.palette()
        palette.setColor(QPalette.Window, QColor(124, 230, 177))
        self.setPalette(palette)

        # Создаем основной виджет и помещаем его в MainWindow
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.setWindowTitle('Расчёт траектории робота')
        # app = QApplication([])
        # screen = QDesktopWidget().screenGeometry()
        screen_geometry = QtGui.QGuiApplication.primaryScreen().geometry()
        screen_width = screen_geometry.width() - 10
        screen_height = screen_geometry.height() - 100
        self.setGeometry(0, 0, screen_width, screen_height)

        # Основной layout окна
        main_layout = QGridLayout(self.central_widget)

        # Создаем виджет времени и размещаем его
        self.datetime_widget = DateTimeWidget()
        # self.datetime_widget.setParent(self)
        # self.datetime_widget.move(0, 45)  # Позиция в левом верхнем углу

        self.weather_widget = WeatherWidget()
        # self.weather_widget.setParent(self)
        # self.weather_widget.move(0, 175)  # Позиция под виджетом времени

        # Создание и добавление виджета гороскопа
        self.horoscope_widget = HoroscopeWidget()
        # self.horoscope_widget.setParent(self)
        # self.horoscope_widget.move(1000, 175)
        self.horoscope_widget.setVisible(False)

        main_layout.addWidget(self.datetime_widget, 0, 0, alignment=Qt.AlignLeft | Qt.AlignTop)
        main_layout.addWidget(self.weather_widget, 1, 0, alignment=Qt.AlignLeft | Qt.AlignTop)
        main_layout.addWidget(self.horoscope_widget, 1, 1, alignment=Qt.AlignCenter | Qt.AlignTop)

        main_layout.setRowStretch(1, 1)

        self.setLayout(main_layout)

        self.setGeometry(0, 0, 605, 605)

        # Стили для меню (голубые кнопки меню)
        self.setStyleSheet("""
               QMenuBar {
                   background-color: #287571;  /* Цвет фона для меню */
               }

               QMenuBar::item {
                   background-color: #3ab0aa;  /* Голубой цвет для пунктов меню */
                   color: black;  /* Цвет текста */
                   border :1px solid ;
                   border-top-color : #3ab0aa;
                   border-bottom-color : #3ab0aa;
               }

               QMenuBar::item:selected {
                   background-color: #87CEFA;  /* Светло-голубой цвет при наведении */
               }

               QMenu {
                   background-color: #1abd61;  /* Цвет фона выпадающего меню */
               }

               QMenu::item:selected {
                   background-color: #87CEFA;  /* Цвет при наведении в выпадающем меню */
               }
           """)
        # Центрируем окно на экране
        qr = self.frameGeometry()
        cp = QtGui.QGuiApplication.primaryScreen().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def create_menu_bar(self):
        # Создаем меню-бар
        menu_bar = self.menuBar()
        menu_bar.setFont(QFont('Times', 17))
        menu_bar.setStyleSheet("border :5px solid ;"
                       "border-top-color : orange; "
                       "border-left-color : orange;"
                       "border-right-color : orange;"
                       "border-bottom-color : orange")
        # Создаем разделы меню
        burger_button = menu_bar.addAction(" ≡ ")
        burger_button.setFont(QFont('Times', 15))
        menu_bar.setStyleSheet(
            "border :5px solid;"
            "border-top-color : blue;"
            "border-left-color : orange;"
            "border-right-color : orange;"
            "border-bottom-color : blue"
        )

        # Добавляем пункт меню для проверки обновлений
        update_action = QAction("Проверить обновления", self)
        menu_bar.addAction(update_action)
        update_action.triggered.connect(check_for_update)

        data_menu = menu_bar.addMenu(' Данные ')
        data_menu.setFont(QFont('Times', 15))
        data_menu.setStyleSheet("border :5px solid ;"
                               "border-top-color : blue; "
                               "border-left-color :orange;"
                               "border-right-color :orange;"
                               "border-bottom-color : blue")
        mode_menu = menu_bar.addMenu(' Режим работы ')
        mode_menu.setFont(QFont('Times', 15))
        mode_menu.setStyleSheet("border :5px solid ;"
                                "border-top-color : blue; "
                                "border-left-color :orange;"
                                "border-right-color :orange;"
                                "border-bottom-color : blue")
        parameters_menu = menu_bar.addMenu(' Параметры ')
        parameters_menu.setFont(QFont('Times', 15))
        parameters_menu.setStyleSheet("border :5px solid ;"
                                "border-top-color : blue; "
                                "border-left-color :orange;"
                                "border-right-color :orange;"
                                "border-bottom-color : blue")
        calculate_menu = menu_bar.addMenu(' Расчет ')
        calculate_menu.setFont(QFont('Times', 15))
        calculate_menu.setStyleSheet("border :5px solid ;"
                                "border-top-color : blue; "
                                "border-left-color :orange;"
                                "border-right-color :orange;"
                                "border-bottom-color : blue")

        #Добавляем дейтвие к кнопке создатели
        burger_button.triggered.connect(self.show_creators_info)

        # Добавляем действия в меню "Режим работы"
        robot_type_action = QAction("Тип робота", self)
        mode_menu.addAction(robot_type_action)
        robot_type_action.triggered.connect(self.show_robot_type_dialog)

        movement_type_action = QAction("Тип движения", self)
        mode_menu.addAction(movement_type_action)
        movement_type_action.triggered.connect(self.show_movement_type_dialog)

        # Добавляем действия в меню "Данные"
        icons_path = os.path.dirname(__file__)

        # Создание действия для загрузки данных
        load_data_action = QAction(QIcon(os.path.join(icons_path, "o.png")), "Загрузить данные", self)
        load_data_action.setShortcut("Ctrl+O")

        # Создание действия для сохранения данных
        save_data_action = QAction(QIcon(os.path.join(icons_path, "ss.png")), "Сохранить как", self)
        save_data_action.setShortcut("Ctrl+Shift+S")

        # Создание действия для обычного сохранения
        save_action = QAction(QIcon(os.path.join(icons_path, "s.png")), "Сохранить", self)
        save_action.setShortcut("Ctrl+S")

        data_menu.addAction(load_data_action)
        data_menu.addAction(save_action)
        data_menu.addAction(save_data_action)

        # Привязываем действия к методам
        load_data_action.triggered.connect(self.load_data)
        save_data_action.triggered.connect(self.save_data_as)
        save_action.triggered.connect(self.save_data)

        # Добавляем разделы в меню "Параметры"
        robot_menu = parameters_menu.addMenu("Робот")
        robot_menu.setFont(QFont('Times', 13))


        robot_cartesian_action = robot_menu.addMenu("Декартовый")
        robot_cartesian_action.setFont(QFont('Times', 11))
        cartesian_params_action = QAction("Конструктивные параметры", self)
        cartesian_limits_action = QAction("Ограничения по координатам", self)
        robot_cartesian_action.addAction(cartesian_params_action)
        robot_cartesian_action.addAction(cartesian_limits_action)
        # Привязка действий к методам
        cartesian_params_action.triggered.connect(self.show_cartesian_params_dialog)
        cartesian_limits_action.triggered.connect(self.show_cartesian_limits_dialog)


        # Добавление меню "Цилиндрический" в меню робота
        robot_cylindrical_action = robot_menu.addMenu("Цилиндрический")
        robot_cylindrical_action.setFont(QFont('Times', 11))
        # Создание действий для "Цилиндрический"
        cylindrical_params_action = QAction("Конструктивные параметры", self)
        cylindrical_limits_action = QAction("Ограничения по координатам", self)
        # Добавление действий в меню
        robot_cylindrical_action.addAction(cylindrical_params_action)
        robot_cylindrical_action.addAction(cylindrical_limits_action)
        # Привязка действий к методам
        cylindrical_params_action.triggered.connect(self.show_cylindrical_params_dialog)
        cylindrical_limits_action.triggered.connect(self.show_cylindrical_limits_dialog)

        # Добавление меню "Скара" в меню робота
        robot_scara_action = robot_menu.addMenu("Скара")
        robot_scara_action.setFont(QFont('Times', 11))
        scara_params_action = QAction("Конструктивные параметры", self)
        robot_scara_action.addAction(scara_params_action)
        # Добавляем действие для открытия окна ограничений по координатам SCARA
        scara_limits_action = QAction("Ограничения по координатам", self)
        robot_scara_action.addAction(scara_limits_action)
        scara_limits_action.triggered.connect(self.show_scara_limits_dialog)
        scara_params_action.triggered.connect(self.show_scara_params_dialog)

        # Добавление меню "Колер" в меню робота
        robot_coler_action = robot_menu.addMenu("Колер")
        robot_coler_action.setFont(QFont('Times', 11))
        coler_params_action = QAction("Конструктивные параметры", self)
        robot_coler_action.addAction(coler_params_action)
        # Добавляем действие для открытия окна ограничений по координатам SCARA
        coler_limits_action = QAction("Ограничения по координатам", self)
        robot_coler_action.addAction(coler_limits_action)
        coler_limits_action.triggered.connect(self.show_coler_limits_dialog)
        coler_params_action.triggered.connect(self.show_coler_params_dialog)

        # Система управления
        system_menu = parameters_menu.addMenu("Система управления")
        motor_params_action = QAction("Параметры двигателей", self)
        motor_params_action.setFont(QFont('Times', 13))
        system_menu.addAction(motor_params_action)
        motor_params_action.triggered.connect(self.show_motor_params_dialog)
        # Пункт меню "Параметры регуляторов"
        regulator_params_action = QAction("Параметры регуляторов", self)
        regulator_params_action.setFont(QFont('Times', 13))
        system_menu.addAction(regulator_params_action)
        regulator_params_action.triggered.connect(self.show_regulator_params_dialog)

        # Добавляем действие для Вычислителя
        calculator_action = QAction("Вычислитель", self)
        parameters_menu.addAction(calculator_action)
        calculator_action.triggered.connect(self.show_calculator_dialog)

        # Движение
        movement_menu = parameters_menu.addMenu("Движение")
        position_action = QAction("Позиционное", self)
        contour_action = QAction("Контурное", self)
        position_action.setFont(QFont('Times', 13))
        contour_action.setFont(QFont('Times', 13))
        movement_menu.addAction(position_action)
        movement_menu.addAction(contour_action)
        # Привязываем действие к методу
        position_action.triggered.connect(self.show_cyclegram_dialog)
        contour_action.triggered.connect(self.show_trajectory_type_dialog)

        # Путь к папке с иконками
        icons_path = os.path.dirname(__file__)

        # Действие для графика
        graph_action = QAction(QIcon(os.path.join(icons_path, "graf 2.jpg")), "График", self)
        graph_action.setShortcut("Ctrl+G")
        calculate_menu.addAction(graph_action)
        graph_action.triggered.connect(self.show_graph_settings_dialog)

        workspace_action = calculate_menu.addAction(QIcon(os.path.join(icons_path, "graf.jpg")), "Рабочая область")
        # Привязываем действие для рабочей области
        workspace_action.triggered.connect(self.draw_workspace)

    def prompt_load_last_file(self):
        last_file_path = "last_file.txt"
        if os.path.exists(last_file_path):
            with open(last_file_path, "r") as f:
                saved_path = f.read().strip()

            if saved_path and os.path.exists(saved_path):
                file_name = os.path.basename(saved_path)
                reply = QMessageBox.question(
                    self,
                    "Загрузить последний файл?",
                    f"Загрузить последний файл?\n{file_name}",
                    QMessageBox.Yes | QMessageBox.No
                )
                if reply == QMessageBox.Yes:
                    self.load_data_from_path(saved_path)

    # def showEvent(self, event):
    #     super().showEvent(event)
    #     if not self.update_checked:  # Проверяем, не была ли уже выполнена проверка
    #         check_for_update()
    #         self.update_checked = True

    def keyPressEvent(self, event):
        """ Проверка нажатия на клавиатуру """
        if event.key() == QtCore.Qt.Key_H:
            self.horoscope_view = self.horoscope_visible(self.horoscope_view)
        elif event.key() == QtCore.Qt.Key_T:
            self.t = tetris()

    def horoscope_visible(self, view):
        """ Влючение/выключение отображения гороскопа """
        if view:
            # self.s = self.weather_widget.pos()
            # print(self.weather_widget.pos(), 0)
            self.horoscope_widget.setVisible(True)
            view = False
        else:
            self.horoscope_widget.setVisible(False)
            # self.resize(self.s)
            # print(self.s)
            view = True
        return view

    def show_creators_info(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("Создатели")

        layout = QVBoxLayout()
        creators_label = QLabel("<b>Создатели Бюджет разработки:</b>")
        creators_label.setFont(QFont('Times', 14))
        creators_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(creators_label)

        image_label = QLabel()
        image_path = os.path.join(os.path.dirname(__file__), "ohota.jpg")

        pixmap = QPixmap(image_path)
        if pixmap.isNull():
            print("Ошибка: Изображение не найдено или не удалось загрузить.")
        else:
            image_label.setPixmap(pixmap)
            image_label.setAlignment(Qt.AlignCenter)

        layout.addWidget(image_label)

        dialog.setLayout(layout)
        dialog.exec_()

    def show_cartesian_help(self):
        # Окно с подсказкой по Декарт
        cartesian_help_dialog = QDialog(self)
        cartesian_help_dialog.setWindowTitle("Подсказка Декарт")
        cartesian_help_layout = QVBoxLayout()

        cartesian_help_text = QLabel("Декартовый робот имеет 4 степени подвижности, причем 3 степени обеспечивают\n"
    "перемещение рабочего органа по трем взаимноперпендикулярным осям X, Y, Z, а четвертая выполняет\n"
    "операцию вращения (ориентации) рабочего органа.\n"
    "Упрощенное изображение робота представлено на рисунке, где q1, q2, q4 - обобщенные координаты\n"
    "(перемещения вдоль соответственно осей X, Y, Z), а q3 - угол поворота схвата.\n"
    "Конструктивно робот состоит из неподвижной платформы и трехзвенной механической руки со схватом.\n"
    "1 и 2 звенья перемещаются в горизонтальной плоскости вдоль соответственно осей X и Y, а 3 звено может\n"
    "перемещаться в вертикальной плоскости вдоль оси Z и вращаться относительно шарнира 0.\n"
    "Все звенья манипулятора предполагаются абсолютно твердыми.\n\n"
    "Управление движением робота осуществляется с помощью четырех независимых электромеханических электроприводов,\n"
    "каждый из которых состоит из электрического двигателя постоянного тока с независимым возбуждением и редуктора.\n\n"
    "Уравнения динамики робота легко могут быть получены из уравнений Лагранжа 2 рода и имеют следующий вид:\n"
    "A1 * q1'' = M1\n"
    "A2 * q2'' = M2\n"
    "A3 * q3'' = M3\n"
    "A4 * q4'' = M4\n\n"
    "M1, M2, M3, M4 - это обобщенные силы, соответствующие обобщенным переменным.\n"
    "A1, A2, A3, A4 - это коэффициенты, зависящие от конструктивных параметров робота и имеют вид:\n"
    "A1 = n1 + n2 + n3 + n1d * n1²\n"
    "A2 = m2 + m3 + m2д * n2²\n"
    "A3 = J3 + J3д * n3²\n"
    "A4 = m3 + m4д * n4²\n\n"
    "Обобщенные координаты qi не должны выходить за пределы диапазона допустимых значений.\n",
                                     self)
        cartesian_help_text.setFont(QFont('Times', 12))
        cartesian_help_layout.addWidget(cartesian_help_text)

        cartesian_help_dialog.setLayout(cartesian_help_layout)
        cartesian_help_dialog.exec_()

    def show_scara_help(self):
        # Окно с подсказкой по Скара
        scara_help_dialog = QDialog(self)
        scara_help_dialog.setWindowTitle("Подсказка Скара")
        scara_help_layout = QVBoxLayout()

        scara_help_text = QLabel("Робот с кинематической схемой СКАРА имеет 4 степени подвижности.\n"
    "Конструктивно робот состоит из неподвижной платформы и трехзвенной механической руки со схватом.\n"
    "Упрощенное изображение робота представлено на рисунке.\n"
    "1 и 2 звенья манипулятора соединены шарниром 02 и перемещаются в горизонтальной плоскости.\n"
    "3 звено соединено со 2-м звеном с помощью шарнира 03 и может вращаться вокруг 03 в горизонтальной плоскости\n"
    "и одновременно перемещаться в вертикальной плоскости.1 звено соединено с платформой с помощью шарнира 01.\n"
    "Оси шарниров 01, 02, 03 предполагаются параллельными.Будем считать звенья абсолютно твердыми телами.\n"
    "Управление движением робота осуществляется с помощью 4-х независимых электромеханических приводов A1, A2, A3, A4,\n"
    "каждый из которых состоит из электродвигателя постоянного тока с независимым возбуждением и редуктора.\n"
    "Приводы Di (i:=1..3) управляют поворотами соответствующих звеньев робота в горизонтальной плоскости,\n"
    "привод A4 управляет вертикальными перемещениями 3 звена.\n"
    "В качестве обобщенных координат будем использовать:\n"
    "q1, q2, q3 - углы поворота звеньев, q4 - линейное перемещение третьего звена вокруг вертикальной оси.\n"
    "Уравнения динамики робота могут быть получены из уравнений Лагранжа 2-го рода и выглядят следующим образом:\n"
    "q1'' * (A1 + A2 * cos(q2)) + q2 * (A3 + A2 / 2 * cos(q2)) + q3 * A4 - A2 * q1 * q2 * sin(q2)\n"
    "in(q2) - A2 / 2 * q2² * sin(q2) = M1;\n"
    "q1'' * (A3 + A2 / 2 * cos(q2)) + q2 * A5 + q3 * A6 + A2 / 2 * q1² * sin(q2) = M2;\n"
    "A7 * q1'' + A8 * q2'' + A9 * q3 = M3;\n"
    "A10 * q4 = M4;\n"
    "M1, M2, M3, M4 – обобщённые моменты, соответствующие обобщённым координатам.\n"
    "Ai – коэффициенты, зависящие от конструктивных параметров робота и имеют вид:\n"
    "A1 = J01 + J1g * n1² + n2 * a1² + n2 * r1² + Jc2 + Jc3 + n3 * (a1² + a2²);\n"
    "A2 = 2 * n2 * a1 * r2 + n3 * a1 * a2;\n"
    "A3 = n2 * r2² + Jc2 + Jc3 * n3;\n"
    "A4 = Jc3;\n"
    "A5 = n2 * r2 + Jc2 + Jg2 * n2² + Jc3 * n3 * a2²;\n"
    "A6 = Jc3;\n"
    "A7 = Jc3;\n"
    "A8 = Jc3;\n"
    "A9 = Jc3 + Jg3 * n3²;\n"
    "A10 = n3 + n4 * n4².\n"
    "Обобщённые координаты qi не должны выходить за пределы диапазона допустимых значений.\n", self)
        scara_help_text.setFont(QFont('Times', 12))
        scara_help_layout.addWidget(scara_help_text)

        scara_help_dialog.setLayout(scara_help_layout)
        scara_help_dialog.exec_()
    def show_cylindrical_help(self):
        # Окно с подсказкой по Декарт
        cylindrical_help_dialog = QDialog(self)
        cylindrical_help_dialog.setWindowTitle("Подсказка Цилиндр")
        cylindrical_help_layout = QVBoxLayout()

        cylindrical_help_text = QLabel("Робот с кинематической схемой ЦИЛИНДР имеет 4 степени подвижности.\n"
    "Упрощенное изображение робота представлено на рисунке.\n"
    "Конструктивно робот состоит из неподвижной платформы и трехзвенной механической руки со схватом.\n"
    "1 звено может вращаться вокруг шарнира О1 в горизонтальной плоскости; 2 звено совершает поступательное перемещение;\n"
    "3 звено может перемещаться в вертикальной плоскости и одновременно вращаться вокруг шарнира О2\n"
    "в горизонтальной плоскости. Все звенья робота предполагаются абсолютно твердыми телами.\n"
    "Управление движением робота осуществляется с помощью четырех независимых электромеханических электроприводов,\n"
    "каждый из которых состоит из электрического двигателя постоянного тока с независимым возбуждением и редуктора.\n"
    "Уравнения динамики робота могут быть получены из уравнений Лагранжа 2 рода и имеют следующий вид:\n"
    "A1 * q1'' + q2'' * (A2 * q2 + A3 * q2') + q1' * (A2 * q2' + 2 * A3 * q2' * q2) + A6 * q3'' = M1\n"
    "A4 * q2'' - q1'² * (A2 / 2 + A3 * q3) = M2\n"
    "A6 * q1' + A5 * q3'' = M3\n"
    "A7 * q4'' = M4\n"
    "Где коэффициенты Ai зависят от конструктивных параметров робота и определяются из следующих выражений:\n"
    "A1 = (Jo1 + J1д * n1²) + Jc2 + Jc3 + n2 * r2² + n3 * a2²\n"
    "A2 = n2 * r2 + n3 * a2\n"
    "A3 = n2 + n3\n"
    "A4 = n2 + n2д * n2² + n3\n"
    "A5 = Jc3 + J3д * n3²\n"
    "A6 = Jc3\n"
    "A7 = n3 + m4д * n4²\n"
    "Обобщенные координаты qi не должны выходить за пределы диапазона допустимых значений.\n",
                                     self)
        cylindrical_help_text.setFont(QFont('Times', 12))
        cylindrical_help_layout.addWidget(cylindrical_help_text)

        cylindrical_help_dialog.setLayout(cylindrical_help_layout)
        cylindrical_help_dialog.exec_()

    def show_construct_params_help(self):
        construct_params_help_dialog = QDialog(self)
        construct_params_help_dialog.setWindowTitle("Подсказка Цилиндр")
        construct_params_help_layout = QVBoxLayout()

        construct_params_help_text = QLabel( "К конструктивным параметрам двигателей относятся:\n"
        "Jg - момент инерции ротора двигателя;\n"
        "Umax - максимально возможное значение напряжения, подаваемого на двигатель;\n"
        "n - коэффициент редукции редуктора;\n"
        "Ku, Kq - вспомогательные коэффициенты.\n\n"
        "Помощь\n"
        "Ku и Kq определяются из следующего соотношения:\n"
        "M = KuU - Kqq,\n"
        "где:\n"
        "M - момент, развиваемый двигателем;\n"
        "U - напряжение, подаваемое на двигатель;\n"
        "q - угловая скорость вращения двигателя.\n\n"
        "Для базового варианта исходных данных для управления 1 и 2 степенями подвижности робота\n"
        "используется двигатель ДР-72-Н2-02, а для управления 3 и 4 степенями подвижности робота\n"
        "используется двигатель ДПМ-30-Н2-04.\n",
        self)
        construct_params_help_text.setFont(QFont('Times', 12))
        construct_params_help_layout.addWidget(construct_params_help_text)

        construct_params_help_dialog.setLayout(construct_params_help_layout)
        construct_params_help_dialog.exec_()

    def show_cartesian_params_dialog(self):
        # Создание диалогового окна для ввода конструктивных параметров
        dialog = QDialog(self)
        dialog.setWindowTitle("Конструктивные параметры Декартового робота")

        layout = QVBoxLayout()

        # Создание сетки для полей ввода
        grid_layout = QGridLayout()

        # Создание полей ввода для масс
        self.massd_1_input = QLineEdit(self)
        self.massd_1_input.setFont(QFont('Times', 14))
        self.massd_1_input.setText(str(self.massd_1) if self.massd_1 is not None else "")

        self.massd_2_input = QLineEdit(self)
        self.massd_2_input.setFont(QFont('Times', 14))
        self.massd_2_input.setText(str(self.massd_2) if self.massd_2 is not None else "")

        self.massd_3_input = QLineEdit(self)
        self.massd_3_input.setFont(QFont('Times', 14))
        self.massd_3_input.setText(str(self.massd_3) if self.massd_3 is not None else "")

        self.momentd_1_input = QLineEdit(self)
        self.momentd_1_input.setFont(QFont('Times', 14))
        self.momentd_1_input.setText(str(self.momentd_1) if self.momentd_1 is not None else "")

        # Добавление полей в сетку с метками
        temp = QLabel("Масса 1-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 0, 0)
        grid_layout.addWidget(self.massd_1_input, 0, 1)

        temp = QLabel("Масса 3-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 1, 0)
        grid_layout.addWidget(self.massd_3_input, 1, 1)

        temp = QLabel("Масса 2-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 0, 2)
        grid_layout.addWidget(self.massd_2_input, 0, 3)

        temp = QLabel("Момент инерции")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 1, 2)
        grid_layout.addWidget(self.momentd_1_input, 1, 3)

        layout.addLayout(grid_layout)

        # Создание кнопок ОК и Отменить
        button_layout = QHBoxLayout()
        ok_button = QPushButton("ОК")
        ok_button.setStyleSheet("background-color : #18f22e")
        ok_button.setFont(QFont('Times', 14))
        cancel_button = QPushButton("Отменить")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        help_button = QPushButton("Помощь")
        help_button.setStyleSheet("background-color : #87CEEB")
        help_button.setFont(QFont('Times', 14))

        # Связываем кнопку ОК с функцией проверки данных
        ok_button.clicked.connect(lambda: self.apply_cartesian_params(dialog))
        help_button.clicked.connect(self.show_cartesian_params_help_dialog)
        cancel_button.clicked.connect(dialog.reject)

        button_layout.addWidget(ok_button)
        button_layout.addWidget(help_button)
        button_layout.addWidget(cancel_button)

        layout.addLayout(button_layout)
        dialog.setLayout(layout)

        # Открываем диалоговое окно и проверяем, был ли он принят
        if dialog.exec_() == QDialog.Accepted:
            # Если диалог был принят, сохраняем данные
            massesd = self.apply_cartesian_params(dialog)
            if massesd:  # Проверяем, что данные корректны
                self.massd_1, self.massd_2, self.massd_3, self.momentd_1 = massesd
                print("Данные успешно сохранены.")
        else:
            # Если окно было закрыто без сохранения
            print("Изменения не были сохранены.")

    def show_cartesian_params_help_dialog(self):

        help_dialog = QDialog(self)
        help_dialog.setWindowTitle("Подсказка")
        help_layout = QVBoxLayout()

        # Основной текст подсказки с выделением
        help_text = QLabel(self)
        help_text.setFont(QFont('Times', 12))

        help_text.setText("""Конструктивные параметры робота ДЕКАРТ:<br>
                            m1 - масса 2 звена;<br>
                            m2 - масса 2 звена;<br>
                            m3 - масса 3 звена;<br>
                            Jc3 - момент инерции 3 звена относительно центра инерции 3 звена.<br>""")

        help_layout.addWidget(help_text)

        # Кнопки для открытия дополнительных подсказок
        button_layout = QHBoxLayout()

        cartesian_button = QPushButton("Декарт")
        cartesian_button.setFont(QFont('Times', 14))
        cartesian_button.setStyleSheet("background-color: lightblue")
        cartesian_button.clicked.connect(self.show_cartesian_help)

        construct_params_button = QPushButton("Конструктивные параметры")
        construct_params_button.setFont(QFont('Times', 14))
        construct_params_button.setStyleSheet("background: rgb(169, 87, 201)")
        construct_params_button.clicked.connect(self.show_construct_params_help)

        button_layout.addWidget(cartesian_button)
        button_layout.addWidget(construct_params_button)

        cancel_button = QPushButton("Закрыть")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        cancel_button.clicked.connect(help_dialog.reject)
        button_layout.addWidget(cancel_button)

        help_layout.addLayout(button_layout)

        help_dialog.setLayout(help_layout)
        help_dialog.exec_()

    def show_cartesian_limits_dialog(self):
        # Создание диалогового окна для ограничений по координатам
        dialog = QDialog(self)
        dialog.setWindowTitle("Ограничения по координатам")

        layout = QVBoxLayout()

        # Сетка для полей ввода
        grid_layout = QGridLayout()

        # Создание полей ввода для ограничений по координатам
        self.x_min_input = QLineEdit(self)
        self.x_min_input.setFont(QFont('Times', 14))
        self.x_max_input = QLineEdit(self)
        self.x_max_input.setFont(QFont('Times', 14))
        self.y_min_input = QLineEdit(self)
        self.y_min_input.setFont(QFont('Times', 14))
        self.y_max_input = QLineEdit(self)
        self.y_max_input.setFont(QFont('Times', 14))
        self.z_min_input = QLineEdit(self)
        self.z_min_input.setFont(QFont('Times', 14))
        self.z_max_input = QLineEdit(self)
        self.z_max_input.setFont(QFont('Times', 14))
        self.q_min_input = QLineEdit(self)
        self.q_min_input.setFont(QFont('Times', 14))
        self.q_max_input = QLineEdit(self)
        self.q_max_input.setFont(QFont('Times', 14))

        # Устанавливаем значения по умолчанию, если они есть
        self.x_min_input.setText(str(self.x_min) if self.x_min is not None else "")
        self.x_max_input.setText(str(self.x_max) if self.x_max is not None else "")
        self.y_min_input.setText(str(self.y_min) if self.y_min is not None else "")
        self.y_max_input.setText(str(self.y_max) if self.y_max is not None else "")
        self.z_min_input.setText(str(self.z_min) if self.z_min is not None else "")
        self.z_max_input.setText(str(self.z_max) if self.z_max is not None else "")
        self.q_min_input.setText(str(self.q_min) if self.q_min is not None else "")
        self.q_max_input.setText(str(self.q_max) if self.q_max is not None else "")

        # Добавление полей в сетку с метками
        temp = QLabel("X Мин:")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 0, 0)
        grid_layout.addWidget(self.x_min_input, 0, 1)
        temp = QLabel("X Макс:")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 0, 2)
        grid_layout.addWidget(self.x_max_input, 0, 3)

        temp = QLabel("Y Мин:")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 1, 0)
        grid_layout.addWidget(self.y_min_input, 1, 1)
        temp = QLabel("Y Макс:")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 1, 2)
        grid_layout.addWidget(self.y_max_input, 1, 3)

        temp = QLabel("Z Мин:")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 2, 0)
        grid_layout.addWidget(self.z_min_input, 2, 1)
        temp = QLabel("Z Макс:")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 2, 2)
        grid_layout.addWidget(self.z_max_input, 2, 3)

        temp = QLabel("Q Мин:")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 3, 0)
        grid_layout.addWidget(self.q_min_input, 3, 1)
        temp = QLabel("Q Макс:")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 3, 2)
        grid_layout.addWidget(self.q_max_input, 3, 3)

        layout.addLayout(grid_layout)

        # Создание кнопок ОК, Отменить и Помощь
        button_layout = QHBoxLayout()
        ok_button = QPushButton("ОК")
        ok_button.setStyleSheet("background-color : #18f22e")
        ok_button.setFont(QFont('Times', 14))
        cancel_button = QPushButton("Отменить")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        help_button = QPushButton("Помощь")
        help_button.setStyleSheet("background-color : #87CEEB")
        help_button.setFont(QFont('Times', 14))

        ok_button.clicked.connect(lambda: self.apply_cartesian_limits(dialog))
        cancel_button.clicked.connect(dialog.reject)
        help_button.clicked.connect(self.show_cartesian_limits_help_dialog)

        button_layout.addWidget(ok_button)
        button_layout.addWidget(help_button)
        button_layout.addWidget(cancel_button)

        layout.addLayout(button_layout)
        dialog.setLayout(layout)
        dialog.exec_()

    def show_cartesian_limits_help_dialog(self):

        help_dialog = QDialog(self)
        help_dialog.setWindowTitle("Подсказка")
        help_layout = QVBoxLayout()

        # Основной текст подсказки с выделением
        help_text = QLabel(self)
        help_text.setFont(QFont('Times', 12))

        help_text.setText("""На обобщенные координаты робота Декарт наложены следующие ограничения:<br>
        xmin - минимально возможное значение линейного перемещения 1 звена;<br>
        xmax - максимально возможное значение линейного перемещения 1 звена;<br>
        ymin - минимально возможное значение линейного перемещения 2 звена;<br>
        ymax - максимально возможное значение линейного перемещения 2 звена;<br>
        q3min - минимально возможное значение угла поворота 3 звена;<br>
        q3max - максимально возможное значение угла поворота 3 звена;<br>
        zmin - минимально возможное значение линейного перемещения 3 звена;<br>
        zmax - максимально возможное значение линейного перемещения 3 звена.<br>
        """)

        help_layout.addWidget(help_text)

        # Кнопки для открытия дополнительных подсказок
        button_layout = QHBoxLayout()

        cartesian_button = QPushButton("Декарт")
        cartesian_button.setFont(QFont('Times', 14))
        cartesian_button.setStyleSheet("background-color: lightblue")
        cartesian_button.clicked.connect(self.show_cartesian_help)

        button_layout.addWidget(cartesian_button)

        cancel_button = QPushButton("Закрыть")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        cancel_button.clicked.connect(help_dialog.reject)
        button_layout.addWidget(cancel_button)

        help_layout.addLayout(button_layout)

        help_dialog.setLayout(help_layout)
        help_dialog.exec_()

    def apply_cartesian_limits(self, dialog):
        # Проверка и сохранение введенных значений
        try:
            # Получаем значения из полей ввода
            x_min = float(self.x_min_input.text())
            x_max = float(self.x_max_input.text())
            y_min = float(self.y_min_input.text())
            y_max = float(self.y_max_input.text())
            z_min = float(self.z_min_input.text())
            z_max = float(self.z_max_input.text())
            q_min = float(self.q_min_input.text())
            q_max = float(self.q_max_input.text())

            # Проверка диапазона значений для координат
            if not (0 <= x_min <= 1 and 0 <= x_max <= 1):
                raise ValueError("Значения X Мин и Макс должны быть в диапазоне от 0 до 1.")
            if not (0 <= y_min <= 1 and 0 <= y_max <= 1):
                raise ValueError("Значения Y Мин и Макс должны быть в диапазоне от 0 до 1.")
            if not (0 <= z_min <= 1 and 0 <= z_max <= 1):
                raise ValueError("Значения Z Мин и Макс должны быть в диапазоне от 0 до 1.")

            # Проверка диапазона значений для углов
            if not (-3.142 <= q_min <= 3.142 and -3.142 <= q_max <= 3.142):
                raise ValueError("Значения Q Мин и Макс должны быть в диапазоне от -3.142 до 3.142.")

            # Проверка порядка значений
            if x_min > x_max:
                raise ValueError("Минимум X не может быть больше максимума X.")
            if y_min > y_max:
                raise ValueError("Минимум Y не может быть больше максимума Y.")
            if z_min > z_max:
                raise ValueError("Минимум Z не может быть больше максимума Z.")
            if q_min > q_max:
                raise ValueError("Минимум Q не может быть больше максимума Q.")

            # Если все значения корректны, сохраняем их
            self.x_min, self.x_max = x_min, x_max
            self.y_min, self.y_max = y_min, y_max
            self.z_min, self.z_max = z_min, z_max
            self.q_min, self.q_max = q_min, q_max

            self.trajectory_calculator.set_cartesian_limits(x_min, x_max, y_min, y_max, z_min, z_max, q_min, q_max)

            dialog.accept()  # Закрываем диалог

        except ValueError as e:
            QMessageBox.warning(self, "Ошибка", str(e))

    def show_scara_params_dialog(self):
        # Создание диалогового окна для ввода конструктивных параметров SCARA
        dialog = QDialog(self)
        dialog.setWindowTitle("Конструктивные параметры Скара")

        layout = QVBoxLayout()
        grid_layout = QGridLayout()

        # Поля ввода
        self.moment_1_input = QLineEdit(self)
        self.moment_1_input.setFont(QFont('Times', 14))
        self.moment_1_input.setText(str(self.moment_1) if self.moment_1 is not None else "")

        self.moment_2_input = QLineEdit(self)
        self.moment_2_input.setFont(QFont('Times', 14))
        self.moment_2_input.setText(str(self.moment_2) if self.moment_2 is not None else "")

        self.moment_3_input = QLineEdit(self)
        self.moment_3_input.setFont(QFont('Times', 14))
        self.moment_3_input.setText(str(self.moment_3) if self.moment_3 is not None else "")

        self.length_1_input = QLineEdit(self)
        self.length_1_input.setFont(QFont('Times', 14))
        self.length_1_input.setText(str(self.length_1) if self.length_1 is not None else "")

        self.length_2_input = QLineEdit(self)
        self.length_2_input.setFont(QFont('Times', 14))
        self.length_2_input.setText(str(self.length_2) if self.length_2 is not None else "")

        self.distance_input = QLineEdit(self)
        self.distance_input.setFont(QFont('Times', 14))
        self.distance_input.setText(str(self.distance) if self.distance is not None else "")

        self.masss_2_input = QLineEdit(self)
        self.masss_2_input.setFont(QFont('Times', 14))
        self.masss_2_input.setText(str(self.masss_2) if self.masss_2 is not None else "")

        self.masss_3_input = QLineEdit(self)
        self.masss_3_input.setFont(QFont('Times', 14))
        self.masss_3_input.setText(str(self.masss_3) if self.masss_3 is not None else "")

        # Добавление полей в сетку

        temp = QLabel("Момент 1-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 0, 0)
        grid_layout.addWidget(self.moment_1_input, 0, 1)

        temp = QLabel("Момент 2-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 1, 0)
        grid_layout.addWidget(self.moment_2_input, 1, 1)

        temp = QLabel("Момент 3-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 2, 0)
        grid_layout.addWidget(self.moment_3_input, 2, 1)

        temp = QLabel("Длина 1-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 3, 0)
        grid_layout.addWidget(self.length_1_input, 3, 1)

        temp = QLabel("Длина 2-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 0, 2)
        grid_layout.addWidget(self.length_2_input, 0, 3)

        temp = QLabel("Расстояние")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 1, 2)
        grid_layout.addWidget(self.distance_input, 1, 3)

        temp = QLabel("Масса 2-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 2, 2)
        grid_layout.addWidget(self.masss_2_input, 2, 3)

        temp = QLabel("Масса 3-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 3, 2)
        grid_layout.addWidget(self.masss_3_input, 3, 3)

        layout.addLayout(grid_layout)

        # Кнопки OK и Отменить
        button_layout = QHBoxLayout()
        ok_button = QPushButton("ОК")
        ok_button.setStyleSheet("background-color : #18f22e")
        ok_button.setFont(QFont('Times', 14))
        cancel_button = QPushButton("Отменить")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        help_button = QPushButton("Помощь")
        help_button.setStyleSheet("background-color : #87CEEB")
        help_button.setFont(QFont('Times', 14))

        ok_button.clicked.connect(lambda: self.apply_scara_params(dialog))
        help_button.clicked.connect(self.show_scara_params_help_dialog)
        cancel_button.clicked.connect(dialog.reject)

        button_layout.addWidget(ok_button)
        button_layout.addWidget(help_button)
        button_layout.addWidget(cancel_button)

        layout.addLayout(button_layout)
        dialog.setLayout(layout)

        # Открываем диалоговое окно
        dialog.exec_()
    def show_scara_params_help_dialog(self):

        help_dialog = QDialog(self)
        help_dialog.setWindowTitle("Подсказка")
        help_layout = QVBoxLayout()

        # Основной текст подсказки с выделением
        help_text = QLabel(self)
        help_text.setFont(QFont('Times', 12))

        help_text.setText(""" J01 - момент инерции 1 звена относительно шарнира O1 при вращении в горизонтальной плоскости;<br>
        Jc2 - момент инерции 2 звена относительно центра инерции 2 звена;<br>
        Jc3 - момент инерции 3 звена относительно центра инерции 3 звена;<br>
        a1 - длина 1 звена;<br>
        a2 - длина 2 звена;<br>
        r2 - расстояние от начала 2 звена до его центра инерции;<br>
        m2 - масса 2 звена;<br>
        m3 - масса 3 звена.<br>""")

        help_layout.addWidget(help_text)

        # Кнопки для открытия дополнительных подсказок
        button_layout = QHBoxLayout()

        scara_button = QPushButton("Скара")
        scara_button.setFont(QFont('Times', 14))
        scara_button.setStyleSheet("background-color: lightgreen")
        scara_button.clicked.connect(self.show_scara_help)

        construct_params_button = QPushButton("Конструктивные параметры")
        construct_params_button.setFont(QFont('Times', 14))
        construct_params_button.setStyleSheet("background: rgb(169, 87, 201)")
        construct_params_button.clicked.connect(self.show_construct_params_help)

        button_layout.addWidget(scara_button)
        button_layout.addWidget(construct_params_button)

        cancel_button = QPushButton("Закрыть")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        cancel_button.clicked.connect(help_dialog.reject)
        button_layout.addWidget(cancel_button)

        help_layout.addLayout(button_layout)

        help_dialog.setLayout(help_layout)
        help_dialog.exec_()

    def apply_scara_params(self, dialog):
        try:
            # Считываем введенные значения
            moment_1 = float(self.moment_1_input.text())
            moment_2 = float(self.moment_2_input.text())
            moment_3 = float(self.moment_3_input.text())
            length_1 = float(self.length_1_input.text())
            length_2 = float(self.length_2_input.text())
            distance = float(self.distance_input.text())
            masss_2 = float(self.masss_2_input.text())
            masss_3 = float(self.masss_3_input.text())

            # Проверяем значения на ограничения
            if not (0 <= moment_1 <= 1):
                raise ValueError("Момент 1-го звена не в диапазоне от 0 до 1")
            if not (0 <= moment_2 <= 1):
                raise ValueError("Момент 2-го звена не в диапазоне от 0 до 1")
            if not (0 <= moment_3 <= 1):
                raise ValueError("Момент 3-го звена не в диапазоне от 0 до 1")
            if not (0 <= length_1 <= 1):
                raise ValueError("Длина 1-го звена не в диапазоне от 0 до 1")
            if not (0 <= length_2 <= 1):
                raise ValueError("Длина 2-го звена не в диапазоне от 0 до 1")
            if not (0 <= distance <= 1):
                raise ValueError("Расстояние не в диапазоне от 0 до 1")
            if not (0 <= masss_2 <= 20):
                raise ValueError("Масса 2-го звена не в диапазоне от 0 до 20")
            if not (0 <= masss_3 <= 20):
                raise ValueError("Масса 3-го звена не в диапазоне от 0 до 20")

            # Если все проверки пройдены, сохраняем значения
            self.moment_1 = moment_1
            self.moment_2 = moment_2
            self.moment_3 = moment_3
            self.length_1 = length_1
            self.length_2 = length_2
            self.distance = distance
            self.masss_2 = masss_2
            self.masss_3 = masss_3
            self.trajectory_calculator.set_scara_params(moment_1, moment_2, moment_3, length_1, length_2, distance, masss_2, masss_3)

            dialog.accept()


        except ValueError as e:
            # Показываем окно с ошибкой, если значение выходит за допустимый диапазон
            QMessageBox.warning(self, "Ошибка", str(e))

    def show_scara_limits_dialog(self):
        # Создание диалогового окна для ввода ограничений по координатам
        dialog = QDialog(self)
        dialog.setWindowTitle("Ограничения по координатам Скара")

        layout = QVBoxLayout()

        # Создание сетки для полей ввода
        grid_layout = QGridLayout()

        # Создание полей ввода для ограничений
        self.q1s_min_input = QLineEdit(self)
        self.q1s_max_input = QLineEdit(self)
        self.q2s_min_input = QLineEdit(self)
        self.q2s_max_input = QLineEdit(self)
        self.q3s_min_input = QLineEdit(self)
        self.q3s_max_input = QLineEdit(self)
        self.zs_min_input = QLineEdit(self)
        self.zs_max_input = QLineEdit(self)

        self.q1s_min_input.setFont(QFont('Times', 14))
        self.q1s_max_input.setFont(QFont('Times', 14))
        self.q2s_min_input.setFont(QFont('Times', 14))
        self.q2s_max_input.setFont(QFont('Times', 14))
        self.q3s_min_input.setFont(QFont('Times', 14))
        self.q3s_max_input.setFont(QFont('Times', 14))
        self.zs_min_input.setFont(QFont('Times', 14))
        self.zs_max_input.setFont(QFont('Times', 14))

        # Установка значений в поля ввода
        self.q1s_min_input.setText(str(self.q1s_min))
        self.q1s_max_input.setText(str(self.q1s_max))
        self.q2s_min_input.setText(str(self.q2s_min))
        self.q2s_max_input.setText(str(self.q2s_max))
        self.q3s_min_input.setText(str(self.q3s_min))
        self.q3s_max_input.setText(str(self.q3s_max))
        self.zs_min_input.setText(str(self.zs_min))
        self.zs_max_input.setText(str(self.zs_max))

        # Добавление полей в сетку с метками
        temp = QLabel("Q1min")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 0, 0)
        grid_layout.addWidget(self.q1s_min_input, 0, 1)

        temp = QLabel("Q2min")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 1, 0)
        grid_layout.addWidget(self.q2s_min_input, 1, 1)

        temp = QLabel("Q3min")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 2, 0)
        grid_layout.addWidget(self.q3s_min_input, 2, 1)

        temp = QLabel("Zmin")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 3, 0)
        grid_layout.addWidget(self.zs_min_input, 3, 1)

        temp = QLabel("Q1max")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 0, 2)
        grid_layout.addWidget(self.q1s_max_input, 0, 3)

        temp = QLabel("Q2max")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 1, 2)
        grid_layout.addWidget(self.q2s_max_input, 1, 3)

        temp = QLabel("Q3max")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 2, 2)
        grid_layout.addWidget(self.q3s_max_input, 2, 3)

        temp = QLabel("Zmax")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 3, 2)
        grid_layout.addWidget(self.zs_max_input, 3, 3)

        layout.addLayout(grid_layout)

        # Создание кнопок ОК и Отменить
        button_layout = QHBoxLayout()
        ok_button = QPushButton("ОК")
        ok_button.setStyleSheet("background-color : #18f22e")
        ok_button.setFont(QFont('Times', 14))
        cancel_button = QPushButton("Отменить")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        help_button = QPushButton("Помощь")
        help_button.setStyleSheet("background-color : #87CEEB")
        help_button.setFont(QFont('Times', 14))

        ok_button.clicked.connect(lambda: self.apply_scara_limits(dialog))
        cancel_button.clicked.connect(dialog.reject)
        help_button.clicked.connect(self.show_scara_limits_help_dialog)

        button_layout.addWidget(ok_button)
        button_layout.addWidget(help_button)
        button_layout.addWidget(cancel_button)

        layout.addLayout(button_layout)
        dialog.setLayout(layout)

        # Открываем диалоговое окно и заполняем поля данными
        if dialog.exec_() == QDialog.Accepted:
            self.q1s_min = self.q1s_min_input.text()
            self.q1s_max = self.q1s_max_input.text()
            self.q2s_min = self.q2s_min_input.text()
            self.q2s_max = self.q2s_max_input.text()
            self.q3s_min = self.q3s_min_input.text()
            self.q3s_max = self.q3s_max_input.text()
            self.zs_min = self.zs_min_input.text()
            self.zs_max = self.zs_max_input.text()

    def show_scara_limits_help_dialog(self):

        help_dialog = QDialog(self)
        help_dialog.setWindowTitle("Подсказка")
        help_layout = QVBoxLayout()

        # Основной текст подсказки с выделением
        help_text = QLabel(self)
        help_text.setFont(QFont('Times', 12))

        help_text.setText("""На обобщенные координаты робота Скара наложены следующие ограничения:<br>
                            q1min - минимально возможное значение угла поворота 1 звена;<br>
                            q1max - максимально возможное значение угла поворота 1 звена;<br>
                            q2min - минимально возможное значение угла поворота 2 звена;<br>
                            q2max - максимально возможное значение угла поворота 2 звена;<br>
                            q3min - минимально возможное значение угла поворота 3 звена;<br>
                            q3max - максимально возможное значение угла поворота 3 звена;<br>
                            zmin - минимально возможное значение линейного перемещения 3 звена;<br>
                            zmax - максимально возможное значение линейного перемещения 3 звена.<br>
                            """)
        help_layout.addWidget(help_text)

        # Кнопки для открытия дополнительных подсказок
        button_layout = QHBoxLayout()

        scara_button = QPushButton("Скара")
        scara_button.setFont(QFont('Times', 14))
        scara_button.setStyleSheet("background-color: lightgreen")
        scara_button.clicked.connect(self.show_scara_help)

        button_layout.addWidget(scara_button)

        cancel_button = QPushButton("Закрыть")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        cancel_button.clicked.connect(help_dialog.reject)
        button_layout.addWidget(cancel_button)

        help_layout.addLayout(button_layout)

        help_dialog.setLayout(help_layout)
        help_dialog.exec_()

    def apply_scara_limits(self, dialog):
        try:
            # Считываем введенные значения
            q1s_min = float(self.q1s_min_input.text())
            q1s_max = float(self.q1s_max_input.text())
            q2s_min = float(self.q2s_min_input.text())
            q2s_max = float(self.q2s_max_input.text())
            q3s_min = float(self.q3s_min_input.text())
            q3s_max = float(self.q3s_max_input.text())
            zs_min = float(self.zs_min_input.text())
            zs_max = float(self.zs_max_input.text())

            # Проверяем значения на ограничения
            if not (-3.142 <= q1s_min <= 3.142):
                raise ValueError("Значение Q1min не в диапазоне от -3.142 до 3.142")
            if not (-3.142 <= q1s_max <= 3.142):
                raise ValueError("Значение Q1max не в диапазоне от -3.142 до 3.142")
            if not (-3.142 <= q2s_min <= 3.142):
                raise ValueError("Значение Q2min не в диапазоне от -3.142 до 3.142")
            if not (-3.142 <= q2s_max <= 3.142):
                raise ValueError("Значение Q2max не в диапазоне от -3.142 до 3.142")
            if not (-3.142 <= q3s_min <= 3.142):
                raise ValueError("Значение Q3min не в диапазоне от -3.142 до 3.142")
            if not (-3.142 <= q3s_max <= 3.142):
                raise ValueError("Значение Q3max не в диапазоне от -3.142 до 3.142")
            if not (0 <= zs_min <= 1):
                raise ValueError("Значение Zmin не в диапазоне от 0 до 1")
            if not (0 <= zs_max <= 1):
                raise ValueError("Значение Zmax не в диапазоне от 0 до 1")

            # Проверяем, чтобы значения max были больше или равны соответствующим min
            if q1s_max < q1s_min:
                raise ValueError("Q1max должно быть больше или равно Q1min")
            if q2s_max < q2s_min:
                raise ValueError("Q2max должно быть больше или равно Q2min")
            if q3s_max < q3s_min:
                raise ValueError("Q3max должно быть больше или равно Q3min")
            if zs_max < zs_min:
                raise ValueError("Zmax должно быть больше или равно Zmin")

            # Если все проверки пройдены, сохраняем значения
            self.q1s_min = q1s_min
            self.q1s_max = q1s_max
            self.q2s_min = q2s_min
            self.q2s_max = q2s_max
            self.q3s_min = q3s_min
            self.q3s_max = q3s_max
            self.zs_min = zs_min
            self.zs_max = zs_max
            self.workspace_calculator.set_scara_limits(q1s_min, q1s_max, q2s_min, q2s_max, q3s_min, q3s_max, zs_min,
                                                       zs_max)
            self.trajectory_calculator.set_scara_limits(q1s_min, q1s_max, q2s_min, q2s_max, q3s_min, q3s_max, zs_min,
                                                       zs_max)

            # Закрываем диалоговое окно
            dialog.accept()

        except ValueError as e:
            # Показываем окно с ошибкой, если значение выходит за допустимый диапазон
            QMessageBox.warning(self, "Ошибка", str(e))

    def show_cylindrical_params_dialog(self):
        # Создание диалогового окна для ввода конструктивных параметров SCARA
        dialog = QDialog(self)
        dialog.setWindowTitle("Конструктивные параметры Цилиндр")

        layout = QVBoxLayout()
        grid_layout = QGridLayout()

        # Поля ввода
        self.momentc_1_input = QLineEdit(self)
        self.momentc_1_input.setFont(QFont('Times', 14))
        self.momentc_1_input.setText(str(self.momentc_1) if self.momentc_1 is not None else "")

        self.momentc_2_input = QLineEdit(self)
        self.momentc_2_input.setFont(QFont('Times', 14))
        self.momentc_2_input.setText(str(self.momentc_2) if self.momentc_2 is not None else "")

        self.momentc_3_input = QLineEdit(self)
        self.momentc_3_input.setFont(QFont('Times', 14))
        self.momentc_3_input.setText(str(self.momentc_3) if self.momentc_3 is not None else "")

        self.lengthc_1_input = QLineEdit(self)
        self.lengthc_1_input.setFont(QFont('Times', 14))
        self.lengthc_1_input.setText(str(self.lengthc_1) if self.lengthc_1 is not None else "")

        self.lengthc_2_input = QLineEdit(self)
        self.lengthc_2_input.setFont(QFont('Times', 14))
        self.lengthc_2_input.setText(str(self.lengthc_2) if self.lengthc_2 is not None else "")

        self.distancec_input = QLineEdit(self)
        self.distancec_input.setFont(QFont('Times', 14))
        self.distancec_input.setText(str(self.distancec) if self.distancec is not None else "")

        self.massc_2_input = QLineEdit(self)
        self.massc_2_input.setFont(QFont('Times', 14))
        self.massc_2_input.setText(str(self.massc_2) if self.massc_2 is not None else "")

        self.massc_3_input = QLineEdit(self)
        self.massc_3_input.setFont(QFont('Times', 14))
        self.massc_3_input.setText(str(self.massc_3) if self.massc_3 is not None else "")

        # Добавление полей в сетку
        temp = QLabel("Момент 1-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 0, 0)
        grid_layout.addWidget(self.momentc_1_input, 0, 1)

        temp = QLabel("Момент 2-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 1, 0)
        grid_layout.addWidget(self.momentc_2_input, 1, 1)

        temp = QLabel("Момент 3-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 2, 0)
        grid_layout.addWidget(self.momentc_3_input, 2, 1)

        temp = QLabel("Длина 1-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 3, 0)
        grid_layout.addWidget(self.lengthc_1_input, 3, 1)

        temp = QLabel("Длина 2-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 0, 2)
        grid_layout.addWidget(self.lengthc_2_input, 0, 3)

        temp = QLabel("Расстояние")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 1, 2)
        grid_layout.addWidget(self.distancec_input, 1, 3)

        temp = QLabel("Масса 2-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 2, 2)
        grid_layout.addWidget(self.massc_2_input, 2, 3)

        temp = QLabel("Масса 3-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 3, 2)
        grid_layout.addWidget(self.massc_3_input, 3, 3)

        layout.addLayout(grid_layout)

        # Кнопки OK и Отменить
        button_layout = QHBoxLayout()
        ok_button = QPushButton("ОК")
        ok_button.setStyleSheet("background-color : #18f22e")
        ok_button.setFont(QFont('Times', 14))
        cancel_button = QPushButton("Отменить")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        help_button = QPushButton("Помощь")
        help_button.setStyleSheet("background-color : #87CEEB")
        help_button.setFont(QFont('Times', 14))

        ok_button.clicked.connect(lambda: self.apply_cylindrical_params(dialog))
        help_button.clicked.connect(self.show_cylindrical_params_help_dialog)
        cancel_button.clicked.connect(dialog.reject)

        button_layout.addWidget(ok_button)
        button_layout.addWidget(help_button)
        button_layout.addWidget(cancel_button)

        layout.addLayout(button_layout)
        dialog.setLayout(layout)

        # Открываем диалоговое окно
        dialog.exec_()

    def show_cylindrical_params_help_dialog(self):

        help_dialog = QDialog(self)
        help_dialog.setWindowTitle("Подсказка")
        help_layout = QVBoxLayout()

        # Основной текст подсказки с выделением
        help_text = QLabel(self)
        help_text.setFont(QFont('Times', 12))

        help_text.setText("""Конструктивные параметры робота ЦИЛИНДР:<br>
        Jθ1 – момент инерции 1 звена относительно шарнира O1 при вращении в горизонтальной плоскости;<br>
        Jc2 – момент инерции 2 звена относительно центра инерции 2 звена;<br>
        Jc3 – момент инерции 3 звена относительно центра инерции 3 звена;<br><br>
        a1 - длина 1 звена;<br>
        a2 - длина 2 звена;<br>
        r2 - расстояние от начала 2 звена до его центра инерции;<br>
        m2 - масса 2 звена;<br>
        m3 - масса 3 звена.<br>
        """)

        help_layout.addWidget(help_text)

        # Кнопки для открытия дополнительных подсказок
        button_layout = QHBoxLayout()

        cylindrical_button = QPushButton("Цилиндр")
        cylindrical_button.setFont(QFont('Times', 14))
        cylindrical_button.setStyleSheet("background-color: yellow")
        cylindrical_button.clicked.connect(self.show_cylindrical_help)

        construct_params_button = QPushButton("Конструктивные параметры")
        construct_params_button.setFont(QFont('Times', 14))
        construct_params_button.setStyleSheet("background: rgb(169, 87, 201)")
        construct_params_button.clicked.connect(self.show_construct_params_help)

        button_layout.addWidget(cylindrical_button)
        button_layout.addWidget(construct_params_button)

        cancel_button = QPushButton("Закрыть")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        cancel_button.clicked.connect(help_dialog.reject)
        button_layout.addWidget(cancel_button)

        help_layout.addLayout(button_layout)

        help_dialog.setLayout(help_layout)
        help_dialog.exec_()

    def apply_cylindrical_params(self, dialog):
        try:
            # Считываем введенные значения
            momentc_1 = float(self.momentc_1_input.text())
            momentc_2 = float(self.momentc_2_input.text())
            momentc_3 = float(self.momentc_3_input.text())
            lengthc_1 = float(self.lengthc_1_input.text())
            lengthc_2 = float(self.lengthc_2_input.text())
            distancec = float(self.distancec_input.text())
            massc_2 = float(self.massc_2_input.text())
            massc_3 = float(self.massc_3_input.text())

            # Проверяем значения на ограничения
            if not (0 <= momentc_1 <= 1):
                raise ValueError("Момент 1-го звена не в диапазоне от 0 до 1")
            if not (0 <= momentc_2 <= 1):
                raise ValueError("Момент 2-го звена не в диапазоне от 0 до 1")
            if not (0 <= momentc_3 <= 1):
                raise ValueError("Момент 3-го звена не в диапазоне от 0 до 1")
            if not (0 <= lengthc_1 <= 1):
                raise ValueError("Длина 1-го звена не в диапазоне от 0 до 1")
            if not (0 <= lengthc_2 <= 1):
                raise ValueError("Длина 2-го звена не в диапазоне от 0 до 1")
            if not (0 <= distancec <= 1):
                raise ValueError("Расстояние не в диапазоне от 0 до 1")
            if not (0 <= massc_2 <= 20):
                raise ValueError("Масса 2-го звена не в диапазоне от 0 до 20")
            if not (0 <= massc_3 <= 20):
                raise ValueError("Масса 3-го звена не в диапазоне от 0 до 20")

            # Если все проверки пройдены, сохраняем значения
            self.momentc_1 = momentc_1
            self.momentc_2 = momentc_2
            self.momentc_3 = momentc_3
            self.lengthc_1 = lengthc_1
            self.lengthc_2 = lengthc_2
            self.distancec = distancec
            self.massc_2 = massc_2
            self.massc_3 = massc_3
            self.workspace_calculator.set_cylindrical_params(momentc_1, momentc_2, momentc_3, lengthc_1, lengthc_2,
                                                             distancec, massc_2, massc_3)
            self.trajectory_calculator.set_cylindrical_params(momentc_1, momentc_2, momentc_3, lengthc_1, lengthc_2,
                                                             distancec, massc_2, massc_3)

            dialog.accept()


        except ValueError as e:
            # Показываем окно с ошибкой, если значение выходит за допустимый диапазон
            QMessageBox.warning(self, "Ошибка", str(e))

    def show_cylindrical_limits_dialog(self):
        # Создание диалогового окна для ввода ограничений по координатам
        dialog = QDialog(self)
        dialog.setWindowTitle("Ограничения по координатам Цилиндр")

        layout = QVBoxLayout()

        # Создание сетки для полей ввода
        grid_layout = QGridLayout()

        # Создание полей ввода для ограничений
        self.q1c_min_input = QLineEdit(self)
        self.q1c_max_input = QLineEdit(self)
        self.a2c_min_input = QLineEdit(self)
        self.a2c_max_input = QLineEdit(self)
        self.q3c_min_input = QLineEdit(self)
        self.q3c_max_input = QLineEdit(self)
        self.zc_min_input = QLineEdit(self)
        self.zc_max_input = QLineEdit(self)

        self.q1c_min_input.setFont(QFont('Times', 14))
        self.q1c_max_input.setFont(QFont('Times', 14))
        self.a2c_min_input.setFont(QFont('Times', 14))
        self.a2c_max_input.setFont(QFont('Times', 14))
        self.q3c_min_input.setFont(QFont('Times', 14))
        self.q3c_max_input.setFont(QFont('Times', 14))
        self.zc_min_input.setFont(QFont('Times', 14))
        self.zc_max_input.setFont(QFont('Times', 14))

        # Установка значений в поля ввода
        self.q1c_min_input.setText(str(self.q1c_min))
        self.q1c_max_input.setText(str(self.q1c_max))
        self.a2c_min_input.setText(str(self.a2c_min))
        self.a2c_max_input.setText(str(self.a2c_max))
        self.q3c_min_input.setText(str(self.q3c_min))
        self.q3c_max_input.setText(str(self.q3c_max))
        self.zc_min_input.setText(str(self.zc_min))
        self.zc_max_input.setText(str(self.zc_max))

        # Добавление полей в сетку с метками
        temp = QLabel("Q1min")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 0, 0)
        grid_layout.addWidget(self.q1c_min_input, 0, 1)

        temp = QLabel("A1min")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 1, 0)
        grid_layout.addWidget(self.a2c_min_input, 1, 1)

        temp = QLabel("Q3min")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 2, 0)
        grid_layout.addWidget(self.q3c_min_input, 2, 1)

        temp = QLabel("Zmin")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 3, 0)
        grid_layout.addWidget(self.zc_min_input, 3, 1)

        temp = QLabel("Q1max")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 0, 2)
        grid_layout.addWidget(self.q1c_max_input, 0, 3)

        temp = QLabel("A2max")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 1, 2)
        grid_layout.addWidget(self.a2c_max_input, 1, 3)

        temp = QLabel("Q3max")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 2, 2)
        grid_layout.addWidget(self.q3c_max_input, 2, 3)

        temp = QLabel("Zmax")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 3, 2)
        grid_layout.addWidget(self.zc_max_input, 3, 3)

        layout.addLayout(grid_layout)

        # Создание кнопок ОК и Отменить
        button_layout = QHBoxLayout()
        ok_button = QPushButton("ОК")
        ok_button.setStyleSheet("background-color : #18f22e")
        ok_button.setFont(QFont('Times', 14))
        cancel_button = QPushButton("Отменить")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        help_button = QPushButton("Помощь")
        help_button.setStyleSheet("background-color : #87CEEB")
        help_button.setFont(QFont('Times', 14))


        ok_button.clicked.connect(lambda: self.apply_cylindrical_limits(dialog))
        cancel_button.clicked.connect(dialog.reject)
        help_button.clicked.connect(self.show_cylindrical_limits_help_dialog)

        button_layout.addWidget(ok_button)
        button_layout.addWidget(help_button)
        button_layout.addWidget(cancel_button)

        layout.addLayout(button_layout)
        dialog.setLayout(layout)

        # Открываем диалоговое окно и заполняем поля данными
        if dialog.exec_() == QDialog.Accepted:
            self.q1c_min = self.q1c_min_input.text()
            self.q1c_max = self.q1c_max_input.text()
            self.a2c_min = self.a2c_min_input.text()
            self.a2c_max = self.a2c_max_input.text()
            self.q3c_min = self.q3c_min_input.text()
            self.q3c_max = self.q3c_max_input.text()
            self.zc_min = self.zc_min_input.text()
            self.zc_max = self.zc_max_input.text()

    def show_cylindrical_limits_help_dialog(self):

        help_dialog = QDialog(self)
        help_dialog.setWindowTitle("Подсказка")
        help_layout = QVBoxLayout()

        # Основной текст подсказки с выделением
        help_text = QLabel(self)
        help_text.setFont(QFont('Times', 12))

        help_text.setText(""" На обобщенные координаты робота Цилиндр наложены следующие ограничения:<br>
                            q1min - минимально возможное значение угла поворота 1 звена;<br>
                            q1max - максимально возможное значение угла поворота 1 звена;<br><br>
                            a2min - минимально возможное значение линейного перемещения 2 звена;<br>
                            a2max - максимальное возможное значение линейного перемещения 2 звена;<br><br>
                            q3min - минимальное возможное значение угла поворота 3 звена;<br>
                            q3max - максимальное возможное значение угла поворота 3 звена;<br><br>
                            z1min - минимальное возможное значение линейного перемещения 3 звена;<br>
                            zmax - максимально возможное значение линейного перемещения 3 звена.<br>
                            """)

        help_layout.addWidget(help_text)

        # Кнопки для открытия дополнительных подсказок
        button_layout = QHBoxLayout()

        cylindrical_button = QPushButton("Цилиндр")
        cylindrical_button.setFont(QFont('Times', 14))
        cylindrical_button.setStyleSheet("background-color: yellow")
        cylindrical_button.clicked.connect(self.show_cylindrical_help)

        button_layout.addWidget(cylindrical_button)

        cancel_button = QPushButton("Закрыть")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        cancel_button.clicked.connect(help_dialog.reject)
        button_layout.addWidget(cancel_button)

        help_layout.addLayout(button_layout)

        help_dialog.setLayout(help_layout)
        help_dialog.exec_()

    def apply_cylindrical_limits(self, dialog):
        try:
            # Считываем введенные значения
            q1c_min = float(self.q1c_min_input.text())
            q1c_max = float(self.q1c_max_input.text())
            a2c_min = float(self.a2c_min_input.text())
            a2c_max = float(self.a2c_max_input.text())
            q3c_min = float(self.q3c_min_input.text())
            q3c_max = float(self.q3c_max_input.text())
            zc_min = float(self.zc_min_input.text())
            zc_max = float(self.zc_max_input.text())

            # Проверяем значения на ограничения
            if not (-3.142 <= q1c_min <= 3.142):
                raise ValueError("Значение Q1min не в диапазоне от -3.142 до 3.142")
            if not (-3.142 <= q1c_max <= 3.142):
                raise ValueError("Значение Q1max не в диапазоне от -3.142 до 3.142")
            if not (0 <= a2c_min <= 1):
                raise ValueError("Значение A2min не в диапазоне от 0 до 1")
            if not (0 <= a2c_max <= 1):
                raise ValueError("Значение A2max не в диапазоне от 0 до 1")
            if not (-3.142 <= q3c_min <= 3.142):
                raise ValueError("Значение Q3min не в диапазоне от -3.142 до 3.142")
            if not (-3.142 <= q3c_max <= 3.142):
                raise ValueError("Значение Q3max не в диапазоне от -3.142 до 3.142")
            if not (0 <= zc_min <= 1):
                raise ValueError("Значение Zmin не в диапазоне от 0 до 1")
            if not (0 <= zc_max <= 1):
                raise ValueError("Значение Zmax не в диапазоне от 0 до 1")

            # Проверяем, чтобы значения max были больше или равны соответствующим min
            if q1c_max < q1c_min:
                raise ValueError("Q1max должно быть больше или равно Q1min")
            if a2c_max < a2c_min:
                raise ValueError("A2max должно быть больше или равно Q2min")
            if q3c_max < q3c_min:
                raise ValueError("Q3max должно быть больше или равно Q3min")
            if zc_max < zc_min:
                raise ValueError("Zmax должно быть больше или равно Zmin")

            # Если все проверки пройдены, сохраняем значения
            self.q1c_min, self.q1c_max = q1c_min, q1c_max
            self.a2c_min, self.a2c_max = a2c_min, a2c_max
            self.q3c_min, self.q3c_max = q3c_min, q3c_max
            self.zc_min, self.zc_max = zc_min, zc_max

            self.workspace_calculator.set_cylindrical_limits(q1c_min, q1c_max, a2c_min, a2c_max, q3c_min, q3c_max,
                                                             zc_min, zc_max)
            self.trajectory_calculator.set_cylindrical_limits(q1c_min, q1c_max, a2c_min, a2c_max, q3c_min, q3c_max,
                                                             zc_min, zc_max)

            # Закрываем диалоговое окно
            dialog.accept()

        except ValueError as e:
            # Показываем окно с ошибкой, если значение выходит за допустимый диапазон
            QMessageBox.warning(self, "Ошибка", str(e))

    def show_coler_params_dialog(self):
        # Создание диалогового окна для ввода конструктивных параметров Колер
        dialog = QDialog(self)
        dialog.setWindowTitle("Конструктивные параметры Колер")

        layout = QVBoxLayout()
        grid_layout = QGridLayout()

        # Поля ввода
        self.momentcol_1_input = QLineEdit(self)
        self.momentcol_1_input.setFont(QFont('Times', 14))
        self.momentcol_1_input.setText(str(self.momentcol_1) if self.momentcol_1 is not None else "")

        self.momentcol_2_input = QLineEdit(self)
        self.momentcol_2_input.setFont(QFont('Times', 14))
        self.momentcol_2_input.setText(str(self.momentcol_2) if self.momentcol_2 is not None else "")

        self.momentcol_3_input = QLineEdit(self)
        self.momentcol_3_input.setFont(QFont('Times', 14))
        self.momentcol_3_input.setText(str(self.momentcol_3) if self.momentcol_3 is not None else "")

        self.lengthcol_1_input = QLineEdit(self)
        self.lengthcol_1_input.setFont(QFont('Times', 14))
        self.lengthcol_1_input.setText(str(self.lengthcol_1) if self.lengthcol_1 is not None else "")

        self.lengthcol_2_input = QLineEdit(self)
        self.lengthcol_2_input.setFont(QFont('Times', 14))
        self.lengthcol_2_input.setText(str(self.lengthcol_2) if self.lengthcol_2 is not None else "")

        self.distancecol_input = QLineEdit(self)
        self.distancecol_input.setFont(QFont('Times', 14))
        self.distancecol_input.setText(str(self.distancecol) if self.distancecol is not None else "")

        self.masscol_2_input = QLineEdit(self)
        self.masscol_2_input.setFont(QFont('Times', 14))
        self.masscol_2_input.setText(str(self.masscol_2) if self.masscol_2 is not None else "")

        self.masscol_3_input = QLineEdit(self)
        self.masscol_3_input.setFont(QFont('Times', 14))
        self.masscol_3_input.setText(str(self.masscol_3) if self.masscol_3 is not None else "")

        # Добавление полей в сетку
        temp = QLabel("Момент 1-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 0, 0)
        grid_layout.addWidget(self.momentcol_1_input, 0, 1)

        temp = QLabel("Момент 2-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 1, 0)
        grid_layout.addWidget(self.momentcol_2_input, 1, 1)

        temp = QLabel("Момент 3-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 2, 0)
        grid_layout.addWidget(self.momentcol_3_input, 2, 1)

        temp = QLabel("Длина 1-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 3, 0)
        grid_layout.addWidget(self.lengthcol_1_input, 3, 1)

        temp = QLabel("Длина 2-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 0, 2)
        grid_layout.addWidget(self.lengthcol_2_input, 0, 3)

        temp = QLabel("Расстояние")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 1, 2)
        grid_layout.addWidget(self.distancecol_input, 1, 3)

        temp = QLabel("Масса 2-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 2, 2)
        grid_layout.addWidget(self.masscol_2_input, 2, 3)

        temp = QLabel("Масса 3-го звена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 3, 2)
        grid_layout.addWidget(self.masscol_3_input, 3, 3)

        layout.addLayout(grid_layout)

        # Кнопки OK и Отменить
        button_layout = QHBoxLayout()
        ok_button = QPushButton("ОК")
        ok_button.setStyleSheet("background-color : #18f22e")
        ok_button.setFont(QFont('Times', 14))
        cancel_button = QPushButton("Отменить")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        help_button = QPushButton("Помощь")
        help_button.setStyleSheet("background-color : #87CEEB")
        help_button.setFont(QFont('Times', 14))

        ok_button.clicked.connect(lambda: self.apply_coler_params(dialog))
        help_button.clicked.connect(self.show_coler_params_help_dialog)
        cancel_button.clicked.connect(dialog.reject)

        button_layout.addWidget(ok_button)
        button_layout.addWidget(help_button)
        button_layout.addWidget(cancel_button)

        layout.addLayout(button_layout)
        dialog.setLayout(layout)

        # Открываем диалоговое окно
        dialog.exec_()

    def show_coler_params_help_dialog(self):

        help_dialog = QDialog(self)
        help_dialog.setWindowTitle("Подсказка")
        help_layout = QVBoxLayout()

        # Основной текст подсказки с выделением
        help_text = QLabel(self)
        help_text.setFont(QFont('Times', 12))

        help_text.setText("""Конструктивные параметры робота ЦИЛИНДР:<br>
        Jθ1 – момент инерции 1 звена относительно шарнира O1 при вращении в горизонтальной плоскости;<br>
        Jc2 – момент инерции 2 звена относительно центра инерции 2 звена;<br>
        Jc3 – момент инерции 3 звена относительно центра инерции 3 звена;<br><br>
        a1 - длина 1 звена;<br>
        a2 - длина 2 звена;<br>
        r2 - расстояние от начала 2 звена до его центра инерции;<br>
        m2 - масса 2 звена;<br>
        m3 - масса 3 звена.<br>
        """)

        help_layout.addWidget(help_text)

        # Кнопки для открытия дополнительных подсказок
        button_layout = QHBoxLayout()

        cylindrical_button = QPushButton("Цилиндр")
        cylindrical_button.setFont(QFont('Times', 14))
        cylindrical_button.setStyleSheet("background-color: yellow")
        cylindrical_button.clicked.connect(self.show_cylindrical_help)

        construct_params_button = QPushButton("Конструктивные параметры")
        construct_params_button.setFont(QFont('Times', 14))
        construct_params_button.setStyleSheet("background: rgb(169, 87, 201)")
        construct_params_button.clicked.connect(self.show_construct_params_help)

        button_layout.addWidget(cylindrical_button)
        button_layout.addWidget(construct_params_button)

        cancel_button = QPushButton("Закрыть")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        cancel_button.clicked.connect(help_dialog.reject)
        button_layout.addWidget(cancel_button)

        help_layout.addLayout(button_layout)

        help_dialog.setLayout(help_layout)
        help_dialog.exec_()

    def apply_coler_params(self, dialog):
        try:
            # Считываем введенные значения
            momentcol_1 = float(self.momentcol_1_input.text())
            momentcol_2 = float(self.momentcol_2_input.text())
            momentcol_3 = float(self.momentcol_3_input.text())
            lengthcol_1 = float(self.lengthcol_1_input.text())
            lengthcol_2 = float(self.lengthcol_2_input.text())
            distancecol = float(self.distancecol_input.text())
            masscol_2 = float(self.masscol_2_input.text())
            masscol_3 = float(self.masscol_3_input.text())

            # Проверяем значения на ограничения
            if not (0 <= momentcol_1 <= 1):
                raise ValueError("Момент 1-го звена не в диапазоне от 0 до 1")
            if not (0 <= momentcol_2 <= 1):
                raise ValueError("Момент 2-го звена не в диапазоне от 0 до 1")
            if not (0 <= momentcol_3 <= 1):
                raise ValueError("Момент 3-го звена не в диапазоне от 0 до 1")
            if not (0 <= lengthcol_1 <= 1):
                raise ValueError("Длина 1-го звена не в диапазоне от 0 до 1")
            if not (0 <= lengthcol_2 <= 1):
                raise ValueError("Длина 2-го звена не в диапазоне от 0 до 1")
            if not (0 <= distancecol <= 1):
                raise ValueError("Расстояние не в диапазоне от 0 до 1")
            if not (0 <= masscol_2 <= 20):
                raise ValueError("Масса 2-го звена не в диапазоне от 0 до 20")
            if not (0 <= masscol_3 <= 20):
                raise ValueError("Масса 3-го звена не в диапазоне от 0 до 20")

            # Если все проверки пройдены, сохраняем значения
            self.momentcol_1 = momentcol_1
            self.momentcol_2 = momentcol_2
            self.momentcol_3 = momentcol_3
            self.lengthcol_1 = lengthcol_1
            self.lengthcol_2 = lengthcol_2
            self.distancecol = distancecol
            self.masscol_2 = masscol_2
            self.masscol_3 = masscol_3
            self.workspace_calculator.set_coler_params(momentcol_1, momentcol_2, momentcol_3, lengthcol_1, lengthcol_2,
                                                             distancecol, masscol_2, masscol_3)
            self.trajectory_calculator.set_coler_params(momentcol_1, momentcol_2, momentcol_3, lengthcol_1, lengthcol_2,
                                                             distancecol, masscol_2, masscol_3)

            dialog.accept()


        except ValueError as e:
            # Показываем окно с ошибкой, если значение выходит за допустимый диапазон
            QMessageBox.warning(self, "Ошибка", str(e))

    def show_coler_limits_dialog(self):
        # Создание диалогового окна для ввода ограничений по координатам
        dialog = QDialog(self)
        dialog.setWindowTitle("Ограничения по координатам Колер")

        layout = QVBoxLayout()

        # Создание сетки для полей ввода
        grid_layout = QGridLayout()

        # Создание полей ввода для ограничений
        self.q1col_min_input = QLineEdit(self)
        self.q1col_max_input = QLineEdit(self)
        self.a2col_min_input = QLineEdit(self)
        self.a2col_max_input = QLineEdit(self)
        self.q3col_min_input = QLineEdit(self)
        self.q3col_max_input = QLineEdit(self)
        self.zcol_min_input = QLineEdit(self)
        self.zcol_max_input = QLineEdit(self)

        self.q1col_min_input.setFont(QFont('Times', 14))
        self.q1col_max_input.setFont(QFont('Times', 14))
        self.a2col_min_input.setFont(QFont('Times', 14))
        self.a2col_max_input.setFont(QFont('Times', 14))
        self.q3col_min_input.setFont(QFont('Times', 14))
        self.q3col_max_input.setFont(QFont('Times', 14))
        self.zcol_min_input.setFont(QFont('Times', 14))
        self.zcol_max_input.setFont(QFont('Times', 14))

        # Установка значений в поля ввода
        self.q1col_min_input.setText(str(self.q1col_min))
        self.q1col_max_input.setText(str(self.q1col_max))
        self.a2col_min_input.setText(str(self.a2col_min))
        self.a2col_max_input.setText(str(self.a2col_max))
        self.q3col_min_input.setText(str(self.q3col_min))
        self.q3col_max_input.setText(str(self.q3col_max))
        self.zcol_min_input.setText(str(self.zcol_min))
        self.zcol_max_input.setText(str(self.zcol_max))

        # Добавление полей в сетку с метками

        temp = QLabel("A1min")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 0, 0)
        grid_layout.addWidget(self.a2col_min_input, 0, 1)

        temp = QLabel("A2max")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 0, 2)
        grid_layout.addWidget(self.a2col_max_input, 0, 3)

        temp = QLabel("Q1min")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 1, 0)
        grid_layout.addWidget(self.q1col_min_input, 1, 1)

        temp = QLabel("Q1max")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 1, 2)
        grid_layout.addWidget(self.q1col_max_input, 1, 3)

        temp = QLabel("Q3min")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 2, 0)
        grid_layout.addWidget(self.q3col_min_input, 2, 1)

        temp = QLabel("Zmin")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 3, 0)
        grid_layout.addWidget(self.zcol_min_input, 3, 1)

        temp = QLabel("Q3max")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 2, 2)
        grid_layout.addWidget(self.q3col_max_input, 2, 3)

        temp = QLabel("Zmax")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 3, 2)
        grid_layout.addWidget(self.zcol_max_input, 3, 3)

        layout.addLayout(grid_layout)

        # Создание кнопок ОК и Отменить
        button_layout = QHBoxLayout()
        ok_button = QPushButton("ОК")
        ok_button.setStyleSheet("background-color : #18f22e")
        ok_button.setFont(QFont('Times', 14))
        cancel_button = QPushButton("Отменить")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        help_button = QPushButton("Помощь")
        help_button.setStyleSheet("background-color : #87CEEB")
        help_button.setFont(QFont('Times', 14))


        ok_button.clicked.connect(lambda: self.apply_coler_limits(dialog))
        cancel_button.clicked.connect(dialog.reject)
        help_button.clicked.connect(self.show_coler_limits_help_dialog)

        button_layout.addWidget(ok_button)
        button_layout.addWidget(help_button)
        button_layout.addWidget(cancel_button)

        layout.addLayout(button_layout)
        dialog.setLayout(layout)

        # Открываем диалоговое окно и заполняем поля данными
        if dialog.exec_() == QDialog.Accepted:
            self.a2col_min = self.a2col_min_input.text()
            self.a2col_max = self.a2col_max_input.text()
            self.q1col_min = self.q1col_min_input.text()
            self.q1col_max = self.q1col_max_input.text()
            self.q3col_min = self.q3col_min_input.text()
            self.q3col_max = self.q3col_max_input.text()
            self.zcol_min = self.zcol_min_input.text()
            self.zcol_max = self.zcol_max_input.text()

    def show_coler_limits_help_dialog(self):

        help_dialog = QDialog(self)
        help_dialog.setWindowTitle("Подсказка")
        help_layout = QVBoxLayout()

        # Основной текст подсказки с выделением
        help_text = QLabel(self)
        help_text.setFont(QFont('Times', 12))

        help_text.setText(""" На обобщенные координаты робота Цилиндр наложены следующие ограничения:<br>
                            q1min - минимально возможное значение угла поворота 1 звена;<br>
                            q1max - максимально возможное значение угла поворота 1 звена;<br><br>
                            a2min - минимально возможное значение линейного перемещения 2 звена;<br>
                            a2max - максимальное возможное значение линейного перемещения 2 звена;<br><br>
                            q3min - минимальное возможное значение угла поворота 3 звена;<br>
                            q3max - максимальное возможное значение угла поворота 3 звена;<br><br>
                            z1min - минимальное возможное значение линейного перемещения 3 звена;<br>
                            zmax - максимально возможное значение линейного перемещения 3 звена.<br>
                            """)

        help_layout.addWidget(help_text)

        # Кнопки для открытия дополнительных подсказок
        button_layout = QHBoxLayout()

        cylindrical_button = QPushButton("Цилиндр")
        cylindrical_button.setFont(QFont('Times', 14))
        cylindrical_button.setStyleSheet("background-color: yellow")
        cylindrical_button.clicked.connect(self.show_cylindrical_help)

        button_layout.addWidget(cylindrical_button)

        cancel_button = QPushButton("Закрыть")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        cancel_button.clicked.connect(help_dialog.reject)
        button_layout.addWidget(cancel_button)

        help_layout.addLayout(button_layout)

        help_dialog.setLayout(help_layout)
        help_dialog.exec_()

    def apply_coler_limits(self, dialog):
        try:
            # Считываем введенные значения
            q1col_min = float(self.q1col_min_input.text())
            q1col_max = float(self.q1col_max_input.text())
            a2col_min = float(self.a2col_min_input.text())
            a2col_max = float(self.a2col_max_input.text())
            q3col_min = float(self.q3col_min_input.text())
            q3col_max = float(self.q3col_max_input.text())
            zcol_min = float(self.zcol_min_input.text())
            zcol_max = float(self.zcol_max_input.text())

            # Проверяем значения на ограничения
            if not (-3.142 <= q1col_min <= 3.142):
                raise ValueError("Значение Q1min не в диапазоне от -3.142 до 3.142")
            if not (-3.142 <= q1col_max <= 3.142):
                raise ValueError("Значение Q1max не в диапазоне от -3.142 до 3.142")
            if not (0 <= a2col_min <= 1):
                raise ValueError("Значение A2min не в диапазоне от 0 до 1")
            if not (0 <= a2col_max <= 1):
                raise ValueError("Значение A2max не в диапазоне от 0 до 1")
            if not (-3.142 <= q3col_min <= 3.142):
                raise ValueError("Значение Q3min не в диапазоне от -3.142 до 3.142")
            if not (-3.142 <= q3col_max <= 3.142):
                raise ValueError("Значение Q3max не в диапазоне от -3.142 до 3.142")
            if not (0 <= zcol_min <= 1):
                raise ValueError("Значение Zmin не в диапазоне от 0 до 1")
            if not (0 <= zcol_max <= 1):
                raise ValueError("Значение Zmax не в диапазоне от 0 до 1")

            # Проверяем, чтобы значения max были больше или равны соответствующим min
            if q1col_max < q1col_min:
                raise ValueError("Q1max должно быть больше или равно Q1min")
            if a2col_max < a2col_min:
                raise ValueError("A2max должно быть больше или равно Q2min")
            if q3col_max < q3col_min:
                raise ValueError("Q3max должно быть больше или равно Q3min")
            if zcol_max < zcol_min:
                raise ValueError("Zmax должно быть больше или равно Zmin")

            # Если все проверки пройдены, сохраняем значения
            self.q1col_min, self.q1c_max = q1col_min, q1col_max
            self.a2col_min, self.a2c_max = a2col_min, a2col_max
            self.q3col_min, self.q3c_max = q3col_min, q3col_max
            self.zcol_min, self.zc_max = zcol_min, zcol_max

            self.workspace_calculator.set_coler_limits(q1col_min, q1col_max, a2col_min, a2col_max, q3col_min, q3col_max,
                                                             zcol_min, zcol_max)
            self.trajectory_calculator.set_coler_limits(q1col_min, q1col_max, a2col_min, a2col_max, q3col_min, q3col_max,
                                                             zcol_min, zcol_max)

            # Закрываем диалоговое окно
            dialog.accept()

        except ValueError as e:
            # Показываем окно с ошибкой, если значение выходит за допустимый диапазон
            QMessageBox.warning(self, "Ошибка", str(e))

    def show_motor_params_dialog(self):
        # Создание диалогового окна для параметров двигателей
        dialog = QDialog(self)
        dialog.setWindowTitle("Параметры двигателей")

        layout = QVBoxLayout()
        grid_layout = QGridLayout()

        # Заголовки строк
        row_labels = [
            ["J1", "J2", "J3", "J4"],
            ["n1", "n2", "n3", "n4"],
            ["U1max", "U2max", "U3max", "U4max"],
            ["Ku1", "Ku2", "Ku3", "Ku4"],
            ["Kq1", "Kq2", "Kq3", "Kq4"]
        ]

        # Создание полей ввода с заголовками
        self.motor_inputs = {}
        for row in range(5):
            for col in range(4):
                motor_label = QLabel(row_labels[row][col])
                motor_label.setFont(QFont('Times', 13))
                input_field = QLineEdit(self)
                input_field.setFont(QFont('Times', 14))

                key = f"{row_labels[row][col]}"  # Используем имена заголовков в качестве ключей

                # Устанавливаем значения из атрибутов класса
                if key.startswith("J"):
                    default_value = self.J[col]
                elif key.startswith("n"):
                    default_value = self.n[col]
                elif key.startswith("U"):
                    default_value = self.Umax[col]
                elif key.startswith("Ku"):
                    default_value = self.Ku[col]
                elif key.startswith("Kq"):
                    default_value = self.Kq[col]

                input_field.setText(str(default_value))

                grid_layout.addWidget(motor_label, row, col * 2)
                grid_layout.addWidget(input_field, row, col * 2 + 1)
                self.motor_inputs[key] = input_field

        layout.addLayout(grid_layout)

        # Создание кнопок ОК и Отменить
        button_layout = QHBoxLayout()
        ok_button = QPushButton("ОК")
        ok_button.setStyleSheet("background-color : #18f22e")
        ok_button.setFont(QFont('Times', 14))
        cancel_button = QPushButton("Отменить")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        help_button = QPushButton("Помощь")
        help_button.setStyleSheet("background-color : #87CEEB")
        help_button.setFont(QFont('Times', 14))

        ok_button.clicked.connect(lambda: self.apply_motor_params(dialog))
        help_button.clicked.connect(self.show_motor_params_help_dialog)
        cancel_button.clicked.connect(dialog.reject)

        button_layout.addWidget(ok_button)
        button_layout.addWidget(help_button)
        button_layout.addWidget(cancel_button)

        layout.addLayout(button_layout)
        dialog.setLayout(layout)
        dialog.exec_()

    def show_motor_params_help_dialog(self):

        help_dialog = QDialog(self)
        help_dialog.setWindowTitle("Подсказка")
        help_layout = QVBoxLayout()

        # Основной текст подсказки с выделением
        help_text = QLabel(self)
        help_text.setFont(QFont('Times', 12))

        help_text.setText("""К числу параметров двигателя относятся:  <br>
                            Обратите внимание, что значения должны быть числами. <br>
                            - J — момент инерции ротора; <br>
                            - n — передаточный коэффициент редуктора; <br>
                            - Umax — максимальное напряжение двигателя; <br>
                            - Ku — коэффициент управления по напряжению; <br>
                            - Kq — коэффициент противоЭДС;""")

        help_layout.addWidget(help_text)

        # Кнопки для открытия дополнительных подсказок
        button_layout = QHBoxLayout()

        cancel_button = QPushButton("Закрыть")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        cancel_button.clicked.connect(help_dialog.reject)
        button_layout.addWidget(cancel_button)

        help_layout.addLayout(button_layout)

        help_dialog.setLayout(help_layout)
        help_dialog.exec_()

    def apply_motor_params(self, dialog):
        try:
            # Проверяем диапазоны значений и сохраняем параметры в атрибутах класса
            for key, input_field in self.motor_inputs.items():
                value = float(input_field.text())

                # Проверка для J1-J4 (диапазон 0-1)
                if key.startswith("J") and not (0 <= value <= 1):
                    self.show_error(f"Значение {key} не в диапазоне от 0 до 1")
                    return

                # Проверка для n1-n4 (диапазон 0-500)
                if key.startswith("n") and not (0 <= value <= 500):
                    self.show_error(f"Значение {key} не в диапазоне от 0 до 500")
                    return

                # Проверка для U1max-U4max (диапазон 0-40)
                if key.startswith("U") and "max" in key and not (0 <= value <= 40):
                    self.show_error(f"Значение {key} не в диапазоне от 0 до 40")
                    return

                # Проверка для Ku1-Ku4 (диапазон 0-0.1)
                if key.startswith("Ku") and not (0 <= value <= 0.1):
                    self.show_error(f"Значение {key} не в диапазоне от 0 до 0,1")
                    return

                # Проверка для Kq1-Kq4 (диапазон 0-0.1)
                if key.startswith("Kq") and not (0 <= value <= 0.1):
                    self.show_error(f"Значение {key} не в диапазоне от 0 до 0,1")
                    return

            # Сохраняем значения в соответствующие атрибуты класса
            for key, input_field in self.motor_inputs.items():
                value = float(input_field.text())

                if key.startswith("J"):
                    index = int(key[1]) - 1
                    self.J[index] = value
                elif key.startswith("n"):
                    index = int(key[1]) - 1
                    self.n[index] = value
                elif key.startswith("U"):
                    index = int(key[1]) - 1
                    self.Umax[index] = value
                elif key.startswith("Ku"):
                    index = int(key[2]) - 1
                    self.Ku[index] = value
                elif key.startswith("Kq"):
                    index = int(key[2]) - 1
                    self.Kq[index] = value

            self.trajectory_calculator.get_motor_params(self.J, self.n, self.Umax, self.Ku, self.Kq)
            dialog.accept()

        except ValueError:
            self.show_error("Некорректный ввод. Введите числовое значение.")

    def show_error(self, message):
        error_dialog = QMessageBox(self)
        error_dialog.setWindowTitle("Ошибка")
        error_dialog.setText(message)
        error_dialog.setIcon(QMessageBox.Warning)
        error_dialog.exec_()

    def show_regulator_params_dialog(self):
        # Создание диалогового окна для параметров регуляторов
        dialog = QDialog(self)
        dialog.setWindowTitle("Параметры регуляторов")

        layout = QVBoxLayout()
        grid_layout = QGridLayout()

        # Заголовки строк
        row_labels = [
            ["Kп1", "Kп2", "Kп3", "Kп4"],
            ["Kи1", "Kи2", "Kи3", "Kи4"],
            ["Kд1", "Kд2", "Kд3", "Kд4"]
        ]

        # Создание полей ввода с заголовками
        self.regulator_inputs = {}
        for row in range(3):
            for col in range(4):
                motor_label = QLabel(row_labels[row][col])
                motor_label.setFont(QFont('Times', 13))
                input_field = QLineEdit(self)
                input_field.setFont(QFont('Times', 14))

                key = f"{row_labels[row][col]}"  # Используем имена заголовков в качестве ключей

                # Устанавливаем значения из атрибутов класса
                if key.startswith("Kп"):
                    default_value = self.Kp[col]
                elif key.startswith("Kи"):
                    default_value = self.Ki[col]
                elif key.startswith("Kд"):
                    default_value = self.Kd[col]

                input_field.setText(str(default_value))

                grid_layout.addWidget(motor_label, row, col * 2)
                grid_layout.addWidget(input_field, row, col * 2 + 1)
                self.regulator_inputs[key] = input_field

        layout.addLayout(grid_layout)

        # Создание кнопок ОК и Отменить
        button_layout = QHBoxLayout()
        ok_button = QPushButton("ОК")
        ok_button.setStyleSheet("background-color : #18f22e")
        ok_button.setFont(QFont('Times', 14))
        cancel_button = QPushButton("Отменить")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        help_button = QPushButton("Помощь")
        help_button.setStyleSheet("background-color : #87CEEB")
        help_button.setFont(QFont('Times', 14))

        ok_button.clicked.connect(lambda: self.apply_regulator_params(dialog))
        help_button.clicked.connect(self.show_regulator_params_help_dialog)
        cancel_button.clicked.connect(dialog.reject)

        button_layout.addWidget(ok_button)
        button_layout.addWidget(help_button)
        button_layout.addWidget(cancel_button)

        layout.addLayout(button_layout)
        dialog.setLayout(layout)
        dialog.exec_()

    def show_regulator_params_help_dialog(self):

        help_dialog = QDialog(self)
        help_dialog.setWindowTitle("Подсказка")
        help_layout = QVBoxLayout()

        # Основной текст подсказки с выделением
        help_text = QLabel(self)
        help_text.setFont(QFont('Times', 12))

        help_text.setText("""Кп, Ки, Кд - коэффициенты ПИД-регулятора. """)

        help_layout.addWidget(help_text)

        # Кнопки для открытия дополнительных подсказок
        button_layout = QHBoxLayout()

        cancel_button = QPushButton("Закрыть")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        cancel_button.clicked.connect(help_dialog.reject)
        button_layout.addWidget(cancel_button)

        help_layout.addLayout(button_layout)

        help_dialog.setLayout(help_layout)
        help_dialog.exec_()

    def apply_regulator_params(self, dialog):
        try:
            # Проверяем и сохраняем параметры регуляторов
            for key, input_field in self.regulator_inputs.items():
                value = float(input_field.text())

                if not (0 <= value <= 500):
                    raise ValueError(f"Значение {key} не в диапазоне от 0 до 500")

                if key.startswith("Kп"):
                    index = int(key[2]) - 1
                    self.Kp[index] = value
                elif key.startswith("Kи"):
                    index = int(key[2]) - 1
                    self.Ki[index] = value
                elif key.startswith("Kд"):
                    index = int(key[2]) - 1
                    self.Kd[index] = value
            self.trajectory_calculator.PID_regulator(self.Kp, self.Ki, self.Kd)

            dialog.accept()

        except ValueError as e:
            self.show_error(str(e))

    def show_error(self, message):
        error_dialog = QMessageBox(self)
        error_dialog.setWindowTitle("Ошибка")
        error_dialog.setText(message)
        error_dialog.setIcon(QMessageBox.Warning)
        error_dialog.exec_()

    def show_cyclegram_dialog(self):
        # Создание диалогового окна для циклограммы
        dialog = QDialog(self)
        dialog.setWindowTitle("Циклограмма")

        layout = QVBoxLayout()

        # Создание сетки для полей ввода
        grid_layout = QGridLayout()

        # Заголовки строк (слева)
        headers = ["t", "q1", "q2", "q3", "q4"]
        for row, header in enumerate(headers):
            temp = QLabel(header)
            temp.setFont(QFont('Times', 13))
            grid_layout.addWidget(temp, row + 1, 0)  # Добавляем заголовки по вертикали (в столбец 0)

        # Создание полей ввода
        self.cycle_inputs = {'t': [], 'q1': [], 'q2': [], 'q3': [], 'q4': []}

        # Добавляем 9 столбцов с 4 строками для каждого параметра
        for col in range(9):  # 9 столбцов данных
            for row, key in enumerate(self.cycle_inputs.keys()):  # 5 строк (t, q1, q2, q3, q4)
                input_field = QLineEdit(self)
                input_field.setFont(QFont('Times', 14))

                # Устанавливаем значения из соответствующих списков
                default_value = getattr(self, key)[col]
                input_field.setText(str(default_value))

                grid_layout.addWidget(input_field, row + 1, col + 1)  # Добавляем в сетку (строки идут по горизонтали)
                self.cycle_inputs[key].append(input_field)

        layout.addLayout(grid_layout)

        # Кнопки ОК и Отменить
        button_layout = QHBoxLayout()
        ok_button = QPushButton("ОК")
        ok_button.setStyleSheet("background-color : #18f22e")
        ok_button.setFont(QFont('Times', 14))
        cancel_button = QPushButton("Отменить")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        help_button = QPushButton("Помощь")
        help_button.setStyleSheet("background-color : #87CEEB")
        help_button.setFont(QFont('Times', 14))
        spline_button = QPushButton("Циклограмма сплайна")
        spline_button.setStyleSheet("background-color : steelblue")
        spline_button.setFont(QFont('Times', 14))

        ok_button.clicked.connect(lambda: self.apply_cycle_data(dialog))
        help_button.clicked.connect(self.show_cyclegram_help_dialog)
        spline_button.clicked.connect(self.show_spline_cyclegram_dialog)
        cancel_button.clicked.connect(dialog.reject)

        button_layout.addWidget(ok_button)
        button_layout.addWidget(help_button)
        button_layout.addWidget(spline_button)
        button_layout.addWidget(cancel_button)

        layout.addLayout(button_layout)
        dialog.setLayout(layout)
        dialog.exec_()

    def show_cyclegram_help_dialog(self):

        help_dialog = QDialog(self)
        help_dialog.setWindowTitle("Подсказка")
        help_layout = QVBoxLayout()

        # Основной текст подсказки с выделением
        help_text = QLabel(self)
        help_text.setFont(QFont('Times', 12))

        help_text.setText("""Задайте циклограмму движения. <br>
                            В первой строке задаются моменты времени <br>
                            (в порядке возрастания), в остальных - <br>
                             программные значения обобщенных координат. """)

        help_layout.addWidget(help_text)

        # Кнопки для открытия дополнительных подсказок
        button_layout = QHBoxLayout()

        cancel_button = QPushButton("Закрыть")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        cancel_button.clicked.connect(help_dialog.reject)
        button_layout.addWidget(cancel_button)

        help_layout.addLayout(button_layout)

        help_dialog.setLayout(help_layout)
        help_dialog.exec_()

    def show_spline_cyclegram_dialog(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("Циклограмма сплайна")
        dialog.setGeometry(200, 200, 800, 500)

        main_layout = QVBoxLayout()

        # Текстовое поле для вывода данных
        text_edit = QTextEdit()
        text_edit.setFont(QFont('Courier', 12))
        text_edit.setReadOnly(True)

        # Получаем массивы данных
        data_lists = {
            "t": getattr(self.trajectory_calculator, "t_spline_copied", []),
            "q1": getattr(self.trajectory_calculator, "q_1_spline_copied", []),
            "q2": getattr(self.trajectory_calculator, "q_2_spline_copied", [])
        }

        max_len = max(len(lst) for lst in data_lists.values())  # Длина самого длинного массива

        # Формируем текст для вывода
        output_text = "   t        q1        q2\n"
        output_text += "-" * 30 + "\n"

        for i in range(max_len):
            row_values = []
            row_values.append(f"{data_lists['t'][i]:.2f}".ljust(10) if i < len(data_lists["t"]) else "  -  ".ljust(10))
            row_values.append(
                f"{data_lists['q1'][i]:.5f}".ljust(15) if i < len(data_lists["q1"]) else "  -  ".ljust(15))
            row_values.append(
                f"{data_lists['q2'][i]:.5f}".ljust(15) if i < len(data_lists["q2"]) else "  -  ".ljust(15))

            output_text += " ".join(row_values) + "\n"

        text_edit.setText(output_text)

        close_button = QPushButton("Закрыть")
        close_button.setStyleSheet("background-color: red")
        close_button.setFont(QFont('Times', 14))
        close_button.setFixedSize(170, 50)
        close_button.clicked.connect(dialog.reject)

        main_layout.addWidget(text_edit)
        main_layout.addWidget(close_button, alignment=Qt.AlignCenter)

        dialog.setLayout(main_layout)
        dialog.exec_()

    def apply_cycle_data(self, dialog):
        try:
            # Сохраняем данные из полей в соответствующие списки
            for key, input_fields in self.cycle_inputs.items():
                for i, input_field in enumerate(input_fields):
                    value = float(input_field.text())
                    getattr(self, key)[i] = value
            type_of_control = "Позиционное"
            self.trajectory_calculator.set_cyclogram(
                self.t[:9],
                self.q1[:9],
                self.q2[:9],
                self.q3[:9],
                self.q4[:9],
                type_of_control
            )

            dialog.accept()

        except ValueError:
            self.show_error("Неверное значение. Пожалуйста, введите числовые значения.")

    def show_error(self, message):
        error_dialog = QMessageBox(self)
        error_dialog.setWindowTitle("Ошибка")
        error_dialog.setText(message)
        error_dialog.setIcon(QMessageBox.Warning)
        error_dialog.exec_()

    def show_trajectory_type_dialog(self):
        # Создание диалогового окна для выбора типа траектории
        dialog = QDialog(self)
        dialog.setWindowTitle("Выбор типа траектории")

        layout = QVBoxLayout()

        # Заголовок для диалогового окна
        title_label = QLabel("Тип траектории")
        title_label.setStyleSheet("font-size: 25px; font-weight: bold;")
        layout.addWidget(title_label)

        # Создание группы радио-кнопок для выбора типа траектории
        self.trajectory_group = QGroupBox()
        trajectory_layout = QHBoxLayout()  # Горизонтальное расположение чекбоксов

        self.line_radio = QRadioButton("Прямая")
        self.line_radio.setStyleSheet("font-size: 25px;")
        self.circle_radio = QRadioButton("Окружность")
        self.circle_radio.setStyleSheet("font-size: 25px;")

        # Восстанавливаем ранее выбранное значение
        if hasattr(self, 'trajectory_type'):  # Проверяем наличие переменной
            if self.trajectory_type == "Прямая":
                self.line_radio.setChecked(True)
            elif self.trajectory_type == "Окружность":
                self.circle_radio.setChecked(True)
        else:
            # Устанавливаем значение по умолчанию, если тип траектории не был выбран
            self.line_radio.setChecked(True)

        # Добавляем радио-кнопки в макет
        trajectory_layout.addWidget(self.line_radio)
        trajectory_layout.addWidget(self.circle_radio)

        self.trajectory_group.setLayout(trajectory_layout)

        # Создание кнопок ОК и Отменить
        button_layout = QHBoxLayout()
        ok_button = QPushButton("ОК")
        ok_button.setStyleSheet("background-color : #18f22e")
        ok_button.setFont(QFont('Times', 14))
        cancel_button = QPushButton("Отменить")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        help_button = QPushButton("Помощь")
        help_button.setStyleSheet("background-color : #87CEEB")
        help_button.setFont(QFont('Times', 14))

        ok_button.clicked.connect(lambda: self.apply_trajectory_type(dialog))  # Обработка выбора типа траектории
        help_button.clicked.connect(self.show_trajectory_type_help_dialog)
        cancel_button.clicked.connect(dialog.reject)  # Закрытие диалога без сохранения

        button_layout.addWidget(ok_button)
        button_layout.addWidget(help_button)
        button_layout.addWidget(cancel_button)

        layout.addWidget(self.trajectory_group)
        layout.addLayout(button_layout)

        dialog.setLayout(layout)
        dialog.exec_()

    def show_trajectory_type_help_dialog(self):

        help_dialog = QDialog(self)
        help_dialog.setWindowTitle("Подсказка")
        help_layout = QVBoxLayout()

        # Основной текст подсказки с выделением
        help_text = QLabel(self)
        help_text.setFont(QFont('Times', 12))

        help_text.setText("""Выберите тип траектории. """)

        help_layout.addWidget(help_text)

        # Кнопки для открытия дополнительных подсказок
        button_layout = QHBoxLayout()

        cancel_button = QPushButton("Закрыть")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        cancel_button.clicked.connect(help_dialog.reject)
        button_layout.addWidget(cancel_button)

        help_layout.addLayout(button_layout)

        help_dialog.setLayout(help_layout)
        help_dialog.exec_()

    def apply_trajectory_type(self, dialog):
        # Сохранение выбранного типа траектории
        if self.line_radio.isChecked():
            self.trajectory_type = "Прямая"
            dialog.accept()
            self.show_line_inputs_dialog()  # Открытие окна с полями ввода для Прямой
        elif self.circle_radio.isChecked():
            self.trajectory_type = "Окружность"
            dialog.accept()
            self.show_circle_inputs_dialog()  # Открытие окна с полями ввода для Окружности
        else:
            dialog.reject()

    def show_line_inputs_dialog(self):
        # Создание диалогового окна для ввода параметров прямой
        line_dialog = QDialog(self)
        line_dialog.setWindowTitle("Введите параметры для Прямой")

        layout = QVBoxLayout()

        # Заголовок для диалогового окна
        title_label = QLabel("Параметры прямой")
        title_label.setStyleSheet("font-size: 18px; font-weight: bold;")
        layout.addWidget(title_label)

        # Создание сеточного макета для полей ввода
        grid_layout = QGridLayout()

        # Создание полей ввода с настройкой шрифта
        self.x1_input = QLineEdit()
        self.x1_input.setFont(QFont('Times', 14))  # Установка шрифта для поля ввода
        self.x2_input = QLineEdit()
        self.x2_input.setFont(QFont('Times', 14))
        self.y1_input = QLineEdit()
        self.y1_input.setFont(QFont('Times', 14))
        self.y2_input = QLineEdit()
        self.y2_input.setFont(QFont('Times', 14))
        self.speed_input = QLineEdit()
        self.speed_input.setFont(QFont('Times', 14))

        # Заполнение полей ввода текущими значениями
        self.x1_input.setText(str(self.line_params[0]))
        self.x2_input.setText(str(self.line_params[1]))
        self.y1_input.setText(str(self.line_params[2]))
        self.y2_input.setText(str(self.line_params[3]))
        self.speed_input.setText(str(self.line_params[4]))

        # Создание и настройка меток
        x1_label = QLabel("x1")
        x1_label.setFont(QFont('Times', 14))
        x2_label = QLabel("x2")
        x2_label.setFont(QFont('Times', 14))
        y1_label = QLabel("y1")
        y1_label.setFont(QFont('Times', 14))
        y2_label = QLabel("y2")
        y2_label.setFont(QFont('Times', 14))
        speed_label = QLabel("Скорость")
        speed_label.setFont(QFont('Times', 14))

        # Добавление полей ввода и меток в сетку
        grid_layout.addWidget(x1_label, 0, 0)
        grid_layout.addWidget(self.x1_input, 1, 0)
        grid_layout.addWidget(x2_label, 2, 0)
        grid_layout.addWidget(self.x2_input, 3, 0)

        grid_layout.addWidget(y1_label, 0, 1)
        grid_layout.addWidget(self.y1_input, 1, 1)
        grid_layout.addWidget(y2_label, 2, 1)
        grid_layout.addWidget(self.y2_input, 3, 1)

        grid_layout.addWidget(speed_label, 0, 3)
        grid_layout.addWidget(self.speed_input, 1,3)

        layout.addLayout(grid_layout)

        # Создание кнопок ОК, Показать и Помощь
        button_layout = QHBoxLayout()
        ok_button = QPushButton("ОК")
        ok_button.setStyleSheet("background-color : #18f22e")
        ok_button.setFont(QFont('Times', 14))
        show_button = QPushButton("Показать")
        show_button.setStyleSheet("background-color : yellow")
        show_button.setFont(QFont('Times', 14))

        ok_button.clicked.connect(lambda: self.apply_line_dialog(line_dialog))  # Обработка выбора параметров
        show_button.clicked.connect(self.show_line_parameters)  # Обработка показа введенных данных

        button_layout.addWidget(ok_button)
        button_layout.addWidget(show_button)

        layout.addLayout(button_layout)
        line_dialog.setLayout(layout)
        line_dialog.exec_()

    def apply_line_dialog(self, dialog):
        # Проверка введенных данных и сохранение при валидных значениях
        try:
            x1 = float(self.x1_input.text() or 0)
            x2 = float(self.x2_input.text() or 0)
            y1 = float(self.y1_input.text() or 0)
            y2 = float(self.y2_input.text() or 0)
            speed = float(self.speed_input.text() or 0)

            if not (0 <= x1 <= 10 and 0 <= x2 <= 10 and 0 <= y1 <= 10 and 0 <= y2 <= 10 and 0 <= speed <= 10):
                raise ValueError("Значение не в диапазоне от 0 до 10")

            self.line_params[0] = x1
            self.line_params[1] = x2
            self.line_params[2] = y1
            self.line_params[3] = y2
            self.line_params[4] = speed

            type_of_control = "Контурное"
            self.trajectory_calculator.set_line_params(x1, x2, y1, y2, speed,type_of_control)


            # Закрытие диалога
            dialog.accept()
        except ValueError as e:
            # Отображение сообщения об ошибке
            self.show_line_error_dialog(str(e))

    def show_line_error_dialog(self, message):
        # Создание диалогового окна с сообщением об ошибке для прямой
        error_dialog = QMessageBox(self)
        error_dialog.setWindowTitle("Ошибка")
        error_dialog.setIcon(QMessageBox.Critical)
        error_dialog.setText(message)
        error_dialog.setStandardButtons(QMessageBox.Ok)
        error_dialog.exec_()

    def show_line_parameters(self):
        # Вывод введенных данных в консоль
        params = {
            "x1": self.line_params[0],
            "x2": self.line_params[1],
            "y1": self.line_params[2],
            "y2": self.line_params[3],
            "Скорость": self.line_params[4]
        }
        print("Введенные параметры прямой:", params)

    def show_circle_inputs_dialog(self):
        # Создание диалогового окна для ввода параметров окружности
        circle_dialog = QDialog(self)
        circle_dialog.setWindowTitle("Введите параметры для Окружности")

        layout = QVBoxLayout()

        # Заголовок для диалогового окна
        title_label = QLabel("Параметры окружности")
        title_label.setStyleSheet("font-size: 20px; font-weight: bold;")
        layout.addWidget(title_label)

        # Создание сеточного макета для полей ввода
        grid_layout = QGridLayout()

        # Поля ввода
        self.x_input = QLineEdit()
        self.x_input.setFont(QFont('Times', 14))
        self.x_input.setText(str(self.circle_params[0]))

        self.y_input = QLineEdit()
        self.y_input.setFont(QFont('Times', 14))
        self.y_input.setText(str(self.circle_params[1]))

        self.radius_input = QLineEdit()
        self.radius_input.setFont(QFont('Times', 14))
        self.radius_input.setText(str(self.circle_params[2]))

        self.circle_speed_input = QLineEdit()
        self.circle_speed_input.setFont(QFont('Times', 14))
        self.circle_speed_input.setText(str(self.circle_params[3]))

        # Создание и настройка надписей
        x_label = QLabel("x")
        x_label.setFont(QFont('Times', 16))  # Увеличение размера текста
        y_label = QLabel("y")
        y_label.setFont(QFont('Times', 16))
        radius_label = QLabel("Радиус")
        radius_label.setFont(QFont('Times', 16))
        speed_label = QLabel("Скорость")
        speed_label.setFont(QFont('Times', 16))

        # Добавление полей ввода в сетку
        grid_layout.addWidget(x_label, 0, 0)
        grid_layout.addWidget(self.x_input, 1, 0)
        grid_layout.addWidget(y_label, 0, 1)
        grid_layout.addWidget(self.y_input, 1, 1)
        grid_layout.addWidget(radius_label, 0, 2)
        grid_layout.addWidget(self.radius_input, 1, 2)
        grid_layout.addWidget(speed_label, 0, 3)
        grid_layout.addWidget(self.circle_speed_input, 1, 3)

        layout.addLayout(grid_layout)

        # Создание кнопок ОК и Показать
        button_layout = QHBoxLayout()
        ok_button = QPushButton("ОК")
        ok_button.setStyleSheet("background-color : #18f22e")
        ok_button.setFont(QFont('Times', 14))
        show_button = QPushButton("Показать")
        show_button.setStyleSheet("background-color : yellow")
        show_button.setFont(QFont('Times', 14))

        ok_button.clicked.connect(lambda: self.apply_circle_dialog(circle_dialog))  # Обработка выбора параметров
        show_button.clicked.connect(self.show_circle_parameters)  # Обработка показа введенных данных

        button_layout.addWidget(ok_button)
        button_layout.addWidget(show_button)

        layout.addLayout(button_layout)
        circle_dialog.setLayout(layout)
        circle_dialog.exec_()

    def apply_circle_dialog(self, dialog):
        # Проверка введенных данных и сохранение при валидных значениях
        try:
            x = float(self.x_input.text() or 0)
            y = float(self.y_input.text() or 0)
            radius = float(self.radius_input.text() or 0)
            speed = float(self.circle_speed_input.text() or 0)

            if not (0 <= x <= 1000 and 0 <= y <= 1000 and 0 <= radius <= 1000 and 0 <= speed <= 1000):
                raise ValueError("Значение не в диапазоне от 0 до 1000")

            self.circle_params[0] = x
            self.circle_params[1] = y
            self.circle_params[2] = radius
            self.circle_params[3] = speed
            type_of_control = "Контурное"
            self.trajectory_calculator.set_circle_params(x,y,radius, speed, type_of_control)
            # Закрытие диалога
            dialog.accept()
        except ValueError as e:
            # Отображение сообщения об ошибке
            self.show_circle_error_dialog(str(e))

    def show_circle_error_dialog(self, message):
        # Создание диалогового окна с сообщением об ошибке для окружности
        error_dialog = QMessageBox(self)
        error_dialog.setWindowTitle("Ошибка")
        error_dialog.setIcon(QMessageBox.Critical)
        error_dialog.setText(message)
        error_dialog.setStandardButtons(QMessageBox.Ok)
        error_dialog.exec_()

    def show_circle_parameters(self):
        # Вывод введенных данных в консоль
        params = {
            "x": self.circle_params[0],
            "y": self.circle_params[1],
            "Радиус": self.circle_params[2],
            "Скорость": self.circle_params[3]
        }
        print("Введенные параметры окружности:", params)

    def show_graph_settings_dialog(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("Настройки графика")

        layout = QVBoxLayout()

        # Группа радиокнопок
        type_group = QGroupBox("Тип координат")
        type_group.setFont(QFont('Times', 14))
        type_layout = QVBoxLayout()

        # Основные
        self.general_radio = QRadioButton("Обобщённые от времени")
        self.plane_radio = QRadioButton("Декартовы на плоскости")

        # Дополнительные
        additional_label = QLabel("Дополнительные")
        additional_label.setFont(QFont('Times', 12))

        self.time_radio = QRadioButton("Декартовы от времени")
        self.speed_radio = QRadioButton("Обобщённые скорости")
        self.acceleration_radio = QRadioButton("Обобщённые ускорения")

        # Внутренние
        internal_label = QLabel("Внутренние")
        internal_label.setFont(QFont('Times', 12))

        self.voltage_radio = QRadioButton("Напряжение")
        self.voltage_star_radio = QRadioButton("Напряжение* (Напряжение - ЭДС)")
        self.current_radio = QRadioButton("Ток")
        self.motor_moment_radio = QRadioButton("Момент электродвижущий")
        self.load_moment_radio = QRadioButton("Момент нагрузки")
        self.moment_star_radio = QRadioButton("Момент* (МЭ - М нагрузки)")

        self.radio_group = QButtonGroup()
        for rb in [
            self.general_radio, self.plane_radio,
            self.time_radio, self.speed_radio, self.acceleration_radio,
            self.voltage_radio, self.voltage_star_radio, self.current_radio,
            self.motor_moment_radio, self.load_moment_radio, self.moment_star_radio
        ]:
            self.radio_group.addButton(rb)

        # Установка по умолчанию
        self.general_radio.setChecked(True)

        # Добавление виджетов
        type_layout.addWidget(self.general_radio)
        type_layout.addWidget(self.plane_radio)

        type_layout.addWidget(additional_label)
        type_layout.addWidget(self.time_radio)
        type_layout.addWidget(self.speed_radio)
        type_layout.addWidget(self.acceleration_radio)

        type_layout.addWidget(internal_label)
        type_layout.addWidget(self.voltage_radio)
        type_layout.addWidget(self.voltage_star_radio)
        type_layout.addWidget(self.current_radio)
        type_layout.addWidget(self.motor_moment_radio)
        type_layout.addWidget(self.load_moment_radio)
        type_layout.addWidget(self.moment_star_radio)

        type_group.setLayout(type_layout)
        layout.addWidget(type_group)

        # Сплайн
        spline_group = QGroupBox("Отображение сплайна")
        spline_group.setFont(QFont('Times', 14))
        spline_layout = QVBoxLayout()
        self.spline_checkbox = QCheckBox("Показать сплайн")
        spline_layout.addWidget(self.spline_checkbox)
        spline_group.setLayout(spline_layout)
        layout.addWidget(spline_group)

        # Кнопки
        ok_button = QPushButton("ОК")
        ok_button.setStyleSheet("background-color: #18f22e")
        ok_button.setFont(QFont('Times', 14))
        ok_button.clicked.connect(lambda: self.apply_graph_settings(dialog))

        cancel_button = QPushButton("Отмена")
        cancel_button.setStyleSheet("background-color: red")
        cancel_button.setFont(QFont('Times', 14))
        cancel_button.clicked.connect(dialog.reject)

        help_button = QPushButton("Помощь")
        help_button.setStyleSheet("background-color : #87CEEB")
        help_button.setFont(QFont('Times', 14))
        help_button.clicked.connect(self.show_graph_settings_help_dialog)

        layout.addWidget(ok_button)
        layout.addWidget(help_button)
        layout.addWidget(cancel_button)

        dialog.setLayout(layout)
        dialog.exec_()

    def show_graph_settings_help_dialog(self):

        help_dialog = QDialog(self)
        help_dialog.setWindowTitle("Подсказка")
        help_layout = QVBoxLayout()

        # Основной текст подсказки с выделением
        help_text = QLabel(self)
        help_text.setFont(QFont('Times', 12))

        help_text.setText("""В данном режиме необходимо выбрать тип и количество  <br>
                            выводимых переменных и запустить расчёт. <br>""")

        help_layout.addWidget(help_text)

        # Кнопки для открытия дополнительных подсказок
        button_layout = QHBoxLayout()

        cancel_button = QPushButton("Закрыть")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        cancel_button.clicked.connect(help_dialog.reject)
        button_layout.addWidget(cancel_button)

        help_layout.addLayout(button_layout)

        help_dialog.setLayout(help_layout)
        help_dialog.exec_()

    def apply_graph_settings(self, dialog):
        try:
            # Определяем тип робота на основе переменной self.robot_type
            if self.robot_type == "Декартовый":
                self.trajectory_calculator.set_robot_type("Декартовый")
            elif self.robot_type == "Скара":
                self.trajectory_calculator.set_robot_type("Скара")
            elif self.robot_type == "Цилиндрический":
                self.trajectory_calculator.set_robot_type("Цилиндрический")
            elif self.robot_type == "Колер":
                self.trajectory_calculator.set_robot_type("Колер")
            else:
                print("Не выбран тип робота.")
                dialog.reject()
                return

            self.spline = self.spline_checkbox.isChecked()
            self.trajectory_calculator.set_spline(self.spline)

            # Определяем выбранный тип координат с помощью радиокнопок
            if self.general_radio.isChecked():
                self.selected_coordinate_type = "Обобщённые"
                self.trajectory_calculator.draw_obobshennie_coordinates_from_time()
            elif self.plane_radio.isChecked():
                self.selected_coordinate_type = "Декартовы на плоскости"
                self.trajectory_calculator.draw_on_decart_plane()
            elif self.time_radio.isChecked():
                self.selected_coordinate_type = "Декартовы от времени"
                self.trajectory_calculator.draw_decart_coordinates_from_time()
            elif self.speed_radio.isChecked():
                self.selected_coordinate_type = "Обобщённые скорости"
                self.trajectory_calculator.draw_obobshennie_speed_from_time()
            elif self.acceleration_radio.isChecked():
                self.selected_coordinate_type = "Обобщённые ускорения"
                self.trajectory_calculator.draw_obobshennie_acceleration_from_time()
            elif self.voltage_radio.isChecked():
                self.selected_coordinate_type = "Напряжение"
                self.trajectory_calculator.draw_voltage_from_time()
            elif self.voltage_star_radio.isChecked():
                self.selected_coordinate_type = "Напряжение* (Напряжение - ЭДС)"
                self.trajectory_calculator.draw_voltage_star_from_time()
            elif self.current_radio.isChecked():
                self.selected_coordinate_type = "Ток"
                self.trajectory_calculator.draw_current_from_time()
            elif self.motor_moment_radio.isChecked():
                self.selected_coordinate_type = "Момент электродвижущий"
                self.trajectory_calculator.draw_motor_moment_from_time()
            elif self.load_moment_radio.isChecked():
                self.selected_coordinate_type = "Момент нагрузки"
                self.trajectory_calculator.draw_load_moment_from_time()
            elif self.moment_star_radio.isChecked():
                self.selected_coordinate_type = "Момент* (МЭ - М нагрузки)"
                self.trajectory_calculator.draw_moment_star_from_time()


            # Формируем текст для вывода в окно
            results_text = (
                "Первое звено робота:\n"
                f"Ошибки: {self.trajectory_calculator.error_1}\n"
                f"Средняя ошибка: {self.trajectory_calculator.avg_error_1}\n"
                f"Медианная ошибка: {self.trajectory_calculator.median_error_1}\n"
                f"Время регулирования: {self.trajectory_calculator.reg_time_1}\n"
                f"Среднее время регулирования: {self.trajectory_calculator.avg_reg_time_1}\n"
                f"Медианное время регулирования: {self.trajectory_calculator.median_reg_time_1}\n\n"

                "Второе звено робота:\n"
                f"Ошибки: {self.trajectory_calculator.error_2}\n"
                f"Средняя ошибка: {self.trajectory_calculator.avg_error_2}\n"
                f"Медианная ошибка: {self.trajectory_calculator.median_error_2}\n"
                f"Время регулирования: {self.trajectory_calculator.reg_time_2}\n"
                f"Среднее время регулирования: {self.trajectory_calculator.avg_reg_time_2}\n"
                f"Медианное время регулирования: {self.trajectory_calculator.median_reg_time_2}\n\n"
            )

            if self.spline:
                results_text += (
                    "Первое звено робота (сплайн):\n"
                    f"Ошибки: {self.trajectory_calculator.spline_error_1}\n"
                    f"Средняя ошибка: {self.trajectory_calculator.spline_avg_error_1}\n"
                    f"Медианная ошибка: {self.trajectory_calculator.spline_median_error_1}\n"
                    f"Время регулирования: {self.trajectory_calculator.spline_reg_time_1}\n"
                    f"Среднее время регулирования: {self.trajectory_calculator.spline_avg_reg_time_1}\n"
                    f"Медианное время регулирования: {self.trajectory_calculator.spline_median_reg_time_1}\n\n"

                    "Второе звено робота (сплайн):\n"
                    f"Ошибки: {self.trajectory_calculator.spline_error_2}\n"
                    f"Средняя ошибка: {self.trajectory_calculator.spline_avg_error_2}\n"
                    f"Медианная ошибка: {self.trajectory_calculator.spline_median_error_2}\n"
                    f"Время регулирования: {self.trajectory_calculator.spline_reg_time_2}\n"
                    f"Среднее время регулирования: {self.trajectory_calculator.spline_avg_reg_time_2}\n"
                    f"Медианное время регулирования: {self.trajectory_calculator.spline_median_reg_time_2}\n"
                )

            # Окно с ошибками
            self.show_errors_window(results_text)

            # Завершаем диалог
            dialog.accept()

        except Exception as e:
            print(f"Произошла ошибка: {e}")
            dialog.reject()

    def show_errors_window(self, text):
        self.errors_window = QWidget()
        self.errors_window.setWindowTitle("Результаты расчёта")
        self.errors_window.setGeometry(300, 300, 600, 400)

        layout = QVBoxLayout()

        text_edit = QTextEdit()
        text_edit.setText(text)
        text_edit.setReadOnly(True)

        font = QFont('Courier', 12)
        text_edit.setFont(font)
        layout.addWidget(text_edit)

        close_button = QPushButton("Закрыть")
        close_button.setStyleSheet("background-color : red")
        close_button.setFont(QFont('Times', 14))
        close_button.clicked.connect(self.errors_window.close)
        layout.addWidget(close_button)

        self.errors_window.setLayout(layout)
        self.errors_window.show()

    def draw_workspace(self):
        # Сохраняем параметры в WorkspaceCalculator перед отрисовкой

        # Устанавливаем тип робота
        self.workspace_calculator.set_robot_type(self.robot_type)

        # Вызываем отрисовку рабочей области на основе выбранного типа робота.
        self.workspace_calculator.draw_workspace()

    def load_data(self):
        # Открываем диалог для загрузки файла
        file_name, _ = QFileDialog.getOpenFileName(self, "Загрузить данные", "", "Text Files (*.txt);;All Files (*)")

        if file_name:
            try:
                with open(file_name, 'r') as file:
                    lines = file.readlines()

                    # Записываем тип робота
                    self.robot_type = lines[0].strip()

                    # Обрабатываем каждую строку из файла
                    self.massd_1 = float(lines[1].strip())
                    self.massd_2 = float(lines[2].strip())
                    self.massd_3 = float(lines[3].strip())
                    self.momentd_1 = float(lines[4].strip())

                    self.x_min = float(lines[5].strip())
                    self.y_min = float(lines[6].strip())
                    self.q_min = float(lines[7].strip())
                    self.z_min = float(lines[8].strip())
                    self.x_max = float(lines[9].strip())
                    self.y_max = float(lines[10].strip())
                    self.q_max = float(lines[11].strip())
                    self.z_max = float(lines[12].strip())

                    self.moment_1 = float(lines[13].strip())
                    self.moment_2 = float(lines[14].strip())
                    self.moment_3 = float(lines[15].strip())
                    self.length_1 = float(lines[16].strip())
                    self.length_2 = float(lines[17].strip())
                    self.distance = float(lines[18].strip())
                    self.masss_2 = float(lines[19].strip())
                    self.masss_3 = float(lines[20].strip())

                    self.q1s_min = float(lines[21].strip())
                    self.q1s_max = float(lines[22].strip())
                    self.q2s_min = float(lines[23].strip())
                    self.q2s_max = float(lines[24].strip())
                    self.q3s_min = float(lines[25].strip())
                    self.q3s_max = float(lines[26].strip())
                    self.zs_min = float(lines[27].strip())
                    self.zs_max = float(lines[28].strip())

                    self.momentc_1 = float(lines[29].strip())
                    self.momentc_2 = float(lines[30].strip())
                    self.momentc_3 = float(lines[31].strip())
                    self.lengthc_1 = float(lines[32].strip())
                    self.lengthc_2 = float(lines[33].strip())
                    self.distancec = float(lines[34].strip())
                    self.massc_2 = float(lines[35].strip())
                    self.massc_3 = float(lines[36].strip())

                    self.q1c_min = float(lines[37].strip())
                    self.q1c_max = float(lines[38].strip())
                    self.a2c_min = float(lines[39].strip())
                    self.a2c_max = float(lines[40].strip())
                    self.q3c_min = float(lines[41].strip())
                    self.q3c_max = float(lines[42].strip())
                    self.zc_min = float(lines[43].strip())
                    self.zc_max = float(lines[44].strip())

                    self.momentcol_1 = float(lines[45].strip())
                    self.momentcol_2 = float(lines[46].strip())
                    self.momentcol_3 = float(lines[47].strip())
                    self.lengthcol_1 = float(lines[48].strip())
                    self.lengthcol_2 = float(lines[49].strip())
                    self.distancecol = float(lines[50].strip())
                    self.masscol_2 = float(lines[51].strip())
                    self.masscol_3 = float(lines[52].strip())

                    self.q1col_min = float(lines[53].strip())
                    self.q1col_max = float(lines[54].strip())
                    self.a2col_min = float(lines[55].strip())
                    self.a2col_max = float(lines[56].strip())
                    self.q3col_min = float(lines[57].strip())
                    self.q3col_max = float(lines[58].strip())
                    self.zcol_min = float(lines[59].strip())
                    self.zcol_max = float(lines[60].strip())

                    self.J = list(map(float, lines[61].strip().split(',')))
                    self.n = list(map(float, lines[62].strip().split(',')))
                    self.Umax = list(map(float, lines[63].strip().split(',')))
                    self.Ku = list(map(float, lines[64].strip().split(',')))
                    self.Kq = list(map(float, lines[65].strip().split(',')))

                    self.Kp = list(map(float, lines[66].strip().split(',')))
                    self.Ki = list(map(float, lines[67].strip().split(',')))
                    self.Kd = list(map(float, lines[68].strip().split(',')))

                    self.t = list(map(float, lines[69].strip().split(',')))
                    self.q1 = list(map(float, lines[70].strip().split(',')))
                    self.q2 = list(map(float, lines[71].strip().split(',')))
                    self.q3 = list(map(float, lines[72].strip().split(',')))
                    self.q4 = list(map(float, lines[73].strip().split(',')))

                    self.trajectory_type = lines[74].strip()

                    self.workspace_calculator.set_robot_type(
                        self.robot_type
                    )

                    self.trajectory_calculator.set_robot_type(
                        self.robot_type
                    )

                    self.line_params = list(map(float, lines[75].strip().split(',')))
                    self.circle_params = list(map(float, lines[76].strip().split(',')))

                    self.workspace_calculator.set_scara_limits(
                        self.q1s_min, self.q1s_max,
                        self.q2s_min, self.q2s_max,
                        self.q3s_min, self.q3s_max,
                        self.zs_min, self.zs_max
                    )
                    self.workspace_calculator.set_cylindrical_limits(
                        self.q1c_min, self.q1c_max,
                        self.a2c_min, self.a2c_max,
                        self.q3c_min, self.q3c_max,
                        self.zc_min, self.zc_max
                    )
                    self.workspace_calculator.set_coler_limits(
                        self.q1col_min, self.q1col_max,
                        self.a2col_min, self.a2col_max,
                        self.q3col_min, self.q3col_max,
                        self.zcol_min, self.zcol_max
                    )

                    self.workspace_calculator.set_coler_params(
                        self.momentcol_1, self.momentcol_2,
                        self.momentcol_3, self.lengthcol_1,
                        self.lengthcol_2, self.distancecol,
                        self.masscol_2, self.masscol_3
                    )

                    self.trajectory_calculator.set_cyclogram(
                        self.t, self.q1,
                        self.q2, self.q3,
                        self.q4
                    )

                    self.trajectory_calculator.set_cartesian_limits(
                        self.x_min,
                        self.x_max,
                        self.y_min,
                        self.y_max,
                        self.z_min,
                        self.z_max,
                        self.q_min,
                        self.q_max
                    )

                    self.trajectory_calculator.set_cartesian_params(
                        self.massd_1, self.massd_2,
                        self.massd_3, self.momentd_1
                    )

                    self.trajectory_calculator.set_scara_limits(
                        self.q1s_min, self.q1s_max,
                        self.q2s_min, self.q2s_max,
                        self.q3s_min, self.q3s_max,
                        self.zs_min, self.zs_max
                    )
                    self.trajectory_calculator.set_cylindrical_limits(
                        self.q1c_min, self.q1c_max,
                        self.a2c_min, self.a2c_max,
                        self.q3c_min, self.q3c_max,
                        self.zc_min, self.zc_max
                    )
                    self.trajectory_calculator.set_scara_params(
                        self.moment_1, self.moment_2,
                        self.moment_3, self.length_1,
                        self.length_2, self.distance,
                        self.masss_2, self.masss_3
                    )

                    self.trajectory_calculator.set_cylindrical_params(
                        self.momentc_1, self.momentc_2,
                        self.momentc_3, self.lengthc_1,
                        self.lengthc_2, self.distancec,
                        self.massc_2, self.massc_3
                    )

                    self.trajectory_calculator.set_coler_limits(
                        self.q1col_min, self.q1col_max,
                        self.a2col_min, self.a2col_max,
                        self.q3col_min, self.q3col_max,
                        self.zcol_min, self.zcol_max
                    )

                    self.trajectory_calculator.set_coler_params(
                        self.momentcol_1, self.momentcol_2,
                        self.momentcol_3, self.lengthcol_1,
                        self.lengthcol_2, self.distancecol,
                        self.masscol_2, self.masscol_3
                    )
                    #Добавлять сюда

                    self.trajectory_calculator.PID_regulator(
                        self.Kp,
                        self.Ki,
                        self.Kd
                    )

                    self.trajectory_calculator.get_motor_params(
                        self.J,
                        self.n,
                        self.Umax,
                        self.Ku,
                        self.Kq
                    )


                    self.current_file_path = file_name  # Сохраняем путь к файлу
                    # Сохраняем путь к последнему открытому файлу
                    with open("last_file.txt", "w") as f:
                        f.write(self.current_file_path)
                    QMessageBox.information(self, "Успех", "Данные успешно загружены!")
            except Exception as e:
                QMessageBox.critical(self, "Ошибка", f"Ошибка при загрузке данных: {e}")

    def load_data_from_path(self, file_name):
        try:
            with open(file_name, 'r') as file:
                lines = file.readlines()

                self.robot_type = lines[0].strip()

                # Обрабатываем каждую строку из файла
                self.massd_1 = float(lines[1].strip())
                self.massd_2 = float(lines[2].strip())
                self.massd_3 = float(lines[3].strip())
                self.momentd_1 = float(lines[4].strip())

                self.x_min = float(lines[5].strip())
                self.y_min = float(lines[6].strip())
                self.q_min = float(lines[7].strip())
                self.z_min = float(lines[8].strip())
                self.x_max = float(lines[9].strip())
                self.y_max = float(lines[10].strip())
                self.q_max = float(lines[11].strip())
                self.z_max = float(lines[12].strip())

                self.moment_1 = float(lines[13].strip())
                self.moment_2 = float(lines[14].strip())
                self.moment_3 = float(lines[15].strip())
                self.length_1 = float(lines[16].strip())
                self.length_2 = float(lines[17].strip())
                self.distance = float(lines[18].strip())
                self.masss_2 = float(lines[19].strip())
                self.masss_3 = float(lines[20].strip())

                self.q1s_min = float(lines[21].strip())
                self.q1s_max = float(lines[22].strip())
                self.q2s_min = float(lines[23].strip())
                self.q2s_max = float(lines[24].strip())
                self.q3s_min = float(lines[25].strip())
                self.q3s_max = float(lines[26].strip())
                self.zs_min = float(lines[27].strip())
                self.zs_max = float(lines[28].strip())

                self.momentc_1 = float(lines[29].strip())
                self.momentc_2 = float(lines[30].strip())
                self.momentc_3 = float(lines[31].strip())
                self.lengthc_1 = float(lines[32].strip())
                self.lengthc_2 = float(lines[33].strip())
                self.distancec = float(lines[34].strip())
                self.massc_2 = float(lines[35].strip())
                self.massc_3 = float(lines[36].strip())

                self.q1c_min = float(lines[37].strip())
                self.q1c_max = float(lines[38].strip())
                self.a2c_min = float(lines[39].strip())
                self.a2c_max = float(lines[40].strip())
                self.q3c_min = float(lines[41].strip())
                self.q3c_max = float(lines[42].strip())
                self.zc_min = float(lines[43].strip())
                self.zc_max = float(lines[44].strip())

                self.momentcol_1 = float(lines[45].strip())
                self.momentcol_2 = float(lines[46].strip())
                self.momentcol_3 = float(lines[47].strip())
                self.lengthcol_1 = float(lines[48].strip())
                self.lengthcol_2 = float(lines[49].strip())
                self.distancecol = float(lines[50].strip())
                self.masscol_2 = float(lines[51].strip())
                self.masscol_3 = float(lines[52].strip())

                self.q1col_min = float(lines[53].strip())
                self.q1col_max = float(lines[54].strip())
                self.a2col_min = float(lines[55].strip())
                self.a2col_max = float(lines[56].strip())
                self.q3col_min = float(lines[57].strip())
                self.q3col_max = float(lines[58].strip())
                self.zcol_min = float(lines[59].strip())
                self.zcol_max = float(lines[60].strip())

                self.J = list(map(float, lines[61].strip().split(',')))
                self.n = list(map(float, lines[62].strip().split(',')))
                self.Umax = list(map(float, lines[63].strip().split(',')))
                self.Ku = list(map(float, lines[64].strip().split(',')))
                self.Kq = list(map(float, lines[65].strip().split(',')))

                self.Kp = list(map(float, lines[66].strip().split(',')))
                self.Ki = list(map(float, lines[67].strip().split(',')))
                self.Kd = list(map(float, lines[68].strip().split(',')))

                self.t = list(map(float, lines[69].strip().split(',')))
                self.q1 = list(map(float, lines[70].strip().split(',')))
                self.q2 = list(map(float, lines[71].strip().split(',')))
                self.q3 = list(map(float, lines[72].strip().split(',')))
                self.q4 = list(map(float, lines[73].strip().split(',')))

                self.trajectory_type = lines[74].strip()

                self.workspace_calculator.set_robot_type(
                    self.robot_type
                )

                self.trajectory_calculator.set_robot_type(
                    self.robot_type
                )

                self.line_params = list(map(float, lines[75].strip().split(',')))
                self.circle_params = list(map(float, lines[76].strip().split(',')))

                self.workspace_calculator.set_scara_limits(
                    self.q1s_min, self.q1s_max,
                    self.q2s_min, self.q2s_max,
                    self.q3s_min, self.q3s_max,
                    self.zs_min, self.zs_max
                )
                self.workspace_calculator.set_cylindrical_limits(
                    self.q1c_min, self.q1c_max,
                    self.a2c_min, self.a2c_max,
                    self.q3c_min, self.q3c_max,
                    self.zc_min, self.zc_max
                )
                self.workspace_calculator.set_coler_limits(
                    self.q1col_min, self.q1col_max,
                    self.a2col_min, self.a2col_max,
                    self.q3col_min, self.q3col_max,
                    self.zcol_min, self.zcol_max
                )

                self.workspace_calculator.set_coler_params(
                    self.momentcol_1, self.momentcol_2,
                    self.momentcol_3, self.lengthcol_1,
                    self.lengthcol_2, self.distancecol,
                    self.masscol_2, self.masscol_3
                )

                self.trajectory_calculator.set_cyclogram(
                    self.t, self.q1,
                    self.q2, self.q3,
                    self.q4
                )

                self.trajectory_calculator.set_cartesian_limits(
                    self.x_min,
                    self.x_max,
                    self.y_min,
                    self.y_max,
                    self.z_min,
                    self.z_max,
                    self.q_min,
                    self.q_max
                )

                self.trajectory_calculator.set_cartesian_params(
                    self.massd_1, self.massd_2,
                    self.massd_3, self.momentd_1
                )

                self.trajectory_calculator.set_scara_limits(
                    self.q1s_min, self.q1s_max,
                    self.q2s_min, self.q2s_max,
                    self.q3s_min, self.q3s_max,
                    self.zs_min, self.zs_max
                )
                self.trajectory_calculator.set_cylindrical_limits(
                    self.q1c_min, self.q1c_max,
                    self.a2c_min, self.a2c_max,
                    self.q3c_min, self.q3c_max,
                    self.zc_min, self.zc_max
                )
                self.trajectory_calculator.set_scara_params(
                    self.moment_1, self.moment_2,
                    self.moment_3, self.length_1,
                    self.length_2, self.distance,
                    self.masss_2, self.masss_3
                )

                self.trajectory_calculator.set_cylindrical_params(
                    self.momentc_1, self.momentc_2,
                    self.momentc_3, self.lengthc_1,
                    self.lengthc_2, self.distancec,
                    self.massc_2, self.massc_3
                )

                self.trajectory_calculator.set_coler_limits(
                    self.q1col_min, self.q1col_max,
                    self.a2col_min, self.a2col_max,
                    self.q3col_min, self.q3col_max,
                    self.zcol_min, self.zcol_max
                )

                self.trajectory_calculator.set_coler_params(
                    self.momentcol_1, self.momentcol_2,
                    self.momentcol_3, self.lengthcol_1,
                    self.lengthcol_2, self.distancecol,
                    self.masscol_2, self.masscol_3
                )
                #Добавлять сюда

                self.trajectory_calculator.PID_regulator(
                    self.Kp,
                    self.Ki,
                    self.Kd
                )

                self.trajectory_calculator.get_motor_params(
                    self.J,
                    self.n,
                    self.Umax,
                    self.Ku,
                    self.Kq
                )

                self.current_file_path = file_name
                with open("last_file.txt", "w") as f:
                    f.write(file_name)
                QMessageBox.information(self, "Успех", "Данные успешно загружены!")
        except Exception as e:
            QMessageBox.critical(self, "Ошибка", f"Ошибка при загрузке данных: {e}")

    def save_data(self):
        # Проверяем, если файл уже был сохранен
        if self.current_file_path:
            try:
                with open(self.current_file_path, 'w') as file:

                    #Записываем тип робота
                    file.write(f"{self.robot_type}\n")

                    # Записываем параметры декартового робота
                    file.write(f"{self.massd_1}\n")
                    file.write(f"{self.massd_2}\n")
                    file.write(f"{self.massd_3}\n")
                    file.write(f"{self.momentd_1}\n")

                    # Записываем ограничения по координатам
                    file.write(f"{self.x_min}\n")
                    file.write(f"{self.y_min}\n")
                    file.write(f"{self.q_min}\n")
                    file.write(f"{self.z_min}\n")
                    file.write(f"{self.x_max}\n")
                    file.write(f"{self.y_max}\n")
                    file.write(f"{self.q_max}\n")
                    file.write(f"{self.z_max}\n")

                    # Записываем параметры SCARA
                    file.write(f"{self.moment_1}\n")
                    file.write(f"{self.moment_2}\n")
                    file.write(f"{self.moment_3}\n")
                    file.write(f"{self.length_1}\n")
                    file.write(f"{self.length_2}\n")
                    file.write(f"{self.distance}\n")
                    file.write(f"{self.masss_2}\n")
                    file.write(f"{self.masss_3}\n")

                    # Записываем ограничения по координатам SCARA
                    file.write(f"{self.q1s_min}\n")
                    file.write(f"{self.q1s_max}\n")
                    file.write(f"{self.q2s_min}\n")
                    file.write(f"{self.q2s_max}\n")
                    file.write(f"{self.q3s_min}\n")
                    file.write(f"{self.q3s_max}\n")
                    file.write(f"{self.zs_min}\n")
                    file.write(f"{self.zs_max}\n")

                    # Записываем параметры Цилиндрического робота
                    file.write(f"{self.momentc_1}\n")
                    file.write(f"{self.momentc_2}\n")
                    file.write(f"{self.momentc_3}\n")
                    file.write(f"{self.lengthc_1}\n")
                    file.write(f"{self.lengthc_2}\n")
                    file.write(f"{self.distancec}\n")
                    file.write(f"{self.massc_2}\n")
                    file.write(f"{self.massc_3}\n")

                    # Записываем ограничения по координатам Цилиндрического робота
                    file.write(f"{self.q1c_min}\n")
                    file.write(f"{self.q1c_max}\n")
                    file.write(f"{self.a2c_min}\n")
                    file.write(f"{self.a2c_max}\n")
                    file.write(f"{self.q3c_min}\n")
                    file.write(f"{self.q3c_max}\n")
                    file.write(f"{self.zc_min}\n")
                    file.write(f"{self.zc_max}\n")

                    # Записываем параметры робота Колер
                    file.write(f"{self.momentcol_1}\n")
                    file.write(f"{self.momentcol_2}\n")
                    file.write(f"{self.momentcol_3}\n")
                    file.write(f"{self.lengthcol_1}\n")
                    file.write(f"{self.lengthcol_2}\n")
                    file.write(f"{self.distancecol}\n")
                    file.write(f"{self.masscol_2}\n")
                    file.write(f"{self.masscol_3}\n")

                    # Записываем ограничения по координатам робота Колер
                    file.write(f"{self.q1col_min}\n")
                    file.write(f"{self.q1col_max}\n")
                    file.write(f"{self.a2col_min}\n")
                    file.write(f"{self.a2col_max}\n")
                    file.write(f"{self.q3col_min}\n")
                    file.write(f"{self.q3col_max}\n")
                    file.write(f"{self.zcol_min}\n")
                    file.write(f"{self.zcol_max}\n")

                    # Записываем параметры двигателей
                    file.write(','.join(map(str, self.J)) + '\n')
                    file.write(','.join(map(str, self.n)) + '\n')
                    file.write(','.join(map(str, self.Umax)) + '\n')
                    file.write(','.join(map(str, self.Ku)) + '\n')
                    file.write(','.join(map(str, self.Kq)) + '\n')

                    # Записываем параметры регуляторов
                    file.write(','.join(map(str, self.Kp)) + '\n')
                    file.write(','.join(map(str, self.Ki)) + '\n')
                    file.write(','.join(map(str, self.Kd)) + '\n')

                    # Записываем циклические параметры
                    file.write(','.join(map(str, self.t)) + '\n')
                    file.write(','.join(map(str, self.q1)) + '\n')
                    file.write(','.join(map(str, self.q2)) + '\n')
                    file.write(','.join(map(str, self.q3)) + '\n')
                    file.write(','.join(map(str, self.q4)) + '\n')

                    # Записываем параметры траектории
                    file.write(f"{self.trajectory_type}\n")

                    # Записываем параметры прямой
                    file.write(','.join(map(str, self.line_params)) + '\n')

                    # Записываем параметры окружности
                    file.write(','.join(map(str, self.circle_params)) + '\n')

                QMessageBox.information(self, "Успех", "Данные успешно сохранены!")
            except Exception as e:
                QMessageBox.critical(self, "Ошибка", f"Ошибка при сохранении данных: {e}")
        else:
            # Если файл не был сохранен, вызываем "Сохранить как"
            self.save_data_as()

    def save_data_as(self):
        file_name, _ = QFileDialog.getSaveFileName(self, "Сохранить данные как", "",
                                                   "Text Files (*.txt);;All Files (*)")
        print(f"Выбранное имя файла: {file_name}")  # Отладочное сообщение

        if file_name:
            base_name = os.path.basename(file_name)
            print(f"Базовое имя файла: {base_name}")  # Отладочное сообщение

            # Обновите имя видеофайла
            video_path = os.path.join(os.path.dirname(__file__), "Natura.mp4")
            print(f"Путь к видеофайлу: {video_path}")  # Отладочное сообщение

            # Проверяем, совпадает ли имя файла с одним из нужных
            if base_name == "228.txt" or base_name == "stariy_bog.txt":
                # Проверяем существует ли файл перед запуском
                if not os.path.isfile(video_path):
                    print(f"Файл не найден: {os.path.isfile(video_path)}")  # Отладочное сообщение
                    QMessageBox.critical(self, "Ошибка", "Видеофайл не найден!")
                    return

                # Устанавливаем системный звук на 50%
                self.set_system_volume(50)

                # Запускаем видеофайл
                try:
                    print("Попытка запустить видео.")
                    os.startfile(video_path)
                    print("Видео должно запуститься.")
                except Exception as e:
                    QMessageBox.critical(self, "Ошибка", f"Ошибка при запуске видео: {e}")

            try:
                with open(file_name, 'w') as file:

                    #Сохраняем тип робота
                    file.write(f"{self.robot_type}\n")

                    # Записываем параметры декартового робота
                    file.write(f"{self.massd_1}\n")
                    file.write(f"{self.massd_2}\n")
                    file.write(f"{self.massd_3}\n")
                    file.write(f"{self.momentd_1}\n")

                    # Записываем ограничения по координатам
                    file.write(f"{self.x_min}\n")
                    file.write(f"{self.y_min}\n")
                    file.write(f"{self.q_min}\n")
                    file.write(f"{self.z_min}\n")
                    file.write(f"{self.x_max}\n")
                    file.write(f"{self.y_max}\n")
                    file.write(f"{self.q_max}\n")
                    file.write(f"{self.z_max}\n")

                    # Записываем параметры SCARA
                    file.write(f"{self.moment_1}\n")
                    file.write(f"{self.moment_2}\n")
                    file.write(f"{self.moment_3}\n")
                    file.write(f"{self.length_1}\n")
                    file.write(f"{self.length_2}\n")
                    file.write(f"{self.distance}\n")
                    file.write(f"{self.masss_2}\n")
                    file.write(f"{self.masss_3}\n")

                    # Записываем ограничения по координатам SCARA
                    file.write(f"{self.q1s_min}\n")
                    file.write(f"{self.q1s_max}\n")
                    file.write(f"{self.q2s_min}\n")
                    file.write(f"{self.q2s_max}\n")
                    file.write(f"{self.q3s_min}\n")
                    file.write(f"{self.q3s_max}\n")
                    file.write(f"{self.zs_min}\n")
                    file.write(f"{self.zs_max}\n")

                    # Записываем параметры Цилиндрического робота
                    file.write(f"{self.momentc_1}\n")
                    file.write(f"{self.momentc_2}\n")
                    file.write(f"{self.momentc_3}\n")
                    file.write(f"{self.lengthc_1}\n")
                    file.write(f"{self.lengthc_2}\n")
                    file.write(f"{self.distancec}\n")
                    file.write(f"{self.massc_2}\n")
                    file.write(f"{self.massc_3}\n")

                    # Записываем ограничения по координатам Цилиндрического робота
                    file.write(f"{self.q1c_min}\n")
                    file.write(f"{self.q1c_max}\n")
                    file.write(f"{self.a2c_min}\n")
                    file.write(f"{self.a2c_max}\n")
                    file.write(f"{self.q3c_min}\n")
                    file.write(f"{self.q3c_max}\n")
                    file.write(f"{self.zc_min}\n")
                    file.write(f"{self.zc_max}\n")

                    # Записываем параметры робота Колер
                    file.write(f"{self.momentcol_1}\n")
                    file.write(f"{self.momentcol_2}\n")
                    file.write(f"{self.momentcol_3}\n")
                    file.write(f"{self.lengthcol_1}\n")
                    file.write(f"{self.lengthcol_2}\n")
                    file.write(f"{self.distancecol}\n")
                    file.write(f"{self.masscol_2}\n")
                    file.write(f"{self.masscol_3}\n")

                    # Записываем ограничения по координатам робота Колер
                    file.write(f"{self.q1c_min}\n")
                    file.write(f"{self.q1c_max}\n")
                    file.write(f"{self.a2c_min}\n")
                    file.write(f"{self.a2c_max}\n")
                    file.write(f"{self.q3c_min}\n")
                    file.write(f"{self.q3c_max}\n")
                    file.write(f"{self.zc_min}\n")
                    file.write(f"{self.zc_max}\n")

                    # Записываем параметры двигателей
                    file.write(','.join(map(str, self.J)) + '\n')
                    file.write(','.join(map(str, self.n)) + '\n')
                    file.write(','.join(map(str, self.Umax)) + '\n')
                    file.write(','.join(map(str, self.Ku)) + '\n')
                    file.write(','.join(map(str, self.Kq)) + '\n')

                    # Записываем параметры регуляторов
                    file.write(','.join(map(str, self.Kp)) + '\n')
                    file.write(','.join(map(str, self.Ki)) + '\n')
                    file.write(','.join(map(str, self.Kd)) + '\n')

                    # Записываем циклические параметры
                    file.write(','.join(map(str, self.t)) + '\n')
                    file.write(','.join(map(str, self.q1)) + '\n')
                    file.write(','.join(map(str, self.q2)) + '\n')
                    file.write(','.join(map(str, self.q3)) + '\n')
                    file.write(','.join(map(str, self.q4)) + '\n')

                    # Записываем параметры траектории
                    file.write(f"{self.trajectory_type}\n")

                    # Записываем параметры прямой
                    file.write(','.join(map(str, self.line_params)) + '\n')

                    # Записываем параметры окружности
                    file.write(','.join(map(str, self.circle_params)) + '\n')

                self.current_file_path = file_name  # Сохраняем путь к файлу
                QMessageBox.information(self, "Успех", "Данные успешно сохранены!")
            except Exception as e:
                QMessageBox.critical(self, "Ошибка", f"Ошибка при сохранении данных: {e}")

    def set_system_volume(self, level):
        #Устанавливает системную громкость на указанный уровень (от 0 до 100) с использованием pycaw.
        try:
            devices = AudioUtilities.GetSpeakers()
            interface = devices.Activate(
                IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
            volume = cast(interface, POINTER(IAudioEndpointVolume))

            # Проверяем, выключен ли звук, и включаем его
            current_mute_state = volume.GetMute()
            if current_mute_state:
                volume.SetMute(False, None)
                print("Звук был выключен, включён.")

            # Устанавливаем громкость на указанный уровень (в диапазоне от 0.0 до 1.0)
            volume_level = level / 100.0
            volume.SetMasterVolumeLevelScalar(volume_level, None)
            print(f"Громкость установлена на {level}%")
        except Exception as e:
            QMessageBox.warning(self, "Ошибка", f"Не удалось установить громкость: {e}")
            print(f"Ошибка установки громкости: {e}")

    def show_robot_type_dialog(self):
        # Создание диалогового окна для выбора типа робота
        dialog = QDialog(self)
        dialog.setWindowTitle("Выбор типа робота")
        dialog.setGeometry(300, 300, 290, 130)

        layout = QVBoxLayout()

        # Заголовок для диалогового окна
        title_label = QLabel("Тип робота")
        title_label.setStyleSheet("font-size: 25px; font-weight: bold;")
        layout.addWidget(title_label)

        # Создание группы радио-кнопок для выбора типа робота
        self.robot_group = QGroupBox()
        robot_layout = QFormLayout()

        self.cartesian_radio = QRadioButton("Декартовый")
        self.cartesian_radio.setStyleSheet("font-size: 25px;")
        self.cylindrical_radio = QRadioButton("Цилиндрический")
        self.cylindrical_radio.setStyleSheet("font-size: 25px;")
        self.scara_radio = QRadioButton("Скара")
        self.scara_radio.setStyleSheet("font-size: 25px;")
        self.coler_radio = QRadioButton("Колер")
        self.coler_radio.setStyleSheet("font-size: 25px;")

        # Восстанавливаем ранее выбранное значение
        if self.robot_type == "Декартовый":
            self.cartesian_radio.setChecked(True)
        elif self.robot_type == "Цилиндрический":
            self.cylindrical_radio.setChecked(True)
        elif self.robot_type == "Скара":
            self.scara_radio.setChecked(True)
        elif self.robot_type == "Колер":
            self.coler_radio.setChecked(True)
        else:
            self.cartesian_radio.setChecked(True)

        # Добавляем радио-кнопки в макет
        robot_layout.addRow(self.cartesian_radio)
        robot_layout.addRow(self.cylindrical_radio)
        robot_layout.addRow(self.scara_radio)
        robot_layout.addRow(self.coler_radio)

        self.robot_group.setLayout(robot_layout)

        # Создание кнопки ОК
        ok_button = QPushButton("ОК")
        ok_button.setStyleSheet("background-color : #18f22e")
        ok_button.setFont(QFont('Times', 14))
        ok_button.clicked.connect(lambda: self.apply_robot_type(dialog))

        help_button = QPushButton("Помощь")
        help_button.setStyleSheet("background-color : #87CEEB")
        help_button.setFont(QFont('Times', 14))
        help_button.clicked.connect(self.show_robot_type_help_dialog)

        layout.addWidget(self.robot_group)
        layout.addWidget(help_button)
        layout.addWidget(ok_button)

        dialog.setLayout(layout)
        dialog.exec_()

    def show_robot_type_help_dialog(self):

        help_dialog = QDialog(self)
        help_dialog.setWindowTitle("Подсказка")
        help_layout = QVBoxLayout()

        # Основной текст подсказки с выделением
        help_text = QLabel(self)
        help_text.setFont(QFont('Times', 12))

        help_text.setText("""Выберите тип робота, с которым вы будете работать в дальнейшем: <br>
                            Декарт, Скара, Цилиндр. <br>
                            <font color='red'>Для дополнительной помощи выберите одну из следующих тем:</font>""")

        help_layout.addWidget(help_text)

        # Кнопки для открытия дополнительных подсказок
        button_layout = QHBoxLayout()

        cartesian_button = QPushButton("Декарт")
        cartesian_button.setFont(QFont('Times', 14))
        cartesian_button.setStyleSheet("background-color: lightblue")
        cartesian_button.clicked.connect(self.show_cartesian_help)

        scara_button = QPushButton("Скара")
        scara_button.setFont(QFont('Times', 14))
        scara_button.setStyleSheet("background-color: lightgreen")
        scara_button.clicked.connect(self.show_scara_help)

        cylindrical_button = QPushButton("Цилиндр")
        cylindrical_button.setFont(QFont('Times', 14))
        cylindrical_button.setStyleSheet("background-color: yellow")
        cylindrical_button.clicked.connect(self.show_cylindrical_help)

        button_layout.addWidget(cartesian_button)
        button_layout.addWidget(scara_button)
        button_layout.addWidget(cylindrical_button)

        cancel_button = QPushButton("Закрыть")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        cancel_button.clicked.connect(help_dialog.reject)
        button_layout.addWidget(cancel_button)

        help_layout.addLayout(button_layout)

        help_dialog.setLayout(help_layout)
        help_dialog.exec_()

    def show_movement_type_dialog(self):
        # Создание диалогового окна для выбора типа движения
        dialog = QDialog(self)
        dialog.setWindowTitle("Выбор типа движения")
        dialog.setGeometry(300, 300, 310, 150)

        layout = QVBoxLayout()

        # Заголовок для диалогового окна
        title_label = QLabel("Тип движения")
        title_label.setStyleSheet("font-size: 25px; font-weight: bold;")
        layout.addWidget(title_label)

        # Создание группы радио-кнопок для выбора типа движения
        self.movement_group = QGroupBox()
        movement_layout = QFormLayout()

        self.position_radio = QRadioButton("Позиционное")
        self.position_radio.setStyleSheet("font-size: 25px;")
        self.contour_radio = QRadioButton("Контурное")
        self.contour_radio.setStyleSheet("font-size: 25px;")

        # Восстанавливаем ранее выбранное значение
        if self.movement_type == "Позиционное":
            self.position_radio.setChecked(True)
        elif self.movement_type == "Контурное":
            self.contour_radio.setChecked(True)
        else:
            self.position_radio.setChecked(True)

        # Добавляем радио-кнопки в макет
        movement_layout.addRow(self.position_radio)
        movement_layout.addRow(self.contour_radio)

        self.movement_group.setLayout(movement_layout)

        # Создание кнопки ОК
        ok_button = QPushButton("ОК")
        ok_button.setStyleSheet("background-color : #18f22e")
        ok_button.setFont(QFont('Times', 14))
        ok_button.clicked.connect(lambda: self.apply_movement_type(dialog))

        help_button = QPushButton("Помощь")
        help_button.setStyleSheet("background-color : #87CEEB")
        help_button.setFont(QFont('Times', 14))
        help_button.clicked.connect(self.show_movement_type_help_dialog)

        layout.addWidget(self.movement_group)
        layout.addWidget(help_button)
        layout.addWidget(ok_button)

        dialog.setLayout(layout)
        dialog.exec_()

    def show_movement_type_help_dialog(self):

        help_dialog = QDialog(self)
        help_dialog.setWindowTitle("Подсказка")
        help_layout = QVBoxLayout()

        # Основной текст подсказки с выделением
        help_text = QLabel(self)
        help_text.setFont(QFont('Times', 12))

        help_text.setText("""Выберите тип движения: Позиционное или Контурное. <br>
                            <font color='red'>Для дополнительной помощи выберите одну из следующих тем:</font>""")

        help_layout.addWidget(help_text)

        # Кнопки для открытия дополнительных подсказок
        button_layout = QHBoxLayout()

        position_button = QPushButton("Позиционное")
        position_button.setFont(QFont('Times', 14))
        position_button.setStyleSheet("background-color: lightblue")
        position_button.clicked.connect(self.show_position_help)

        contur_button = QPushButton("Контурное")
        contur_button.setFont(QFont('Times', 14))
        contur_button.setStyleSheet("background-color: lightgreen")
        contur_button.clicked.connect(self.show_contur_help)

        button_layout.addWidget(position_button)
        button_layout.addWidget(contur_button)

        cancel_button = QPushButton("Закрыть")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        cancel_button.clicked.connect(help_dialog.reject)
        button_layout.addWidget(cancel_button)

        help_layout.addLayout(button_layout)

        help_dialog.setLayout(help_layout)
        help_dialog.exec_()

    def show_position_help(self):
        # Окно с подсказкой по Декарт
        position_help_dialog = QDialog(self)
        position_help_dialog.setWindowTitle("Подсказка позиционное движение")
        position_help_layout = QVBoxLayout()

        position_help_text = QLabel("Позиционное управление \n"
    "Циклограмма представляет собой прямоугольную матрицу, в которой заданы\n"
    "операцию вращения (ориентации) рабочего органа.\n"
    "последовательность моментов времени и соответствующие значения\n"
    "обобщенных координат. При задании циклограммы следует задавать значения \n"
    "моментов времени в возрастающем порядке.\n",
                                     self)
        position_help_text.setFont(QFont('Times', 12))
        position_help_layout.addWidget(position_help_text)

        position_help_dialog.setLayout(position_help_layout)
        position_help_dialog.exec_()

    def show_contur_help(self):
        # Окно с подсказкой по Декарт
        contur_help_dialog = QDialog(self)
        contur_help_dialog.setWindowTitle("Подсказка контурное движение")
        contur_help_layout = QVBoxLayout()

        contur_help_text = QLabel("Контурное управление \n"
    "Контурное управление предполагает задание траектории движения рабочего\n"
    "органа робота и его скорости. Траектории движения рабочего органа могут быть\n"
    "двух видов - прямая линия и окружность.\n"
    "При задании параметров траектории следует учитывать, что эта траектория  \n"
    "должна целиком лежать в пределах рабочей зоны робота.\n",
                                     self)
        contur_help_text.setFont(QFont('Times', 12))
        contur_help_layout.addWidget(contur_help_text)

        contur_help_dialog.setLayout(contur_help_layout)
        contur_help_dialog.exec_()

    def show_calculator_dialog(self):
        # Создание диалогового окна для вычислителя
        dialog = QDialog(self)
        dialog.setWindowTitle("Вычислитель")

        layout = QVBoxLayout()

        # Сетка для ввода значений
        grid_layout = QGridLayout()

        # Создание полей ввода
        self.bit_depth_input = QLineEdit(self)
        self.bit_depth_input.setFont(QFont('Times', 14))
        self.bit_depth_input.setText(self.calculator_values['bit_depth'])

        self.exchange_cycle_input = QLineEdit(self)
        self.exchange_cycle_input.setFont(QFont('Times', 14))
        self.exchange_cycle_input.setText(self.calculator_values['exchange_cycle'])

        self.control_cycle_input = QLineEdit(self)
        self.control_cycle_input.setFont(QFont('Times', 14))
        self.control_cycle_input.setText(self.calculator_values['control_cycle'])

        self.filter_constant_input = QLineEdit(self)
        self.filter_constant_input.setFont(QFont('Times', 14))
        self.filter_constant_input.setText(self.calculator_values['filter_constant'])

        # Добавление полей в сетку с метками
        temp = QLabel("Разрядность")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 0, 0)
        grid_layout.addWidget(self.bit_depth_input, 0, 1)

        temp = QLabel("Такт обмена")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 1, 0)
        grid_layout.addWidget(self.exchange_cycle_input, 1, 1)

        temp = QLabel("Такт управления")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 0, 2)
        grid_layout.addWidget(self.control_cycle_input, 0, 3)

        temp = QLabel("Постоянная фильтра")
        temp.setFont(QFont('Times', 13))
        grid_layout.addWidget(temp, 1, 2)
        grid_layout.addWidget(self.filter_constant_input, 1, 3)

        layout.addLayout(grid_layout)

        # Создание кнопок ОК и Отменить
        button_layout = QHBoxLayout()
        ok_button = QPushButton("ОК")
        ok_button.setStyleSheet("background-color : #18f22e")
        ok_button.setFont(QFont('Times', 14))
        cancel_button = QPushButton("Отменить")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        help_button = QPushButton("Помощь")
        help_button.setStyleSheet("background-color : #87CEEB")
        help_button.setFont(QFont('Times', 14))

        help_button.clicked.connect(self.show_calculator_help_dialog)
        ok_button.clicked.connect(lambda: self.apply_calculator_values(dialog))
        cancel_button.clicked.connect(dialog.reject)

        button_layout.addWidget(ok_button)
        button_layout.addWidget(help_button)
        button_layout.addWidget(cancel_button)

        layout.addLayout(button_layout)
        dialog.setLayout(layout)
        dialog.exec_()

    def show_calculator_help_dialog(self):

        help_dialog = QDialog(self)
        help_dialog.setWindowTitle("Подсказка")
        help_layout = QVBoxLayout()

        # Основной текст подсказки с выделением
        help_text = QLabel(self)
        help_text.setFont(QFont('Times', 12))

        help_text.setText("""В рамках данного пакета прикладных программ имитируется работа вычислительных устройств, <br>
                            управляющих работой робота. За основу принята двухуровневая иерархическая схема,<br>
                            при которой центральное вычислительное устройство (ЦВУ) решает задачи координации работы<br>
                            вычислительных устройств (ВУ), решает траекторные задачи и выдает уставки на ВУ.<br>
                            ВУ, в свою очередь, осуществляет управление приводами робота.<br><br>
                            К параметрам ЦВУ и ВУ относятся:<br>
                            - p - разрядность ЭВМ (определяет точность решения траектории прямой и обратной задач кинематики);<br>
                            - tоб - цикл обмена ЦВУ и ВУ (такт выдачи установок на ВУ);<br>
                            - tупр - цикл управления (такт выдачи управляющих сигналов импульсов);<br>
                            - tф - постоянная экспоненциального фильтра, используемого скоростных перегрузок<br>
                            механических узлов робота.""")

        help_layout.addWidget(help_text)

        # Кнопки для открытия дополнительных подсказок
        button_layout = QHBoxLayout()


        cancel_button = QPushButton("Закрыть")
        cancel_button.setStyleSheet("background-color : red")
        cancel_button.setFont(QFont('Times', 14))
        cancel_button.clicked.connect(help_dialog.reject)
        button_layout.addWidget(cancel_button)

        help_layout.addLayout(button_layout)

        help_dialog.setLayout(help_layout)
        help_dialog.exec_()

    def apply_cartesian_params(self, dialog):
        # Проверка корректности значений масс без сохранения данных
        try:
            massd_1 = float(self.massd_1_input.text())
            if massd_1 < 0 or massd_1 > 20:
                raise ValueError("Значение массы 1-го звена не в диапазоне от 0 до 20.")

            massd_2 = float(self.massd_2_input.text())
            if massd_2 < 0 or massd_2 > 20:
                raise ValueError("Значение массы 2-го звена не в диапазоне от 0 до 20.")

            massd_3 = float(self.massd_3_input.text())
            if massd_3 < 0 or massd_3 > 20:
                raise ValueError("Значение массы 3-го звена не в диапазоне от 0 до 20.")

            momentd_1 = float(self.momentd_1_input.text())
            if momentd_1 < 0 or momentd_1 > 20:
                raise ValueError("Значение массы 4-го звена не в диапазоне от 0 до 20.")

            # Если все значения корректны, принимаем диалог


            self.massd_1 = massd_1
            self.massd_2 = massd_2
            self.massd_3 = massd_3
            self.momentd_1 = momentd_1
            self.trajectory_calculator.set_cartesian_params(massd_1, massd_2, massd_3, momentd_1)

            dialog.accept()
            return massd_1, massd_2, massd_3, momentd_1

        except ValueError as e:
            QMessageBox.warning(self, "Ошибка", str(e))
            return None  # Возвращаем None, если есть ошибка


    def apply_robot_type(self, dialog):
        # Применяем выбранный тип робота и сохраняем
        if self.scara_radio.isChecked():
            self.robot_type = "Скара"
        elif self.cartesian_radio.isChecked():
            self.robot_type = "Декартовый"
        elif self.cylindrical_radio.isChecked():
            self.robot_type = "Цилиндрический"
        elif self.coler_radio.isChecked():
            self.robot_type = "Колер"
        dialog.accept()

    def apply_movement_type(self, dialog):
        # Применяем выбранный тип движения и сохраняем
        if self.position_radio.isChecked():
            self.movement_type = "Позиционное"
        elif self.contour_radio.isChecked():
            self.movement_type = "Контурное"
        dialog.accept()

    def apply_calculator_values(self, dialog):
        # Проверка корректности значения "Разрядность"
        try:
            bit_depth = int(self.bit_depth_input.text())
            if bit_depth < 2 or bit_depth > 32:
                raise ValueError("Значение не в диапазоне от 2 до 32.")
        except ValueError as e:
            QMessageBox.warning(self, "Ошибка", str(e))
            return  # Оставляем окно открытым

        # Проверка корректности значения "Такт обмена"
        try:
            exchange_cycle = float(self.exchange_cycle_input.text())
            if exchange_cycle < 0 or exchange_cycle > 1:
                raise ValueError("Значение не в диапазоне от 0 до 1.")
        except ValueError as e:
            QMessageBox.warning(self, "Ошибка", str(e))
            return

        # Проверка корректности значения "Такт управления"
        try:
            control_cycle = float(self.control_cycle_input.text())
            if control_cycle < 0 or control_cycle > 1:
                raise ValueError("Значение не в диапазоне от 0 до 1.")
        except ValueError as e:
            QMessageBox.warning(self, "Ошибка", str(e))
            return

        # Проверка корректности значения "Постоянная фильтра"
        try:
            filter_constant = float(self.filter_constant_input.text())
            if filter_constant < 0 or filter_constant > 1:
                raise ValueError("Значение не в диапазоне от 0 до 1.")
        except ValueError as e:
            QMessageBox.warning(self, "Ошибка", str(e))
            return

        # Сохраняем значения
        self.calculator_values['bit_depth'] = self.bit_depth_input.text()
        self.calculator_values['exchange_cycle'] = self.exchange_cycle_input.text()
        self.calculator_values['control_cycle'] = self.control_cycle_input.text()
        self.calculator_values['filter_constant'] = self.filter_constant_input.text()

        dialog.accept()

class tetris:
    def get_record(self):

        record_path = os.path.join(os.path.dirname(__file__), 'record.txt')
        if not os.path.exists(record_path):
            with open(record_path, 'w') as f:
                f.write('0')
            return '0'

        with open(record_path) as f:
            return f.readline().strip()

    def set_record(self, record, score):
        record_path = os.path.join(os.path.dirname(__file__), 'record.txt')
        rec = max(int(record), score)
        with open(record_path, 'w') as f:
            f.write(str(rec))

    def check_boarders(self, figure, i, W, H, field):
        if figure[i].x < 0 or figure[i].x > W - 1:
            return False
        elif figure[i].y > H - 1 or field[figure[i].y][figure[i].x]:
            return False
        return True

    def __init__(self):
        W, H = 10, 20
        tile = 35
        game_res = W * tile, H * tile
        RES = 600, 740
        FPS = 60

        pygame.init()
        sc = pygame.display.set_mode(RES)
        game_sc = pygame.Surface(game_res)
        clock = pygame.time.Clock()

        # Путь к папке с ресурсами
        assets_path = os.path.dirname(__file__)
        font_path = os.path.join(assets_path, 'Samson.ttf')
        bg_path = os.path.join(assets_path, '3.jpg')
        game_bg_path = os.path.join(assets_path, '1.jpg')

        # Проверка на наличие файлов
        if not os.path.exists(font_path):
            raise FileNotFoundError(f"Файл шрифта Samson.ttf не найден по пути: {font_path}")
        if not os.path.exists(bg_path):
            raise FileNotFoundError(f"Фон 3.jpg не найден по пути: {bg_path}")
        if not os.path.exists(game_bg_path):
            raise FileNotFoundError(f"Фон 1.jpg не найден по пути: {game_bg_path}")

        # Загрузка фона и шрифтов
        bg = pygame.image.load(bg_path).convert()
        game_bg = pygame.image.load(game_bg_path).convert()
        main_font = pygame.font.Font(font_path, 65)
        font = pygame.font.Font(font_path, 45)
        info_font = pygame.font.Font(font_path, 40)

        title_tetris = main_font.render('TETRIS', True, pygame.Color('orange'))
        title_score = font.render('score', True, pygame.Color('red'))
        title_record = font.render('record', True, pygame.Color('aqua'))
        title_info = info_font.render('r - restart', True, pygame.Color('green'))

        grid = [pygame.Rect(x * tile, y * tile, tile, tile) for x in range(W) for y in range(H)]

        # Фигуры
        figures_pos = [[(-1, 0), (-2, 0), (0, 0), (1, 0)],
                       [(0, -1), (-1, -1), (-1, 0), (0, 0)],
                       [(-1, 0), (-1, 1), (0, 0), (0, -1)],
                       [(0, 0), (-1, 0), (0, 1), (-1, -1)],
                       [(0, 0), (0, -1), (0, 1), (-1, -1)],
                       [(0, 0), (0, -1), (0, 1), (1, -1)],
                       [(0, 0), (0, -1), (0, 1), (-1, 0)]]
        figures = [[pygame.Rect(x + W // 2, y + 1, 1, 1) for x, y in fig_pos] for fig_pos in figures_pos]

        figure_rect = pygame.Rect(0, 0, tile - 2, tile - 2)
        field = [[0 for i in range(W)] for j in range(H)]

        anim_count, anim_speed, anim_limit = 0, 60, 2000
        figure = deepcopy(choice(figures))
        next_figure = deepcopy(choice(figures))

        score = 0
        lines = 0
        scores = {0: 0, 1: 100, 2: 300, 3: 700, 4: 1500}
        get_color = lambda: (randrange(30, 256), randrange(30, 256), randrange(30, 256))

        color = get_color()
        next_color = get_color()

        while True:
            record = self.get_record()
            dx = 0
            rotate = False
            sc.blit(bg, (0, 0))
            sc.blit(game_sc, (20, 20))
            game_sc.blit(game_bg, (0, 0))

            # задержка после удаления линии
            for i in range(lines):
                pygame.time.wait(200)

            # управление
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return  # выйти из игры, сохраняя приложение активным
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_LEFT or event.key == pygame.K_a:
                        dx = -1
                    elif event.key == pygame.K_RIGHT or event.key == pygame.K_d:
                        dx = 1
                    elif event.key == pygame.K_DOWN or event.key == pygame.K_s:
                        anim_limit = 100
                    elif event.key == pygame.K_UP or event.key == pygame.K_w:
                        rotate = True
                    elif event.key == pygame.K_r:
                        anim_speed = 100000

            # смещение фигуры по оси x
            figure_old = deepcopy(figure)
            for i in range(4):
                figure[i].x += dx
                if not self.check_boarders(figure, i, W, H, field):
                    figure = deepcopy(figure_old)
                    break

            # смещение фигуры по оси y
            anim_count += anim_speed
            if anim_count > anim_limit:
                anim_count = 0
                figure_old = deepcopy(figure)
                for i in range(4):
                    figure[i].y += 1
                    if not self.check_boarders(figure, i, W, H, field):
                        for j in range(4):
                            field[figure_old[j].y][figure_old[j].x] = color
                        figure = next_figure
                        color = next_color
                        next_figure = deepcopy(choice(figures))
                        next_color = get_color()
                        anim_limit = 2000
                        break

            # вращение фигуры
            center = figure[0]
            figure_old = deepcopy(figure)
            if rotate:
                for i in range(4):
                    x = figure[i].y - center.y
                    y = figure[i].x - center.x
                    figure[i].x = center.x - x
                    figure[i].y = center.y + y
                    if not self.check_boarders(figure, i, W, H, field):
                        figure = deepcopy(figure_old)
                        break

            # проверка линий
            line = H - 1
            lines = 0
            for row in range(H - 1, -1, -1):
                count = 0
                for i in range(W):
                    if field[row][i]:
                        count += 1
                    field[line][i] = field[row][i]
                if count < W:
                    line -= 1
                else:
                    anim_speed += 3
                    lines += 1

            # добавление очков
            score += scores[lines]

            # прорисовка сетки поля
            [pygame.draw.rect(game_sc, (60, 60, 60), i_rect, 1) for i_rect in grid]

            # прорисовка фигуры
            for i in range(4):
                figure_rect.x = figure[i].x * tile
                figure_rect.y = figure[i].y * tile
                pygame.draw.rect(game_sc, color, figure_rect)

            # прорисовка упавших фигур
            for y, raw in enumerate(field):
                for x, col in enumerate(raw):
                    if col:
                        figure_rect.x, figure_rect.y = x * tile, y * tile
                        pygame.draw.rect(game_sc, col, figure_rect)

            # прорисовка следующей фигуры
            for i in range(4):
                figure_rect.x = next_figure[i].x * tile + 310
                figure_rect.y = next_figure[i].y * tile + 120
                pygame.draw.rect(sc, next_color, figure_rect)

            # прорисовка надписей
            sc.blit(title_tetris, (385, 30))
            sc.blit(title_info, (385, 300))
            sc.blit(title_score, (430, 450))
            sc.blit(font.render(str(score), True, pygame.Color('white')), (455, 510))
            sc.blit(title_record, (420, 600))
            sc.blit(font.render(record, True, pygame.Color('gold')), (445, 650))

            # проверка на конец игры
            for i in range(W):
                if field[0][i]:
                    self.set_record(record, score)
                    field = [[0 for j in range(W)] for i in range(H)]
                    anim_count, anim_speed, anim_limit = 0, 60, 2000
                    score = 0
                    for i_rect in grid:
                        pygame.draw.rect(game_sc, get_color(), i_rect)
                        sc.blit(game_sc, (20, 20))
                        pygame.display.flip()
                        clock.tick(100)

            pygame.display.flip()
            clock.tick(FPS)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotTrajectoryApp()
    window.show()
    app.exit(app.exec_())
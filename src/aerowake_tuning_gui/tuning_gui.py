import argparse, os
from python_qt_binding import QT_BINDING
from python_qt_binding.QtCore import qDebug, QTimer # ???
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from PyQt5.QtGui import *
from PyQt5.QtCore import *

from .rosPubSubs import CommandPubSub

from math import ceil

PWD = os.path.dirname(os.path.abspath(__file__))
keymap = {'ROSflight':   [0,                            # menu index
                          'Throttle (Unitless)',        # F_title
                          0.85,                         # F_max
                          0.0,                          # F_min
                          0.01,                         # F_inc
                          '-',                          # F_unit
                          'Roll (deg)',                 # x_title
                          30,                           # x_max
                          -30,                          # x_min
                          1,                            # x_inc
                          'deg',                        # x_unit
                          'Pitch (deg)',                # y_title
                          30,                           # y_max
                          -30,                          # y_min
                          1,                            # y_inc
                          'deg',                        # y_unit
                          'Yawrate (deg/s)',            # z_title
                          20,                           # z_max
                          -20,                          # z_min
                          1,                            # z_inc
                          'deg/s'],                     # z_unit
          'Aerowake':    [1,                            # menu index
                          'Altitude (m)',               # F_title
                          5.0,                          # F_max
                          0.0,                          # F_min
                          0.05,                         # F_inc
                          'm',                          # F_unit
                          'Pitch (deg)',                # x_title
                          30,                           # x_max
                          -30,                          # x_min
                          1,                            # x_inc
                          'deg',                        # x_unit
                          'Y-Velocity (m/s)',           # y_title
                          0.5,                          # y_max
                          -0.5,                         # y_min
                          0.01,                         # y_inc
                          'm/s',                        # y_unit
                          'Yaw (deg)',                  # z_title
                          180,                          # z_max
                          -180,                         # z_min
                          3,                            # z_inc
                          'deg'],                       # z_unit
          'ROScopterPOS':[2,                            # menu index
                          'Altitude (m)',               # F_title
                          15.0, # 5.0,                          # F_max
                          0.0,                          # F_min
                          0.05,                         # F_inc
                          'm',                          # F_unit
                          'X-Position (m)',             # x_title
                          30,                            # x_max
                          -30,                           # x_min
                          0.05,                         # x_inc
                          'm',                          # x_unit
                          'Y-Position (m)',             # y_title
                          5,                            # y_max
                          -5,                           # y_min
                          0.05,                         # y_inc
                          'deg',                        # y_unit
                          'Yaw (deg)',                  # z_title
                          180,                          # z_max
                          -180,                         # z_min
                          3,                            # z_inc
                          'deg'],                       # z_unit
          'ROScopterVEL':[3,                            # menu index
                          'Altitude (m)',               # F_title
                          5.0,                          # F_max
                          0.0,                          # F_min
                          0.05,                         # F_inc
                          'm',                          # F_unit
                          'X-Velocity (m/s)',           # x_title
                          0.5,                          # x_max
                          -0.5,                         # x_min
                          0.01,                         # x_inc
                          'm/s',                        # x_unit
                          'Y-Velocity (m/s)',           # y_title
                          0.5,                          # y_max
                          -0.5,                         # y_min
                          0.01,                         # y_inc
                          'm/s',                        # y_unit
                          'Yawrate (deg/s)',            # z_title
                          20,                           # z_max
                          -20,                          # z_min
                          1,                            # z_inc
                          'deg/s'],                     # z_unit
         }

class TuningGUI(QWidget):
    def __init__(self):
        super(TuningGUI, self).__init__()
        CommandPubSub.initialize()

        uifname = 'tuning_widget.ui'
        self.ui_file = os.path.join(PWD, 'resources', uifname)
        loadUi(self.ui_file, self)
        self.setObjectName('Tuning Widget')
        self.env = ''
        self.F_val = 0.0
        self.x_val = 0.0
        self.y_val = 0.0
        self.z_val = 0.0
        self.load_environment(self.comboBox.currentText())

        self.comboBox.currentIndexChanged[str].connect(self.handleModeChange)

        self.armed = True
        self.toggleArmed() # to set armed to False immediately
        self.ARMBUTTON.clicked.connect(self.toggleArmed)

        self.F_slider.valueChanged[int].connect(self.handleFslider)
        self.F_up.clicked.connect(self.handleFup)
        self.F_down.clicked.connect(self.handleFdown)

        self.x_slider.valueChanged[int].connect(self.handlexslider)
        self.x_up.clicked.connect(self.handlexup)
        self.x_down.clicked.connect(self.handlexdown)

        self.y_slider.valueChanged[int].connect(self.handleyslider)
        self.y_up.clicked.connect(self.handleyup)
        self.y_down.clicked.connect(self.handleydown)

        self.z_slider.valueChanged[int].connect(self.handlezslider)
        self.z_up.clicked.connect(self.handlezup)
        self.z_down.clicked.connect(self.handlezdown)

    def toggleArmed(self):
        self.armed = not self.armed
        CommandPubSub.setArmed(self.armed)
        if self.armed:
            self.ARMBUTTON.setStyleSheet("background-color: red")
            self.ARMBUTTON.setText('DISARM')
        else:
            self.ARMBUTTON.setStyleSheet("background-color: green")
            self.ARMBUTTON.setText('ARM')

    def handleModeChange(self, mode):
        if self.okay_to_change():
            self.load_environment(mode)
        else:
            self.comboBox.setCurrentIndex(keymap[self.env][0])

    def handleFslider(self, value):
        F_max = float(keymap[self.env][2])
        F_min = float(keymap[self.env][3])
        self.F_val = (F_max - F_min) * value / 1000.0 + F_min
        CommandPubSub.setF(self.F_val)
        self.F_label.setText(str(self.F_val))

    def FincrementInfo(self):
        increment = float(self.F_increment.toPlainText())
        sliderval = int(self.F_slider.value())
        F_max = float(keymap[self.env][2])
        F_min = float(keymap[self.env][3])
        val_increment = int(ceil(increment / (F_max - F_min) * 1000.0))
        return sliderval, val_increment

    def handleFup(self):
        sliderval, val_increment = self.FincrementInfo()
        self.F_slider.setValue(min(1000, sliderval + val_increment))

    def handleFdown(self):
        sliderval, val_increment = self.FincrementInfo()
        self.F_slider.setValue(max(0, sliderval - val_increment))

    def handlexslider(self, value):
        x_max = float(keymap[self.env][7])
        x_min = float(keymap[self.env][8])
        self.x_val = (x_max - x_min) * (value + 500.0) / 1000.0 + x_min
        CommandPubSub.setx(self.x_val)
        self.x_label.setText(str(self.x_val))

    def xincrementInfo(self):
        increment = float(self.x_increment.toPlainText())
        sliderval = int(self.x_slider.value())
        x_max = float(keymap[self.env][7])
        x_min = float(keymap[self.env][8])
        val_increment = int(ceil(increment / (x_max - x_min) * 1000.0))
        return sliderval, val_increment

    def handlexup(self):
        sliderval, val_increment = self.xincrementInfo()
        self.x_slider.setValue(min(500, sliderval + val_increment))

    def handlexdown(self):
        sliderval, val_increment = self.xincrementInfo()
        self.x_slider.setValue(max(-500, sliderval - val_increment))

    def handleyslider(self, value):
        y_max = float(keymap[self.env][12])
        y_min = float(keymap[self.env][13])
        self.y_val = (y_max - y_min) * (value + 500.0) / 1000.0 + y_min
        CommandPubSub.sety(self.y_val)
        self.y_label.setText(str(self.y_val))

    def yincrementInfo(self):
        increment = float(self.y_increment.toPlainText())
        sliderval = int(self.y_slider.value())
        y_max = float(keymap[self.env][12])
        y_min = float(keymap[self.env][13])
        val_increment = int(ceil(increment / (y_max - y_min) * 1000.0))
        return sliderval, val_increment

    def handleyup(self):
        sliderval, val_increment = self.yincrementInfo()
        self.y_slider.setValue(min(500, sliderval + val_increment))

    def handleydown(self):
        sliderval, val_increment = self.yincrementInfo()
        self.y_slider.setValue(max(-500, sliderval - val_increment))

    def handlezslider(self, value):
        z_max = float(keymap[self.env][17])
        z_min = float(keymap[self.env][18])
        self.z_val = (z_max - z_min) * (value + 500.0) / 1000.0 + z_min
        CommandPubSub.setz(self.z_val)
        self.z_label.setText(str(self.z_val))

    def zincrementInfo(self):
        increment = float(self.z_increment.toPlainText())
        sliderval = int(self.z_slider.value())
        z_max = float(keymap[self.env][17])
        z_min = float(keymap[self.env][18])
        val_increment = int(ceil(increment / (z_max - z_min) * 1000.0))
        return sliderval, val_increment

    def handlezup(self):
        sliderval, val_increment = self.zincrementInfo()
        self.z_slider.setValue(min(500, sliderval + val_increment))

    def handlezdown(self):
        sliderval, val_increment = self.zincrementInfo()
        self.z_slider.setValue(max(-500, sliderval - val_increment))

    def okay_to_change(self):
        return not CommandPubSub.getArmed()

    def load_environment(self, env):
        self.env = env
        CommandPubSub.setMode(self.env)
        F_title   = keymap[self.env][1]
        F_max = str(keymap[self.env][2])
        F_min = str(keymap[self.env][3])
        F_inc = str(keymap[self.env][4])
        F_unit    = keymap[self.env][5]
        x_title   = keymap[self.env][6]
        x_max = str(keymap[self.env][7])
        x_min = str(keymap[self.env][8])
        x_inc = str(keymap[self.env][9])
        x_unit    = keymap[self.env][10]
        y_title   = keymap[self.env][11]
        y_max = str(keymap[self.env][12])
        y_min = str(keymap[self.env][13])
        y_inc = str(keymap[self.env][14])
        y_unit    = keymap[self.env][15]
        z_title   = keymap[self.env][16]
        z_max = str(keymap[self.env][17])
        z_min = str(keymap[self.env][18])
        z_inc = str(keymap[self.env][19])
        z_unit    = keymap[self.env][20]
        self.load_data([F_title, x_title, y_title, z_title],
                       [F_max, x_max, y_max, z_max],
                       [F_min, x_min, y_min, z_min],
                       [F_inc, x_inc, y_inc, z_inc],
                       [F_unit, x_unit, y_unit, z_unit])

    def load_data(self, list_title, list_max_label, list_min_label, list_increment, list_inc_unit):
        self.F_slider.setValue(0)
        self.x_slider.setValue(0)
        self.y_slider.setValue(0)
        self.z_slider.setValue(0)
        self.F_title.setText(list_title[0])
        self.x_title.setText(list_title[1])
        self.y_title.setText(list_title[2])
        self.z_title.setText(list_title[3])
        self.F_max_label.setText(list_max_label[0])
        self.x_max_label.setText(list_max_label[1])
        self.y_max_label.setText(list_max_label[2])
        self.z_max_label.setText(list_max_label[3])
        self.F_min_label.setText(list_min_label[0])
        self.x_min_label.setText(list_min_label[1])
        self.y_min_label.setText(list_min_label[2])
        self.z_min_label.setText(list_min_label[3])
        self.F_increment.setPlainText(list_increment[0])
        self.x_increment.setPlainText(list_increment[1])
        self.y_increment.setPlainText(list_increment[2])
        self.z_increment.setPlainText(list_increment[3])
        self.F_inc_unit.setText(list_inc_unit[0])
        self.x_inc_unit.setText(list_inc_unit[1])
        self.y_inc_unit.setText(list_inc_unit[2])
        self.z_inc_unit.setText(list_inc_unit[3])

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

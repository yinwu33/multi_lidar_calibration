# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'cloud_filter.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!
import sys

import rospy

from PyQt5 import QtCore, QtGui, QtWidgets

ns = "/lidar_1/"


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.widget = QtWidgets.QWidget(self.centralwidget)
        self.widget.setObjectName("widget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.widget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.widget_x = QtWidgets.QWidget(self.widget)
        self.widget_x.setObjectName("widget_x")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget_x)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_x = QtWidgets.QLabel(self.widget_x)
        self.label_x.setObjectName("label_x")
        self.horizontalLayout.addWidget(self.label_x)
        self.horizontalSlider_x = QtWidgets.QSlider(self.widget_x)
        self.horizontalSlider_x.setMinimum(-5000)
        self.horizontalSlider_x.setMaximum(5000)
        self.horizontalSlider_x.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_x.setObjectName("horizontalSlider_x")
        self.horizontalLayout.addWidget(self.horizontalSlider_x)
        self.doubleSpinBox_x = QtWidgets.QDoubleSpinBox(self.widget_x)
        self.doubleSpinBox_x.setMinimum(-50.0)
        self.doubleSpinBox_x.setMaximum(50.0)
        self.doubleSpinBox_x.setObjectName("doubleSpinBox_x")
        self.horizontalLayout.addWidget(self.doubleSpinBox_x)
        self.verticalLayout.addWidget(self.widget_x)
        self.widget_y = QtWidgets.QWidget(self.widget)
        self.widget_y.setObjectName("widget_y")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.widget_y)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_y = QtWidgets.QLabel(self.widget_y)
        self.label_y.setObjectName("label_y")
        self.horizontalLayout_2.addWidget(self.label_y)
        self.horizontalSlider_y = QtWidgets.QSlider(self.widget_y)
        self.horizontalSlider_y.setMinimum(-5000)
        self.horizontalSlider_y.setMaximum(5000)
        self.horizontalSlider_y.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_y.setObjectName("horizontalSlider_y")
        self.horizontalLayout_2.addWidget(self.horizontalSlider_y)
        self.doubleSpinBox_y = QtWidgets.QDoubleSpinBox(self.widget_y)
        self.doubleSpinBox_y.setMinimum(-50.0)
        self.doubleSpinBox_y.setMaximum(50.0)
        self.doubleSpinBox_y.setObjectName("doubleSpinBox_y")
        self.horizontalLayout_2.addWidget(self.doubleSpinBox_y)
        self.verticalLayout.addWidget(self.widget_y)
        self.widget_z = QtWidgets.QWidget(self.widget)
        self.widget_z.setObjectName("widget_z")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.widget_z)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_z = QtWidgets.QLabel(self.widget_z)
        self.label_z.setObjectName("label_z")
        self.horizontalLayout_3.addWidget(self.label_z)
        self.horizontalSlider_z = QtWidgets.QSlider(self.widget_z)
        self.horizontalSlider_z.setMinimum(-5000)
        self.horizontalSlider_z.setMaximum(5000)
        self.horizontalSlider_z.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_z.setObjectName("horizontalSlider_z")
        self.horizontalLayout_3.addWidget(self.horizontalSlider_z)
        self.doubleSpinBox_z = QtWidgets.QDoubleSpinBox(self.widget_z)
        self.doubleSpinBox_z.setMinimum(-50.0)
        self.doubleSpinBox_z.setMaximum(50.0)
        self.doubleSpinBox_z.setObjectName("doubleSpinBox_z")
        self.horizontalLayout_3.addWidget(self.doubleSpinBox_z)
        self.verticalLayout.addWidget(self.widget_z)
        self.widget_a = QtWidgets.QWidget(self.widget)
        self.widget_a.setObjectName("widget_a")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.widget_a)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label_a = QtWidgets.QLabel(self.widget_a)
        self.label_a.setObjectName("label_a")
        self.horizontalLayout_4.addWidget(self.label_a)
        self.horizontalSlider_a = QtWidgets.QSlider(self.widget_a)
        self.horizontalSlider_a.setMinimum(-5000)
        self.horizontalSlider_a.setMaximum(5000)
        self.horizontalSlider_a.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_a.setObjectName("horizontalSlider_a")
        self.horizontalLayout_4.addWidget(self.horizontalSlider_a)
        self.doubleSpinBox_a = QtWidgets.QDoubleSpinBox(self.widget_a)
        self.doubleSpinBox_a.setMinimum(-50.0)
        self.doubleSpinBox_a.setMaximum(50.0)
        self.doubleSpinBox_a.setObjectName("doubleSpinBox_a")
        self.horizontalLayout_4.addWidget(self.doubleSpinBox_a)
        self.verticalLayout.addWidget(self.widget_a)
        self.widget_b = QtWidgets.QWidget(self.widget)
        self.widget_b.setObjectName("widget_b")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.widget_b)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label_b = QtWidgets.QLabel(self.widget_b)
        self.label_b.setObjectName("label_b")
        self.horizontalLayout_5.addWidget(self.label_b)
        self.horizontalSlider_b = QtWidgets.QSlider(self.widget_b)
        self.horizontalSlider_b.setMinimum(-5000)
        self.horizontalSlider_b.setMaximum(5000)
        self.horizontalSlider_b.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_b.setObjectName("horizontalSlider_b")
        self.horizontalLayout_5.addWidget(self.horizontalSlider_b)
        self.doubleSpinBox_b = QtWidgets.QDoubleSpinBox(self.widget_b)
        self.doubleSpinBox_b.setMinimum(-50.0)
        self.doubleSpinBox_b.setMaximum(50.0)
        self.doubleSpinBox_b.setObjectName("doubleSpinBox_b")
        self.horizontalLayout_5.addWidget(self.doubleSpinBox_b)
        self.verticalLayout.addWidget(self.widget_b)
        self.widget_c = QtWidgets.QWidget(self.widget)
        self.widget_c.setObjectName("widget_c")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(self.widget_c)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.label_c = QtWidgets.QLabel(self.widget_c)
        self.label_c.setObjectName("label_c")
        self.horizontalLayout_6.addWidget(self.label_c)
        self.horizontalSlider_c = QtWidgets.QSlider(self.widget_c)
        self.horizontalSlider_c.setMinimum(-5000)
        self.horizontalSlider_c.setMaximum(5000)
        self.horizontalSlider_c.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_c.setObjectName("horizontalSlider_c")
        self.horizontalLayout_6.addWidget(self.horizontalSlider_c)
        self.doubleSpinBox_c = QtWidgets.QDoubleSpinBox(self.widget_c)
        self.doubleSpinBox_c.setMinimum(-50.0)
        self.doubleSpinBox_c.setMaximum(50.0)
        self.doubleSpinBox_c.setObjectName("doubleSpinBox_c")
        self.horizontalLayout_6.addWidget(self.doubleSpinBox_c)
        self.verticalLayout.addWidget(self.widget_c)
        self.verticalLayout_2.addWidget(self.widget)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label_x.setText(_translate("MainWindow", "x"))
        self.label_y.setText(_translate("MainWindow", "y"))
        self.label_z.setText(_translate("MainWindow", "z"))
        self.label_a.setText(_translate("MainWindow", "a"))
        self.label_b.setText(_translate("MainWindow", "b"))
        self.label_c.setText(_translate("MainWindow", "c"))

    def setFunctions(self):
        # horizental connection
        self.horizontalSlider_x.valueChanged.connect(
            self.valueChanged_horizentalSlider_x)
        self.horizontalSlider_y.valueChanged.connect(
            self.valueChanged_horizentalSlider_y)
        self.horizontalSlider_z.valueChanged.connect(
            self.valueChanged_horizentalSlider_z)
        self.horizontalSlider_a.valueChanged.connect(
            self.valueChanged_horizentalSlider_a)
        self.horizontalSlider_b.valueChanged.connect(
            self.valueChanged_horizentalSlider_b)
        self.horizontalSlider_c.valueChanged.connect(
            self.valueChanged_horizentalSlider_c)

        # doubleSpinbox connection
        self.doubleSpinBox_x.valueChanged.connect(self.valueChanged_spinBox_x)
        self.doubleSpinBox_y.valueChanged.connect(self.valueChanged_spinBox_y)
        self.doubleSpinBox_z.valueChanged.connect(self.valueChanged_spinBox_z)
        self.doubleSpinBox_a.valueChanged.connect(self.valueChanged_spinBox_a)
        self.doubleSpinBox_b.valueChanged.connect(self.valueChanged_spinBox_b)
        self.doubleSpinBox_c.valueChanged.connect(self.valueChanged_spinBox_c)

    def valueChanged_spinBox_x(self):
        value = self.doubleSpinBox_x.value()
        self.horizontalSlider_x.setValue(value*100)

    def valueChanged_spinBox_y(self):
        value = self.doubleSpinBox_y.value()
        self.horizontalSlider_y.setValue(value*100)

    def valueChanged_spinBox_z(self):
        value = self.doubleSpinBox_z.value()
        self.horizontalSlider_z.setValue(value*100)

    def valueChanged_spinBox_a(self):
        value = self.doubleSpinBox_a.value()
        self.horizontalSlider_a.setValue(value*100)

    def valueChanged_spinBox_b(self):
        value = self.doubleSpinBox_b.value()
        self.horizontalSlider_b.setValue(value*100)

    def valueChanged_spinBox_c(self):
        value = self.doubleSpinBox_c.value()
        self.horizontalSlider_c.setValue(value*100)

    def valueChanged_horizentalSlider_x(self):
        value = self.horizontalSlider_x.value()/100
        rospy.set_param(ns+"x", value)
        self.doubleSpinBox_x.setValue(value)

    def valueChanged_horizentalSlider_y(self):
        value = self.horizontalSlider_y.value()/100
        rospy.set_param(ns+"y", value)
        self.doubleSpinBox_y.setValue(value)

    def valueChanged_horizentalSlider_z(self):
        value = self.horizontalSlider_z.value()/100
        rospy.set_param(ns+"z", value)
        self.doubleSpinBox_z.setValue(value)

    def valueChanged_horizentalSlider_a(self):
        value = self.horizontalSlider_a.value()/100
        rospy.set_param(ns+"a", value)
        self.doubleSpinBox_a.setValue(value)

    def valueChanged_horizentalSlider_b(self):
        value = self.horizontalSlider_b.value()/100
        rospy.set_param(ns+"b", value)
        self.doubleSpinBox_b.setValue(value)

    def valueChanged_horizentalSlider_c(self):
        value = self.horizontalSlider_c.value()/100
        rospy.set_param(ns+"c", value)
        self.doubleSpinBox_c.setValue(value)


def main():
    rospy.init_node("cloud_crop_node")

    global ns
    ns = rospy.get_namespace()
    rospy.loginfo(f"Using Namespace: {ns}")

    app = QtWidgets.QApplication(sys.argv)
    ui = Ui_MainWindow()
    windows = QtWidgets.QMainWindow()

    ui.setupUi(windows)

    ui.setFunctions()
    windows.setWindowTitle(ns)

    windows.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

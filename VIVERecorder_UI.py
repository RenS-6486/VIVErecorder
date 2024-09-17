import sys
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QTextEdit
from PyQt6.QtCore import QTimer
import time
import openvr
import csv
import math
import numpy as np
from win_precise_time import sleep
import matplotlib.pyplot as plt
from collections import deque
from PyQt6 import QtGui

# Define the sampling rate (in Hz)
sampling_rate = 120


def precise_wait(duration):
    # Wait for a specified duration with high precision.
    now = time.time()
    end = now + duration
    if duration >= 0.001:
        sleep(duration)
    while now < end:
        now = time.time()

def convert_to_quaternion(pose_mat):
    # Convert pose matrix to quaternion and position.
    r_w = math.sqrt(abs(1 + pose_mat[0][0] + pose_mat[1][1] + pose_mat[2][2])) / 2
    if r_w == 0:
        r_w = 0.0001

    r_x = (pose_mat[2][1] - pose_mat[1][2]) / (4 * r_w)
    r_y = (pose_mat[0][2] - pose_mat[2][0]) / (4 * r_w)
    r_z = (pose_mat[1][0] - pose_mat[0][1]) / (4 * r_w)

    x = pose_mat[0][3]
    y = pose_mat[1][3]
    z = pose_mat[2][3]

    return [x, y, z, r_w, r_x, r_y, r_z]


class VRSystemManager:
    def __init__(self):
        self.vr_system = None
        self.initialized = False

    def initialize_vr_system(self):
        # Initialize the VR system
        try:
            openvr.init(openvr.VRApplication_Other)
            self.vr_system = openvr.VRSystem()
            print(f"Starting Capture:")
            self.initialized = True
        except Exception as e:
            print(f"Failed to initialize VR system: {e}")
            self.initialized = False
            return False
        return True

    def get_tracker_data(self):
        # Retrieve tracker data from the VR system.
        poses = self.vr_system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
        return poses

    def print_discovered_objects(self):
        # Print information about discovered VR devices.
        for device_index in range(openvr.k_unMaxTrackedDeviceCount):
            device_class = self.vr_system.getTrackedDeviceClass(device_index)
            if device_class != openvr.TrackedDeviceClass_Invalid:
                serial_number = self.vr_system.getStringTrackedDeviceProperty(
                    device_index, openvr.Prop_SerialNumber_String)
                model_number = self.vr_system.getStringTrackedDeviceProperty(
                    device_index, openvr.Prop_ModelNumber_String)
                print(f"Device {device_index}: {serial_number} ({model_number})")

    def shutdown_vr_system(self):
        # Shutdown the VR system.
        if self.vr_system:
            openvr.shutdown()
            self.initialized = False

class CSVLogger:
    def __init__(self):
        self.file = None
        self.csv_writer = None
        self.initialized = False

    def init_csv(self, filename):
        # Initialize the CSV file for logging tracker data.
        try:
            self.file = open(filename, 'w', newline='')
            self.initialized = True
            self.csv_writer = csv.writer(self.file)
            self.csv_writer.writerow(['Time', 'PositionX', 'PositionY', 'PositionZ', 'RotationW', 'RotationX', 'RotationY', 'RotationZ'])
        except Exception as e:
            print(f"Failed to initialize CSV file: {e}")
            self.initialized = False
            return False
        return True

    def log_data_csv(self, current_time, position):
        # Log tracker data to CSV file.
        try:
            self.csv_writer.writerow([current_time, *position])
        except Exception as e:
            print(f"Failed to write data to CSV file: {e}")

    def close_csv(self):
        # Close the CSV file if it's open.
        if self.file:
            self.file.close()
            self.initialized = False


class LivePlotter:
    def __init__(self):
        self.fig = None
        self.ax1 = None
        self.ax2 = None
        self.ax3 = None
        self.x_data = deque()
        self.y_data = deque()
        self.z_data = deque()
        self.time_data = deque()
        self.first = True
        self.firstx = 0
        self.firsty = 0
        self.firstz = 0
        self.start_time = time.time()
        self.vive_PosVIVE = np.zeros([3])

    def init_live_plot(self):
        # Initialize the 2D plot for VIVE tracker data.
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1)
        plt.subplots_adjust(hspace=0.4)
        self.ax1.set_title('X Position')
        self.ax2.set_title('Y Position')
        self.ax3.set_title('Z Position')
        self.x_line, = self.ax1.plot([], [], 'r-')
        self.y_line, = self.ax2.plot([], [], 'g-')
        self.z_line, = self.ax3.plot([], [], 'b-')
        self.x_data = deque()
        self.y_data = deque()
        self.z_data = deque()
        self.time_data = deque()
        self.start_time = time.time()


    def update_live_plot(self, vive_PosVIVE):
        # Update the 2D plot with new VIVE tracker data.
        current_time = time.time()
        self.x_data.append(vive_PosVIVE[0])
        self.y_data.append(vive_PosVIVE[1])
        self.z_data.append(vive_PosVIVE[2])
        self.time_data.append(current_time - self.start_time)

        self.x_line.set_data(self.time_data, self.x_data)
        self.y_line.set_data(self.time_data, self.y_data)
        self.z_line.set_data(self.time_data, self.z_data)

        if self.first:
            self.firstx = self.x_data[0]
            self.firsty = self.y_data[0]
            self.firstz = self.z_data[0]
            self.first = False

        self.ax1.set_xlim(self.time_data[0], self.time_data[-1])
        self.ax1.set_ylim([self.firstx - 1.5, self.firstx + 1.5])

        self.ax2.set_xlim(self.time_data[0], self.time_data[-1])
        self.ax2.set_ylim([self.firsty - 1.5, self.firsty + 1.5])

        self.ax3.set_xlim(self.time_data[0], self.time_data[-1])
        self.ax3.set_ylim([self.firstz - 1.5, self.firstz + 1.5])

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def init_3d_plot(self):
        # Initialize the 3D live plot for VIVE tracker data.
        self.fig_3d = plt.figure()
        self.ax_3d = self.fig_3d.add_subplot(111, projection='3d')
        self.ax_3d.view_init(elev=1, azim=180, roll=None, vertical_axis='y')

        self.maxlen_3d = 100
        self.x_data_3d = deque(maxlen=self.maxlen_3d)
        self.y_data_3d = deque(maxlen=self.maxlen_3d)
        self.z_data_3d = deque(maxlen=self.maxlen_3d)

        self.line_3d, = self.ax_3d.plot([], [], [], 'r-')

        self.ax_3d.set_xlabel('X')
        self.ax_3d.set_ylabel('Y')
        self.ax_3d.set_zlabel('Z')
        self.ax_3d.set_title('3D Tracker Position')

        self.x_quiver = None
        self.y_quiver = None
        self.z_quiver = None


    def update_3d_plot(self, pose, vive_PosVIVE):
        # Update the 3D live plot with new VIVE tracker data.
        x, y, z = vive_PosVIVE

        self.x_data_3d.append(x)
        self.y_data_3d.append(y)
        self.z_data_3d.append(z)

        self.line_3d.set_data(self.x_data_3d, self.y_data_3d)
        self.line_3d.set_3d_properties(self.z_data_3d)

        R = np.zeros((3, 3))
        for i in range(3):
            for j in range(3):
                R[i, j] = pose[i][j]
        origin = np.array([[0, 0, 0]])
        x_axis = np.array([[1, 0, 0]])
        y_axis = np.array([[0, 1, 0]])
        z_axis = np.array([[0, 0, 1]])

        # Rotate the axes
        x_axis_rot = (R @ x_axis.T).T
        y_axis_rot = (R @ y_axis.T).T
        z_axis_rot = (R @ z_axis.T).T

        if self.x_quiver:
            self.x_quiver.remove()
        self.x_quiver = self.ax_3d.quiver(*[x,y,z], *x_axis_rot[0],
                                          color='tab:orange', length=0.1, normalize=True, label='X-axis')
        if self.y_quiver:
            self.y_quiver.remove()
        self.y_quiver = self.ax_3d.quiver(*[x,y,z], *y_axis_rot[0],
                                          color='tab:green', length=0.1, normalize=True, label='Y-axis')
        if self.z_quiver:
            self.z_quiver.remove()
        self.z_quiver = self.ax_3d.quiver(*[x,y,z], *z_axis_rot[0],
                                          color='tab:blue', length=0.1, normalize=True, label='Z-axis')

        if len(self.x_data_3d) > 1:
            self.ax_3d.set_xlim(-0.5, 0.5)
            self.ax_3d.set_ylim(-0.5, 0.5)
            self.ax_3d.set_zlim(-0.5, 0.5)

        self.fig_3d.canvas.draw()
        self.fig_3d.canvas.flush_events()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle('VIVE Tracker Pose Recorder')
        self.resize(1280, 720)
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QHBoxLayout()
        central_widget.setLayout(layout)

        self.canvas_layout = QVBoxLayout()
        layout.addLayout(self.canvas_layout)

        self.csv_logger = CSVLogger()
        self.plotter = LivePlotter()
        self.vr_manager = VRSystemManager()
        self.log_data = True

        if not self.vr_manager.initialize_vr_system():
            return
        print(self.vr_manager.print_discovered_objects())

        self.plotter.init_3d_plot()
        self.canvas = FigureCanvas(self.plotter.fig_3d)
        self.canvas_layout.addWidget(self.canvas)

        button_layout = QVBoxLayout()
        layout.addLayout(button_layout)
        self.button1 = QPushButton("Start Tracking")
        self.button1.setFixedSize(400, 100)  # Set a fixed size for clarity
        self.button1.clicked.connect(self.button1_clicked)
        self.button1.setFont(QtGui.QFont("Arial", 20))
        self.status = 0
        button_layout.addWidget(self.button1)

        self.button2 = QPushButton("3D plot")
        self.button2.setFixedSize(400, 100)  # Set a fixed size for clarity
        self.button2.clicked.connect(self.button2_clicked)
        self.button2.setFont(QtGui.QFont("Arial", 20))
        self.plot_status = 1
        button_layout.addWidget(self.button2)

        self.plot_text_lst = ['No Plot', '3D Plot', '2D Plot']
        self.timer = QTimer()  # `self` is the parent, ensuring proper memory management
        self.timer.setInterval(8)  # Set interval to 1000 ms (1 second)
        self.timer.timeout.connect(self.update_plot)  # Connect timeout signal to slot

    def button1_clicked(self):
        if self.status == 0:
            readable_time = time.strftime("%Y-%m-%d %H-%M-%S", time.localtime())
            self.csv_logger.init_csv(readable_time + ' ' + self.plot_text_lst[self.plot_status] + '.csv')
            self.status = 1
            self.button1.setText("Stop Tracking")
            if self.plot_status == 1:
                self.plotter.init_3d_plot()
                self.canvas_layout.removeWidget(self.canvas)
                self.canvas.deleteLater()
                self.canvas = FigureCanvas(self.plotter.fig_3d)
                self.canvas_layout.addWidget(self.canvas)
            elif self.plot_status == 2:
                self.plotter.init_live_plot()
                self.canvas_layout.removeWidget(self.canvas)
                self.canvas.deleteLater()
                self.canvas = FigureCanvas(self.plotter.fig)
                self.canvas_layout.addWidget(self.canvas)
            else:
                self.canvas_layout.removeWidget(self.canvas)
                self.canvas.deleteLater()
                self.canvas = QTextEdit()
                self.canvas.setFont(QtGui.QFont("Arial", 15))
                self.canvas.setReadOnly(True)
                self.canvas_layout.addWidget(self.canvas)
            self.timer.start()
        elif self.status == 1:
            self.status = 0
            self.button1.setText("Start Tracking")
            self.timer.stop()
            self.csv_logger.close_csv()

    def button2_clicked(self):
        if self.plot_status == 0:
            self.plotter.init_3d_plot()
            self.canvas_layout.removeWidget(self.canvas)
            self.canvas.deleteLater()
            self.canvas = FigureCanvas(self.plotter.fig_3d)
            self.canvas_layout.addWidget(self.canvas)
            self.button2.setText("3D Plot")
            self.plot_status = 1
        elif self.plot_status == 1:
            self.plotter.init_live_plot()
            self.canvas_layout.removeWidget(self.canvas)
            self.canvas.deleteLater()
            self.canvas = FigureCanvas(self.plotter.fig)
            self.canvas_layout.addWidget(self.canvas)
            self.button2.setText("2D Plot")
            self.plot_status = 2
        else:
            self.canvas_layout.removeWidget(self.canvas)
            self.canvas.deleteLater()
            self.canvas = QTextEdit()
            self.canvas.setFont(QtGui.QFont("Arial", 15))
            self.canvas.setReadOnly(True)
            self.canvas_layout.addWidget(self.canvas)
            self.button2.setText("No Plot")
            self.plot_status = 0

    def update_plot(self):
        poses = self.vr_manager.get_tracker_data()
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            if poses[i].bPoseIsValid:
                device_class = self.vr_manager.vr_system.getTrackedDeviceClass(i)
                if device_class == openvr.TrackedDeviceClass_GenericTracker:
                    current_time = time.time()
                    position = convert_to_quaternion(poses[i].mDeviceToAbsoluteTracking)
                    if self.plot_status == 2:
                        self.plotter.update_live_plot(position[:3])
                    elif self.plot_status == 1:
                        self.plotter.update_3d_plot(poses[i].mDeviceToAbsoluteTracking, position[:3])
                    else:
                        message = f"{current_time}: position {position[:3]}\n\n"
                        self.canvas.append(message)
                    if self.log_data:
                        self.csv_logger.log_data_csv(current_time, position)
                    break


if __name__ == "__main__":
    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec()
    print("Stopping data collection...")
    if window.vr_manager.initialized:
        window.vr_manager.shutdown_vr_system()
    if window.csv_logger.initialized:
        window.csv_logger.close_csv()
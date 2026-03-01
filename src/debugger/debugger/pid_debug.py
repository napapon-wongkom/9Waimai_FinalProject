import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import os
os.environ['PYQTGRAPH_QT_LIB'] = 'PyQt5'
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
import sys

timestamp = 0
PIDsetpoint = 0
PIDinput = 0
PIDoutput = 0
velocity_mps_left = 0
velocity_mps_right = 0
L_PWM_out = 0
R_PWM_out = 0

class PID_debugger(Node):
    def __init__(self):
        super().__init__('PID_debugger')

        self.subscription = self.create_subscription(
                Float32MultiArray,
                'uart_receive_data',
                self.data_callback,
                10
            )
        
        

        self.get_logger().info("PID Debug node started.")

    def data_callback(self, msg):
        if len(msg.data) >=11:
            self.latest_data = {
                't':msg.data[0]/1000.0,
                'sp':msg.data[8],
                'pv':msg.data[9],
                'out':msg.data[10]
            }


class PIDTuner(QtWidgets.QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.setWindowTitle("PID Live Tuner - RPi 5")

        # UI Setup
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QtWidgets.QVBoxLayout(self.central_widget)

        # Top Plot: Velocity Difference (Input vs Setput)

        self.plot_input = pg.PlotWidget(title="Velocity Difference (Right - Left)")
        self.plot_input.addLegend()
        self.curve_sp = self.plot_input.plot(pen='y', name="Setpoint (0)")
        self.curve_pv = self.plot_input.plot(pen='c', name="Velocity Diff (PV)")

        # Bottom Plot: PID Output (The Correction)
        self.plot_output = pg.PlotWidget(title="PID Corrention Output")
        self.curve_out = self.plot_output.plot(pen='m', name="Output (Correction)")

        self.layout.addWidget(self.plot_input)
        self.layout.addWidget(self.plot_output)

        self.max_points = 500
        self.time_data = []
        self.pv_data = []
        self.out_data = []
        self.sp_data = []

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(10) # 100Hz update check

    def update(self):
        # Check ROS for new messages
        rclpy.spin_once(self.node, timeout_sec=0)

        if self.node.latest_data:
            d = self.node.latest_data
            self.time_data.append(d['t'])
            self.sp_data.append(d['sp'])
            self.pv_data.append(d['pv'])
            self.out_data.append(d['out'])

            if len(self.time_data) > self.max_points:
                self.time_data.pop(0)
                self.sp_data.pop(0)
                self.pv_data.pop(0)
                self.out_data.pop(0)

            self.curve_sp.setData(self.time_data, self.sp_data)
            self.curve_pv.setData(self.time_data, self.pv_data)
            self.curve_out.setData(self.time_data, self.out_data)
        

def main(args=None):
    rclpy.init(args=args)

    ros_node = PID_debugger()

    app = QtWidgets.QApplication(sys.argv)
    window = PIDTuner(ros_node)
    window.show()
    try:
        app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

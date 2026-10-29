#!/usr/bin/env python3
"""Real-time control dashboard: every setpoint of CheetahController next to its measurement.

  ros2 run quadru_control control_dashboard.py [--window 6] [--tau-max 35]

Tabs:
  Joint torque   commanded (~/diag_control) vs measured (/joint_states effort), +-tau_max.
                 In Gazebo the measured effort echoes the command (ideal actuator); on the robot it
                 is the drive's measured torque.
  Swing tracking foot position setpoint vs actual (hip-relative, body frame x forward), and the
                 swing tracking error.
  Ground forces  MPC force vs measured contact force (Gazebo bumpers), planned vs measured contact.
  Body           command, MPC reference, estimate and ground truth (Gazebo) of the body state.
  MPC            QP solve time vs MPC period, QP status.
All curves use the message timestamps (simulated time in Gazebo).
"""
import argparse
import math
import threading

import numpy as np
import pyqtgraph as pg
import rclpy
from gazebo_msgs.msg import ContactsState
from nav_msgs.msg import Odometry
from pyqtgraph.Qt import QtCore, QtWidgets
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

CONTROLLER = 'mevius_mpc_controller'
LEGS = ['leg1 (FR)', 'leg2 (FL)', 'leg3 (HR)', 'leg4 (HL)']
JOINTS = ['abduction', 'flex', 'knee']
JOINT_NAMES = [f'leg{i + 1}_{j}' for i in range(4) for j in JOINTS]
CONTACT_THRESHOLD_N = 5.0
TRUNK_COM = np.array([-0.0520205, 0.00128595, 0.509659])   # trunk link CoM, trunk frame (URDF)
SI_UNITS = ('m', 'N', 'm/s', 's')
# ~/diag_control layout (cheetah_controller.cpp, kControlDiagLayout).
FIELDS = [('t', 1), ('tau', 12), ('q', 12), ('qd', 12), ('p_des', 12), ('p', 12), ('v_des', 12),
          ('v', 12), ('swing', 4), ('f_mpc', 12), ('x_ref', 12), ('x_est', 12), ('v_cmd', 3),
          ('mpc', 3)]
IDX = {}
_k = 0
for _name, _n in FIELDS:
    IDX[_name] = slice(_k, _k + _n)
    _k += _n
CONTROL_SIZE = _k

COLORS = {'cmd': (31, 119, 180), 'meas': (255, 127, 14), 'ref': (44, 160, 44),
          'est': (214, 39, 40), 'truth': (90, 90, 90), 'limit': (200, 0, 0)}


class Ring:
    """Fixed-size time series buffer (rows = samples)."""

    def __init__(self, capacity, width):
        self.t = np.full(capacity, np.nan)
        self.x = np.full((capacity, width), np.nan)
        self.i = 0
        self.full = False
        self.lock = threading.Lock()

    def push(self, t, row):
        with self.lock:
            self.t[self.i] = t
            self.x[self.i] = row
            self.i = (self.i + 1) % len(self.t)
            self.full = self.full or self.i == 0

    def snapshot(self):
        with self.lock:
            if self.full:
                order = np.r_[self.i:len(self.t), 0:self.i]
                return self.t[order].copy(), self.x[order].copy()
            return self.t[:self.i].copy(), self.x[:self.i].copy()


class Listener(Node):
    def __init__(self, window):
        super().__init__('control_dashboard')
        self.control = Ring(int(window * 500) + 100, CONTROL_SIZE)
        self.joints = Ring(int(window * 1000) + 100, 12)
        self.bumpers = [Ring(int(window * 200) + 100, 1) for _ in range(4)]
        self.truth = Ring(int(window * 100) + 50, 3)   # forward speed, lateral speed, yaw rate
        self.create_subscription(Float64MultiArray, f'/{CONTROLLER}/diag_control', self.on_control, 100)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 100)
        self.create_subscription(Odometry, '/chien/ground_truth_odom', self.on_truth, qos_profile_sensor_data)
        for i in range(4):
            self.create_subscription(
                ContactsState, f'/chien/leg{i + 1}_foot_contact',
                lambda m, i=i: self.on_bumper(i, m), qos_profile_sensor_data)
        self._joint_index = None

    def on_control(self, msg):
        if len(msg.data) == CONTROL_SIZE:
            self.control.push(msg.data[0], np.asarray(msg.data))

    def on_joints(self, msg):
        if len(msg.effort) != len(msg.name):
            return
        if self._joint_index is None or len(self._joint_index) != 12:
            names = list(msg.name)
            if not all(n in names for n in JOINT_NAMES):
                return
            self._joint_index = [names.index(n) for n in JOINT_NAMES]
        effort = np.asarray(msg.effort)[self._joint_index]
        self.joints.push(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, effort)

    def on_bumper(self, leg, msg):
        # One state per physics step since the last message: average, do not sum.
        forces = [s.total_wrench.force for s in msg.states]
        f = np.mean([math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z) for w in forces]) if forces else 0.0
        self.bumpers[leg].push(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, [f])

    def on_truth(self, msg):
        q = msg.pose.pose.orientation
        R = np.array([
            [1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.w * q.z), 2 * (q.x * q.z + q.w * q.y)],
            [2 * (q.x * q.y + q.w * q.z), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z - q.w * q.x)],
            [2 * (q.x * q.z - q.w * q.y), 2 * (q.y * q.z + q.w * q.x), 1 - 2 * (q.x * q.x + q.y * q.y)]])
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        omega = np.array([w.x, w.y, w.z])
        # Ground truth is the trunk link ORIGIN (0.51 m below its CoM): move it to the CoM.
        v_com = np.array([v.x, v.y, v.z]) + np.cross(omega, R @ TRUNK_COM)
        # Robot forward = -x of the trunk link, left = -y.
        self.truth.push(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                        [v_com @ -R[:, 0], v_com @ -R[:, 1], w.z])


def body_frame(vx, vy, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return c * vx + s * vy, -s * vx + c * vy


class Dashboard(QtWidgets.QMainWindow):
    def __init__(self, node, window, tau_max):
        super().__init__()
        self.node, self.window, self.tau_max = node, window, tau_max
        self.paused = False
        self.setWindowTitle('quadru_control — control dashboard')
        pg.setConfigOptions(antialias=False, background='w', foreground='k')

        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        bar = QtWidgets.QHBoxLayout()
        self.pause_button = QtWidgets.QPushButton('Pause')
        self.pause_button.clicked.connect(self.toggle_pause)
        bar.addWidget(self.pause_button)
        bar.addWidget(QtWidgets.QLabel('Window (s)'))
        self.window_box = QtWidgets.QDoubleSpinBox()
        self.window_box.setRange(0.5, window)
        self.window_box.setValue(min(3.0, window))
        self.window_box.setSingleStep(0.5)
        bar.addWidget(self.window_box)
        self.status = QtWidgets.QLabel('waiting for /%s/diag_control ...' % CONTROLLER)
        bar.addWidget(self.status, 1)
        layout.addLayout(bar)
        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs)
        self.setCentralWidget(central)

        self.curves = {}
        self.plots = []
        self._span = None
        self._build_torque_tab()
        self._build_swing_tab()
        self._build_force_tab()
        self._build_body_tab()
        self._build_mpc_tab()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(40)   # 25 Hz

    # ------------------------------------------------------------------ layout
    def _plot(self, grid, row, col, title, unit):
        p = grid.addPlot(row=row, col=col, title=title)
        p.showGrid(x=True, y=True, alpha=0.3)
        if unit in SI_UNITS:
            p.setLabel('left', units=unit)
        else:
            p.setLabel('left', unit)
            p.getAxis('left').enableAutoSIPrefix(False)
        p.setLabel('bottom', 'time', units='s')
        p.setClipToView(True)
        p.setDownsampling(auto=True, mode='peak')
        p.addLegend(offset=(5, 5), labelTextSize='7pt')
        self.plots.append(p)
        return p

    def _curve(self, plot, key, name, color, style=QtCore.Qt.SolidLine, width=1.5):
        c = plot.plot([], [], name=name, pen=pg.mkPen(color, width=width, style=style))
        self.curves[key] = c
        return c

    def _tab(self, name):
        grid = pg.GraphicsLayoutWidget()
        self.tabs.addTab(grid, name)
        return grid

    def _build_torque_tab(self):
        grid = self._tab('Joint torque')
        for i in range(4):
            for j in range(3):
                p = self._plot(grid, i, j, f'{LEGS[i]} {JOINTS[j]}', 'N·m')
                k = 3 * i + j
                self._curve(p, ('tau_cmd', k), 'command', COLORS['cmd'])
                self._curve(p, ('tau_meas', k), 'measured', COLORS['meas'], QtCore.Qt.DashLine)
                for s in (1, -1):
                    p.addLine(y=s * self.tau_max, pen=pg.mkPen(COLORS['limit'], style=QtCore.Qt.DotLine))

    def _build_swing_tab(self):
        grid = self._tab('Swing tracking')
        for i in range(4):
            for a, axis in enumerate('xyz'):
                p = self._plot(grid, i, a, f'{LEGS[i]} foot {axis} (hip-relative)', 'm')
                self._curve(p, ('p_des', i, a), 'setpoint', COLORS['cmd'])
                self._curve(p, ('p', i, a), 'actual', COLORS['meas'], QtCore.Qt.DashLine)
            p = self._plot(grid, i, 3, f'{LEGS[i]} swing error |p_des - p|', 'mm')
            self._curve(p, ('swing_err', i), 'in swing', COLORS['est'])

    def _build_force_tab(self):
        grid = self._tab('Ground forces')
        for i in range(4):
            p = self._plot(grid, i, 0, f'{LEGS[i]} ground force |F|', 'N')
            self._curve(p, ('f_mpc', i), 'MPC', COLORS['cmd'])
            self._curve(p, ('f_meas', i), 'measured (bumper)', COLORS['meas'], QtCore.Qt.DashLine)
            p = self._plot(grid, i, 1, f'{LEGS[i]} MPC force components (world)', 'N')
            for a, axis in enumerate('xyz'):
                self._curve(p, ('f_mpc_axis', i, a), f'F{axis}', [COLORS['ref'], COLORS['truth'], COLORS['cmd']][a])
            p = self._plot(grid, i, 2, f'{LEGS[i]} contact', '0/1')
            p.setYRange(-0.1, 1.2)
            self._curve(p, ('c_plan', i), 'planned', COLORS['cmd'])
            self._curve(p, ('c_meas', i), 'measured', COLORS['meas'], QtCore.Qt.DashLine)

    def _build_body_tab(self):
        grid = self._tab('Body')
        specs = [('vfwd', 'forward speed', 'm/s', True, True), ('vlat', 'lateral speed', 'm/s', True, True),
                 ('wz', 'yaw rate', 'rad/s', True, True), ('z', 'CoM height (above foot plane)', 'm', False, False),
                 ('roll', 'roll', 'deg', False, False), ('pitch', 'pitch', 'deg', False, False)]
        for n, (key, title, unit, has_cmd, has_truth) in enumerate(specs):
            p = self._plot(grid, n // 2, n % 2, title, unit)
            if has_cmd:
                self._curve(p, (key, 'cmd'), 'command', (0, 0, 0), QtCore.Qt.DotLine)
            self._curve(p, (key, 'ref'), 'MPC reference', COLORS['ref'])
            self._curve(p, (key, 'est'), 'estimate', COLORS['est'])
            if has_truth:
                self._curve(p, (key, 'truth'), 'ground truth', COLORS['truth'], QtCore.Qt.DashLine)

    def _build_mpc_tab(self):
        grid = self._tab('MPC')
        p = self._plot(grid, 0, 0, 'QP solve time', 'ms')
        self._curve(p, ('mpc_ms',), 'solve time', COLORS['cmd'])
        self._curve(p, ('mpc_period',), 'MPC period', COLORS['limit'], QtCore.Qt.DashLine)
        p = self._plot(grid, 1, 0, 'QP status', '1 = solved')
        p.setYRange(-0.1, 1.2)
        self._curve(p, ('mpc_ok',), 'solved', COLORS['cmd'])

    # ------------------------------------------------------------------ update
    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.setText('Resume' if self.paused else 'Pause')

    def _set(self, key, t, y):
        self.curves[key].setData(t, y, connect='finite')

    def refresh(self):
        if self.paused:
            return
        t, x = self.node.control.snapshot()
        if len(t) < 2:
            return
        t_now = t[-1]
        span = self.window_box.value()
        if span != self._span:
            self._span = span
            for p in self.plots:
                p.setXRange(-span, 0.0, padding=0.0)
        m = t > t_now - span
        t, x = t[m] - t_now, x[m]
        f = {name: x[:, IDX[name]] for name, _ in FIELDS}
        self.status.setText(f'sim time {t_now:.2f} s   |   {len(t) / max(span, 1e-3):.0f} samples/s')

        # Joint torque
        tj, xj = self.node.joints.snapshot()
        mj = tj > t_now - span
        for k in range(12):
            self._set(('tau_cmd', k), t, f['tau'][:, k])
            self._set(('tau_meas', k), tj[mj] - t_now, xj[mj, k])

        # Swing tracking
        swing = f['swing'] > 0.5
        for i in range(4):
            err = np.linalg.norm(f['p_des'][:, 3 * i:3 * i + 3] - f['p'][:, 3 * i:3 * i + 3], axis=1) * 1000
            self._set(('swing_err', i), t, np.where(swing[:, i], err, np.nan))
            for a in range(3):
                self._set(('p_des', i, a), t, f['p_des'][:, 3 * i + a])
                self._set(('p', i, a), t, f['p'][:, 3 * i + a])

        # Ground forces
        for i in range(4):
            fm = f['f_mpc'][:, 3 * i:3 * i + 3]
            self._set(('f_mpc', i), t, np.linalg.norm(fm, axis=1))
            for a in range(3):
                self._set(('f_mpc_axis', i, a), t, fm[:, a])
            tb, xb = self.node.bumpers[i].snapshot()
            mb = tb > t_now - span
            self._set(('f_meas', i), tb[mb] - t_now, xb[mb, 0])
            self._set(('c_plan', i), t, (~swing[:, i]).astype(float))
            self._set(('c_meas', i), tb[mb] - t_now, (xb[mb, 0] > CONTACT_THRESHOLD_N).astype(float))

        # Body (MPC state order: roll, pitch, yaw, x, y, z, wx, wy, wz, vx, vy, vz)
        ref, est = f['x_ref'], f['x_est']
        yaw = est[:, 2]
        rf, rl = body_frame(ref[:, 9], ref[:, 10], yaw)
        ef, el = body_frame(est[:, 9], est[:, 10], yaw)
        self._set(('vfwd', 'cmd'), t, f['v_cmd'][:, 0])
        self._set(('vfwd', 'ref'), t, rf)
        self._set(('vfwd', 'est'), t, ef)
        self._set(('vlat', 'cmd'), t, f['v_cmd'][:, 1])
        self._set(('vlat', 'ref'), t, rl)
        self._set(('vlat', 'est'), t, el)
        self._set(('wz', 'cmd'), t, f['v_cmd'][:, 2])
        self._set(('wz', 'ref'), t, ref[:, 8])
        self._set(('wz', 'est'), t, est[:, 8])
        tt, xt = self.node.truth.snapshot()
        mt = tt > t_now - span
        for n, key in enumerate(('vfwd', 'vlat', 'wz')):
            self._set((key, 'truth'), tt[mt] - t_now, xt[mt, n])
        self._set(('z', 'ref'), t, ref[:, 5])
        self._set(('z', 'est'), t, est[:, 5])
        self._set(('roll', 'ref'), t, np.degrees(ref[:, 0]))
        self._set(('roll', 'est'), t, np.degrees(est[:, 0]))
        self._set(('pitch', 'ref'), t, np.degrees(ref[:, 1]))
        self._set(('pitch', 'est'), t, np.degrees(est[:, 1]))

        # MPC
        self._set(('mpc_ms',), t, f['mpc'][:, 1])
        self._set(('mpc_period',), t, f['mpc'][:, 2])
        self._set(('mpc_ok',), t, f['mpc'][:, 0])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--window', type=float, default=6.0, help='history kept (s)')
    parser.add_argument('--tau-max', type=float, default=35.0, help='torque limit drawn (N·m)')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = Listener(args.window)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    app = QtWidgets.QApplication([])
    win = Dashboard(node, args.window, args.tau_max)
    win.resize(1500, 950)
    win.show()
    app.exec_()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

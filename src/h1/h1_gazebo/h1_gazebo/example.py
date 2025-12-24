#!/usr/bin/env python3
import sys, math, time, threading
from typing import List, Dict
from functools import partial

# --- ROS2 ---
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# --- Qt ---
from PyQt5 import QtWidgets, QtCore

# ========================= ROS worker =========================
class PDWorker(Node):
    def __init__(self, controller_name: str, joint_order: List[str]):
        super().__init__('joint_pd_slider_gui')
        self.controller_name = controller_name
        self.order = joint_order
        self.N = len(self.order)
        self.name_to_idx: Dict[str,int] = {n:i for i,n in enumerate(self.order)}

        # pubs/subs
        self.pub = self.create_publisher(Float64MultiArray,
                                         f'/{self.controller_name}/commands', 10)
        self.sub = self.create_subscription(JointState, '/joint_states',
                                            self.js_cb, 100)

        # state
        self.q  = [0.0]*self.N
        self.dq = [0.0]*self.N
        self.dq_f = [0.0]*self.N
        self.have_state = False

        # targets (rad)
        self.q_des = [0.0]*self.N
        self.lock = threading.Lock()

        # default gains (Isaac-like)
        self.Kp = [80.0]*self.N
        self.Kd = [3.0]*self.N

        # guards
        self.dt = 0.01               # 100 Hz
        self.vel_tau = 0.02          # 20 ms LPF for dq
        self.alpha = self.dt/(self.vel_tau+self.dt)
        self.tau_max = [400.0]*self.N
        self.slew_max = 800.0        # Nm/s
        self.prev_tau = [0.0]*self.N

        # timer
        self.timer = self.create_timer(self.dt, self.step)
        self.get_logger().info(f"Publishing efforts to /{self.controller_name}/commands")

    def js_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        name_to_vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        for i,n in enumerate(self.order):
            self.q[i]  = float(name_to_pos.get(n, self.q[i]))
            self.dq[i] = float(name_to_vel.get(n, self.dq[i]))
        self.have_state = True

    def set_joint_target_deg(self, name: str, deg: float):
        # called from GUI thread
        with self.lock:
            self.q_des[self.name_to_idx[name]] = math.radians(deg)

    def set_group_gains(self, group: str, kp: float, kd: float):
        with self.lock:
            for i,n in enumerate(self.order):
                if group == 'legs' and any(s in n for s in ['hip','knee','ankle']):
                    self.Kp[i], self.Kd[i] = kp, kd
                elif group == 'arms' and any(s in n for s in ['shoulder','elbow','wrist']):
                    self.Kp[i], self.Kd[i] = kp, kd
                elif group == 'torso' and ('torso' in n):
                    self.Kp[i], self.Kd[i] = kp, kd

    def set_global_gains(self, kp: float, kd: float):
        with self.lock:
            for i in range(self.N):
                self.Kp[i], self.Kd[i] = kp, kd

    def step(self):
        if not self.have_state:
            return

        # filter velocities
        for i in range(self.N):
            self.dq_f[i] = (1.0-self.alpha)*self.dq_f[i] + self.alpha*self.dq[i]

        with self.lock:
            qd = list(self.q_des)
            Kp = list(self.Kp)
            Kd = list(self.Kd)

        tau_cmd = [0.0]*self.N
        for i in range(self.N):
            e  = qd[i] - self.q[i]
            ed = - self.dq_f[i]
            tau = Kp[i]*e + Kd[i]*ed
            # clamp + slew
            tmax = self.tau_max[i]
            tau = max(-tmax, min(tmax, tau))
            dmax = self.slew_max*self.dt
            tau = max(self.prev_tau[i]-dmax, min(self.prev_tau[i]+dmax, tau))
            self.prev_tau[i] = tau
            tau_cmd[i] = tau

        msg = Float64MultiArray()
        msg.data = tau_cmd
        self.pub.publish(msg)

# ========================= GUI =========================
class SliderGui(QtWidgets.QWidget):
    def __init__(self, worker: PDWorker, joint_order: List[str]):
        super().__init__()
        self.worker = worker
        self.order = joint_order

        self.setWindowTitle("Joint PD Slider GUI")
        self.setMinimumWidth(640)

        layout = QtWidgets.QVBoxLayout(self)

        # Gains controls
        gains_box = QtWidgets.QGroupBox("Gains")
        gb = QtWidgets.QGridLayout()
        self.global_kp = self._slider_with_label(0, 300, 80, "Kp global")
        self.global_kd = self._slider_with_label(0, 50, 3,   "Kd global")
        self.global_kp['slider'].valueChanged.connect(self._update_global_gains)
        self.global_kd['slider'].valueChanged.connect(self._update_global_gains)
        gb.addWidget(self.global_kp['widget'], 0, 0)
        gb.addWidget(self.global_kd['widget'], 0, 1)

        # group gains
        self.leg_kp = self._slider_with_label(0, 300, 110, "Kp legs")
        self.leg_kd = self._slider_with_label(0, 50, 4,    "Kd legs")
        self.arm_kp = self._slider_with_label(0, 200, 40,  "Kp arms")
        self.arm_kd = self._slider_with_label(0, 50,  2,   "Kd arms")
        self.torso_kp = self._slider_with_label(0, 200, 80, "Kp torso")
        self.torso_kd = self._slider_with_label(0, 50,  3,  "Kd torso")

        self.leg_kp['slider'].valueChanged.connect(self._update_group_gains)
        self.leg_kd['slider'].valueChanged.connect(self._update_group_gains)
        self.arm_kp['slider'].valueChanged.connect(self._update_group_gains)
        self.arm_kd['slider'].valueChanged.connect(self._update_group_gains)
        self.torso_kp['slider'].valueChanged.connect(self._update_group_gains)
        self.torso_kd['slider'].valueChanged.connect(self._update_group_gains)

        gb.addWidget(self.leg_kp['widget'],   1, 0)
        gb.addWidget(self.leg_kd['widget'],   1, 1)
        gb.addWidget(self.arm_kp['widget'],   2, 0)
        gb.addWidget(self.arm_kd['widget'],   2, 1)
        gb.addWidget(self.torso_kp['widget'], 3, 0)
        gb.addWidget(self.torso_kd['widget'], 3, 1)
        gains_box.setLayout(gb)
        layout.addWidget(gains_box)

        # Sliders per joint (degrees)
        joint_box = QtWidgets.QGroupBox("Joint targets (deg)")
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        jb_widget = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(jb_widget)

        self.joint_sliders = {}
        for i, name in enumerate(self.order):
            widget = self._slider_with_label(-180, 180, 0, name, unit="°")
            slider = widget['slider']
            label  = widget['value']
            slider.valueChanged.connect(partial(self._on_joint_change, name, label))
            row = i // 2
            col = (i % 2)
            grid.addWidget(widget['widget'], row, col)
            self.joint_sliders[name] = slider
        scroll.setWidget(jb_widget)
        vb = QtWidgets.QVBoxLayout()
        vb.addWidget(scroll)
        joint_box.setLayout(vb)
        layout.addWidget(joint_box)

        # presets
        btns = QtWidgets.QHBoxLayout()
        for text, func in [
            ("Zero", self._preset_zero),
            ("Neutral", self._preset_neutral),
            ("Crouch", self._preset_crouch),
        ]:
            b = QtWidgets.QPushButton(text); b.clicked.connect(func)
            btns.addWidget(b)
        layout.addLayout(btns)

        self.setLayout(layout)

        # initialize group gains in worker
        self._update_global_gains()
        self._update_group_gains()
        self._preset_neutral()  # start near neutral

    # ---- helpers ----
    def _slider_with_label(self, minv, maxv, init, title, unit=""):
        box = QtWidgets.QGroupBox(title)
        v = QtWidgets.QVBoxLayout()
        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal); slider.setMinimum(minv); slider.setMaximum(maxv); slider.setValue(init)
        value = QtWidgets.QLabel(f"{init}{unit}")
        slider.valueChanged.connect(lambda x: value.setText(f"{x}{unit}"))
        v.addWidget(slider); v.addWidget(value)
        box.setLayout(v)
        return {"widget": box, "slider": slider, "value": value}

    def _on_joint_change(self, name, value_label, val):
        self.worker.set_joint_target_deg(name, float(val))

    def _update_global_gains(self):
        kp = float(self.global_kp['slider'].value())
        kd = float(self.global_kd['slider'].value())
        self.worker.set_global_gains(kp, kd)

    def _update_group_gains(self):
        self.worker.set_group_gains('legs',  float(self.leg_kp['slider'].value()),   float(self.leg_kd['slider'].value()))
        self.worker.set_group_gains('arms',  float(self.arm_kp['slider'].value()),   float(self.arm_kd['slider'].value()))
        self.worker.set_group_gains('torso', float(self.torso_kp['slider'].value()), float(self.torso_kd['slider'].value()))

    # ---- presets ----
    def _preset_zero(self):
        for n,s in self.joint_sliders.items():
            s.setValue(0)

    def _preset_neutral(self):
        # mild stance
        self._preset_zero()
        for side in ('left','right'):
            self.joint_sliders[f'{side}_hip_pitch_joint'].setValue(15)
            self.joint_sliders[f'{side}_knee_joint'].setValue(30)
            self.joint_sliders[f'{side}_ankle_pitch_joint'].setValue(-15)

    def _preset_crouch(self):
        self._preset_zero()
        for side in ('left','right'):
            self.joint_sliders[f'{side}_hip_pitch_joint'].setValue(60)
            self.joint_sliders[f'{side}_knee_joint'].setValue(110)
            self.joint_sliders[f'{side}_ankle_pitch_joint'].setValue(-50)

# ========================= main =========================
def main():
    # --- joint order must match your controller YAML ---
    joint_order = [
        'torso_joint',
        'left_hip_yaw_joint','left_hip_pitch_joint','left_hip_roll_joint','left_knee_joint','left_ankle_pitch_joint','left_ankle_roll_joint',
        'right_hip_yaw_joint','right_hip_pitch_joint','right_hip_roll_joint','right_knee_joint','right_ankle_pitch_joint','right_ankle_roll_joint',
        'left_shoulder_pitch_joint','left_shoulder_roll_joint','left_shoulder_yaw_joint','left_elbow_joint','left_wrist_roll_joint','left_wrist_pitch_joint','left_wrist_yaw_joint',
        'right_shoulder_pitch_joint','right_shoulder_roll_joint','right_shoulder_yaw_joint','right_elbow_joint','right_wrist_roll_joint','right_wrist_pitch_joint','right_wrist_yaw_joint'
    ]
    controller_name = 'joint_effort_command_controller'

    # ROS init
    rclpy.init()

    # worker node + executor in background thread
    worker = PDWorker(controller_name, joint_order)
    executor = MultiThreadedExecutor()
    executor.add_node(worker)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Qt app
    app = QtWidgets.QApplication(sys.argv)
    gui = SliderGui(worker, joint_order)
    gui.show()
    ret = app.exec_()

    # shutdown
    executor.shutdown()
    worker.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)

if __name__ == '__main__':
    main()

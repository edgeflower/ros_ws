import signal
import sys

import rclpy

from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtWidgets import QMainWindow
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QLineEdit
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QGroupBox
from python_qt_binding.QtWidgets import QCheckBox
from python_qt_binding.QtWidgets import QSpinBox
from python_qt_binding.QtWidgets import QDoubleSpinBox
from python_qt_binding.QtWidgets import QComboBox
from control_panel.publisher import Panel_Publisher

RANGE = 10000
LINE_EDIT_WIDTH = 45

# Defined by style - currently using the default style
DEFAULT_WINDOW_MARGIN = 11
DEFAULT_CHILD_MARGIN = 9
DEFAULT_BTN_HEIGHT = 25

# Calculate default minimums for window sizing
MIN_WIDTH = DEFAULT_CHILD_MARGIN * 4 + DEFAULT_WINDOW_MARGIN * 2
MIN_HEIGHT = (
    DEFAULT_BTN_HEIGHT * 2 + DEFAULT_WINDOW_MARGIN * 2 + DEFAULT_CHILD_MARGIN * 2
)


class ControlPanelGui(QMainWindow):
    def __init__(self, title, publisher: Panel_Publisher):
        super(ControlPanelGui, self).__init__()

        self.setWindowTitle(title)
        
        self.title2=QLabel(self)
        self.title2.setText("GAME STATUS")
        self.title2.setStyleSheet("font-size: 15px;") # color: red;

        self.text4=QLabel(self)
        self.text4.setText("game_progess")
        self.game_progess_text = QLabel(self)
        self.game_progess_text.setText("GAME_UNSTARTED")
        self.game_progess_text.setStyleSheet("color: blue;") # color: red;
        self.game_progess = QLineEdit(self)
        self.game_progess.setText("0")

        self.text5=QLabel(self)
        self.text5.setText("stage_remaining_time")
        self.timer_text = QLabel(self)
        self.timer_text.setText("0")
        self.timer_text.setStyleSheet("color: red;") # color: red;
        self.stage_remaining_time = QLineEdit(self)
        self.stage_remaining_time.setText("32767")

        self.text12=QLabel(self)
        self.text12.setText("current_hp")
        self.current_hp = QLineEdit(self)
        self.current_hp.setText("600")
        self.PubGameStatusButton=QPushButton("set game status", self)
        self.PubGameStatusButton.clicked.connect(self.PubGameStatusEvent)

        self.text6=QLabel(self)
        self.text6.setText("bullet_remaining_num_17mm")
        self.bullet_remaining_num_17mm = QLineEdit(self)
        self.bullet_remaining_num_17mm.setText("400")
        
        self.text7=QLabel(self)
        self.text7.setText("my_outpost_hp")
        self.my_outpost_hp=QLineEdit(self)
        self.my_outpost_hp.setText("1000")

        self.text8=QLabel(self)
        self.text8.setText("enemy_outpost_hp")
        self.enemy_outpost_hp=QLineEdit(self)
        self.enemy_outpost_hp.setText("1000")

        self.text9=QLabel(self)
        self.text9.setText("my_base_hp")
        self.my_base_hp=QLineEdit(self)
        self.my_base_hp.setText("1000")

        self.text10=QLabel(self)
        self.text10.setText("enemy_base_hp")
        self.enemy_base_hp=QLineEdit(self)
        self.enemy_base_hp.setText("1000")

        # ==================== 决策系统控制面板 ====================
        self.decision_group = QGroupBox("DECISION SYSTEM", self)
        self.decision_group.setStyleSheet("font-size: 15px; font-weight: bold;")

        # 机器人状态显示
        self.robot_status_text = QLabel(self)
        self.robot_status_text.setText("STATUS: --")
        self.robot_status_text.setStyleSheet("color: orange;")

        # 姿态显示
        self.posture_text = QLabel(self)
        self.posture_text.setText("POSTURE: --")
        self.posture_text.setStyleSheet("color: blue; font-weight: bold;")

        # 姿态控制按钮
        self.posture_buttons_layout = QHBoxLayout()

        self.btn_posture_offense = QPushButton("OFFENSE (1)", self)
        self.btn_posture_offense.setStyleSheet("background-color: #ffcccc;")
        self.btn_posture_offense.clicked.connect(lambda: self.set_posture(1))

        self.btn_posture_defense = QPushButton("DEFENSE (2)", self)
        self.btn_posture_defense.setStyleSheet("background-color: #ccffcc;")
        self.btn_posture_defense.clicked.connect(lambda: self.set_posture(2))

        self.btn_posture_patrol = QPushButton("PATROL (3)", self)
        self.btn_posture_patrol.setStyleSheet("background-color: #ccccff;")
        self.btn_posture_patrol.clicked.connect(lambda: self.set_posture(3))

        self.posture_buttons_layout.addWidget(self.btn_posture_offense)
        self.posture_buttons_layout.addWidget(self.btn_posture_defense)
        self.posture_buttons_layout.addWidget(self.btn_posture_patrol)

        # 模拟控制区域
        self.sim_group = QGroupBox("SIMULATION", self)

        # 热量控制
        self.heat_layout = QHBoxLayout()
        self.heat_label = QLabel("Shooter Heat:")
        self.heat_spinbox = QSpinBox(self)
        self.heat_spinbox.setRange(0, 1000)
        self.heat_spinbox.setValue(0)
        self.heat_spinbox.valueChanged.connect(self.update_robot_status)
        self.heat_layout.addWidget(self.heat_label)
        self.heat_layout.addWidget(self.heat_spinbox)

        # 被攻击控制
        self.attacked_checkbox = QCheckBox("Is Attacked", self)
        self.attacked_checkbox.setStyleSheet("color: red; font-weight: bold;")
        self.attacked_checkbox.stateChanged.connect(self.update_robot_status)

        # 弹丸余量控制
        self.shot_layout = QHBoxLayout()
        self.shot_label = QLabel("Shot Allowance:")
        self.shot_spinbox = QSpinBox(self)
        self.shot_spinbox.setRange(0, 10000)
        self.shot_spinbox.setValue(400)
        self.shot_spinbox.valueChanged.connect(self.update_robot_status)
        self.shot_layout.addWidget(self.shot_label)
        self.shot_layout.addWidget(self.shot_spinbox)

        # 敌人目标检测控制区域，对应 /target_tracking 的 armor_interfaces/Target
        self.armor_group = QGroupBox("ENEMY TARGET (/target_tracking)", self)

        self.enemy_target_text = QLabel(self)
        self.enemy_target_text.setText("ENEMY: NO TARGET")
        self.enemy_target_text.setStyleSheet("color: gray;")

        # 跟踪状态和元数据
        self.tracking_checkbox = QCheckBox("Tracking Target", self)
        self.tracking_checkbox.setStyleSheet("color: blue; font-weight: bold;")
        self.tracking_checkbox.stateChanged.connect(self.update_armor_target)

        self.target_meta_layout = QHBoxLayout()
        self.target_id_label = QLabel("ID:")
        self.target_id_combo = QComboBox(self)
        self.target_id_combo.setEditable(True)
        self.target_id_combo.addItems(["", "1", "2", "3", "4", "5", "7", "outpost", "base"])
        self.target_id_combo.currentTextChanged.connect(self.update_armor_target)
        self.tracking_status_label = QLabel("Status:")
        self.tracking_status_spinbox = QSpinBox(self)
        self.tracking_status_spinbox.setRange(0, 10)
        self.tracking_status_spinbox.setValue(1)
        self.tracking_status_spinbox.valueChanged.connect(self.update_armor_target)
        self.armors_num_label = QLabel("Armors:")
        self.armors_num_spinbox = QSpinBox(self)
        self.armors_num_spinbox.setRange(0, 8)
        self.armors_num_spinbox.setValue(1)
        self.armors_num_spinbox.valueChanged.connect(self.update_armor_target)
        self.target_meta_layout.addWidget(self.target_id_label)
        self.target_meta_layout.addWidget(self.target_id_combo)
        self.target_meta_layout.addWidget(self.tracking_status_label)
        self.target_meta_layout.addWidget(self.tracking_status_spinbox)
        self.target_meta_layout.addWidget(self.armors_num_label)
        self.target_meta_layout.addWidget(self.armors_num_spinbox)

        # 置信度控制：ConfidenceHysteresis 在行为树里用它进入 HIGH/GRACE_PERIOD/LOW
        self.confidence_layout = QHBoxLayout()
        self.confidence_label = QLabel("Confidence:")
        self.confidence_spinbox = QSpinBox(self)
        self.confidence_spinbox.setRange(0, 100)
        self.confidence_spinbox.setValue(0)
        self.confidence_spinbox.setSuffix("%")
        self.confidence_spinbox.valueChanged.connect(self.update_armor_target)
        self.confidence_layout.addWidget(self.confidence_label)
        self.confidence_layout.addWidget(self.confidence_spinbox)

        # 目标位置控制，单位按行为树滤波节点使用的米处理
        self.pos_layout = QHBoxLayout()
        self.pos_label = QLabel("Position (m):")
        self.pos_x = QDoubleSpinBox(self)
        self.pos_x.setRange(-20.0, 20.0)
        self.pos_x.setDecimals(2)
        self.pos_x.setSingleStep(0.1)
        self.pos_x.setPrefix("x:")
        self.pos_x.valueChanged.connect(self.update_armor_target)
        self.pos_y = QDoubleSpinBox(self)
        self.pos_y.setRange(-20.0, 20.0)
        self.pos_y.setDecimals(2)
        self.pos_y.setSingleStep(0.1)
        self.pos_y.setPrefix("y:")
        self.pos_y.valueChanged.connect(self.update_armor_target)
        self.pos_z = QDoubleSpinBox(self)
        self.pos_z.setRange(-5.0, 5.0)
        self.pos_z.setDecimals(2)
        self.pos_z.setSingleStep(0.1)
        self.pos_z.setPrefix("z:")
        self.pos_z.valueChanged.connect(self.update_armor_target)
        self.pos_layout.addWidget(self.pos_label)
        self.pos_layout.addWidget(self.pos_x)
        self.pos_layout.addWidget(self.pos_y)
        self.pos_layout.addWidget(self.pos_z)

        self.vel_layout = QHBoxLayout()
        self.vel_label = QLabel("Velocity:")
        self.vel_x = QDoubleSpinBox(self)
        self.vel_x.setRange(-20.0, 20.0)
        self.vel_x.setDecimals(2)
        self.vel_x.setSingleStep(0.1)
        self.vel_x.setPrefix("vx:")
        self.vel_x.valueChanged.connect(self.update_armor_target)
        self.vel_y = QDoubleSpinBox(self)
        self.vel_y.setRange(-20.0, 20.0)
        self.vel_y.setDecimals(2)
        self.vel_y.setSingleStep(0.1)
        self.vel_y.setPrefix("vy:")
        self.vel_y.valueChanged.connect(self.update_armor_target)
        self.vel_z = QDoubleSpinBox(self)
        self.vel_z.setRange(-20.0, 20.0)
        self.vel_z.setDecimals(2)
        self.vel_z.setSingleStep(0.1)
        self.vel_z.setPrefix("vz:")
        self.vel_z.valueChanged.connect(self.update_armor_target)
        self.vel_layout.addWidget(self.vel_label)
        self.vel_layout.addWidget(self.vel_x)
        self.vel_layout.addWidget(self.vel_y)
        self.vel_layout.addWidget(self.vel_z)

        self.armor_motion_layout = QHBoxLayout()
        self.yaw_label = QLabel("Yaw:")
        self.yaw_spinbox = QDoubleSpinBox(self)
        self.yaw_spinbox.setRange(-31.42, 31.42)
        self.yaw_spinbox.setDecimals(3)
        self.yaw_spinbox.setSingleStep(0.1)
        self.yaw_spinbox.valueChanged.connect(self.update_armor_target)
        self.v_yaw_label = QLabel("VYaw:")
        self.v_yaw_spinbox = QDoubleSpinBox(self)
        self.v_yaw_spinbox.setRange(-31.42, 31.42)
        self.v_yaw_spinbox.setDecimals(3)
        self.v_yaw_spinbox.setSingleStep(0.1)
        self.v_yaw_spinbox.valueChanged.connect(self.update_armor_target)
        self.radius_1_label = QLabel("R1:")
        self.radius_1_spinbox = QDoubleSpinBox(self)
        self.radius_1_spinbox.setRange(0.0, 5.0)
        self.radius_1_spinbox.setDecimals(2)
        self.radius_1_spinbox.setSingleStep(0.05)
        self.radius_1_spinbox.valueChanged.connect(self.update_armor_target)
        self.radius_2_label = QLabel("R2:")
        self.radius_2_spinbox = QDoubleSpinBox(self)
        self.radius_2_spinbox.setRange(0.0, 5.0)
        self.radius_2_spinbox.setDecimals(2)
        self.radius_2_spinbox.setSingleStep(0.05)
        self.radius_2_spinbox.valueChanged.connect(self.update_armor_target)
        self.dz_label = QLabel("dz:")
        self.dz_spinbox = QDoubleSpinBox(self)
        self.dz_spinbox.setRange(-5.0, 5.0)
        self.dz_spinbox.setDecimals(2)
        self.dz_spinbox.setSingleStep(0.05)
        self.dz_spinbox.valueChanged.connect(self.update_armor_target)
        self.armor_motion_layout.addWidget(self.yaw_label)
        self.armor_motion_layout.addWidget(self.yaw_spinbox)
        self.armor_motion_layout.addWidget(self.v_yaw_label)
        self.armor_motion_layout.addWidget(self.v_yaw_spinbox)
        self.armor_motion_layout.addWidget(self.radius_1_label)
        self.armor_motion_layout.addWidget(self.radius_1_spinbox)
        self.armor_motion_layout.addWidget(self.radius_2_label)
        self.armor_motion_layout.addWidget(self.radius_2_spinbox)
        self.armor_motion_layout.addWidget(self.dz_label)
        self.armor_motion_layout.addWidget(self.dz_spinbox)

        self.enemy_preset_layout = QHBoxLayout()
        self.btn_enemy_high = QPushButton("HIGH TARGET", self)
        self.btn_enemy_high.setStyleSheet("background-color: #ccffcc;")
        self.btn_enemy_high.clicked.connect(
            lambda: self.apply_enemy_preset(True, 80, 2.0, 0.0, 0.0, "1", 1, 1))
        self.btn_enemy_low = QPushButton("LOW CONF", self)
        self.btn_enemy_low.setStyleSheet("background-color: #fff2cc;")
        self.btn_enemy_low.clicked.connect(
            lambda: self.apply_enemy_preset(True, 20, 2.0, 0.0, 0.0, "1", 1, 1))
        self.btn_enemy_clear = QPushButton("NO TARGET", self)
        self.btn_enemy_clear.setStyleSheet("background-color: #eeeeee;")
        self.btn_enemy_clear.clicked.connect(
            lambda: self.apply_enemy_preset(False, 0, 0.0, 0.0, 0.0, "", 0, 0))
        self.enemy_preset_layout.addWidget(self.btn_enemy_high)
        self.enemy_preset_layout.addWidget(self.btn_enemy_low)
        self.enemy_preset_layout.addWidget(self.btn_enemy_clear)

        # 装甲板控制布局
        self.armor_layout = QVBoxLayout()
        self.armor_layout.addWidget(self.enemy_target_text)
        self.armor_layout.addWidget(self.tracking_checkbox)
        self.armor_layout.addLayout(self.target_meta_layout)
        self.armor_layout.addLayout(self.confidence_layout)
        self.armor_layout.addLayout(self.pos_layout)
        self.armor_layout.addLayout(self.vel_layout)
        self.armor_layout.addLayout(self.armor_motion_layout)
        self.armor_layout.addLayout(self.enemy_preset_layout)
        self.armor_group.setLayout(self.armor_layout)

        # ==================== RFID 控制区域 ====================
        self.rfid_group = QGroupBox("RFID (/rfid)", self)
        self.rfid_checkboxes = {}

        self.rfid_field_groups = {
            '基础点': [
                ('base_self', '己基地'), ('highland_self', '己高地'),
                ('highland_enemy', '敌高地'), ('slope_self', '己斜坡'),
                ('slope_enemy', '敌斜坡'),
            ],
            '飞坡': [
                ('fly_self_front', '己前'), ('fly_self_back', '己后'),
                ('fly_enemy_front', '敌前'), ('fly_enemy_back', '敌后'),
            ],
            '中央高地': [
                ('center_low_self', '己低'), ('center_high_self', '己高'),
                ('center_low_enemy', '敌低'), ('center_high_enemy', '敌高'),
            ],
            '公路': [
                ('road_low_self', '己低'), ('road_high_self', '己高'),
                ('road_low_enemy', '敌低'), ('road_high_enemy', '敌高'),
            ],
            '战略点': [
                ('fortress_self', '己堡垒'), ('outpost_self', '己前哨'),
                ('resource_isolated', '资源孤'), ('resource_overlap', '资源叠'),
                ('supply_self', '己补给'), ('supply_enemy', '敌补给'),
                ('center_bonus', '中央奖励'),
            ],
            '敌方': [
                ('fortress_enemy', '敌堡垒'), ('outpost_enemy', '敌前哨'),
            ],
            '隧道己': [
                ('tunnel_self_1', '1'), ('tunnel_self_2', '2'),
                ('tunnel_self_3', '3'), ('tunnel_self_4', '4'),
                ('tunnel_self_5', '5'), ('tunnel_self_6', '6'),
            ],
            '隧道敌': [
                ('tunnel_enemy_1', '1'), ('tunnel_enemy_2', '2'),
                ('tunnel_enemy_3', '3'), ('tunnel_enemy_4', '4'),
                ('tunnel_enemy_5', '5'), ('tunnel_enemy_6', '6'),
            ],
        }

        rfid_layout = QVBoxLayout()
        for group_name, fields in self.rfid_field_groups.items():
            row = QHBoxLayout()
            group_label = QLabel(f"{group_name}:")
            group_label.setStyleSheet("font-weight: bold;")
            group_label.setFixedWidth(60)
            row.addWidget(group_label)
            for field_name, label in fields:
                cb = QCheckBox(label, self)
                cb.stateChanged.connect(self.update_rfid)
                self.rfid_checkboxes[field_name] = cb
                row.addWidget(cb)
            row.addStretch()
            rfid_layout.addLayout(row)

        self.rfid_clear_btn = QPushButton("Clear All", self)
        self.rfid_clear_btn.clicked.connect(self.clear_rfid)
        rfid_layout.addWidget(self.rfid_clear_btn)

        self.rfid_group.setLayout(rfid_layout)

        # 模拟控制布局
        self.sim_layout = QVBoxLayout()
        self.sim_layout.addLayout(self.heat_layout)
        self.sim_layout.addWidget(self.attacked_checkbox)
        self.sim_layout.addLayout(self.shot_layout)
        self.sim_group.setLayout(self.sim_layout)

        # 决策系统布局
        self.decision_layout = QVBoxLayout()
        self.decision_layout.addWidget(self.robot_status_text)
        self.decision_layout.addWidget(self.posture_text)
        self.decision_layout.addLayout(self.posture_buttons_layout)
        self.decision_group.setLayout(self.decision_layout)

        # 左列：GAME STATUS + ENEMY TARGET
        self.left_layout = QVBoxLayout()
        self.left_layout.addWidget(self.title2)
        self.left_layout.addWidget(self.text4)
        self.left_layout.addWidget(self.game_progess_text)
        self.left_layout.addWidget(self.game_progess)
        self.left_layout.addWidget(self.text5)
        self.left_layout.addWidget(self.timer_text)
        self.left_layout.addWidget(self.stage_remaining_time)
        self.left_layout.addWidget(self.text12)
        self.left_layout.addWidget(self.current_hp)
        self.left_layout.addWidget(self.text6)
        self.left_layout.addWidget(self.bullet_remaining_num_17mm)
        self.left_layout.addWidget(self.text7)
        self.left_layout.addWidget(self.my_outpost_hp)
        self.left_layout.addWidget(self.text8)
        self.left_layout.addWidget(self.enemy_outpost_hp)
        self.left_layout.addWidget(self.text9)
        self.left_layout.addWidget(self.my_base_hp)
        self.left_layout.addWidget(self.text10)
        self.left_layout.addWidget(self.enemy_base_hp)
        self.left_layout.addWidget(self.PubGameStatusButton)
        self.left_layout.addWidget(self.armor_group)

        # 右列：DECISION SYSTEM + SIMULATION + RFID
        self.right_layout = QVBoxLayout()
        self.right_layout.addWidget(self.decision_group)
        self.right_layout.addWidget(self.sim_group)
        self.right_layout.addWidget(self.rfid_group)

        # 双列拼合
        self.columns_layout = QHBoxLayout()
        self.columns_layout.addLayout(self.left_layout)
        self.columns_layout.addLayout(self.right_layout)

        self.central_widget = QWidget()
        self.central_widget.setLayout(self.columns_layout)
        self.setCentralWidget(self.central_widget)
        self.publisher = publisher


    def PubGameStatusEvent(self,event):
        self.publisher.get_logger().info("Pub Game Status!")
        self.timer_text.setText(self.stage_remaining_time.text())
        self.publisher.set_game_state(
            int(self.game_progess.text()),
            #uint16 stage_remain_time
            int(self.stage_remaining_time.text()),
            #uint16 current_hp
            int(self.current_hp.text()),
            #uint16 projectile_allowance_17mm
            int(self.bullet_remaining_num_17mm.text()),
            #uint16 my_outpost_hp
            int(self.my_outpost_hp.text()),
            #uint16 enemy_outpost_hp
            int(self.enemy_outpost_hp.text()),
            #uint16 my_base_hp
            int(self.my_base_hp.text()),
            #uint16 enemy_base_hp
            int(self.enemy_base_hp.text())
        )

    def set_posture(self, posture):
        """设置机器人姿态"""
        self.publisher.set_posture_cmd(posture)

    def update_robot_status(self):
        """更新机器人状态模拟数据"""
        shooter_heat = self.heat_spinbox.value()
        is_attacked = self.attacked_checkbox.isChecked()
        shot_allowance = self.shot_spinbox.value()
        self.publisher.set_robot_status(shooter_heat, is_attacked, shot_allowance)

    def update_rfid(self):
        """更新 RFID 数据"""
        if not hasattr(self, 'publisher'):
            return
        rfid_dict = {name: cb.isChecked() for name, cb in self.rfid_checkboxes.items()}
        self.publisher.set_rfid(**rfid_dict)

    def clear_rfid(self):
        """清除所有 RFID 标志"""
        for cb in self.rfid_checkboxes.values():
            cb.blockSignals(True)
            cb.setChecked(False)
        for cb in self.rfid_checkboxes.values():
            cb.blockSignals(False)
        self.update_rfid()

    def update_armor_target(self):
        """更新敌人目标模拟数据"""
        if not hasattr(self, "publisher"):
            return

        self.publisher.set_armor_target(
            tracking=self.tracking_checkbox.isChecked(),
            confidence=self.confidence_spinbox.value() / 100.0,
            x=float(self.pos_x.value()),
            y=float(self.pos_y.value()),
            z=float(self.pos_z.value()),
            target_id=self.target_id_combo.currentText().strip(),
            tracking_status=int(self.tracking_status_spinbox.value()),
            armors_num=int(self.armors_num_spinbox.value()),
            vx=float(self.vel_x.value()),
            vy=float(self.vel_y.value()),
            vz=float(self.vel_z.value()),
            yaw=float(self.yaw_spinbox.value()),
            v_yaw=float(self.v_yaw_spinbox.value()),
            radius_1=float(self.radius_1_spinbox.value()),
            radius_2=float(self.radius_2_spinbox.value()),
            dz=float(self.dz_spinbox.value())
        )

    def apply_enemy_preset(self, tracking, confidence_percent, x, y, z,
                           target_id, tracking_status, armors_num):
        """应用敌人目标快捷状态"""
        widgets = [
            self.tracking_checkbox, self.confidence_spinbox, self.pos_x, self.pos_y,
            self.pos_z, self.target_id_combo, self.tracking_status_spinbox,
            self.armors_num_spinbox
        ]
        for widget in widgets:
            widget.blockSignals(True)

        self.tracking_checkbox.setChecked(tracking)
        self.confidence_spinbox.setValue(confidence_percent)
        self.pos_x.setValue(x)
        self.pos_y.setValue(y)
        self.pos_z.setValue(z)
        self.target_id_combo.setCurrentText(target_id)
        self.tracking_status_spinbox.setValue(tracking_status)
        self.armors_num_spinbox.setValue(armors_num)

        for widget in widgets:
            widget.blockSignals(False)

        self.update_armor_target()

    def update_loop(self):
        rclpy.spin_once(self.publisher, timeout_sec=0.0)

        self.timer_text.setText(str(self.publisher.game_status.stage_remain_time))

        progress = self.publisher.game_status.game_progress
        if progress == 0:
            self.game_progess_text.setText("GAME_UNSTARTED")
        elif progress == 1:
            self.game_progess_text.setText("GAME_READY")
        elif progress == 2:
            self.game_progess_text.setText("GAME_INITIAL")
        elif progress == 3:
            self.game_progess_text.setText("GAME_START_COUNTDOWN")
        elif progress == 4:
            self.game_progess_text.setText("GAME_RUNNING")
        elif progress == 5:
            self.game_progess_text.setText("GAME_STOP")
        else:
            self.game_progess_text.setText("UNKNOWN")

        self.robot_status_text.setText(
            f"STATUS: HP:{self.publisher.robot_status.current_hp} | "
            f"Heat:{self.publisher.robot_status.shooter_heat} | "
            f"{'ATTACKED' if self.publisher.robot_status.is_attacked else 'OK'}"
        )

        self.posture_text.setText(f"POSTURE: {self.publisher.get_posture_text()}")

        target = self.publisher.armor_target
        target_state = "TRACKING" if target.tracking else "NO TARGET"
        self.enemy_target_text.setText(
            f"ENEMY: {target_state} | id:{target.id or '--'} | "
            f"status:{target.tracking_status} | armors:{target.armors_num} | "
            f"conf:{target.confidence:.2f} | "
            f"pos:({target.position.x:.2f}, {target.position.y:.2f}, {target.position.z:.2f})"
        )
        if target.confidence >= 0.5:
            self.enemy_target_text.setStyleSheet("color: green; font-weight: bold;")
        elif target.confidence > 0.0:
            self.enemy_target_text.setStyleSheet("color: orange; font-weight: bold;")
        else:
            self.enemy_target_text.setStyleSheet("color: gray;")

        if self.publisher.robot_status.is_attacked:
            self.robot_status_text.setStyleSheet("color: red; font-weight: bold;")
        else:
            self.robot_status_text.setStyleSheet("color: green;")

def main():
    rclpy.init()

    app = QApplication(sys.argv)
    panel_pub = Panel_Publisher()
    panel = ControlPanelGui("Control Panel", panel_pub)

    panel.show()

    timer = QTimer()
    timer.timeout.connect(panel.update_loop)
    timer.start(10)

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    ret = app.exec_()
    timer.stop()
    panel_pub.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()

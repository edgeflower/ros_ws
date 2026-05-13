import rclpy
from rclpy.node import Node
from rm_decision_interfaces.msg import (
    GameStatus, AllRobotHP, RobotEconomy,
    RobotStatus, SentryPostureStatus, SentryPostureCmd,
    FriendLocation, RFIDParse
)
from rm_decision_interfaces.srv import SetSentryPosture
from armor_interfaces.msg import Armor, Armors, Target
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header


class Panel_Publisher(Node):

    def __init__(self, node_name='control_panel_pub'):
        super().__init__(node_name)

        # Game Status publishers
        self.game_status_publisher_ = self.create_publisher(GameStatus, 'game_status', 10)
        self.all_robot_hp_publisher_ = self.create_publisher(AllRobotHP, 'all_robot_hp', 10)
        self.robot_economy_publisher_ = self.create_publisher(RobotEconomy, 'robot_economy', 10)

        # Robot Location publisher (对应参考节点 publishRobotLocation)
        self.robot_location_publisher_ = self.create_publisher(FriendLocation, 'robot_location', 10)

        # RFID publisher (对应参考节点 publishRfid)
        self.rfid_publisher_ = self.create_publisher(RFIDParse, 'rfid', 10)

        self.timer_ = self.create_timer(1, self.timer_callback)

        # Decision system publishers
        self.posture_cmd_publisher_ = self.create_publisher(SentryPostureCmd, 'sentry_posture', 10)
        self.robot_status_publisher_ = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.armor_target_publisher_ = self.create_publisher(Target, 'target_tracking', 10)

        # SentryPostureStatus publisher (对应参考节点 publishRobotInfo 中发布)
        self.posture_status_publisher_ = self.create_publisher(
            SentryPostureStatus, 'sentry_posture_status', 10)

        # SetSentryPosture service server (对应参考节点 handleSetSentryPosture)
        self.set_posture_service_ = self.create_service(
            SetSentryPosture, 'set_sentry_posture', self.set_posture_callback)

        # Decision system subscribers (用于接收真实系统状态，可选)
        self.posture_status_subscriber_ = self.create_subscription(
            SentryPostureStatus,
            'sentry_posture_status',
            self.posture_status_callback,
            10
        )

        # Game Status data
        self.game_status = GameStatus()
        self.game_status.game_progress = 0
        self.game_status.stage_remain_time = 32767

        self.all_robot_hp = AllRobotHP()
        self.all_robot_hp.red_1_robot_hp = 200
        self.all_robot_hp.red_2_robot_hp = 200
        self.all_robot_hp.red_3_robot_hp = 200
        self.all_robot_hp.red_4_robot_hp = 200
        self.all_robot_hp.red_7_robot_hp = 700
        self.all_robot_hp.red_outpost_hp = 1500
        self.all_robot_hp.red_base_hp = 4000
        self.all_robot_hp.blue_1_robot_hp = 200
        self.all_robot_hp.blue_2_robot_hp = 200
        self.all_robot_hp.blue_3_robot_hp = 200
        self.all_robot_hp.blue_4_robot_hp = 200
        self.all_robot_hp.blue_7_robot_hp = 400
        self.all_robot_hp.blue_outpost_hp = 1500
        self.all_robot_hp.blue_base_hp = 4000

        self.robot_economy = RobotEconomy()
        self.robot_economy.projectile_allowance_17mm = 400
        self.robot_economy.remaining_gold_coin = 0

        # Robot Status
        self.robot_status = RobotStatus()
        self.robot_status.robot_id = 7
        self.robot_status.current_hp = 400
        self.robot_status.shooter_heat = 0
        self.robot_status.team_color = False
        self.robot_status.is_attacked = False
        self.robot_status.shot_allowance = 300

        # Posture Status
        self.posture_status = SentryPostureStatus()

        # Robot Location
        self.robot_location = FriendLocation()

        # RFID
        self.rfid = RFIDParse()

        # Armor Target
        self.armor_target = Target()
        self.armor_target.header = Header()
        self.armor_target.header.stamp = self.get_clock().now().to_msg()
        self.armor_target.header.frame_id = "gimbal_yaw"
        self.armor_target.tracking = False
        self.armor_target.tracking_status = 0
        self.armor_target.confidence = 0.0
        self.armor_target.id = ""
        self.armor_target.armors_num = 0
        self.armor_target.position = Point()
        self.armor_target.velocity = Vector3()
        self.armor_target.yaw = 0.0
        self.armor_target.v_yaw = 0.0
        self.armor_target.radius_1 = 0.0
        self.armor_target.radius_2 = 0.0
        self.armor_target.dz = 0.0

        # UI references
        self.current_hp = 400
        self.my_outpost_hp = 1500
        self.enemy_outpost_hp = 1500
        self.my_base_hp = 4000
        self.enemy_base_hp = 4000

    def set_game_state(self, game_progress, stage_remain_time, current_hp,
                       projectile_allowance_17mm, my_outpost_hp, enemy_outpost_hp,
                       my_base_hp, enemy_base_hp):
        """设置游戏状态数据"""
        self.game_status.game_progress = game_progress
        self.game_status.stage_remain_time = stage_remain_time

        self.all_robot_hp.red_7_robot_hp = current_hp
        self.all_robot_hp.red_outpost_hp = my_outpost_hp
        self.all_robot_hp.red_base_hp = my_base_hp
        self.all_robot_hp.blue_outpost_hp = enemy_outpost_hp
        self.all_robot_hp.blue_base_hp = enemy_base_hp

        self.robot_economy.projectile_allowance_17mm = projectile_allowance_17mm

        self.robot_status.current_hp = current_hp
        self.robot_status.shot_allowance = projectile_allowance_17mm

        self.current_hp = current_hp
        self.my_outpost_hp = my_outpost_hp
        self.enemy_outpost_hp = enemy_outpost_hp
        self.my_base_hp = my_base_hp
        self.enemy_base_hp = enemy_base_hp

    def set_robot_status(self, shooter_heat=0, is_attacked=False, shot_allowance=400):
        """设置机器人状态"""
        self.robot_status.shooter_heat = shooter_heat
        self.robot_status.is_attacked = is_attacked
        self.robot_status.shot_allowance = shot_allowance

    def set_rfid(self, **kwargs):
        """设置 RFID 数据"""
        for key, value in kwargs.items():
            if hasattr(self.rfid, key):
                setattr(self.rfid, key, bool(value))

    def set_armor_target(self, tracking=False, confidence=0.0, x=0.0, y=0.0, z=0.0,
                         target_id="", tracking_status=0, armors_num=None,
                         vx=0.0, vy=0.0, vz=0.0, yaw=0.0, v_yaw=0.0,
                         radius_1=0.0, radius_2=0.0, dz=0.0,
                         frame_id="gimbal_yaw"):
        """设置敌人目标（模拟 /target_tracking 的 armor_interfaces/Target）"""
        if armors_num is None:
            armors_num = 1 if tracking else 0

        self.armor_target.header.stamp = self.get_clock().now().to_msg()
        self.armor_target.header.frame_id = frame_id
        self.armor_target.tracking = bool(tracking)
        self.armor_target.tracking_status = int(tracking_status)
        self.armor_target.confidence = max(0.0, min(1.0, float(confidence)))
        self.armor_target.id = str(target_id)
        self.armor_target.armors_num = int(armors_num)

        self.armor_target.position.x = float(x)
        self.armor_target.position.y = float(y)
        self.armor_target.position.z = float(z)
        self.armor_target.velocity.x = float(vx)
        self.armor_target.velocity.y = float(vy)
        self.armor_target.velocity.z = float(vz)

        self.armor_target.yaw = float(yaw)
        self.armor_target.v_yaw = float(v_yaw)
        self.armor_target.radius_1 = float(radius_1)
        self.armor_target.radius_2 = float(radius_2)
        self.armor_target.dz = float(dz)

    def timer_callback(self):
        """定时器回调：倒计时和发布消息"""
        if self.game_status.stage_remain_time > 0:
            self.game_status.stage_remain_time -= 1
        else:
            self.game_status.game_progress = 5

        self.armor_target.header.stamp = self.get_clock().now().to_msg()

        self.game_status_publisher_.publish(self.game_status)
        self.all_robot_hp_publisher_.publish(self.all_robot_hp)
        self.robot_economy_publisher_.publish(self.robot_economy)
        self.robot_status_publisher_.publish(self.robot_status)
        self.armor_target_publisher_.publish(self.armor_target)
        self.robot_location_publisher_.publish(self.robot_location)
        self.rfid_publisher_.publish(self.rfid)

    def posture_status_callback(self, msg):
        """回调：接收姿态状态（用于显示）"""
        self.posture_status = msg

    def set_posture_cmd(self, posture, override_mode=True):
        """发送姿态命令（topic + 发布 status）"""
        cmd = SentryPostureCmd()
        cmd.posture = posture
        cmd.override_mode = override_mode
        self.posture_cmd_publisher_.publish(cmd)

        self.posture_status.current_posture = posture
        self.posture_status_publisher_.publish(self.posture_status)
        self.get_logger().info(f"Sent posture command: {posture} (override={override_mode})")

    def set_posture_callback(self, request, response):
        """Service回调：对应参考节点 handleSetSentryPosture"""
        self.posture_status.current_posture = request.posture
        self.posture_status_publisher_.publish(self.posture_status)
        response.accepted = True
        response.message = f"Posture set to {request.posture}"
        self.get_logger().info(
            f"SetSentryPosture service called: posture={request.posture}, override={request.override_mode}")
        return response

    def get_robot_status_text(self):
        """获取机器人状态文本"""
        status_list = []
        if self.robot_status.is_attacked:
            status_list.append("ATTACKED")
        status_list.append(f"HP: {self.robot_status.current_hp}")
        status_list.append(f"Heat: {self.robot_status.shooter_heat}")
        return " | ".join(status_list)

    def get_posture_text(self):
        """获取当前姿态文本"""
        posture_names = {1: "ATTACK", 2: "DEFENSE", 3: "MOVE"}
        posture = self.posture_status.current_posture
        return posture_names.get(posture, f"UNKNOWN({posture})")

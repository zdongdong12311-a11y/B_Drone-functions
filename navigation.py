#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry, Path
from tf import transformations

try:
    from quadrotor_msgs.msg import PositionCommand
    HAS_EGO_MSGS = True
except ImportError:
    HAS_EGO_MSGS = False
    PositionCommand = None


class NavigationController:
    """无人机自主导航控制器 (主逻辑线程 + 独立设定点流线程)"""

    # EGO 轨迹点 -> MAVROS: 使用 位置+速度+偏航角
    # IGNORE_AFX(64)|IGNORE_AFY(128)|IGNORE_AFZ(256)|IGNORE_YAW_RATE(2048) = 2496
    EGO_TYPE_MASK = 2496
    # 纯位置设定点 (起飞/悬停/降落): 忽略速度+加速度+偏航角速率 = 2552
    POS_TYPE_MASK = 2552

    def __init__(self):
        # ---- 飞行基础参数 ----
        self.takeoff_height = float(rospy.get_param('~takeoff_height', 0.8))
        self.stream_rate = float(rospy.get_param('~stream_rate', 20.0))
        self.fcu_timeout = float(rospy.get_param('~fcu_timeout', 30.0))
        self.prestream_time = float(rospy.get_param('~prestream_time', 5.0))
        self.pose_timeout = float(rospy.get_param('~pose_timeout', 3.0))

        # ---- 航点参数 ----
        self.waypoint_xy_tol = float(rospy.get_param('~waypoint_xy_tol', 0.3))
        self.waypoint_z_tol = float(rospy.get_param('~waypoint_z_tol', 0.2))
        self.waypoint_timeout = float(rospy.get_param('~waypoint_timeout', 120.0))

        # ---- 起降参数 ----
        self.takeoff_timeout = float(rospy.get_param('~takeoff_timeout', 30.0))
        self.land_timeout = float(rospy.get_param('~land_timeout', 60.0))
        self.land_descent_speed = float(rospy.get_param('~land_descent_speed', 0.4))  # m/s
        self.land_floor_z = float(rospy.get_param('~land_floor_z', 0.15))
        self.land_exit_z = float(rospy.get_param('~land_exit_z', 0.2))
        self.touchdown_z = float(rospy.get_param('~touchdown_z', 0.05))

        # ---- EGO 目标下发参数 ----
        self.goal_topic = rospy.get_param('~goal_topic', '/move_base_simple/goal')
        self.goal_frame_id = rospy.get_param('~goal_frame_id', 'camera_init')
        self.position_cmd_topic = rospy.get_param('~position_cmd_topic', '/position_cmd')
        self.goal_ack_timeout = float(rospy.get_param('~goal_ack_timeout', 2.0))
        self.goal_retry_timeout = float(rospy.get_param('~goal_retry_timeout', 10.0))
        self.goal_fail_action = str(rospy.get_param('~goal_fail_action', 'skip')).lower()
        if self.goal_fail_action not in ('skip', 'land'):
            rospy.logwarn("无效 ~goal_fail_action='%s', 回退为 'skip'", self.goal_fail_action)
            self.goal_fail_action = 'skip'
        self.ego_cmd_timeout = float(rospy.get_param('~ego_cmd_timeout', 0.5))
        # v2.1: true=直接发 Path 到 EGO FSM 话题(推荐, 无需中转节点);
        #       false=发 PoseStamped 到 /move_base_simple/goal(需 goal_relay 翻译)
        self.goal_direct = bool(rospy.get_param('~goal_direct', True))
        self.ego_waypoints_topic = rospy.get_param(
            '~ego_waypoints_topic', '/waypoint_generator/waypoints')

        # ---- 里程计一致性检查 ----
        self.odom_check_topic = rospy.get_param('~odom_check_topic', '/Odometry')
        self.odom_divergence_threshold = float(rospy.get_param('~odom_divergence_threshold', 0.5))
        self.odom_divergence_emergency = bool(rospy.get_param('~odom_divergence_emergency', False))

        # ---- 内部状态 ----
        self.current_state = State()
        self.current_position = PoseStamped()
        self.pose_received = False
        self.last_pose_time = rospy.Time(0)
        self.now_yaw = 0.0
        self.rate = rospy.Rate(self.stream_rate)
        self.nav_state = "IDLE"  # IDLE/TAKEOFF/NAVIGATING/HOVER/LANDING/DONE

        self._emergency_land_triggered = False
        self._mission_abort = False

        # 设定点流共享状态 (线程安全)
        self._sp_lock = threading.Lock()
        self._hover_sp = None            # 悬停/起飞/降落位置设定点
        self._ego_sp = None              # EGO 轨迹点转换的设定点
        self._ego_sp_time = rospy.Time(0)
        self._stream_stop = False

        # ---- 发布者 ----
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
        self.ego_path_pub = rospy.Publisher(self.ego_waypoints_topic, Path, queue_size=1)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # ---- 订阅者 ----
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.current_position_callback)
        rospy.Subscriber(self.position_cmd_topic, PositionCommand, self.position_cmd_callback)
        if self.odom_divergence_threshold > 0.0:
            rospy.Subscriber(self.odom_check_topic, Odometry, self.odom_callback)

        # ---- 服务客户端 ----
        try:
            rospy.wait_for_service('/mavros/set_mode', timeout=30)
            self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            rospy.wait_for_service('/mavros/cmd/arming', timeout=30)
            self.arm_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        except rospy.ROSException as e:
            rospy.logfatal("MAVROS 服务不可用: %s", e)
            raise

        # ---- 航点文件 ----
        self.waypoint_file = self._get_waypoint_path()

        # ---- 启动设定点流线程 ----
        self._stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._stream_thread.start()

        rospy.loginfo("初始化完成: 起飞高度=%.2fm 流频率=%.0fHz 容差(XY/Z)=(%.2f/%.2f)m "
                      "目标模式=%s", self.takeoff_height, self.stream_rate,
                      self.waypoint_xy_tol, self.waypoint_z_tol,
                      "直接Path->EGO" if self.goal_direct else "PoseStamped->relay")

    # ===================== 回调函数 =====================

    def state_callback(self, msg):
        self.current_state = msg
        # 连接断开立即触发紧急降落 (非阻塞, 仅置位 + 切 AUTO.LAND)
        if not msg.connected and self.pose_received:
            self._trigger_emergency_land("MAVROS 连接断开")

    def current_position_callback(self, msg):
        self.current_position = msg
        self.pose_received = True
        self.last_pose_time = rospy.Time.now()
        q = [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w]
        try:
            self.now_yaw = transformations.euler_from_quaternion(q)[2]
        except (ValueError, ZeroDivisionError):
            pass

    def position_cmd_callback(self, msg):
        """EGO 轨迹点 → 缓存为设定点, 由流线程统一发布 (回调里不发 MAVROS)"""
        sp = PositionTarget()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = self.EGO_TYPE_MASK
        sp.position.x = msg.position.x
        sp.position.y = msg.position.y
        sp.position.z = msg.position.z
        sp.velocity.x = msg.velocity.x
        sp.velocity.y = msg.velocity.y
        sp.velocity.z = msg.velocity.z
        sp.yaw = msg.yaw
        with self._sp_lock:
            self._ego_sp = sp
            self._ego_sp_time = rospy.Time.now()

    def odom_callback(self, msg):
        """FAST-LIO 与 PX4 EKF 位置一致性交叉检查 (检测坐标系未对齐)"""
        if self.odom_divergence_threshold <= 0.0 or not self._pose_is_fresh(1.0):
            return
        p = self.current_position.pose.position
        q = msg.pose.pose.position
        div = math.sqrt((q.x - p.x) ** 2 + (q.y - p.y) ** 2 + (q.z - p.z) ** 2)
        if div > self.odom_divergence_threshold:
            rospy.logwarn_throttle(5.0,
                "里程计偏差过大: FAST-LIO 与 PX4 EKF 相差 %.2f m (阈值 %.2f m)! "
                "请检查 EKF2_EV_CTRL / 坐标系对齐!", div, self.odom_divergence_threshold)
            if self.odom_divergence_emergency and self.current_state.armed:
                self._trigger_emergency_land("里程计一致性检查失败")

    # ===================== 设定点流线程 =====================

    def _stream_loop(self):
        """独立线程, 全程以 stream_rate 发布当前设定点:
        - NAVIGATING 且 EGO 指令新鲜 → 转发 EGO 轨迹点
        - 其它情况 → 发布悬停/起飞/降落位置设定点
        任何阶段都不存在发布空窗, 避免 PX4 OFFBOARD failsafe 误触发
        """
        rate = rospy.Rate(self.stream_rate)
        while not rospy.is_shutdown() and not self._stream_stop:
            now = rospy.Time.now()
            with self._sp_lock:
                ego, ego_time = self._ego_sp, self._ego_sp_time
                hover = self._hover_sp

            sp = None
            if (self.nav_state == "NAVIGATING" and ego is not None
                    and (now - ego_time).to_sec() <= self.ego_cmd_timeout):
                sp = ego
            elif hover is not None:
                sp = hover

            if sp is not None:
                sp.header.stamp = now
                self.setpoint_pub.publish(sp)
            try:
                rate.sleep()
            except rospy.ROSException:
                break

    # ===================== 工具函数 =====================

    def _get_waypoint_path(self):
        ros_param_path = rospy.get_param('~waypoint_file', '')
        if ros_param_path and os.path.isfile(ros_param_path):
            return ros_param_path
        env_path = os.environ.get('DRONE_WAYPOINT_FILE', '')
        if env_path and os.path.isfile(env_path):
            return env_path

        script_dir = os.path.dirname(os.path.abspath(__file__))
        candidates = [os.path.join(script_dir, 'point.txt'), os.path.expanduser('~/point.txt')]
        for path in candidates:
            if os.path.isfile(path):
                return path

        fallback = os.path.join(script_dir, 'point.txt')
        rospy.logwarn("未找到航点文件，使用默认路径: %s", fallback)
        return fallback

    def _make_position_target(self, x, y, z, yaw):
        sp = PositionTarget()
        sp.header.stamp = rospy.Time.now()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = self.POS_TYPE_MASK
        sp.position.x = x
        sp.position.y = y
        sp.position.z = z
        sp.yaw = yaw
        return sp

    def _set_hover(self, x, y, z, yaw=None):
        """更新流线程使用的悬停设定点 (起飞/悬停/降落/回退均走这里)"""
        if yaw is None:
            yaw = self.now_yaw
        with self._sp_lock:
            self._hover_sp = self._make_position_target(x, y, z, yaw)

    def _update_hover_to_current(self):
        p = self.current_position.pose.position
        self._set_hover(p.x, p.y, p.z, self.now_yaw)

    def _clear_ego(self):
        with self._sp_lock:
            self._ego_sp = None
            self._ego_sp_time = rospy.Time(0)

    def _ego_cmd_is_fresh(self):
        with self._sp_lock:
            return (rospy.Time.now() - self._ego_sp_time).to_sec() <= self.ego_cmd_timeout

    def _ego_received_after(self, t):
        with self._sp_lock:
            return self._ego_sp_time > t

    def _pose_is_fresh(self, max_age=1.0):
        return self.pose_received and (rospy.Time.now() - self.last_pose_time).to_sec() <= max_age

    def _check_timeout(self, start_time, timeout):
        return (rospy.Time.now() - start_time).to_sec() > timeout

    def _at_target(self, x, y, z):
        """到达判定: XY 与 Z 容差分离"""
        p = self.current_position.pose.position
        return (math.hypot(p.x - x, p.y - y) < self.waypoint_xy_tol
                and abs(p.z - z) < self.waypoint_z_tol)

    def _set_mode(self, mode):
        try:
            return bool(self.set_mode_client(custom_mode=mode).mode_sent)
        except rospy.ServiceException as e:
            rospy.logwarn("set_mode(%s) 调用失败: %s", mode, e)
            return False

    def _try_disarm(self):
        try:
            return bool(self.arm_client(False).success)
        except rospy.ServiceException as e:
            rospy.logwarn("上锁请求失败: %s", e)
            return False

    @staticmethod
    def load_waypoints(filepath):
        waypoints = []
        with open(filepath, 'r') as f:
            for line_num, line in enumerate(f, 1):
                stripped = line.strip()
                if not stripped or stripped.startswith('#'):
                    continue
                parts = stripped.split()
                if len(parts) >= 4:
                    try:
                        x, y, z, t = (float(p) for p in parts[:4])
                        waypoints.append((x, y, z, t))
                    except ValueError:
                        rospy.logwarn("航点文件第 %d 行解析失败: %s", line_num, stripped)
                else:
                    rospy.logwarn("航点文件第 %d 行格式无效: %s", line_num, stripped)
        return waypoints

    # ===================== 安全保护 =====================

    def _trigger_emergency_land(self, reason="未知原因"):
        """非阻塞紧急降落: 置位 + 冻结悬停点 + 切 AUTO.LAND, 飞控接管。
        不在回调线程里跑阻塞流程, 与主线程无 setpoint 竞争。"""
        if self._emergency_land_triggered:
            return
        self._emergency_land_triggered = True
        self.nav_state = "LANDING"
        rospy.logfatal("=" * 40)
        rospy.logfatal("!!! 紧急降落已触发: %s !!!", reason)
        rospy.logfatal("=" * 40)
        # 冻结当前位置为悬停点, 维持设定点流, 防止模式切换生效前先触发 OFFBOARD 失控保护
        p = self.current_position.pose.position
        self._set_hover(p.x, p.y, p.z, self.now_yaw)
        self._set_mode('AUTO.LAND')

    def _check_emergency(self):
        """主循环内调用: 检查紧急标志与位姿新鲜度"""
        if self._emergency_land_triggered:
            return True
        if self.pose_received and not self._pose_is_fresh(self.pose_timeout):
            self._trigger_emergency_land("位姿数据超时 (>%.1fs)" % self.pose_timeout)
            return True
        return False

    def _on_shutdown(self):
        self._stream_stop = True
        if self.current_state.armed and not self._emergency_land_triggered:
            rospy.logwarn("节点退出时飞机仍在空中, 尽力请求 AUTO.LAND...")
            self._set_mode('AUTO.LAND')
        rospy.loginfo("navigation 节点退出 (设定点流停止后 PX4 failsafe 也会接管)")

    # ===================== 基础飞行控制 =====================

    def wait_for_fcu_ready(self, timeout=None):
        if timeout is None:
            timeout = self.fcu_timeout
        rospy.loginfo("等待 MAVROS 连接和本地位姿...")
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.current_state.connected and self._pose_is_fresh():
                rospy.loginfo("MAVROS 已连接，本地位姿可用。")
                return True
            if self._check_timeout(start_time, timeout):
                rospy.logerr("等待 MAVROS/本地位姿超时。")
                return False
            self.rate.sleep()
        return False

    def set_offboard_and_arm(self, climb_z):
        """预流设定点 → 切 OFFBOARD → 解锁 (均带重试)"""
        if not self.wait_for_fcu_ready():
            return False

        lock_x = self.current_position.pose.position.x
        lock_y = self.current_position.pose.position.y
        lock_yaw = self.now_yaw
        self._set_hover(lock_x, lock_y, climb_z, lock_yaw)

        rospy.loginfo("预发布设定点 (%.0fs)...", self.prestream_time)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and not self._check_timeout(start_time, self.prestream_time):
            if not self.current_state.connected:
                rospy.logerr("预发布期间 FCU 断开。")
                return False
            self.rate.sleep()  # 发布由流线程完成, 此处只计时

        ok = False
        for _ in range(3):
            if self._set_mode('OFFBOARD'):
                ok = True
                break
            rospy.sleep(0.5)
        if not ok:
            rospy.logerr("切换 OFFBOARD 失败。")
            return False

        ok = False
        for _ in range(3):
            try:
                if self.arm_client(True).success:
                    ok = True
                    break
            except rospy.ServiceException as e:
                rospy.logwarn("解锁请求异常: %s", e)
            rospy.sleep(0.5)
        if not ok:
            rospy.logerr("解锁失败。")
            return False

        rospy.loginfo("已切入 OFFBOARD 并解锁。")
        return True

    # ===================== 任务逻辑 =====================

    def takeoff(self, height=None):
        target_z = self.takeoff_height if height is None else height
        self.nav_state = "TAKEOFF"
        rospy.loginfo("起飞至 %.2f 米...", target_z)

        if not self.set_offboard_and_arm(target_z):
            return False

        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self._check_emergency():
                return False
            if self.current_state.mode != "OFFBOARD":
                rospy.logwarn_throttle(2.0, "当前模式 '%s' 不是 OFFBOARD (可能被遥控切出)!",
                                       self.current_state.mode)

            current_z = self.current_position.pose.position.z
            if abs(current_z - target_z) < self.waypoint_z_tol:
                rospy.loginfo("已到达目标高度: %.2f 米", current_z)
                self.nav_state = "HOVER"
                return True
            if self._check_timeout(start_time, self.takeoff_timeout):
                rospy.logerr("起飞超时 (当前高度 %.2f m)。", current_z)
                return False
            self.rate.sleep()
        return False

    def send_ego_goal(self, x, y, z):
        """向 EGO-Planner 下发目标。
        goal_direct=True : 直接发布单航点 Path 到 EGO FSM 订阅的话题,
                           不再依赖 waypoint_generator / goal_relay;
        goal_direct=False: 兼容旧模式, 发 PoseStamped 到 /move_base_simple/goal
                           (需要 goal_relay 在场翻译)。"""
        if self.goal_direct:
            path = Path()
            path.header.stamp = rospy.Time.now()
            path.header.frame_id = self.goal_frame_id
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            path.poses.append(pose)
            self.ego_path_pub.publish(path)
        else:
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = self.goal_frame_id
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = z

            p = self.current_position.pose.position
            dx, dy = x - p.x, y - p.y
            yaw = math.atan2(dy, dx) if (abs(dx) > 1e-6 or abs(dy) > 1e-6) else self.now_yaw
            qx, qy, qz, qw = transformations.quaternion_from_euler(0.0, 0.0, yaw)
            goal.pose.orientation.x = qx
            goal.pose.orientation.y = qy
            goal.pose.orientation.z = qz
            goal.pose.orientation.w = qw
            self.goal_pub.publish(goal)

    def navigation_target(self, x, y, z, hover_time=2.0):
        """导航至目标点并悬停。返回 False 表示任务需要中止 (紧急/放弃)"""
        self.nav_state = "NAVIGATING"

        # ---- 阶段1: 下发目标并等待 EGO 确认 (重发直到收到新轨迹指令或超时) ----
        self._clear_ego()
        rospy.loginfo("下发规划目标 x=%.2f y=%.2f z=%.2f (悬停 %.1fs)", x, y, z, hover_time)

        accepted = False
        retry_start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self._check_emergency():
                return False
            self._update_hover_to_current()  # 等待期间保持位置悬停
            self.send_ego_goal(x, y, z)
            goal_time = rospy.Time.now()

            # 等待 EGO 的首条新轨迹指令作为 ACK
            ack_deadline = goal_time + rospy.Duration(self.goal_ack_timeout)
            while not rospy.is_shutdown() and rospy.Time.now() < ack_deadline:
                if self._check_emergency():
                    return False
                self._update_hover_to_current()
                if self._ego_received_after(goal_time):
                    accepted = True
                    break
                rospy.sleep(0.05)
            if accepted:
                break

            if self._check_timeout(retry_start, self.goal_retry_timeout):
                rospy.logerr("EGO-Planner %.0fs 内未响应目标 (无轨迹输出)!", self.goal_retry_timeout)
                if self.goal_fail_action == 'land':
                    rospy.logerr("goal_fail_action='land' → 中止任务并降落")
                    self._mission_abort = True
                    return False
                rospy.logwarn("goal_fail_action='skip' → 跳过该航点")
                break

        # ---- 阶段2: 跟随 EGO 轨迹 (流线程自动转发, 中断自动回退悬停) ----
        arrived = False
        if accepted:
            rospy.loginfo("目标已被 EGO-Planner 接受, 开始跟随轨迹...")
            start_time = rospy.Time.now()
            while not rospy.is_shutdown():
                if self._check_emergency():
                    return False
                # 持续把回退悬停点刷新到当前位置: 轨迹一旦中断即原点悬停
                self._update_hover_to_current()

                if not self._ego_cmd_is_fresh():
                    rospy.logwarn_throttle(3.0, "EGO 轨迹指令中断, 已回退为位置悬停!")

                if self._at_target(x, y, z):
                    p = self.current_position.pose.position
                    rospy.loginfo("到达航点! XY误差 %.2f m, Z误差 %.2f m",
                                  math.hypot(p.x - x, p.y - y), abs(p.z - z))
                    arrived = True
                    break
                if self._check_timeout(start_time, self.waypoint_timeout):
                    rospy.logwarn("航点导航超时 (%.0fs), 跳转下一航点。", self.waypoint_timeout)
                    break
                self.rate.sleep()

        # ---- 阶段3: 悬停 (超时跳过时悬停在当前位置, 而非未到达的目标点) ----
        self.nav_state = "HOVER"
        if arrived:
            self._set_hover(x, y, z)
        else:
            self._update_hover_to_current()
        rospy.loginfo("悬停 %.1f 秒...", hover_time)
        hover_start = rospy.Time.now()
        while not rospy.is_shutdown() and not self._check_timeout(hover_start, hover_time):
            if self._check_emergency():
                return False
            self.rate.sleep()

        rospy.loginfo("航点任务完成。")
        return True

    def hover_in_place(self, duration):
        """原地悬停 (无航点时使用)"""
        self.nav_state = "HOVER"
        self._update_hover_to_current()
        rospy.loginfo("原地悬停 %.1f 秒...", duration)
        start = rospy.Time.now()
        while not rospy.is_shutdown() and not self._check_timeout(start, duration):
            if self._check_emergency():
                return False
            self.rate.sleep()
        return True

    def land_at_current_position(self):
        self.nav_state = "LANDING"
        rospy.loginfo("启动安全降落...")

        if not self._pose_is_fresh(2.0):
            rospy.logwarn("位姿不可用, 直接请求 AUTO.LAND")
            self._set_mode('AUTO.LAND')
            return

        lock_x = self.current_position.pose.position.x
        lock_y = self.current_position.pose.position.y
        lock_yaw = self.now_yaw
        current_z = self.current_position.pose.position.z
        target_z = current_z
        dz_per_tick = self.land_descent_speed / self.stream_rate
        start_time = rospy.Time.now()

        # ---- 受控下降 (紧急降落时跳过: AUTO.LAND 已接管) ----
        if not self._emergency_land_triggered and current_z > self.land_exit_z:
            rospy.loginfo("受控垂直下降 (锁定 XY/Yaw, %.2f m/s)...", self.land_descent_speed)
            while not rospy.is_shutdown() and current_z > self.land_exit_z:
                target_z = max(target_z - dz_per_tick, self.land_floor_z)
                self._set_hover(lock_x, lock_y, target_z, lock_yaw)
                if not self._pose_is_fresh(self.pose_timeout):
                    rospy.logerr("降落中位姿超时, 切换 AUTO.LAND!")
                    self._set_mode('AUTO.LAND')
                    break
                if self._check_timeout(start_time, self.land_timeout):
                    rospy.logwarn("受控下降超时, 切换 AUTO.LAND!")
                    break
                current_z = self.current_position.pose.position.z
                self.rate.sleep()

        # ---- 最终阶段: AUTO.LAND 触地 (带重试) + 等待确认 + 上锁 ----
        deadline = rospy.Time.now() + rospy.Duration(10.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self._set_mode('AUTO.LAND'):
                rospy.loginfo("AUTO.LAND 已请求, 等待落地...")
                break
            rospy.sleep(1.0)

        wait_start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self._pose_is_fresh(2.0) and self.current_position.pose.position.z < self.touchdown_z:
                rospy.loginfo("已落地 (z=%.2f m)。", self.current_position.pose.position.z)
                break
            if self._check_timeout(wait_start, 30.0):
                rospy.logwarn("等待落地确认超时, 请人工确认!")
                break
            self.rate.sleep()

        rospy.sleep(2.0)  # 等待飞控落地状态稳定
        if self.current_state.armed:
            if self._try_disarm():
                rospy.loginfo("已上锁。")
            else:
                rospy.logwarn("自动上锁失败, 请手动上锁!")
        self.nav_state = "DONE"


def main():
    rospy.init_node('navigation_controller', anonymous=False)

    if not HAS_EGO_MSGS:
        rospy.logfatal("未检测到 quadrotor_msgs, 无法接收 EGO-Planner 轨迹! "
                       "请先 source EGO-Planner 工作空间。")
        return

    try:
        nav = NavigationController()
    except rospy.ROSException as e:
        rospy.logfatal("初始化失败: %s", e)
        return

    rospy.on_shutdown(nav._on_shutdown)

    # ---- 起飞 (失败时安全处置, 不把飞机晾在半空) ----
    if not nav.takeoff():
        rospy.logerr("起飞失败, 执行安全处置 (降落+上锁)。")
        nav.land_at_current_position()
        return

    # ---- 加载航点 ----
    try:
        waypoints = NavigationController.load_waypoints(nav.waypoint_file)
    except IOError:
        rospy.logerr("打开航点文件失败: %s, 执行降落。", nav.waypoint_file)
        nav.land_at_current_position()
        return

    if not waypoints:
        rospy.logwarn("航点为空, 悬停后降落。")
        nav.hover_in_place(3.0)
        nav.land_at_current_position()
        return

    # ---- 航点导航 ----
    rospy.loginfo("共 %d 个航点, 开始执行...", len(waypoints))
    for i, (x, y, z, t) in enumerate(waypoints):
        if nav._emergency_land_triggered or nav._mission_abort:
            break
        rospy.loginfo("=" * 15 + " 航点 #%d/%d " + "=" * 15, i + 1, len(waypoints))
        nav.navigation_target(x, y, z, t)

    # ---- 收尾降落 (无论正常结束还是紧急中断, 统一走这里, 内部自动适配) ----
    rospy.loginfo("任务结束, 开始降落。")
    nav.land_at_current_position()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

# WARNING! Code for journal experiment.
# This code runs repetative experiments and resets the system periodically

import os
import time
import numpy as np
from pinocchio import Quaternion
from scipy.spatial.transform import Rotation as SciR
import pickle
import yaml

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from pydrake.math import (
    RollPitchYaw,
    RotationMatrix,
)

from pydrake.all import (
    AngleAxis,
    PiecewisePolynomial,
    Quaternion,
    RigidTransform
)
from ament_index_python.packages import get_package_share_directory

from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from common_msgs.msg import HardwareStates, JointTrajectoryWrapper
from leap_ros2.utils import time_msg_to_float

class JournalExperimentNode(Node):
    def __init__(self):
        node_name = 'journal_experiment_node'
        super().__init__(node_name)

        test_method = yaml.safe_load(
            open(os.path.join(get_package_share_directory('leap_ros2'), 'config', 'low_level_ctrl_params.yaml'), 'r'),)['algo_type']
        self.test_method = test_method
        print(f"Test method {test_method}")

        # preparation
        self.rand_so3_data = np.load('./data/quat_targets-250318.npy')
        self.rand_so3_data = self.rand_so3_data[0:10]
        if test_method in ['mjpc']:
            # rotate the goal w.r.t. z-axis np.pi
            for i in range(len(self.rand_so3_data)):
                r = SciR.from_quat(self.rand_so3_data[i][[1, 2, 3, 0]])
                r = SciR.from_euler('z', np.pi) * r
                self.rand_so3_data[i] = r.as_quat()[[3, 0, 1, 2]]
        self.current_goal_id = -1
        self.is_manipulating = False
        self.is_resetting = False

        # parameters
        self.timeout = 60.0
        self.error_thres_deg = 8.0

        # statistics
        self.success_count = 0
        self.timeout_count = 0
        self.completion_times = []  # 成功试验的首次达标时间
        self.min_errors = []  # 记录每次试验的最小误差
        self.mean_tci_velocity = []  # 记录每次试验的平均速度 TCI
        self.mean_tci_torque = []  # 记录每次试验的平均力矩 TCI

        # buffers
        self.t_last_reset = -1
        self.reset_buffers()
        
        # 当前试验的追踪变量
        self.current_trial_min_error = float('inf')  # 当前试验的最小误差
        self.first_success_time = None  # 首次达到成功标准的时间
        self.has_reached_success = False  # 是否已达到过成功标准

        self.curr_quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.curr_quat_target = np.array([1.0, 0.0, 0.0, 0.0])
        
        # Manipulability Matrix
        self.current_M = np.eye(3)  # 当前的可操作性矩阵
        self.M_valid = False  # M 矩阵是否有效

        high_level_node_name = ''
        if test_method in ['ours']:
            high_level_node_name = 'highlevel_controller_node'
        elif test_method in ['mjpc']:
            high_level_node_name = 'mjpc_controller_node'

        low_level_node_name = 'low_level_node'
        sim_node_name = 'mujoco_ros'

        self.sub_cbg = MutuallyExclusiveCallbackGroup()
        self.srv_cbg = MutuallyExclusiveCallbackGroup()
        self.timer_cbg = MutuallyExclusiveCallbackGroup()

        # high level reset
        self.high_level_reset_client = self.create_client(Trigger, f'{high_level_node_name}/reset', callback_group=self.srv_cbg)
        if not self.high_level_reset_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {high_level_node_name}/reset not found!')

        if test_method in ['mjpc']:
            self.high_level_start_client = self.create_client(Trigger, f'{high_level_node_name}/start', callback_group=self.srv_cbg)
            if not self.high_level_start_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'Service {high_level_node_name}/start not found!')

        if test_method in ['ours']:
            # low level reset
            self.low_level_reset_client = self.create_client(Trigger, f'{low_level_node_name}/reset', callback_group=self.srv_cbg)
            if not self.low_level_reset_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'Service {low_level_node_name}/reset not found!')

        # sim reset
        self.sim_reset_client = self.create_client(Trigger, f'{sim_node_name}/reset', callback_group=self.srv_cbg)
        if not self.sim_reset_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {sim_node_name}/reset not found!')

        # high level param
        self.high_level_param_client = self.create_client(SetParameters, f'{high_level_node_name}/set_parameters', callback_group=self.srv_cbg)
        if not self.high_level_param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('High level parameter service is not initialized!')

        # external control
        self.reset_srv = self.create_service(Trigger, f'{node_name}/call_reset', self.call_reset_task)
        self.start_task_srv = self.create_service(Trigger, f'{node_name}/call_start_task', self.call_start_task)

        # create a folder in data/ with current time
        dir_name = f'./data/log/{time.strftime("%Y%m%d-%H%M%S")}'
        if not os.path.exists(dir_name):
            os.makedirs(dir_name)
        self.dir_name = dir_name

        # subscriber
        if test_method in ['ours']:
            state_msg = HardwareStates
            state_topic = '/hardware_states'
        elif test_method in ['mjpc']:
            state_msg = Float64MultiArray
            state_topic = '/object_quat'

        self.state_sub = self.create_subscription(
            state_msg, state_topic,
            self.state_callback,
            10,
            callback_group=self.sub_cbg
        )

        self.Manipulability_m_sub = self.create_subscription(
            Float64MultiArray,
            '/manipulability_m',
            self.manipulability_m_callback,
            10,
            callback_group=self.sub_cbg
        )
        
        self.cmd_sub = self.create_subscription(
            JointState,
            '/mujoco_ros/joint_commands',
            self.cmd_callback,
            10,
            callback_group=self.sub_cbg
        )

        if test_method in ['ours']:
            self.traj_sub = self.create_subscription(
                JointTrajectoryWrapper,
                '/high_level_traj',
                self.traj_callback,
                qos_profile=QoSProfile(
                    depth=10,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST
                ),
                callback_group=self.sub_cbg
            )

        self.get_logger().info(f'{node_name} ready!')

        self.start_time = self.get_ros_time()
        self.total_reset_times = 0
        self.last_reset_time = self.start_time
        self.reset_period = 10.0

        # start_task
        self.reset_task()
        self.start_task()

        self.timer = self.create_timer(1.0, callback=self.timer_callback, callback_group=self.timer_cbg)

    def get_ros_time(self):
        """ Return ROS time in float """
        return time_msg_to_float(self.get_clock().now().to_msg())
    
    def reset_buffers(self):
        self.t_last_rev_state = -1  # state
        self.dt_recv_state = []
        self.state_buf = []
        self.state_stamp = []

        self.t_last_rev_cmd = -1    # low level
        self.dt_recv_cmd = []
        self.cmd_buf = []
        self.cmd_stamp = []

        self.t_last_rev_traj = -1   # high level
        self.dt_recv_traj = []
        
        # TCI buffers
        self.tci_velocity_buf = []
        self.tci_torque_buf = []
        self.tci_stamp = []
    
    # ---------- callbacks ----------
    def manipulability_m_callback(self, msg:Float64MultiArray):
        """接收并缓存可操作性矩阵 M"""
        if not self.is_manipulating:
            return
        
        M = np.array(msg.data).reshape(3, 3)
        
        # 检查 M 是否有效（正定矩阵）
        try:
            eigvals = np.linalg.eigvalsh(M)
            if np.all(eigvals > 1e-6):  # 所有特征值都为正
                self.current_M = M.copy()
                self.M_valid = True
            else:
                self.M_valid = False
        except:
            self.M_valid = False

    def state_callback(self, msg:HardwareStates):
        if not self.is_manipulating:
            return
        
        t_now = self.get_ros_time()
        elapsed_time = t_now - self.t_last_reset

        if isinstance(msg, HardwareStates):
            self.curr_quat[:] = np.array(msg.q.data)[0:4]
            self.state_buf.append(np.array(msg.q.data).tolist())
            self.state_stamp.append(elapsed_time)
        elif isinstance(msg, Float64MultiArray):
            self.curr_quat[:] = np.array(msg.data)[0:4]
            self.state_buf.append(np.array(msg.data).tolist())
            self.state_stamp.append(elapsed_time)
        
        # 计算当前时刻的取向误差
        _, current_error = self.check_task_completion()
        
        # 更新当前试验的最小误差
        if current_error < self.current_trial_min_error:
            self.current_trial_min_error = current_error
        
        # 检查是否首次达到成功标准
        if not self.has_reached_success and current_error < self.error_thres_deg:
            self.has_reached_success = True
            self.first_success_time = elapsed_time
            self.get_logger().info(
                f'First reached success threshold at {elapsed_time:.2f}s '
                f'(error: {current_error:.2f}°)'
            )
        
        # 计算 TCI (如果 M 矩阵有效)
        if self.M_valid:
            tci_vel = self.compute_tci(self.current_M, self.curr_quat, self.curr_quat_target, mode='velocity')
            tci_torque = self.compute_tci(self.current_M, self.curr_quat, self.curr_quat_target, mode='torque')
            
            self.tci_velocity_buf.append(tci_vel)
            self.tci_torque_buf.append(tci_torque)
            self.tci_stamp.append(elapsed_time)

    def cmd_callback(self, msg:JointState):
        if not self.is_manipulating:
            return
        
        t_now = self.get_ros_time()
        if self.t_last_rev_cmd < 0:
            self.t_last_rev_cmd = t_now
        else:
            self.dt_recv_cmd.append(t_now - self.t_last_rev_cmd)
            self.t_last_rev_cmd = t_now

        self.cmd_buf.append(np.array(msg.position).tolist())
        self.cmd_stamp.append(t_now - self.t_last_reset)

    def traj_callback(self, msg):
        if not self.is_manipulating:
            return
        
        t_now = self.get_ros_time()
        if self.t_last_rev_traj < 0:
            self.t_last_rev_traj = t_now
        else:
            self.dt_recv_traj.append(t_now - self.t_last_rev_traj)
            self.t_last_rev_traj = t_now

    # -------------------------------
    
    def call_reset_task(self, request, response):
        self.reset_task()
        response.success = True
        return response
    
    def call_start_task(self, request, response):
        self.start_task()
        response.success = True
        return response
    
    def reset_task(self):
        self.is_manipulating = False

        # reset goal
        new_target = self.rand_so3_data[self.current_goal_id + 1]
        self.reset_so3_target(new_target)
        self.get_logger().info(f'New SO(3) target: {new_target}')

        request = Trigger.Request()

        # reset high level
        future = self.high_level_reset_client.call_async(request)

        if self.test_method in ['ours']:
            # reset low level
            future = self.low_level_reset_client.call_async(request)

        # reset sim
        future = self.sim_reset_client.call_async(request)

        # wait until all reset are done
        time.sleep(2.0)
    
    def start_task(self):
        self.get_logger().info(f'Starting goal {self.current_goal_id + 1}...')

        if self.test_method in ['ours']:
            request_start_task = SetParameters.Request()
            request_start_task.parameters = [Parameter(name='start_task', value=ParameterValue(bool_value=True, type=ParameterType.PARAMETER_BOOL))]
            future = self.high_level_param_client.call_async(request_start_task)
            future.add_done_callback(self.start_task_callback)
        elif self.test_method in ['mjpc']:
            request = Trigger.Request()
            future = self.high_level_start_client.call_async(request)
            future.add_done_callback(self.start_task_callback)

    def start_task_callback(self, future):
        self.get_logger().info(f'Goal {self.current_goal_id + 1} started!')
        self.t_last_reset = self.get_ros_time()
        # 重置当前试验的追踪变量
        self.current_trial_min_error = float('inf')
        self.first_success_time = None
        self.has_reached_success = False
        self.M_valid = False
        self.is_manipulating = True

    def reset_so3_target(self, quat_target):
        self.curr_quat_target[:] = quat_target  # 保存目标用于检测完成
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name='rand_so3_target', value=ParameterValue(double_array_value=quat_target.tolist(), type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        ]
        future = self.high_level_param_client.call_async(request)

    def check_task_completion(self):
        """检查是否达到目标SO3"""
        # 归一化四元数
        q_curr = self.curr_quat / np.linalg.norm(self.curr_quat)
        q_target = self.curr_quat_target / np.linalg.norm(self.curr_quat_target)
        # print(f"q_curr: {q_curr}")
        # print(f"q_target: {q_target}")
        q_curr = Quaternion(q_curr)
        q_target = Quaternion(q_target)
        dq = q_curr.inverse().multiply(q_target)
        # print(f"dq: {dq}")
        angle_axis = RotationMatrix(dq).ToAngleAxis()
        # print(f"angle_axis: {angle_axis}")
        angle_ = angle_axis.angle()
        angle_degrees = np.degrees(angle_)
        # print(f"Angle Error: {angle_degrees} deg")
        # 判断是否完成
        is_completed = angle_degrees < self.error_thres_deg
        return is_completed, angle_degrees
    
    def compute_tci(self, M, obj_quat, target_quat, mode='velocity'):
        """
        计算 Transmission Capability Index (TCI)
        
        Args:
            M: 3x3 可操作性矩阵
            obj_quat: 当前四元数 [w, x, y, z]
            target_quat: 目标四元数 [w, x, y, z]
            mode: 'velocity' 或 'torque'
        
        Returns:
            tci: TCI 值 (0~1)
        """
        # MuJoCo quat is [w, x, y, z], Scipy is [x, y, z, w]
        r_curr = SciR.from_quat([obj_quat[1], obj_quat[2], obj_quat[3], obj_quat[0]])
        r_targ = SciR.from_quat([target_quat[1], target_quat[2], target_quat[3], target_quat[0]])
        
        # 计算相对旋转: R_err = R_targ * R_curr.T
        r_err = r_targ * r_curr.inv()
        rot_vec = r_err.as_rotvec()  # 旋转向量 (轴 * 角度)
        
        norm = np.linalg.norm(rot_vec)
        if norm < 1e-6:
            # 误差极小时，返回最优 TCI (1.0)
            return 1.0
        else:
            u = rot_vec / norm  # 单位方向向量
        
        tci = 0.0
        try:
            if mode == 'velocity':
                # 速度传输比: 1 / sqrt(u^T M^-1 u)
                # 我们希望最大化它
                M_inv = np.linalg.inv(M)
                term = u.T @ M_inv @ u
                transmission = 1.0 / np.sqrt(term)
                
                # 归一化: 除以椭球最大轴长
                eigvals = np.linalg.eigvalsh(M)
                max_transmission = np.sqrt(np.max(eigvals))
                tci = transmission / (max_transmission + 1e-9)
                
            elif mode == 'torque':
                # 力矩传输比: sqrt(u^T M u)
                term = u.T @ M @ u
                transmission = np.sqrt(term)
                
                # 归一化: 除以椭球最大轴长
                eigvals = np.linalg.eigvalsh(M)
                max_transmission = np.sqrt(np.max(eigvals))
                tci = transmission / (max_transmission + 1e-9)
                
        except Exception as e:
            # 计算失败时返回 0
            tci = 0.0
        
        return np.clip(tci, 0.0, 1.0)

    def post_task_process(self):
        high_freq = 1 / np.mean(self.dt_recv_traj) if len(self.dt_recv_traj) > 0 else 0.0
        low_freq = 1 / np.mean(self.dt_recv_cmd) if len(self.dt_recv_cmd) > 0 else 0.0
        print(f'High level freq: {high_freq:.2f} Hz')
        print(f'Low level freq: {low_freq:.2f} Hz')
        print(f"Cmd buf shape: {np.array(self.cmd_buf).shape}")
        print(f"State buf shape: {np.array(self.state_buf).shape}")
        
        # 计算当前试验的平均 TCI
        mean_tci_vel = np.mean(self.tci_velocity_buf) if len(self.tci_velocity_buf) > 0 else 0.0
        mean_tci_tor = np.mean(self.tci_torque_buf) if len(self.tci_torque_buf) > 0 else 0.0
        
        # 保存到全局统计
        self.mean_tci_velocity.append(mean_tci_vel)
        self.mean_tci_torque.append(mean_tci_tor)
        
        print(f"Mean TCI (velocity): {mean_tci_vel:.4f}")
        print(f"Mean TCI (torque): {mean_tci_tor:.4f}")

        data = {
            'high_freq': high_freq,
            'low_freq': low_freq,
            'cmd_buf': self.cmd_buf,
            'cmd_stamp': self.cmd_stamp,
            'state_buf': self.state_buf,
            'state_stamp': self.state_stamp,
            'min_error_deg': self.current_trial_min_error,
            'first_success_time': self.first_success_time,
            'has_reached_success': self.has_reached_success,
            'target_quat': self.curr_quat_target.tolist(),
            'tci_velocity_buf': self.tci_velocity_buf,
            'tci_torque_buf': self.tci_torque_buf,
            'tci_stamp': self.tci_stamp,
            'mean_tci_velocity': mean_tci_vel,
            'mean_tci_torque': mean_tci_tor
        }

        pickle.dump(data, open(f'{self.dir_name}/goal_{self.current_goal_id}.pkl', 'wb'))

        self.reset_buffers()

    def timer_callback(self):
        # block timer callback when reseting
        if self.is_resetting:
            return
        
        self.is_resetting = True
        # ----------------------------------------
        t_now = self.get_ros_time()
        
        if self.is_manipulating:
            elapsed_time = t_now - self.t_last_reset
            
            # 获取当前时刻的误差（仅用于显示）
            _, current_error = self.check_task_completion()
            
            # 检查是否超时
            if elapsed_time > self.timeout:
                # 时间窗口结束，判定该试验的结果
                # 使用整个过程中的最小误差来判定成功与否
                min_error = self.current_trial_min_error
                is_success = min_error < self.error_thres_deg
                
                # 记录该次试验的最小误差
                self.min_errors.append(min_error)
                
                if is_success:
                    # 任务成功：在规定时间内，最小误差达到了阈值
                    self.success_count += 1
                    # 记录首次达到成功标准的时间
                    self.completion_times.append(self.first_success_time)
                    self.current_goal_id += 1
                    self.get_logger().info(
                        f'✓ Goal {self.current_goal_id} SUCCESS! '
                        f'Min error: {min_error:.2f}°, '
                        f'Time to success: {self.first_success_time:.2f}s, '
                        f'Final error: {current_error:.2f}°'
                    )
                else:
                    # 任务失败：在规定时间内，最小误差未达到阈值
                    self.timeout_count += 1
                    self.current_goal_id += 1
                    self.get_logger().warn(
                        f'✗ Goal {self.current_goal_id} FAILED! '
                        f'Min error: {min_error:.2f}° (threshold: {self.error_thres_deg:.2f}°), '
                        f'Final error: {current_error:.2f}°'
                    )
                
                # 检查是否所有目标都已完成
                if self.current_goal_id >= len(self.rand_so3_data) - 1:
                    self.post_task_process()
                    self.print_statistics()
                    self.get_logger().info('All goals are done!')
                    self.destroy_node()
                    rclpy.shutdown()
                else:
                    self.post_task_process()
                    self.reset_task()
                    self.start_task()
        # ----------------------------------------
        self.is_resetting = False
    
    def print_statistics(self):
        """打印任务统计信息"""
        total_tasks = self.success_count + self.timeout_count
        success_rate = 100.0 * self.success_count / total_tasks if total_tasks > 0 else 0.0
        avg_time = np.mean(self.completion_times) if len(self.completion_times) > 0 else 0.0
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('TASK STATISTICS')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Total tasks: {total_tasks}')
        self.get_logger().info(f'Successful: {self.success_count}')
        self.get_logger().info(f'Failed: {self.timeout_count}')
        self.get_logger().info(f'Success rate: {success_rate:.1f}%')
        self.get_logger().info('-'*60)
        
        # 成功试验的时间统计
        if len(self.completion_times) > 0:
            self.get_logger().info('Time to first success (for successful trials):')
            self.get_logger().info(f'  Average: {avg_time:.2f}s')
            self.get_logger().info(f'  Min/Max: {np.min(self.completion_times):.2f}s / {np.max(self.completion_times):.2f}s')
            self.get_logger().info(f'  Std dev: {np.std(self.completion_times):.2f}s')
        
        # 最小误差统计
        if len(self.min_errors) > 0:
            self.get_logger().info('-'*60)
            self.get_logger().info('Minimum orientation errors (across all trials):')
            self.get_logger().info(f'  Average: {np.mean(self.min_errors):.2f}°')
            self.get_logger().info(f'  Min/Max: {np.min(self.min_errors):.2f}° / {np.max(self.min_errors):.2f}°')
            self.get_logger().info(f'  Std dev: {np.std(self.min_errors):.2f}°')
            self.get_logger().info(f'  Median: {np.median(self.min_errors):.2f}°')
            # 计算有多少试验的最小误差低于阈值
            below_threshold = sum(1 for e in self.min_errors if e < self.error_thres_deg)
            self.get_logger().info(f'  Below {self.error_thres_deg}° threshold: {below_threshold}/{len(self.min_errors)} ({100.0*below_threshold/len(self.min_errors):.1f}%)')
        
        # TCI 统计
        if len(self.mean_tci_velocity) > 0:
            self.get_logger().info('-'*60)
            self.get_logger().info('Transmission Capability Index (TCI) - Velocity Mode:')
            self.get_logger().info(f'  Average: {np.mean(self.mean_tci_velocity):.4f}')
            self.get_logger().info(f'  Min/Max: {np.min(self.mean_tci_velocity):.4f} / {np.max(self.mean_tci_velocity):.4f}')
            self.get_logger().info(f'  Std dev: {np.std(self.mean_tci_velocity):.4f}')
            self.get_logger().info(f'  Median: {np.median(self.mean_tci_velocity):.4f}')
        
        if len(self.mean_tci_torque) > 0:
            self.get_logger().info('-'*60)
            self.get_logger().info('Transmission Capability Index (TCI) - Torque Mode:')
            self.get_logger().info(f'  Average: {np.mean(self.mean_tci_torque):.4f}')
            self.get_logger().info(f'  Min/Max: {np.min(self.mean_tci_torque):.4f} / {np.max(self.mean_tci_torque):.4f}')
            self.get_logger().info(f'  Std dev: {np.std(self.mean_tci_torque):.4f}')
            self.get_logger().info(f'  Median: {np.median(self.mean_tci_torque):.4f}')
        
        self.get_logger().info('='*60 + '\n')
        
        # 保存详细统计数据
        stats_data = {
            'total_tasks': total_tasks,
            'success_count': self.success_count,
            'fail_count': self.timeout_count,
            'success_rate': success_rate,
            'completion_times': self.completion_times,
            'min_errors': self.min_errors,
            'mean_tci_velocity': self.mean_tci_velocity,
            'mean_tci_torque': self.mean_tci_torque,
            'error_threshold_deg': self.error_thres_deg,
            'timeout_sec': self.timeout
        }
        pickle.dump(stats_data, open(f'{self.dir_name}/statistics.pkl', 'wb'))
        self.get_logger().info(f'Statistics saved to {self.dir_name}/statistics.pkl')
            

def main(args=None):
    try:
        rclpy.init(args=args)
        node = JournalExperimentNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except:
        pass
    finally:
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
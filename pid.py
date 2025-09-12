import time
import math

# ========================================
# PID控制器类
# ========================================
class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, max_output=1.0, min_output=-1.0):
        """
        PID控制器初始化
        kp: 比例增益
        ki: 积分增益  
        kd: 微分增益
        max_output: 最大输出限制
        min_output: 最小输出限制
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        
        # PID状态变量
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
    def update(self, setpoint, current_value, dt=None):
        """
        更新PID控制器
        setpoint: 目标值
        current_value: 当前值
        dt: 时间步长（如果为None则自动计算）
        返回: PID输出
        """
        # 计算时间步长
        current_time = time.time()
        if dt is None:
            dt = current_time - self.last_time
            if dt <= 0:
                dt = 0.01  # 默认时间步长
        
        # 计算误差
        error = setpoint - current_value
        
        # 比例项
        proportional = self.kp * error
        
        # 积分项
        self.integral += error * dt
        integral = self.ki * self.integral
        
        # 微分项
        derivative = self.kd * (error - self.previous_error) / dt if dt > 0 else 0.0
        
        # PID输出
        output = proportional + integral + derivative
        
        # 输出限制
        output = max(self.min_output, min(self.max_output, output))
        
        # 更新状态
        self.previous_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        """重置PID控制器状态"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
    
    def set_parameters(self, kp=None, ki=None, kd=None):
        """动态调整PID参数"""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd

# ========================================
# 平衡控制器类
# ========================================
class BalanceController:
    def __init__(self):
        """
        平衡控制器初始化
        使用多个PID控制器来维持机器人平衡
        """
        # 姿态角PID控制器（防抖动微调参数）
        self.roll_pid = PIDController(kp=0.1, ki=0.001, kd=0.02, max_output=0.02, min_output=-0.02)
        self.pitch_pid = PIDController(kp=0.1, ki=0.001, kd=0.02, max_output=0.02, min_output=-0.02)
        self.yaw_pid = PIDController(kp=0.05, ki=0.0005, kd=0.01, max_output=0.01, min_output=-0.01)
        
        # 角速度PID控制器（用于阻尼，防抖动参数）
        self.roll_vel_pid = PIDController(kp=0.02, ki=0.0, kd=0.005, max_output=0.005, min_output=-0.005)
        self.pitch_vel_pid = PIDController(kp=0.02, ki=0.0, kd=0.005, max_output=0.005, min_output=-0.005)
        self.yaw_vel_pid = PIDController(kp=0.01, ki=0.0, kd=0.002, max_output=0.002, min_output=-0.002)
        
        # 目标姿态（理想平衡状态）
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0
        
        # 目标角速度（理想静止状态）
        self.target_roll_vel = 0.0
        self.target_pitch_vel = 0.0
        self.target_yaw_vel = 0.0
        
        # 平衡器启用状态
        self.enabled = True
        
        # 平滑滤波器（减少抖动）
        self.smoothing_factor = 0.1  # 平滑系数，越小越平滑
        self.last_roll_compensation = 0.0
        self.last_pitch_compensation = 0.0
        self.last_yaw_compensation = 0.0
        
        # 调试信息
        self.debug_info = {
            'roll_error': 0.0,
            'pitch_error': 0.0,
            'yaw_error': 0.0,
            'roll_compensation': 0.0,
            'pitch_compensation': 0.0,
            'yaw_compensation': 0.0
        }
    
    def update(self, current_orientation, current_angular_velocity, dt=0.01):
        """
        更新平衡控制器
        current_orientation: 当前姿态角 [roll, pitch, yaw]
        current_angular_velocity: 当前角速度 [roll_vel, pitch_vel, yaw_vel]
        dt: 时间步长
        返回: 姿态补偿量 [roll_compensation, pitch_compensation, yaw_compensation]
        """
        if not self.enabled:
            return [0.0, 0.0, 0.0]
        
        # 提取当前值
        current_roll, current_pitch, current_yaw = current_orientation
        current_roll_vel, current_pitch_vel, current_yaw_vel = current_angular_velocity
        
        # 姿态角PID控制
        roll_compensation = self.roll_pid.update(self.target_roll, current_roll, dt)
        pitch_compensation = self.pitch_pid.update(self.target_pitch, current_pitch, dt)
        yaw_compensation = self.yaw_pid.update(self.target_yaw, current_yaw, dt)
        
        # 角速度阻尼控制
        roll_vel_compensation = self.roll_vel_pid.update(self.target_roll_vel, current_roll_vel, dt)
        pitch_vel_compensation = self.pitch_vel_pid.update(self.target_pitch_vel, current_pitch_vel, dt)
        yaw_vel_compensation = self.yaw_vel_pid.update(self.target_yaw_vel, current_yaw_vel, dt)
        
        # 总补偿量 = 姿态补偿 + 角速度阻尼
        raw_roll_compensation = roll_compensation + roll_vel_compensation
        raw_pitch_compensation = pitch_compensation + pitch_vel_compensation
        raw_yaw_compensation = yaw_compensation + yaw_vel_compensation
        
        # 应用平滑滤波器（减少抖动）
        total_roll_compensation = (self.smoothing_factor * raw_roll_compensation + 
                                 (1 - self.smoothing_factor) * self.last_roll_compensation)
        total_pitch_compensation = (self.smoothing_factor * raw_pitch_compensation + 
                                  (1 - self.smoothing_factor) * self.last_pitch_compensation)
        total_yaw_compensation = (self.smoothing_factor * raw_yaw_compensation + 
                                (1 - self.smoothing_factor) * self.last_yaw_compensation)
        
        # 更新历史值
        self.last_roll_compensation = total_roll_compensation
        self.last_pitch_compensation = total_pitch_compensation
        self.last_yaw_compensation = total_yaw_compensation
        
        # 更新调试信息
        self.debug_info.update({
            'roll_error': self.target_roll - current_roll,
            'pitch_error': self.target_pitch - current_pitch,
            'yaw_error': self.target_yaw - current_yaw,
            'roll_compensation': total_roll_compensation,
            'pitch_compensation': total_pitch_compensation,
            'yaw_compensation': total_yaw_compensation
        })
        
        return [total_roll_compensation, total_pitch_compensation, total_yaw_compensation]
    
    def set_target_orientation(self, roll=None, pitch=None, yaw=None):
        """设置目标姿态角"""
        if roll is not None:
            self.target_roll = roll
        if pitch is not None:
            self.target_pitch = pitch
        if yaw is not None:
            self.target_yaw = yaw
    
    def set_target_angular_velocity(self, roll_vel=None, pitch_vel=None, yaw_vel=None):
        """设置目标角速度"""
        if roll_vel is not None:
            self.target_roll_vel = roll_vel
        if pitch_vel is not None:
            self.target_pitch_vel = pitch_vel
        if yaw_vel is not None:
            self.target_yaw_vel = yaw_vel
    
    def enable(self):
        """启用平衡控制器"""
        self.enabled = True
    
    def disable(self):
        """禁用平衡控制器"""
        self.enabled = False
        # 重置所有PID控制器
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()
        self.roll_vel_pid.reset()
        self.pitch_vel_pid.reset()
        self.yaw_vel_pid.reset()
    
    def set_pid_parameters(self, axis, kp=None, ki=None, kd=None):
        """
        设置指定轴的PID参数
        axis: 'roll', 'pitch', 'yaw', 'roll_vel', 'pitch_vel', 'yaw_vel'
        """
        if axis == 'roll':
            self.roll_pid.set_parameters(kp, ki, kd)
        elif axis == 'pitch':
            self.pitch_pid.set_parameters(kp, ki, kd)
        elif axis == 'yaw':
            self.yaw_pid.set_parameters(kp, ki, kd)
        elif axis == 'roll_vel':
            self.roll_vel_pid.set_parameters(kp, ki, kd)
        elif axis == 'pitch_vel':
            self.pitch_vel_pid.set_parameters(kp, ki, kd)
        elif axis == 'yaw_vel':
            self.yaw_vel_pid.set_parameters(kp, ki, kd)
    
    def get_debug_info(self):
        """获取调试信息"""
        return self.debug_info.copy()
    
    def reset_all(self):
        """重置所有PID控制器和平滑滤波器"""
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()
        self.roll_vel_pid.reset()
        self.pitch_vel_pid.reset()
        self.yaw_vel_pid.reset()
        
        # 重置平滑滤波器
        self.last_roll_compensation = 0.0
        self.last_pitch_compensation = 0.0
        self.last_yaw_compensation = 0.0

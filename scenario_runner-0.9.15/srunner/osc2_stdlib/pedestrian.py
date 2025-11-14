#!/usr/bin/env python

# Copyright (c) 2024 - Enhanced Pedestrian Support for OSC 2.0
# This module provides pedestrian support for OpenSCENARIO 2.0
# by referencing OpenSCENARIO 1.0 implementation patterns

"""
Pedestrian module for OSC 2.0
Provides walk() action and pedestrian-specific behaviors
Based on OpenSCENARIO 1.0 pedestrian control patterns
"""

import math
import carla
import srunner.osc2_stdlib.misc_object as misc
import srunner.scenariomanager.carla_data_provider as carla_provider
from srunner.tools.osc2_helper import OSC2Helper


class Pedestrian:
    """
    Pedestrian class for OSC 2.0
    
    This class provides pedestrian-specific functionality including:
    - walk() action (equivalent to vehicle's drive())
    - Pedestrian-specific modifiers support
    - Integration with CARLA Walker controller
    
    Based on OpenSCENARIO 1.0 pedestrian implementation patterns.
    """
    
    def __init__(self, gender: str = "male") -> None:
        self.gender = gender
        
        # ===== 基础属性 (与Vehicle对齐) =====
        self.model = "walker.pedestrian.*"  # CARLA walker蓝图通配符
        self.rolename = "scenario"
        self.position = misc.Position()
        self.transform = carla.Transform()
        self.speed = 0.0  # 当前速度 (m/s)
        self.autopilot = False  # 行人不使用autopilot
        self.random_location = True
        self.color = None
        self.category = "pedestrian"
        self.args = dict()
        
        # ===== 行人专属属性 =====
        self.walking = False  # 是否正在行走
        self.target_location = None  # 目标位置
        self.waypoints = []  # 行走路径点
        self.walking_speed = 1.4  # 默认行走速度 1.4 m/s (约5 km/h)
        self.running_speed = 3.5  # 跑步速度 3.5 m/s (约12.6 km/h)
        
        # ===== 行走模式 =====
        self.walk_mode = "normal"  # normal, fast, slow, run
        
    # -----------  基础方法（与Vehicle保持一致）  -----------
    
    def set_arg(self, kw):
        """设置参数"""
        self.args.update(kw)

    def get_arg(self, key):
        """获取参数"""
        return self.args.get(key)

    def set_name(self, name: str) -> None:
        """设置行人名称"""
        self.rolename = name

    def get_name(self) -> str:
        """获取行人名称"""
        return self.rolename

    def set_model(self, model: str) -> None:
        """
        设置行人模型
        
        Args:
            model: CARLA walker blueprint name
                   例如: "walker.pedestrian.0001", "walker.pedestrian.0002", etc.
        """
        self.model = model

    def set_position(self, pos: misc.Position) -> None:
        """
        设置行人位置
        
        Args:
            pos: Position对象 (WorldPosition或LanePosition)
        """
        self.position = pos
        
        if isinstance(pos, misc.WorldPosition):
            x = float(pos.x)
            y = float(pos.y)
            z = float(pos.z)
            yaw = float(pos.yaw)
            pitch = float(pos.pitch)
            roll = float(pos.roll)
            
            self.transform = carla.Transform(
                carla.Location(x=x, y=y, z=z),
                carla.Rotation(yaw=yaw, pitch=pitch, roll=roll),
            )
        elif isinstance(pos, misc.LanePosition):
            # LanePosition 转换 - 可使用CarlaDataProvider的方法
            print(f"[Pedestrian {self.rolename}] LanePosition -> Transform conversion needed")
        else:
            print(f"[Pedestrian {self.rolename}] Error: Unsupported position type {type(pos)}")

    def get_transform(self) -> carla.Transform:
        """
        获取行人的Transform
        
        Returns:
            carla.Transform: 行人的当前变换
        """
        # 如果是ego行人，从CARLA获取实时位置
        if OSC2Helper.wait_for_ego and self.rolename == OSC2Helper.ego_name:
            actor = carla_provider.CarlaDataProvider.get_actor_by_name(self.rolename)
            return carla_provider.CarlaDataProvider.get_transform(actor)
        
        # 如果有初始化transform，使用它
        if self.args.get("init_transform"):
            return self.args["init_transform"]
        
        # 默认返回当前transform
        return self.transform

    # -----------  OSC 2.0 行人专属动作方法  -----------
    
    def walk(self, path=None, **kwargs):
        """
        OSC 2.0 行人walk()动作 - 核心方法
        
        这是行人的主要动作方法，等同于车辆的drive()
        参考OSC 1.0的SpeedAction和RoutingAction实现
        
        Args:
            path: Path对象，定义行走路径（可选）
            **kwargs: 包含modifiers的关键字参数
                - speed: SpeedModifier - 行走速度
                - position: PositionModifier - 目标位置
                - lateral: 横向偏移
        
        Usage:
            pedestrian.walk(path) with:
                speed(1.5mps)
                position(10m, ahead_of: ego_vehicle)
        
        Returns:
            self: 支持链式调用
        """
        self.walking = True
        
        # 保存path信息
        if path:
            self.args["path"] = path
        
        # 标记动作类型
        self.args["action_type"] = "walk"
        
        # 处理modifiers（由场景解析器注入）
        if "modifiers" in kwargs:
            self._apply_walk_modifiers(kwargs["modifiers"])
        
        # 存储所有参数供后续处理
        self.args["walk_params"] = kwargs
        
        return self
    
    def _apply_walk_modifiers(self, modifiers: list):
        """
        应用行走修饰符
        
        Args:
            modifiers: 修饰符列表
        """
        # 这个方法会被OSC2场景解析器调用
        # 实际的modifier处理在osc2_scenario.py中完成
        pass
    
    # -----------  高级行人动作 (参考OSC 1.0)  -----------
    
    def walk_straight(self, distance: float, speed: float = None):
        """
        直线行走指定距离
        
        参考OSC 1.0的LongitudinalAction + DistanceAction
        
        Args:
            distance: 行走距离（米）
            speed: 行走速度（m/s），默认使用walking_speed
        """
        if speed is None:
            speed = self.walking_speed
        
        self.speed = speed
        self.walking = True
        self.args["action_type"] = "walk_straight"
        self.args["distance"] = distance
        self.args["target_speed"] = speed
        return self
    
    def walk_to(self, pos: misc.WorldPosition, speed: float = None) -> None:
        """
        行走到指定位置（OSC 2.0标准方法）
        
        参考OSC 1.0的AcquirePositionAction实现
        
        Args:
            pos: 目标位置（WorldPosition）
            speed: 行走速度（m/s），默认使用walking_speed
        """
        self.target_location = carla.Location(
            x=float(pos.x),
            y=float(pos.y),
            z=float(pos.z)
        )
        
        if speed is None:
            speed = self.walking_speed
        
        self.speed = speed
        self.walking = True
        self.args["action_type"] = "walk_to"
        self.args["target_location"] = self.target_location
        self.args["target_speed"] = speed

    def walk_to_position(self, pos: misc.WorldPosition) -> None:
        """
        行走到指定位置（别名方法）
        
        Args:
            pos: 目标位置（WorldPosition）
        """
        self.walk_to(pos)
    
    def stop(self):
        """
        停止行走
        
        参考OSC 1.0的SpeedAction with target_speed=0
        """
        self.speed = 0.0
        self.walking = False
        self.args["action_type"] = "stop"
        return self
    
    def run(self, path=None, **kwargs):
        """
        跑步动作（快速行走）
        
        Args:
            path: 路径对象
            **kwargs: 修饰符参数
        """
        self.walk_mode = "run"
        self.speed = self.running_speed
        return self.walk(path, **kwargs)
    
    def cross_road(self, target_side: str = "opposite", speed: float = None):
        """
        过马路动作
        
        这是一个高级动作，会自动计算过马路的路径
        参考OSC 1.0中的PedestrianCrossingFront场景
        
        Args:
            target_side: 目标侧 ("opposite", "left", "right")
            speed: 过马路速度（m/s），默认使用walking_speed
        """
        if speed is None:
            speed = self.walking_speed
            
        self.speed = speed
        self.args["action_type"] = "cross_road"
        self.args["target_side"] = target_side
        self.args["target_speed"] = speed
        self.walking = True
        return self
    
    def walk_along_route(self, waypoints: list, speed: float = None):
        """
        沿路径点行走
        
        参考OSC 1.0的FollowRouteAction和WaypointFollower
        
        Args:
            waypoints: carla.Location列表或carla.Waypoint列表
            speed: 行走速度（m/s），默认使用walking_speed
        """
        if speed is None:
            speed = self.walking_speed
            
        self.speed = speed
        self.waypoints = waypoints
        self.walking = True
        self.args["action_type"] = "walk_along_route"
        self.args["waypoints"] = waypoints
        self.args["target_speed"] = speed
        return self
    
    def turn_to(self, target_rotation: carla.Rotation):
        """
        转向指定方向
        
        参考OSC 1.0的TeleportAction的rotation部分
        
        Args:
            target_rotation: 目标旋转角度
        """
        self.args["action_type"] = "turn_to"
        self.args["target_rotation"] = target_rotation
        return self
    
    def stand_still(self, duration: float = None):
        """
        站立不动（静止等待）
        
        参考OSC 1.0的SpeedAction with speed=0
        
        Args:
            duration: 站立时长（秒），None表示持续直到下一个动作
        """
        self.speed = 0.0
        self.walking = False
        self.args["action_type"] = "stand_still"
        if duration:
            self.args["duration"] = duration
        return self
    
    def wait(self, duration: float = None):
        """
        等待动作（停留在原地）
        
        Args:
            duration: 等待时长（秒），None表示无限等待
        """
        self.speed = 0.0
        self.walking = False
        self.args["action_type"] = "wait"
        if duration:
            self.args["wait_duration"] = duration
        return self
    
    # -----------  辅助方法  -----------
    
    def calculate_walk_waypoints(self, start_location: carla.Location, end_location: carla.Location, 
                                   num_points: int = 10) -> list:
        """
        计算从起点到终点的行走路径点
        
        参考OSC 1.0中waypoint生成逻辑
        
        Args:
            start_location: 起始位置
            end_location: 结束位置
            num_points: 路径点数量
            
        Returns:
            list: carla.Location列表
        """
        waypoints = []
        for i in range(num_points + 1):
            alpha = i / num_points
            x = start_location.x + (end_location.x - start_location.x) * alpha
            y = start_location.y + (end_location.y - start_location.y) * alpha
            z = start_location.z + (end_location.z - start_location.z) * alpha
            waypoints.append(carla.Location(x=x, y=y, z=z))
        return waypoints
    
    def calculate_crossing_waypoints(self, current_transform: carla.Transform, 
                                       crossing_distance: float = 10.0) -> list:
        """
        计算过马路的路径点
        
        参考OSC 1.0的PedestrianCrossing场景
        
        Args:
            current_transform: 当前变换
            crossing_distance: 过马路距离（米）
            
        Returns:
            list: carla.Location列表
        """
        # 获取横向向量（垂直于行人朝向）
        right_vector = current_transform.rotation.get_right_vector()
        
        # 创建过马路路径点
        waypoints = []
        num_steps = int(crossing_distance / 0.5)  # 每0.5米一个点
        
        for i in range(num_steps + 1):
            offset = (i / num_steps) * crossing_distance
            location = current_transform.location + right_vector * offset
            waypoints.append(location)
            
        return waypoints
    
    def set_walking_speed(self, speed: float):
        """
        设置行走速度
        
        Args:
            speed: 速度 (m/s)
        """
        self.walking_speed = speed
        self.speed = speed
    
    def set_walk_mode(self, mode: str):
        """
        设置行走模式
        
        Args:
            mode: 模式 ("normal", "fast", "slow", "run")
        """
        mode_speeds = {
            "slow": 0.7,    # 慢走
            "normal": 1.4,  # 正常
            "fast": 2.2,    # 快走
            "run": 3.5      # 跑步
        }
        
        if mode in mode_speeds:
            self.walk_mode = mode
            self.walking_speed = mode_speeds[mode]
            self.speed = self.walking_speed
        else:
            print(f"[Pedestrian] Warning: Unknown walk mode '{mode}'")
    
    def get_walking_status(self) -> dict:
        """
        获取行走状态信息
        
        Returns:
            dict: 包含行走状态的字典
        """
        return {
            "walking": self.walking,
            "speed": self.speed,
            "mode": self.walk_mode,
            "target_location": self.target_location,
            "current_transform": self.transform,
            "waypoints": self.waypoints,
            "action_type": self.args.get("action_type", "none")
        }
    
    def is_walking(self) -> bool:
        """检查行人是否正在行走"""
        return self.walking
    
    def get_current_speed(self) -> float:
        """获取当前速度（m/s）"""
        return self.speed
    
    def get_distance_to(self, target_location: carla.Location) -> float:
        """
        计算到目标位置的距离
        
        Args:
            target_location: 目标位置
            
        Returns:
            float: 距离（米）
        """
        current_location = self.transform.location
        dx = target_location.x - current_location.x
        dy = target_location.y - current_location.y
        dz = target_location.z - current_location.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def __repr__(self):
        return (f"<Pedestrian '{self.rolename}' "
                f"model='{self.model}' "
                f"speed={self.speed:.2f}m/s "
                f"walking={self.walking}>")


# ===== 预定义行人子类 =====

class Man(Pedestrian):
    """男性行人"""
    def __init__(self) -> None:
        super().__init__("male")
        # 可以设置特定的男性行人模型
        self.model = "walker.pedestrian.0001"


class Woman(Pedestrian):
    """女性行人（标准拼写）"""
    def __init__(self) -> None:
        super().__init__("female")
        # 可以设置特定的女性行人模型
        self.model = "walker.pedestrian.0002"


class Women(Woman):
    """女性行人（保持向后兼容）"""
    pass


class Child(Pedestrian):
    """儿童行人"""
    def __init__(self) -> None:
        super().__init__("child")
        self.model = "walker.pedestrian.0007"
        # 儿童行走速度较慢
        self.walking_speed = 1.0
        self.running_speed = 2.5


class Elder(Pedestrian):
    """老年行人"""
    def __init__(self) -> None:
        super().__init__("elder")
        self.model = "walker.pedestrian.0009"
        # 老年人行走速度较慢
        self.walking_speed = 0.8
        self.running_speed = 1.5


# ===== 工具函数 =====

def create_pedestrian(gender: str = "male", model: str = None) -> Pedestrian:
    """
    工厂方法：创建行人实例
    
    Args:
        gender: 性别 ("male", "female", "child", "elder")
        model: 可选的自定义模型名称
    
    Returns:
        Pedestrian: 行人实例
    """
    pedestrian_classes = {
        "male": Man,
        "female": Woman,
        "child": Child,
        "elder": Elder
    }
    
    ped_class = pedestrian_classes.get(gender.lower(), Pedestrian)
    ped = ped_class()
    
    if model:
        ped.set_model(model)
    
    return ped
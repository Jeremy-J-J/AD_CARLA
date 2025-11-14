from __future__ import print_function

import copy
import math
import operator
import random
import re
import sys
import carla
from typing import List, Tuple

import py_trees
from agents.navigation.global_route_planner import GlobalRoutePlanner

from srunner.osc2.ast_manager import ast_node
from srunner.osc2.ast_manager.ast_vistor import ASTVisitor

# OSC2
from srunner.osc2.symbol_manager.method_symbol import MethodSymbol
from srunner.osc2.symbol_manager.parameter_symbol import ParameterSymbol
from srunner.osc2.utils.log_manager import (LOG_INFO, LOG_ERROR, LOG_WARNING)
from srunner.osc2.utils.relational_operator import RelationalOperator
from srunner.osc2_dm.physical_types import Physical, Range

# from sqlalchemy import true
# from srunner.osc2_stdlib import event, variables
from srunner.osc2_stdlib.modifier import (
    AccelerationModifier,
    ChangeLaneModifier,
    ChangeSpeedModifier,
    LaneModifier,
    PositionModifier,
    SpeedModifier,
)

# OSC2
from srunner.scenarioconfigs.osc2_scenario_configuration import (
    OSC2ScenarioConfiguration,
)
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorTransformSetter,
    ChangeTargetSpeed,
    LaneChange,
    UniformAcceleration,
    WaypointFollower,
    calculate_distance,
)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    IfTriggerer,
    TimeOfWaitComparison,
)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.openscenario_parser import oneshot_with_check
from srunner.tools.osc2_helper import OSC2Helper

def _safe_get_actor(name: str):
    """
    从 CarlaDataProvider 取出名为 name 的 actor。
    若不存在，返回 None；否则返回 actor 本身。
    """
    try:
        return CarlaDataProvider.get_actor_by_name(name)
    except RuntimeError:
        return None

def para_type_str_sequence(config, arguments, line, column, node):
    retrieval_name = ""
    if isinstance(arguments, List):
        for arg in arguments:
            if isinstance(arg, Tuple):
                if isinstance(arg[1], int):
                    retrieval_name = retrieval_name + "#int"
                elif isinstance(arg[1], float):
                    retrieval_name = retrieval_name + "#float"
                elif isinstance(arg[1], str):
                    retrieval_name = retrieval_name + "#str"
                elif isinstance(arg[1], bool):
                    retrieval_name = retrieval_name + "#bool"
                elif isinstance(arg[1], Physical):
                    physical_type_name = OSC2Helper.find_physical_type(
                        config.physical_dict, arg[1].unit.physical.si_base_exponent
                    )

                    if physical_type_name is None:
                        pass
                    else:
                        physical_type = (
                            node.get_scope().resolve(physical_type_name).name
                        )
                        retrieval_name += "#" + physical_type
                else:
                    pass
            elif isinstance(arg, str):
                retrieval_name = retrieval_name + arg.split(".", 1)[-1]
            else:
                pass
    elif isinstance(arguments, Tuple):
        if isinstance(arguments[1], int):
            retrieval_name = retrieval_name + "#int"
        elif isinstance(arguments[1], float):
            retrieval_name = retrieval_name + "#float"
        elif isinstance(arguments[1], str):
            retrieval_name = retrieval_name + "#str"
        elif isinstance(arguments[1], bool):
            retrieval_name = retrieval_name + "#bool"
        elif isinstance(arguments[1], Physical):
            physical_type_name = OSC2Helper.find_physical_type(
                config.physical_dict, arguments[1].unit.physical.si_base_exponent
            )

            if physical_type_name is None:
                pass
            else:
                physical_type = node.get_scope().resolve(physical_type_name).name
                retrieval_name += "#" + physical_type
        else:
            pass
    elif isinstance(arguments, int):
        retrieval_name = retrieval_name + "#int"
    elif isinstance(arguments, float):
        retrieval_name = retrieval_name + "#float"
    elif isinstance(arguments, str):
        retrieval_name = retrieval_name + "#str"
    elif isinstance(arguments, bool):
        retrieval_name = retrieval_name + "#bool"
    elif isinstance(arguments, Physical):
        physical_type_name = OSC2Helper.find_physical_type(
            config.physical_dict, arguments.unit.physical.si_base_exponent
        )

        if physical_type_name is None:
            pass
        else:
            physical_type = node.get_scope().resolve(physical_type_name).name
            retrieval_name += "#" + physical_type
    else:
        pass
    return retrieval_name


def process_speed_modifier(
    config, modifiers, duration: float, all_duration: float, father_tree
):
    if not modifiers:
        return

    for modifier in modifiers:
        actor_name = modifier.get_actor_name()

        if isinstance(modifier, SpeedModifier):
            # en_value_mps() The speed unit in Carla is m/s, so the default conversion unit is m/s
            target_speed = modifier.get_speed().gen_physical_value()
            # target_speed = float(modifier.get_speed())*0.27777778
            actor = CarlaDataProvider.get_actor_by_name(actor_name)
            car_driving = WaypointFollower(actor, target_speed)
            # car_driving.set_duration(duration)

            father_tree.add_child(car_driving)

            car_config = config.get_car_config(actor_name)
            car_config.set_arg({"target_speed": target_speed})
            LOG_WARNING(
                f"{actor_name} car speed will be set to {target_speed * 3.6} km/h"
            )

            # # _velocity speed, go straight down the driveway, and will hit the wall
            # keep_speed = KeepVelocity(actor, target_speed, duration=father_duration.num)
        elif isinstance(modifier, ChangeSpeedModifier):
            # speed_delta indicates the increment of velocity
            speed_delta = modifier.get_speed().gen_physical_value()
            speed_delta = speed_delta * 3.6
            current_car_conf = config.get_car_config(actor_name)
            current_car_speed = current_car_conf.get_arg("target_speed")
            current_car_speed = current_car_speed * 3.6
            target_speed = current_car_speed + speed_delta
            LOG_WARNING(
                f"{actor_name} car speed will be changed to {target_speed} km/h"
            )

            actor = CarlaDataProvider.get_actor_by_name(actor_name)
            change_speed = ChangeTargetSpeed(actor, target_speed)

            car_driving = WaypointFollower(actor)
            # car_driving.set_duration(duration)

            father_tree.add_child(change_speed)
            father_tree.add_child(car_driving)
        elif isinstance(modifier, AccelerationModifier):
            current_car_conf = config.get_car_config(actor_name)
            current_car_speed = current_car_conf.get_arg("target_speed")
            accelerate_speed = modifier.get_accelerate().gen_physical_value()
            target_velocity = current_car_speed + accelerate_speed * duration
            actor = CarlaDataProvider.get_actor_by_name(actor_name)
            start_time = all_duration - duration
            uniform_accelerate_speed = UniformAcceleration(
                actor, current_car_speed, target_velocity, accelerate_speed, start_time
            )
            print("END ACCELERATION")
            car_driving = WaypointFollower(actor)

            father_tree.add_child(uniform_accelerate_speed)
            father_tree.add_child(car_driving)
        else:
            LOG_WARNING("not implement modifier")


def process_pedestrian_walk_action(config, modifiers, duration: float, all_duration: float, father_tree, actor_name: str):
    """
    处理行人的walk动作，参考OSC 1.0的行人控制实现
    
    基于OpenSCENARIO 1.0中的PedestrianControl和WaypointFollower
    处理行人的速度控制、位置控制和路径跟随
    
    Args:
        config: OSC2场景配置
        modifiers: 修饰符列表（speed, position等）
        duration: 动作持续时间
        all_duration: 总持续时间
        father_tree: 父行为树节点
        actor_name: 行人actor名称
    """
    import time
    import copy
    import operator
    import py_trees
    
    print(f"[Pedestrian Walk] Called for {actor_name}, duration={duration}, modifiers count={len(modifiers) if modifiers else 0}")
    
    # 关键修复：在创建新的 WaypointFollower 之前，主动终止该actor所有运行中的旧 WaypointFollower
    # 这样可以确保新阶段的行为不会被旧阶段的行为覆盖
    actor = _safe_get_actor(actor_name)
    if actor is not None:
        try:
            check_attr = operator.attrgetter(f"running_WF_actor_{actor.id}")
            running_wf = check_attr(py_trees.blackboard.Blackboard())
            if running_wf:
                # 标记所有运行中的 WF 应该被终止
                py_trees.blackboard.Blackboard().set(
                    f"terminate_WF_actor_{actor.id}", 
                    copy.copy(running_wf), 
                    overwrite=True
                )
                LOG_INFO(f"[Pedestrian Walk] Marked {len(running_wf)} old WaypointFollowers for termination for actor {actor_name}")
        except AttributeError:
            # 没有运行中的 WF，这是正常的
            LOG_INFO(f"[Pedestrian Walk] No existing WaypointFollowers for actor {actor_name}")
            pass
    
    if not modifiers:
        LOG_WARNING(f"[Pedestrian Walk] No modifiers provided for {actor_name}, using default behavior")
        # 即使没有modifiers，也应该添加一个基本的行走行为
        if actor and isinstance(actor, carla.Walker):
            unique_name = f"Walk_{actor_name}_{int(time.time() * 1000000)}"
            pedestrian_follower = WaypointFollower(actor, 1.4, name=unique_name)  # 默认速度
            father_tree.add_child(pedestrian_follower)
            LOG_INFO(f"[Pedestrian Walk] Added default WaypointFollower for {actor_name}")
        return
    if actor is None:
        LOG_WARNING(f"[Pedestrian Walk] Actor {actor_name} not found, skipping walk action")
        return
    
    # 检查是否是行人
    if not isinstance(actor, carla.Walker):
        LOG_WARNING(f"[Pedestrian Walk] Actor {actor_name} is not a pedestrian (Walker)")
        return
    
    # 提取速度和位置修饰符
    speed_value = None
    position_info = None
    lateral_offset = 0.0
    
    print(f"[Pedestrian Walk] ===== PROCESSING MODIFIERS FOR {actor_name} =====")
    print(f"[Pedestrian Walk] Total modifiers: {len(modifiers)}")
    
    for i, modifier in enumerate(modifiers):
        print(f"[Pedestrian Walk] Modifier {i}: type={type(modifier).__name__}")
        
        if isinstance(modifier, SpeedModifier):
            # 获取行走速度（m/s）
            speed_value = modifier.get_speed().gen_physical_value()
            LOG_INFO(f"[Pedestrian Walk] {actor_name} walk speed set to {speed_value:.2f} m/s")
            
        elif isinstance(modifier, PositionModifier):
            # 获取目标位置信息
            distance = modifier.get_distance()
            refer_actor, relation = modifier.get_refer_car()
            trigger_point = modifier.get_trigger_point()
            
            print(f"[Pedestrian Walk] PositionModifier details:")
            print(f"[Pedestrian Walk]   - distance: {distance}")
            print(f"[Pedestrian Walk]   - refer_actor: {refer_actor}")
            print(f"[Pedestrian Walk]   - relation: {relation}")
            print(f"[Pedestrian Walk]   - trigger_point: {trigger_point}")
            
            position_info = {
                "distance": distance,
                "refer_actor": refer_actor,
                "relation": relation,
                "trigger_point": trigger_point
            }
            
            # 检查是否有lateral参数（横向偏移）
            if hasattr(modifier, 'args') and 'lateral' in modifier.args:
                lateral_offset = modifier.args['lateral'].gen_physical_value()
                LOG_INFO(f"[Pedestrian Walk] {actor_name} lateral offset: {lateral_offset:.2f} m")
            else:
                LOG_INFO(f"[Pedestrian Walk] No lateral offset specified")
    
    LOG_INFO(f"[Pedestrian Walk] ===== END MODIFIERS PROCESSING =====")
    
    # 默认行走速度（如果未指定）
    if speed_value is None:
        speed_value = 1.4  # 默认行走速度 1.4 m/s
    
    print(f"[Pedestrian Walk] After processing, position_info is: {position_info}")
    print(f"[Pedestrian Walk] Speed value: {speed_value}")
    
    # 处理位置相关的行走
    if position_info:
        print(f"[Pedestrian Walk] position_info is NOT None, entering position processing")
        # 如果有目标位置，计算路径点
        trigger_point = position_info.get("trigger_point", "all")
        
        LOG_INFO(f"[Pedestrian Walk] ========================================")
        LOG_INFO(f"[Pedestrian Walk] Processing position_info for {actor_name}")
        LOG_INFO(f"[Pedestrian Walk] trigger_point: {trigger_point}")
        LOG_INFO(f"[Pedestrian Walk] ========================================")
        
        if trigger_point in ("start", "all"):
            LOG_INFO(f"[Pedestrian Walk] >>> Entering 'at: start' processing for {actor_name}")
            # 在start时设置初始位置
            refer_actor_name = position_info.get("refer_actor")
            relation = position_info.get("relation")
            distance_physical = position_info.get("distance")
            
            if refer_actor_name and distance_physical:
                try:
                    # 获取参考actor的配置
                    refer_config = config.get_car_config(refer_actor_name)
                    refer_transform = refer_config.get_transform()
                    refer_location = refer_transform.location
                    
                    distance_value = distance_physical.gen_physical_value()
                    
                    # 计算行人的目标位置
                    forward_vector = refer_transform.rotation.get_forward_vector()
                    right_vector = refer_transform.rotation.get_right_vector()
                    
                    if relation == "ahead_of":
                        # 在参考车辆前方
                        target_location = refer_location + forward_vector * distance_value
                    elif relation == "behind":
                        # 在参考车辆后方
                        target_location = refer_location - forward_vector * distance_value
                    elif relation == "left_of":
                        # 在参考车辆左侧
                        target_location = refer_location - right_vector * distance_value
                    elif relation == "right_of":
                        # 在参考车辆右侧
                        target_location = refer_location + right_vector * distance_value
                    else:
                        target_location = refer_location
                    
                    # 添加横向偏移
                    if lateral_offset != 0:
                        target_location += right_vector * lateral_offset
                    
                    # 确保Z坐标足够高，避免行人在地面下
                    # 检查参考位置的Z坐标，如果太低则使用合理的高度
                    # 行人需要比车辆更高的spawn点，因为行人模型的原点在脚部附近
                    # 经过测试，至少需要1.2米的配置高度（加上CARLA自动0.2后=1.4米最终高度）
                    if target_location.z < 1.0:
                        # Z坐标太低，设置为足够的高度
                        target_location.z = max(refer_location.z + 0.6, 1.2)
                    
                    # 设置行人的transform
                    target_transform = carla.Transform(
                        target_location,
                        refer_transform.rotation
                    )
                    
                    # 添加设置位置的行为
                    father_tree.add_child(ActorTransformSetter(actor, target_transform))
                    
                    # 保存到配置中 - 这一步非常关键！
                    # 这会影响actor spawn的位置
                    try:
                        ped_config = config.get_car_config(actor_name)
                        # 设置transform属性，这样spawn时会使用这个位置
                        ped_config.transform = target_transform
                        ped_config.random_location = False  # 禁用随机位置
                        # 注意：不要在这里设置init_transform！
                        # init_transform会在_save_initial_transforms中从CARLA actor获取真实位置
                        # 这里设置会覆盖正确的初始位置
                        # ped_config.set_arg({"init_transform": target_transform})
                        ped_config.set_arg({"target_speed": speed_value})
                        LOG_INFO(f"[Pedestrian Walk] Set transform for {actor_name}: {target_transform.location}")
                    except KeyError as e:
                        # 如果行人不在car_config中，跳过
                        LOG_WARNING(f"[Pedestrian Walk] Cannot find config for {actor_name}")
                    
                    LOG_INFO(f"[Pedestrian Walk] {actor_name} position set relative to {refer_actor_name}")
                    
                except (KeyError, AttributeError) as e:
                    LOG_WARNING(f"[Pedestrian Walk] Failed to set position for {actor_name}: {e}")
        
        if trigger_point in ("end", "all"):
            print(f"[Pedestrian Walk] >>> Entering 'at: end' processing for {actor_name}")
            # 在end时计算终点位置并添加行走行为
            # 使用WaypointFollower来控制行人行走
            refer_actor_name = position_info.get("refer_actor")
            relation = position_info.get("relation")
            distance_physical = position_info.get("distance")
            
            LOG_INFO(f"[Pedestrian Walk] 'at: end' parameters:")
            LOG_INFO(f"[Pedestrian Walk]   - refer_actor_name: {refer_actor_name}")
            LOG_INFO(f"[Pedestrian Walk]   - relation: {relation}")
            LOG_INFO(f"[Pedestrian Walk]   - distance_physical: {distance_physical}")
            LOG_INFO(f"[Pedestrian Walk]   - lateral_offset: {lateral_offset}")
            
            if refer_actor_name and distance_physical:
                print(f"[Pedestrian Walk] Valid 'at: end' parameters, proceeding to calculate target position")
                print(f"[Pedestrian Walk] lateral_offset = {lateral_offset}")
                try:
                    # 获取参考actor的当前实时位置
                    # 这样每个阶段都会基于参考actor的当前位置计算目标点
                    refer_actor = _safe_get_actor(refer_actor_name)
                    if refer_actor is not None:
                        # 使用实时位置和旋转
                        refer_transform = CarlaDataProvider.get_transform(refer_actor)
                    else:
                        # 如果无法获取actor，回退到配置
                        refer_config = config.get_car_config(refer_actor_name)
                        refer_transform = refer_config.get_transform()
                    
                    refer_location = refer_transform.location
                    
                    distance_value = distance_physical.gen_physical_value()
                    
                    # 计算目标位置
                    forward_vector = refer_transform.rotation.get_forward_vector()
                    right_vector = refer_transform.rotation.get_right_vector()
                    
                    if relation == "ahead_of":
                        end_location = refer_location + forward_vector * distance_value
                    elif relation == "behind":
                        end_location = refer_location - forward_vector * distance_value
                    elif relation == "left_of":
                        end_location = refer_location - right_vector * distance_value
                    elif relation == "right_of":
                        end_location = refer_location + right_vector * distance_value
                    else:
                        end_location = refer_location
                    
                    # 添加横向偏移
                    if lateral_offset != 0:
                        end_location += right_vector * lateral_offset
                    
                    # 计算行走路径点
                    start_location = CarlaDataProvider.get_location(actor)
                    print(f"[Pedestrian Walk] start_location: {start_location}")
                    print(f"[Pedestrian Walk] end_location: {end_location}")
                    waypoints = _calculate_pedestrian_waypoints(start_location, end_location)
                    print(f"[Pedestrian Walk] Calculated {len(waypoints)} waypoints")
                    
                    # 创建WaypointFollower行为，传入路径点
                    # 重要：每个WaypointFollower必须有唯一的名字，否则在连续阶段中会冲突
                    unique_name = f"WalkTo_{actor_name}_{int(time.time() * 1000000)}"
                    print(f"[Pedestrian Walk] Creating WaypointFollower with name: {unique_name}")
                    pedestrian_follower = WaypointFollower(actor, speed_value, plan=waypoints, name=unique_name)
                    print(f"[Pedestrian Walk] father_tree type: {type(father_tree)}, name: {father_tree.name if hasattr(father_tree, 'name') else 'N/A'}")
                    father_tree.add_child(pedestrian_follower)
                    print(f"[Pedestrian Walk] WaypointFollower created and added to tree")
                    print(f"[Pedestrian Walk] father_tree children count: {len(father_tree.children) if hasattr(father_tree, 'children') else 'N/A'}")
                    
                    LOG_INFO(f"[Pedestrian Walk] ========== CREATING NEW WAYPOINT FOLLOWER ==========")
                    LOG_INFO(f"[Pedestrian Walk] Actor: {actor_name}")
                    LOG_INFO(f"[Pedestrian Walk] Current location: {start_location}")
                    LOG_INFO(f"[Pedestrian Walk] Refer actor {refer_actor_name} current location: {refer_location}")
                    LOG_INFO(f"[Pedestrian Walk] Target end_location: {end_location}")
                    LOG_INFO(f"[Pedestrian Walk] Distance: {distance_value}m, Lateral: {lateral_offset}m")
                    LOG_INFO(f"[Pedestrian Walk] Waypoints count: {len(waypoints)}")
                    LOG_INFO(f"[Pedestrian Walk] WaypointFollower name: {unique_name}")
                    LOG_INFO(f"[Pedestrian Walk] =====================================================")
                    
                except (KeyError, AttributeError) as e:
                    LOG_WARNING(f"[Pedestrian Walk] Failed to calculate end position for {actor_name}: {e}")
                    # 回退到简单的follower
                    unique_name = f"Walk_{actor_name}_{int(time.time() * 1000000)}"
                    pedestrian_follower = WaypointFollower(actor, speed_value, name=unique_name)
                    father_tree.add_child(pedestrian_follower)
            else:
                # 没有完整的位置信息（refer_actor_name 或 distance_physical 为空）
                LOG_INFO(f"[Pedestrian Walk] !!! 'at: end' conditions not met !!!")
                LOG_INFO(f"[Pedestrian Walk] refer_actor_name: {position_info.get('refer_actor')}")
                LOG_INFO(f"[Pedestrian Walk] distance_physical: {position_info.get('distance')}")
                LOG_INFO(f"[Pedestrian Walk] Using simple follower without target position")
                unique_name = f"Walk_{actor_name}_{int(time.time() * 1000000)}"
                pedestrian_follower = WaypointFollower(actor, speed_value, name=unique_name)
                father_tree.add_child(pedestrian_follower)
    else:
        # 没有位置信息，只是以指定速度行走
        LOG_INFO(f"[Pedestrian Walk] position_info is None, using simple follower with speed {speed_value}")
        unique_name = f"Walk_{actor_name}_{int(time.time() * 1000000)}"
        pedestrian_follower = WaypointFollower(actor, speed_value, name=unique_name)
        father_tree.add_child(pedestrian_follower)
        
        # 保存速度到配置
        try:
            ped_config = config.get_car_config(actor_name)
            ped_config.set_arg({"target_speed": speed_value})
        except KeyError:
            pass
    
    LOG_INFO(f"[Pedestrian Walk] Processed walk action for {actor_name} with speed {speed_value:.2f} m/s")


def _calculate_pedestrian_waypoints(start_location: carla.Location, end_location: carla.Location, 
                                     step_size: float = 1.0) -> list:
    """
    计算行人从起点到终点的路径点列表
    
    参考OSC 1.0的路径生成逻辑
    
    Args:
        start_location: 起始位置
        end_location: 结束位置
        step_size: 路径点间距（米）
        
    Returns:
        list: carla.Location列表
    """
    waypoints = []
    
    # 计算总距离
    dx = end_location.x - start_location.x
    dy = end_location.y - start_location.y
    dz = end_location.z - start_location.z
    total_distance = math.sqrt(dx*dx + dy*dy + dz*dz)
    
    if total_distance < 0.1:
        # 距离太近，直接返回终点
        return [end_location]
    
    # 计算需要的路径点数量
    num_points = max(int(total_distance / step_size), 2)
    
    # 生成路径点
    for i in range(num_points + 1):
        alpha = i / num_points
        x = start_location.x + dx * alpha
        y = start_location.y + dy * alpha
        z = start_location.z + dz * alpha
        waypoints.append(carla.Location(x=x, y=y, z=z))
    
    return waypoints


def process_location_modifier(config, modifiers, duration: float, father_tree):
    """
    统一处理位置 / 车道类修饰符，兼容：
      • LaneModifier
      • PositionModifier（可能不含 lane 字段，例如行人）
      • ChangeLaneModifier
    """
    if not modifiers:
        return

    # ────────────────── 工具函数 ──────────────────
    def _lane_id_from_modifier(mod):
        """
        尝试从任意修饰符对象中解析 lane_id，解析不到返回 None
        """
        if hasattr(mod, "get_lane_id"):          # 典型 LaneModifier
            return mod.get_lane_id()

        # PositionModifier 的几种可能存储位置
        for key in ("_args", "args"):
            if hasattr(mod, key):
                args = getattr(mod, key)
                if isinstance(args, dict) and "lane" in args:
                    return int(args["lane"])
        if hasattr(mod, "lane"):
            return int(mod.lane)

        return None

    # ────────────────── 1. change_lane() 立即处理 ──────────────────
    for modifier in modifiers:
        if isinstance(modifier, ChangeLaneModifier):
            npc_name     = modifier.get_actor_name()
            lane_changes = modifier.get_lane_changes()
            direction    = modifier.get_side()

            actor = CarlaDataProvider.get_actor_by_name(npc_name)
            lane_change    = LaneChange(actor, speed=None,
                                        direction=direction,
                                        lane_changes=lane_changes)
            continue_drive = WaypointFollower(actor)

            father_tree.add_child(lane_change)
            father_tree.add_child(continue_drive)
            return

    # ────────────────── 2. start 触发，绝对位置 ──────────────────
    event_start = [
        m for m in modifiers
        if m.get_trigger_point() == "start" and m.get_refer_car() is None
    ]

    for m in event_start:
        car_name = m.get_actor_name()
        lane_id  = _lane_id_from_modifier(m)

        # 无 lane_id（如行人） → 随机找一处行人导航点
        if lane_id is None:
            nav_loc = CarlaDataProvider.get_world().get_random_location_from_navigation()
            if nav_loc is None:
                raise RuntimeError("无法在导航网格上找到可用位置")
            wp = CarlaDataProvider.get_map().get_waypoint(
                    nav_loc, project_to_road=False, lane_type=carla.LaneType.Sidewalk)
            if wp is None:
                wp = CarlaDataProvider.get_map().get_waypoint(nav_loc)
        else:
            wp = CarlaDataProvider.get_waypoint_by_laneid(lane_id)
            if wp is None:
                raise RuntimeError(f"lane_id={lane_id} 未找到有效航路点")

        # 如果此时 actor 还未生成（行人通常如此），TransformSetter 可以先跳过
        actor = CarlaDataProvider.get_actor_by_name(car_name)
        if actor is not None:
            father_tree.add_child(ActorTransformSetter(actor, wp.transform))

        # 仅车辆在 car_config 中登记
        try:
            car_cfg = config.get_car_config(car_name)
            # 重要：这里的init_transform用于spawn前的位置计算
            # 但会被_save_initial_transforms中保存的真实位置覆盖
            # 所以这里设置是安全的，不会影响最终使用的初始位置
            car_cfg.set_arg({"init_transform": wp.transform})
        except KeyError:
            pass  # 行人等非车辆忽略

    # ────────────────── 3. start 触发，带参考车辆 ──────────────────
    start_group = [
        m for m in modifiers
        if m.get_trigger_point() == "start" and m.get_refer_car() is not None
    ]

    init_wp = None
    npc_name = None

    for modifier in start_group:
        npc_name          = modifier.get_actor_name()
        ref_car, location = modifier.get_refer_car()

        ref_cfg = config.get_car_config(ref_car)
        ref_loc = ref_cfg.get_transform().location
        ref_wp  = CarlaDataProvider.get_map().get_waypoint(ref_loc)

        if init_wp is None:
            init_wp = ref_wp

        if location == "left_of":
            init_wp = init_wp.get_left_lane() or init_wp
        elif location == "right_of":
            init_wp = init_wp.get_right_lane() or init_wp
        elif location == "same_as":
            pass
        elif location in ("ahead_of", "behind"):
            dist = modifier.get_distance().gen_physical_value()
            seq  = init_wp.next if location == "ahead_of" else init_wp.previous
            wp_list = seq(dist)
            if not wp_list:
                LOG_WARNING(f"[OSC2] Cannot find waypoint {dist}m {location} for {npc_name}, using current waypoint")
                # 保持当前init_wp不变
            else:
                init_wp = wp_list[0]
        else:
            raise KeyError(f"未知相对位置 {location}")

    if init_wp and npc_name:
        actor = CarlaDataProvider.get_actor_by_name(npc_name)
        if actor is not None:
            father_tree.add_child(ActorTransformSetter(actor, init_wp.transform))

        try:
            car_cfg = config.get_car_config(npc_name)
            # 用于spawn前的位置，会被_save_initial_transforms覆盖
            car_cfg.set_arg({"init_transform": init_wp.transform})
        except KeyError:
            pass

    # ────────────────── 4. end 触发 ──────────────────
    end_group   = [m for m in modifiers if m.get_trigger_point() == "end"]
    end_wp      = None
    end_lane_wp = None

    for modifier in end_group:
        npc_name          = modifier.get_actor_name()
        ref_car, location = modifier.get_refer_car()

        try:
            ref_cfg = config.get_car_config(ref_car)
        except KeyError:
            continue  # 参考对象不是车辆，直接跳过

        ref_wp    = CarlaDataProvider.get_map().get_waypoint(
                        ref_cfg.get_transform().location)
        ref_speed = ref_cfg.get_arg("target_speed")

        drive_dist = ref_speed * float(duration)
        base_wp    = ref_wp.next(drive_dist)[0]

        if location in ("ahead_of", "behind"):
            offset = modifier.get_distance().gen_physical_value()
            seq    = base_wp.next if location == "ahead_of" else base_wp.previous
            end_wp = seq(offset)[0]

        elif location in ("left_of", "right_of", "same_as"):
            lane_func = {
                "left_of":  ref_wp.get_left_lane,
                "right_of": ref_wp.get_right_lane,
                "same_as":  lambda: ref_wp,
            }[location]
            end_lane_wp = lane_func()

    # 4.1 计算到 end_wp 的期望速度
    if end_wp:
        try:
            car_cfg = config.get_car_config(npc_name)
        except KeyError:
            end_wp = None   # 非车辆，无需后续速度处理
        else:
            cur_tf   = car_cfg.get_arg("init_transform")
            grp      = GlobalRoutePlanner(CarlaDataProvider.get_world().get_map(), 0.5)
            distance = calculate_distance(cur_tf.location, end_wp.transform.location, grp)
            need_spd = distance / float(duration)

            car_cfg.set_arg({"desired_speed": need_spd})
            actor = CarlaDataProvider.get_actor_by_name(npc_name)
            if actor is not None:
                father_tree.add_child(WaypointFollower(actor, need_spd))

    # 4.2 末态需要变道
    if end_lane_wp:
        try:
            car_cfg = config.get_car_config(npc_name)
        except KeyError:
            return  # 行人等非车辆无需变道

        cur_tf = car_cfg.get_arg("init_transform")
        cur_wp = CarlaDataProvider.get_map().get_waypoint(cur_tf.location)

        direction = None
        if end_lane_wp.lane_id == getattr(cur_wp.get_left_lane(), "lane_id", None):
            direction = "left"
        elif end_lane_wp.lane_id == getattr(cur_wp.get_right_lane(), "lane_id", None):
            direction = "right"

        if direction:
            actor = CarlaDataProvider.get_actor_by_name(npc_name)
            if actor is not None:
                lane_change = LaneChange(actor, speed=None, direction=direction,
                                         distance_same_lane=5, distance_other_lane=10)
                father_tree.add_child(lane_change)
                father_tree.add_child(WaypointFollower(actor))

            # 用于spawn前的位置，会被_save_initial_transforms覆盖
            car_cfg.set_arg({"init_transform": end_lane_wp.transform})


class OSC2Scenario(BasicScenario):
    """
    Implementation of the osc2 Scenario
    """

    def __init__(
        self,
        world,
        ego_vehicles,
        config: OSC2ScenarioConfiguration,
        osc2_file,
        debug_mode=False,
        criteria_enable=True,
        timeout=300,
    ):
        """
        Setup all relevant parameters and create scenario
        """
        self.config = config
        self.route = None
        self.osc2_file = osc2_file
        self.ast_tree = OSC2Helper.gen_osc2_ast(self.osc2_file)
        # Timeout of scenario in seconds
        self.timeout = timeout
        self.all_duration = float()

        self.other_actors = None

        self.behavior = None

        # Check whether the access is available
        self.visit_power = False
        self.scenario_declaration = config.scenario_declaration
        self.struct_declaration = config.struct_declaration
        # Use struct_parameters to store parameters of type struct, so that we can recognize it in keep constraint
        self.struct_parameters = {}

        super(OSC2Scenario, self).__init__(
            "OSC2Scenario",
            ego_vehicles=ego_vehicles,
            config=config,
            world=world,
            debug_mode=debug_mode,
            terminate_on_failure=False,
            criteria_enable=criteria_enable,
        )

    def set_behavior_tree(self, behavior):
        self.behavior = behavior

    def _initialize_actors(self, config):
        """
        Override父类方法，在spawn actors之前先提取并设置初始位置
        
        OSC2的行人位置信息在行为修饰符中（如 position(..., at: start)），
        需要先解析行为树提取这些信息，然后再spawn actors。
        """
        # 先预解析行为树，提取 at: start 的位置信息
        #self._extract_initial_positions(config)
        
        # 临时方案：为行人设置有效的spawn位置
        # 因为OSC2的位置信息在行为修饰符中，需要更复杂的解析
        self._set_pedestrian_spawn_positions(config)
        
        # 然后调用父类方法spawn actors
        super()._initialize_actors(config)
        
        # 保存所有actor的初始transform，用于后续位置计算
        # 这样即使actor在移动，也可以基于初始位置计算目标点
        self._save_initial_transforms(config)
    
    def _save_initial_transforms(self, config):
        """
        保存所有actor的初始transform到args['init_transform']
        
        这样在后续位置计算时，即使actor移动了，
        也可以使用初始位置作为参考，避免目标点随actor移动
        
        注意：必须在super()._initialize_actors()之后调用，
        因为只有spawn后才能从CARLA获取真实的transform
        """
        # 保存ego vehicles的初始transform（从CARLA actor获取，不是config）
        for ego_vehicle in self.ego_vehicles:
            ego_transform = CarlaDataProvider.get_transform(ego_vehicle)
            # 找到对应的config
            for ego_config in config.ego_vehicles:
                if ego_config.rolename == ego_vehicle.attributes.get('role_name'):
                    if not hasattr(ego_config, 'args') or ego_config.args is None:
                        ego_config.args = {}
                    ego_config.args['init_transform'] = ego_transform
                    break
        
        # 保存other actors的初始transform（从CARLA actor获取，不是config）
        for actor in self.other_actors:
            actor_transform = CarlaDataProvider.get_transform(actor)
            role_name = actor.attributes.get('role_name')
            # 找到对应的config
            for actor_config in config.other_actors:
                if actor_config.rolename == role_name:
                    if not hasattr(actor_config, 'args') or actor_config.args is None:
                        actor_config.args = {}
                    actor_config.args['init_transform'] = actor_transform
                    break
    
    def _set_pedestrian_spawn_positions(self, config):
        """
        为行人设置有效的spawn位置
        
        如果行人的transform是(0,0,0)，则基于ego车辆位置计算一个合理的位置
        """
        if not config.ego_vehicles:
            LOG_WARNING("[OSC2] No ego vehicle found, cannot set pedestrian positions")
            return
        
        ego_config = config.ego_vehicles[0]
        ego_transform = ego_config.get_transform()
        ego_location = ego_transform.location
        
        for i, actor_config in enumerate(config.other_actors):
            if actor_config.category != "pedestrian":
                continue
            
            # 检查是否需要设置位置
            current_loc = actor_config.transform.location
            if current_loc.x != 0 or current_loc.y != 0 or current_loc.z != 0:
                # 已经有位置了
                continue
            
            # 计算一个基于ego的位置：在ego前方右侧，横向分散
            forward_dist = 20.0 + i * 5.0  # 20m, 25m, 30m, ...
            lateral_offset = 4.0 + i * 2.0  # 右侧：4m, 6m, 8m, 10m（正值表示右侧）
            
            forward_vector = ego_transform.rotation.get_forward_vector()
            right_vector = ego_transform.rotation.get_right_vector()
            
            # 计算XY位置
            ped_location = ego_location + forward_vector * forward_dist + right_vector * lateral_offset
            
            # 使用CARLA地图查询获取正确的地面高度
            # 这样可以确保行人spawn在地面上而不是地面下
            try:
                from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
                carla_map = CarlaDataProvider.get_map()
                if carla_map:
                    # 获取该位置的waypoint（会自动对齐到道路）
                    waypoint = carla_map.get_waypoint(ped_location, project_to_road=True, lane_type=carla.LaneType.Any)
                    if waypoint:
                        # 使用waypoint的Z坐标作为地面高度
                        # 为了确保行人在地面上，如果Z值太小（接近0），使用ego的Z值
                        # 注意：行人需要额外的Z偏移来确保站在地面上而不是地面下
                        waypoint_z = waypoint.transform.location.z
                        if abs(waypoint_z) < 0.1:  # Z值太小，可能不准确
                            # 使用ego的Z值，并额外添加偏移确保行人在地面上
                            # CARLA会自动添加0.2的偏移，但这还不够，需要额外偏移
                            ped_location.z = ego_location.z + 0.5  # 额外+0.5米
                        else:
                            # 使用waypoint的Z值，也需要额外偏移
                            ped_location.z = waypoint_z + 0.5  # 额外+0.5米
                        LOG_INFO(f"[OSC2] Set ground height for {actor_config.rolename}: z={ped_location.z:.2f}")
                    else:
                        # 如果找不到waypoint，使用ego的Z坐标+额外偏移
                        ped_location.z = ego_location.z + 0.5
                        LOG_INFO(f"[OSC2] No waypoint found, using ego z for {actor_config.rolename}: z={ped_location.z:.2f}")
                else:
                    ped_location.z = ego_location.z + 0.5
                    LOG_INFO(f"[OSC2] No map available, using ego z for {actor_config.rolename}: z={ped_location.z:.2f}")
            except Exception as e:
                LOG_WARNING(f"[OSC2] Failed to get ground height for {actor_config.rolename}: {e}")
                ped_location.z = ego_location.z + 0.5
            
            ped_transform = carla.Transform(ped_location, ego_transform.rotation)
            actor_config.transform = ped_transform
            actor_config.random_location = False
            
            LOG_INFO(f"[OSC2] Set {actor_config.rolename} spawn position at (x={ped_location.x:.2f}, y={ped_location.y:.2f}, z={ped_location.z:.2f})")
    
    def _extract_initial_positions(self, config):
        """
        预解析行为树，提取 at: start 的位置修饰符并设置actor transform
        
        这个方法在actors spawn之前被调用，确保行人有正确的初始位置。
        """
        try:
            # 创建一个临时的visitor来提取位置信息
            position_extractor = self.PositionExtractor(self)
            position_extractor.visit(self.ast_tree)
            LOG_INFO("[OSC2] Initial positions extracted successfully")
        except Exception as e:
            LOG_WARNING(f"[OSC2] Failed to extract initial positions: {e}")
    
    class PositionExtractor(ASTVisitor):
        """专门用于提取at:start位置信息的visitor"""
        
        def __init__(self, scenario_instance):
            super().__init__()
            self.scenario = scenario_instance
            self.config = scenario_instance.config
        
        def visit_behavior_invocation(self, node: ast_node.BehaviorInvocation):
            """处理behavior invocation，提取位置信息"""
            actor = node.actor
            behavior_name = node.behavior_name
            
            if not actor or behavior_name != "walk":
                # 只处理行人的walk动作
                self.visit_children(node)
                return
            
            # 解析修饰符
            children = list(node.get_children())  # 转换为list
            for child in children:
                if isinstance(child, ast_node.ModifierInvocation):
                    self._process_modifier(actor, child)
            
            self.visit_children(node)
        
        def _process_modifier(self, actor_name, modifier_node):
            """处理修饰符，提取position(..., at: start)"""
            modifier_name = modifier_node.modifier_name
            
            print(f"[DEBUG _process_modifier] actor={actor_name}, modifier={modifier_name}")
            
            if modifier_name != "position":
                print(f"[DEBUG _process_modifier] Skipping non-position modifier: {modifier_name}")
                return
            
            print(f"[DEBUG _process_modifier] Processing position modifier for {actor_name}")
            
            # 解析position修饰符的参数
            arguments = self.visit_children(modifier_node)
            print(f"[DEBUG _process_modifier] arguments type={type(arguments)}, value={arguments}")
            if not arguments:
                print(f"[DEBUG _process_modifier] No arguments, returning")
                return
            
            # 提取参数
            distance = None
            refer_actor = None
            relation = None
            lateral = None
            trigger_point = "all"  # 默认值
            
            if isinstance(arguments, list):
                arguments = OSC2Helper.flat_list(arguments)
                print(f"[DEBUG _process_modifier] Flattened arguments: {arguments}")
                for arg in arguments:
                    print(f"[DEBUG _process_modifier] Processing arg: type={type(arg)}, value={arg}")
                    if isinstance(arg, tuple):
                        key, value = arg
                        print(f"[DEBUG _process_modifier] Tuple arg: key={key}, value={value}")
                        if key == "at":
                            trigger_point = value
                        elif key == "lateral":
                            lateral = value
                        elif key in ("ahead_of", "behind", "left_of", "right_of"):
                            relation = key
                            refer_actor = value
                    elif isinstance(arg, Physical):
                        distance = arg
                        print(f"[DEBUG _process_modifier] Physical distance: {distance}")
            
            print(f"[DEBUG _process_modifier] Extracted: trigger_point={trigger_point}, refer_actor={refer_actor}, relation={relation}, distance={distance}")
            
            # 只处理at:start的情况
            if trigger_point not in ("start", "all"):
                print(f"[DEBUG _process_modifier] trigger_point={trigger_point}, not start/all, returning")
                return
            
            if not refer_actor or not distance:
                print(f"[DEBUG _process_modifier] Invalid: refer_actor={refer_actor}, distance={distance}")
                LOG_WARNING(f"[Position Extract] Invalid position modifier for {actor_name}")
                return
            
            print(f"[DEBUG _process_modifier] Valid position modifier, calculating position...")
            
            # 计算位置
            try:
                refer_config = self.config.get_car_config(refer_actor)
                refer_transform = refer_config.get_transform()
                refer_location = refer_transform.location
                
                distance_value = distance.gen_physical_value()
                lateral_value = lateral.gen_physical_value() if lateral else 0.0
                
                # 计算目标位置
                forward_vector = refer_transform.rotation.get_forward_vector()
                right_vector = refer_transform.rotation.get_right_vector()
                
                if relation == "ahead_of":
                    target_location = refer_location + forward_vector * distance_value
                elif relation == "behind":
                    target_location = refer_location - forward_vector * distance_value
                elif relation == "left_of":
                    target_location = refer_location - right_vector * distance_value
                elif relation == "right_of":
                    target_location = refer_location + right_vector * distance_value
                else:
                    target_location = refer_location
                
                # 添加横向偏移
                if lateral_value != 0:
                    target_location += right_vector * lateral_value
                
                # 设置行人的transform
                target_transform = carla.Transform(
                    target_location,
                    refer_transform.rotation
                )
                
                # 更新配置
                ped_config = self.config.get_car_config(actor_name)
                ped_config.transform = target_transform
                ped_config.random_location = False  # 明确指定位置，不使用随机位置
                
                LOG_INFO(f"[Position Extract] Set {actor_name} initial position: {target_location}")
                
            except Exception as e:
                LOG_WARNING(f"[Position Extract] Failed to calculate position for {actor_name}: {e}")

    class BehaviorInit(ASTVisitor):
        def __init__(self, config_instance) -> None:
            super().__init__()
            self.father_ins = config_instance
            self.root_behavior = None
            self.__cur_behavior = None
            self.__parent_behavior = {}
            self.__duration = 1000000000.0

        def get_behavior_tree(self):
            return self.root_behavior

        def visit_scenario_declaration(self, node: ast_node.ScenarioDeclaration):
            scenario_name = node.qualified_behavior_name

            if scenario_name != "top" and not self.father_ins.visit_power:
                return

            if scenario_name == "top" and self.father_ins.visit_power:
                return

            for child in node.get_children():
                if isinstance(child, ast_node.DoDirective):
                    self.visit_do_directive(child)
                elif isinstance(child, ast_node.ModifierInvocation):
                    self.visit_modifier_invocation(child)
                elif isinstance(child, ast_node.ParameterDeclaration):
                    self.visit_parameter_declaration(child)
                elif isinstance(child, ast_node.KeepConstraintDeclaration):
                    self.visit_keep_constraint_declaration(child)

        def visit_do_directive(self, node: ast_node.DoDirective):
            self.visit_children(node)

        def bool_result(self, option):
            # wait(x < y) @drive_distance Handling of Boolean expressions x < y
            expression_value = re.split("\W+", option)
            symbol = re.search("\W+", option).group()
            if symbol == "<":
                symbol = operator.lt
            elif symbol == ">":
                symbol = operator.gt
            # if len(expression_value) == 2:
            #     x = variables.Variable.get_arg(expression_value[0])
            #     y = variables.Variable.get_arg(expression_value[1])
            x = expression_value[0]
            y = expression_value[1]
            if "ego" in x:
                actor_name = "ego_vehicle"
                actor_ego = CarlaDataProvider.get_actor_by_name(actor_name)
            if "npc" in y:
                actor_name = "npc"
                actor_npc = CarlaDataProvider.get_actor_by_name(actor_name)
            return actor_ego, actor_npc, symbol

        def visit_do_member(self, node: ast_node.DoMember):
            self.__duration = 1000000000.0
            composition_operator = node.composition_operator
            sub_node = None
            if composition_operator in ["serial", "parallel", "one_of"]:
                if composition_operator == "serial":
                    self.__cur_behavior = py_trees.composites.Sequence(
                        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
                        name="serial",
                    )
                elif composition_operator == "parallel":
                    self.__cur_behavior = py_trees.composites.Parallel(
                        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
                        name="parallel",
                    )
                elif composition_operator == "one_of":
                    self.__cur_behavior = py_trees.composites.Sequence(
                        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
                        name="one_of",
                    )
                    do_member_list = []
                    for child in node.get_children():
                        if isinstance(child, ast_node.DoMember):
                            do_member_list.append(child)
                    sub_node = random.choice(do_member_list)
            else:
                raise NotImplementedError(
                    f"no supported scenario operator {composition_operator}"
                )

            if self.root_behavior is None:
                self.root_behavior = self.__cur_behavior
                self.__parent_behavior[node] = self.__cur_behavior
            elif (
                self.root_behavior is not None
                and self.__parent_behavior.get(node) is None
            ):
                self.__parent_behavior[node] = self.root_behavior
                parent = self.__parent_behavior[node]
                parent.add_child(self.__cur_behavior)
            else:
                parent = self.__parent_behavior[node]
                parent.add_child(self.__cur_behavior)

            for child in node.get_children():
                if not isinstance(child, ast_node.AST):
                    continue

                if isinstance(child, (ast_node.DoMember, ast_node.EmitDirective, ast_node.WaitDirective)):
                    self.__parent_behavior[child] = self.__cur_behavior

            if sub_node is None:
                for child in node.get_children():
                    if not isinstance(child, ast_node.AST):
                        continue

                    if isinstance(child, ast_node.DoMember):
                        self.visit_do_member(child)
                    elif isinstance(child, ast_node.NamedArgument):
                        named_arg = self.visit_named_argument(child)
                        if named_arg[0] == "duration":
                            if isinstance(named_arg[1], Physical):
                                self.__duration = named_arg[1].gen_physical_value()
                            else:
                                print(
                                    "[Error] 'duration' parameter must be 'Physical' type"
                                )
                                sys.exit(1)
                    elif isinstance(child, ast_node.BehaviorInvocation):
                        self.visit_behavior_invocation(child)
                    elif isinstance(child, ast_node.WaitDirective):
                        self.visit_wait_directive(child)
                    elif isinstance(child, ast_node.EmitDirective):
                        self.visit_emit_directive(child)
                    elif isinstance(child, ast_node.CallDirective):
                        self.visit_call_directive(child)
                    else:
                        raise NotImplementedError(f"no implentment AST node {child}")
            else:
                if isinstance(sub_node, ast_node.DoMember):
                    self.visit_do_member(sub_node)
                else:
                    raise NotImplementedError("no supported ast node")

            if re.match("\d", str(self.__duration)) and self.__duration != math.inf:
                self.father_ins.all_duration += int(self.__duration)

        def visit_wait_directive(self, node: ast_node.WaitDirective):
            behaviors = py_trees.composites.Sequence(
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL, name="wait"
            )
            subbehavior = py_trees.composites.Sequence(
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL, name="behavior"
            )

            if node.get_child_count() == 1 and isinstance(
                node.get_child(0), ast_node.EventCondition
            ):
                elapsed_condition = self.visit_event_condition(node.get_child(0))
                self.__duration = elapsed_condition.gen_physical_value()
                print(elapsed_condition, self.__duration)
                self.father_ins.all_duration += int(self.__duration)
                waitTriggerer = TimeOfWaitComparison(self.__duration)
                waitTriggerer = oneshot_with_check(
                    variable_name="wait_time", behaviour=waitTriggerer
                )
                subbehavior.add_child(waitTriggerer)
                behaviors.add_child(subbehavior)
                parent = self.__parent_behavior[node]
                parent.add_child(behaviors)
                return

            bool_condition = ""
            for child in node.get_children():
                if not isinstance(child, ast_node.AST):
                    continue
                if isinstance(child, ast_node.EventReference):
                    event_declaration_node, event_name = self.visit_event_reference(
                        child
                    )
                elif isinstance(child, ast_node.EventFieldDecl):
                    pass
                elif isinstance(child, ast_node.EventCondition):
                    # string bool_condition = 'x<y'
                    bool_condition = self.visit_event_condition(child)
                else:
                    raise NotImplementedError(f"no implentment AST node {child}")
            actor_ego, actor_npc, symbol = self.bool_result(bool_condition)
            arguments = self.visit_children(event_declaration_node)
            other_car = arguments[0][1]
            distance = float(arguments[1][1].gen_physical_value())
            ret = getattr(self.father_ins.event, event_name)(other_car, distance)
            ret = oneshot_with_check(variable_name="wait_condition", behaviour=ret)

            wait_triggerer = IfTriggerer(actor_ego, actor_npc, symbol)
            wait_triggerer = oneshot_with_check(
                variable_name="wait", behaviour=wait_triggerer
            )
            subbehavior.add_child(wait_triggerer)
            subbehavior.add_child(ret)
            behaviors.add_child(subbehavior)

            parent = self.__parent_behavior[node]
            parent.add_child(behaviors)

        def visit_emit_directive(self, node: ast_node.EmitDirective):
            behaviors = py_trees.composites.Sequence(
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL, name="emit"
            )
            function_name = node.event_name
            arguments = self.visit_children(node)
            actor = arguments[0][1]
            distance = float(arguments[1][1].gen_physical_value())
            ret = getattr(self.father_ins.event, function_name)(actor, distance)
            ret = oneshot_with_check(variable_name="emit_condition", behaviour=ret)
            behaviors.add_child(ret)

            parent = self.__parent_behavior[node]
            parent.add_child(behaviors)

        def visit_behavior_invocation(self, node: ast_node.BehaviorInvocation):
            actor = node.actor
            behavior_name = node.behavior_name

            behavior_invocation_name = None
            if actor != None:
                behavior_invocation_name = actor + "." + behavior_name
            else:
                behavior_invocation_name = behavior_invocation_name

            if (
                self.father_ins.scenario_declaration.get(behavior_invocation_name)
                is not None
            ):
                self.father_ins.visit_power = True
                scenario_declaration_node = copy.deepcopy(
                    node.get_scope().declaration_address
                )
                # scenario_declaration_node = self.father_ins.scenario_declaration.get(behavior_invocation_name)
                scenario_declaration_node_scope = scenario_declaration_node.get_scope()
                arguments = self.visit_children(node)
                # Stores the value of the argument before the invoked scenario was overwritten， 如a: time=None
                # keyword_args = {}
                if isinstance(arguments, List):
                    for arg in arguments:
                        if isinstance(arg, Tuple):
                            scope = scenario_declaration_node_scope.resolve(arg[0])
                            # keyword_args[arg[0]] = scope.value
                            scope.value = arg[1]
                elif isinstance(arguments, Tuple):
                    scope = scenario_declaration_node_scope.resolve(arguments[0])
                    # keyword_args[arguments[0]] = scope.value
                    scope.value = arguments[1]
                self.visit_scenario_declaration(scenario_declaration_node)
                # Restores the value of the argument before the called scene was overwritten
                # for (name,value) in keyword_args.items():
                #     scope = scenario_declaration_node_scope.resolve(name)
                #     scope.value = value
                del scenario_declaration_node
                return

            behavior = py_trees.composites.Parallel(
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
                name=behavior_invocation_name
                + " duration="
                + str(int(self.__duration)),
            )

            # Create node for timeout
            timeout = TimeOut(
                self.__duration, name="duration=" + str(int(self.__duration))
            )
            behavior.add_child(timeout)

            actor_drive = py_trees.composites.Sequence(
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
                name=behavior_invocation_name,
            )

            modifier_invocation_no_occur = True

            location_modifiers = []
            speed_modifiers = []

            children = node.get_children()
            for child in children:
                if isinstance(child, ast_node.NamedArgument):
                    named_arg = self.visit_named_argument(child)
                    if named_arg[0] == "duration" and isinstance(
                        named_arg[1], Physical
                    ):
                        self.__duration = named_arg[1].gen_physical_value()
                    elif named_arg[0] == "duration":
                        print("[Error] 'duration' parameter must be 'Physical' type")
                        # sys.exit(1)
                elif isinstance(child, ast_node.ModifierInvocation):
                    modifier_invocation_no_occur = False
                    modifier = self.visit_modifier_invocation(child)
                    modifier_name = modifier[0]
                    arguments = modifier[1]

                    if modifier_name == "speed":
                        modifier_ins = SpeedModifier(actor, modifier_name)
                        keyword_args = {}
                        if isinstance(arguments, list):
                            arguments = OSC2Helper.flat_list(arguments)

                            for arg in arguments:
                                if isinstance(arg, tuple):
                                    keyword_args[arg[0]] = arg[1]
                                elif isinstance(arg, Physical):
                                    keyword_args["speed"] = arg
                        elif isinstance(arguments, tuple):
                            keyword_args[arguments[0]] = arguments[1]
                        elif isinstance(arguments, Physical):
                            keyword_args["speed"] = arguments
                        else:
                            raise NotImplementedError(
                                f"no implentment argument of {modifier_name}"
                            )

                        modifier_ins.set_args(keyword_args)

                        speed_modifiers.append(modifier_ins)

                    elif modifier_name == "position":
                        modifier_ins = PositionModifier(actor, modifier_name)
                        keyword_args = {}

                        keyword_args = {}
                        if isinstance(arguments, list):
                            arguments = OSC2Helper.flat_list(arguments)
                            for arg in arguments:
                                if isinstance(arg, tuple):
                                    keyword_args[arg[0]] = arg[1]
                                elif isinstance(arg, Physical):
                                    keyword_args["distance"] = arg
                        elif isinstance(arguments, tuple):
                            keyword_args[arguments[0]] = arguments[1]
                        elif isinstance(arg, Physical):
                            keyword_args["distance"] = arguments
                        else:
                            raise NotImplementedError(
                                f"no implentment argument of {modifier_name}"
                            )

                        modifier_ins.set_args(keyword_args)

                        location_modifiers.append(modifier_ins)

                    elif modifier_name == "lane":
                        modifier_ins = LaneModifier(actor, modifier_name)

                        keyword_args = {}
                        if isinstance(arguments, List):
                            arguments = OSC2Helper.flat_list(arguments)
                            for arg in arguments:
                                if isinstance(arg, Tuple):
                                    keyword_args[arg[0]] = arg[1]
                                else:
                                    keyword_args["lane"] = str(arg)
                        elif isinstance(arguments, Tuple):
                            keyword_args[arguments[0]] = arguments[1]
                        else:
                            keyword_args["lane"] = str(arguments)
                        modifier_ins.set_args(keyword_args)

                        location_modifiers.append(modifier_ins)

                    elif modifier_name == "acceleration":
                        modifier_ins = AccelerationModifier(actor, modifier_name)

                        keyword_args = {}
                        if isinstance(arguments, List):
                            arguments = OSC2Helper.flat_list(arguments)
                            for arg in arguments:
                                if isinstance(arg, Tuple):
                                    keyword_args[arg[0]] = arg[1]
                                else:
                                    keyword_args["acceleration"] = arg
                        elif isinstance(arguments, Tuple):
                            keyword_args[arguments[0]] = arguments[1]
                        else:
                            keyword_args["acceleration"] = arguments
                        modifier_ins.set_args(keyword_args)
                        speed_modifiers.append(modifier_ins)

                    elif modifier_name == "keep_lane":
                        actor_object = CarlaDataProvider.get_actor_by_name(actor)
                        car_driving = WaypointFollower(actor_object)
                        actor_drive.add_child(car_driving)

                        behavior.add_child(actor_drive)
                        # self.__cur_behavior.add_child(behavior)
                        print("Target keep lane.")

                    elif modifier_name == "change_speed":
                        # change_speed([speed: ]<speed>)
                        modifier_ins = ChangeSpeedModifier(actor, modifier_name)

                        keyword_args = {}
                        if isinstance(arguments, Tuple):
                            keyword_args[arguments[0]] = arguments[1]
                        elif isinstance(arguments, Physical):
                            keyword_args["desired_speed"] = arguments
                        else:
                            return f"Needed 1 arguments, but given {len(arguments)}arguments."

                        modifier_ins.set_args(keyword_args)

                        speed_modifiers.append(modifier_ins)

                    elif modifier_name == "change_lane":
                        modifier_ins = ChangeLaneModifier(actor, modifier_name)

                        keyword_args = {}
                        count = 0
                        for _, _ in enumerate(arguments):
                            count += 1
                        if count == 1:
                            keyword_args["side"] = arguments
                        elif count == 2:
                            if isinstance(arguments[0], Tuple):
                                keyword_args[arguments[0][0]] = str(arguments[0][1])
                            else:
                                keyword_args["lane_changes"] = str(arguments[0])
                            if isinstance(arguments[1], Tuple):
                                keyword_args[arguments[1][0]] = arguments[1][1]
                            else:
                                keyword_args["side"] = arguments[1]
                        else:
                            return f"Needed 2 arguments, but given {len(arguments)}arguments."

                        modifier_ins.set_args(keyword_args)

                        location_modifiers.append(modifier_ins)
                    else:
                        raise NotImplementedError(
                            f"no implentment function: {modifier_name}"
                        )

            if modifier_invocation_no_occur:
                car_actor = CarlaDataProvider.get_actor_by_name(actor)
                car_driving = WaypointFollower(car_actor)
                actor_drive.add_child(car_driving)
                behavior.add_child(actor_drive)
                self.__cur_behavior.add_child(behavior)
                return

            # 检查是否是行人的walk动作
            is_pedestrian_walk = False
            LOG_INFO(f"[OSC2] Processing behavior: {behavior_name} for actor: {actor}")
            if behavior_name == "walk":
                LOG_INFO(f"[OSC2] Behavior is 'walk', checking if actor is pedestrian...")
                # 检查actor是否为行人
                actor_obj = _safe_get_actor(actor)
                LOG_INFO(f"[OSC2] Actor object: {actor_obj}, type: {type(actor_obj)}")
                if actor_obj and isinstance(actor_obj, carla.Walker):
                    is_pedestrian_walk = True
                    LOG_INFO(f"[OSC2] Detected pedestrian walk action for {actor}")
                else:
                    LOG_WARNING(f"[OSC2] Actor {actor} is not a Walker or not found!")
            
            # 根据动作类型选择处理方式
            if is_pedestrian_walk:
                # 行人walk动作使用专门的处理函数
                all_modifiers = speed_modifiers + location_modifiers
                process_pedestrian_walk_action(
                    self.father_ins.config,
                    all_modifiers,
                    self.__duration,
                    self.father_ins.all_duration,
                    actor_drive,
                    actor
                )
            else:
                # 车辆drive动作使用原有的处理流程
                process_location_modifier(
                    self.father_ins.config, location_modifiers, self.__duration, actor_drive
                )
                process_speed_modifier(
                    self.father_ins.config,
                    speed_modifiers,
                    self.__duration,
                    self.father_ins.all_duration,
                    actor_drive,
                )

            behavior.add_child(actor_drive)
            self.__cur_behavior.add_child(behavior)

        def visit_modifier_invocation(self, node: ast_node.ModifierInvocation):
            # actor = node.actor
            modifier_name = node.modifier_name
            LOG_INFO(f"modifier invocation name {node.modifier_name}")
            arguments = self.visit_children(node)
            line, column = node.get_loc()
            # retrieval_name = modifier_name + para_type_str_sequence(config=self.father_ins.config,
            # arguments=arguments, line=line, column=column, node=node)
            retrieval_name = modifier_name
            method_scope = node.get_scope().resolve(retrieval_name)
            if (
                method_scope is None
                and modifier_name not in dir(self.father_ins.config.path)
                and modifier_name
                not in (
                    "speed",
                    "lane",
                    "position",
                    "acceleration",
                    "keep_lane",
                    "change_speed",
                    "change_lane",
                )
            ):
                line, column = node.get_loc()
                LOG_ERROR(
                    "Not Find " + modifier_name + " Method Declaration",
                    token=None,
                    line=line,
                    column=column,
                )
            if isinstance(method_scope, MethodSymbol):
                method_declaration_node = copy.deepcopy(
                    method_scope.declaration_address
                )
                method_scope = method_declaration_node.get_scope()
                if isinstance(arguments, List):
                    for arg in arguments:
                        if isinstance(arg, Tuple):
                            scope = method_scope.resolve(arg[0])
                            scope.value = arg[1]
                elif isinstance(arguments, Tuple):
                    scope = method_scope.resolve(arguments[0])
                    scope.value = arguments[1]
                method_value = None
                for child in method_declaration_node.get_children():
                    if isinstance(child, ast_node.MethodBody):
                        method_value = self.visit_method_body(child)
                del method_declaration_node
                if method_value is not None:
                    return method_value
                return
            else:
                pass
            arguments = self.visit_children(node)
            return modifier_name, arguments

        def visit_event_reference(self, node: ast_node.EventReference):
            return (
                copy.deepcopy(
                    node.get_scope().resolve(node.event_path).declaration_address
                ),
                node.event_path,
            )
            # return node.event_path

        def visit_event_field_declaration(self, node: ast_node.EventFieldDecl):
            return super().visit_event_field_declaration(node)

        def visit_event_condition(self, node: ast_node.EventCondition):
            expression = ""
            for child in node.get_children():
                if isinstance(child, ast_node.RelationExpression):
                    flat_arguments = self.visit_relation_expression(child)
                    temp_stack = []
                    for ex in flat_arguments:
                        if ex in RelationalOperator.values():
                            right = temp_stack.pop()
                            left = temp_stack.pop()
                            expression = left + ex + str(right)
                            temp_stack.append(expression)
                        elif ex == "in":
                            right = temp_stack.pop()
                            left = temp_stack.pop()
                            innum = temp_stack.pop()
                            expression = (
                                innum + " " + ex + " [" + left + ", " + right + "]"
                            )
                            temp_stack.append(expression)
                        else:
                            temp_stack.append(ex)
                    expression = temp_stack.pop()
                elif isinstance(child, ast_node.LogicalExpression):
                    expression = self.visit_logical_expression(child)
                elif isinstance(child, ast_node.ElapsedExpression):
                    expression = self.visit_elapsed_expression(child)
                else:
                    pass
            return expression

        def visit_relation_expression(self, node: ast_node.RelationExpression):
            arguments = [self.visit_children(node), node.operator]
            flat_arguments = OSC2Helper.flat_list(arguments)
            return flat_arguments

        def visit_logical_expression(self, node: ast_node.LogicalExpression):
            arguments = [self.visit_children(node), node.operator]
            flat_arguments = OSC2Helper.flat_list(arguments)
            temp_stack = []
            for ex in flat_arguments:
                if ex in ('and', 'or', '=>'):
                    expression = ""
                    length = len(temp_stack) - 1
                    for num in temp_stack:
                        if length > 0:
                            expression = expression + num + " " + ex + " "
                            length = length - 1
                        else:
                            expression = expression + num
                    temp_stack.clear()
                    temp_stack.append(expression)
                elif ex == "not":
                    num = temp_stack.pop()
                    expression = ex + " " + num
                    temp_stack.append(expression)
                else:
                    temp_stack.append(ex)
            logical_expression = temp_stack.pop()
            # return [self.visit_children(node), node.operator]
            return logical_expression

        def visit_elapsed_expression(self, node: ast_node.ElapsedExpression):
            child = node.get_child(0)
            if isinstance(child, ast_node.PhysicalLiteral):
                return self.visit_physical_literal(child)
            elif isinstance(child, ast_node.RangeExpression):
                return self.visit_range_expression(child)
            else:
                return None

        def visit_binary_expression(self, node: ast_node.BinaryExpression):
            arguments = [self.visit_children(node), node.operator]
            flat_arguments = OSC2Helper.flat_list(arguments)
            LOG_INFO(f"{flat_arguments}")
            temp_stack = []
            for ex in flat_arguments:
                if ex in ('+', '-', '*', '/', '%'):
                    right = temp_stack.pop()
                    left = temp_stack.pop()
                    # expression = left + ' ' + ex + ' ' + right
                    if ex == "+":
                        expression = left + right
                    elif ex == "-":
                        expression = left - right
                    elif ex == "*":
                        expression = left * right
                    elif ex == "/":
                        expression = left / right
                    elif ex == "%":
                        expression = left % right
                    else:
                        LOG_INFO(f"undefined Binary Expression operator: {ex}")
                    temp_stack.append(expression)
                else:
                    temp_stack.append(ex)
            binary_expression = temp_stack.pop()
            LOG_INFO(f"Relation Expression Value: {binary_expression}")
            # return [self.visit_children(node), node.operator]
            return binary_expression

        def visit_named_argument(self, node: ast_node.NamedArgument):
            return node.argument_name, self.visit_children(node)

        def visit_positional_argument(self, node: ast_node.PositionalArgument):
            return self.visit_children(node)

        def visit_range_expression(self, node: ast_node.RangeExpression):
            start, end = self.visit_children(node)
            if type(start) != type(end):
                print("[Error] different types between start and end of the range")
                sys.exit(1)

            start_num = None
            end_num = None
            start_unit = None
            end_unit = None
            unit_name = None

            if isinstance(start, Physical):
                start_num = start.num
                end_num = end.num

                start_unit = start.unit
                end_unit = end.unit
            else:
                start_num = start
                end_num = end

            if start_unit is not None and end_unit is not None:
                if start_unit == end_unit:
                    unit_name = start_unit
                else:
                    print("[Error] wrong unit in the range")
                    sys.exit(1)

            if start_num >= end_num:
                print("[Error] wrong start and end in the range")
                sys.exit(1)

            var_range = Range(start_num, end_num)

            if unit_name:
                return Physical(var_range, unit_name)
            else:
                return var_range

        def visit_physical_literal(self, node: ast_node.PhysicalLiteral):
            return Physical(
                self.visit_children(node),
                self.father_ins.config.unit_dict[node.unit_name],
            )

        def visit_integer_literal(self, node: ast_node.IntegerLiteral):
            return int(node.value)

        def visit_float_literal(self, node: ast_node.FloatLiteral):
            return float(node.value)

        def visit_bool_literal(self, node: ast_node.BoolLiteral):
            return node.value == "true"

        def visit_string_literal(self, node: ast_node.StringLiteral):
            return node.value

        def visit_identifier(self, node: ast_node.Identifier):
            return node.name

        def visit_identifier_reference(self, node: ast_node.IdentifierReference):
            para_name = node.name
            para_type = None
            para_value = None
            if node.get_scope() is not None:
                if not hasattr(node.get_scope(), "type"):
                    return para_name
                para_type = node.get_scope().type
                symbol = node.get_scope()
                last_value = None
                cur_value = node.get_scope().value
                while last_value != cur_value and symbol.resolve(cur_value) is not None:
                    symbol = symbol.resolve(cur_value)
                    last_value = cur_value
                    cur_value = symbol.value
                if cur_value is None:
                    return symbol.name
                else:
                    para_value = cur_value
            if para_value is not None:
                if isinstance(para_value, (Physical, float, int)):
                    return para_value
                para_value = para_value.strip('"')
                if re.fullmatch("(^[-]?[0-9]+(\.[0-9]+)?)\s*(\w+)", para_value):
                    # Regular expression ^[-]?[0-9]+(\.[0-9]+)? matching float
                    # para_value_num = re.findall('^[-]?[0-9]+(\.[0-9]+)?', para_value)[0]
                    patter = re.compile("(^[-]?[0-9]+[\.[0-9]+]?)\s*(\w+)")
                    para_value_num, para_value_unit = patter.match(para_value).groups()
                    if para_value_num.count(".") == 1:
                        return Physical(
                            float(para_value_num),
                            self.father_ins.config.unit_dict[para_value_unit],
                        )
                    else:
                        return Physical(
                            int(para_value_num),
                            self.father_ins.config.unit_dict[para_value_unit],
                        )
                elif para_type == "int":
                    return int(para_value)
                elif para_type == "uint":
                    return int(para_value)
                elif para_type == "float":
                    return float(para_value)
                elif para_type == "bool":
                    return para_value == "true"
                elif para_type == "string":
                    return para_value
                else:
                    return para_value
            else:
                return para_name

        def visit_parameter_declaration(self, node: ast_node.ParameterDeclaration):
            para_name = node.field_name
            para_type = node.field_type
            para_value = None
            for child in node.get_children():
                if isinstance(child, ast_node.FunctionApplicationExpression):
                    para_value = self.visit_function_application_expression(child)
            if para_value is not None:
                node.get_scope().value = para_value

            # Save variables of type struct for later access
            if para_type in self.father_ins.struct_declaration:
                self.father_ins.struct_parameters.update(
                    {para_name[0]: self.father_ins.struct_declaration[para_type]}
                )

        def visit_method_declaration(self, node: ast_node.MethodDeclaration):
            pass

        def visit_argument(self, node: ast_node.Argument):
            return node.argument_name, self.visit_children(node)

        def visit_method_body(self, node: ast_node.MethodBody):
            type = node.type
            method_value = None
            if type == "external":
                external_list = []
                for child in node.get_children():
                    if isinstance(child, ast_node.PositionalArgument):
                        line, column = node.get_loc()
                        LOG_ERROR(
                            "not support external format.!",
                            token=None,
                            line=line,
                            column=column,
                        )
                    elif isinstance(child, ast_node.NamedArgument):
                        name, value = self.visit_named_argument(child)
                        external_list.append((name, value))

                exec_context = ""
                module_name = None
                for elem in external_list:
                    if "module" == elem[0]:
                        exec_context += "import " + str(elem[1]) + "\n"
                        module_name = str(elem[1])
                    elif "name" == elem[0]:
                        exec_context += "ret = "
                        if module_name is not None:
                            exec_context += module_name + "." + str(elem[1]) + "("
                        else:
                            exec_context += str(elem[1]) + "("
                    else:
                        exec_context += str(elem[1])
                exec_context += ")\n"

                try:
                    exec_data = {}
                    exec(exec_context, globals(), exec_data)
                    method_value = exec_data["ret"]
                except Exception:
                    line, column = node.get_loc()
                    LOG_ERROR(
                        "not support external format.!",
                        token=None,
                        line=line,
                        column=column,
                    )

                return method_value
            else:
                for child in node.get_children():
                    if isinstance(child, ast_node.BinaryExpression):
                        method_value = self.visit_binary_expression(child)
            if method_value is not None:
                return method_value
            return

        def visit_function_application_expression(
            self, node: ast_node.FunctionApplicationExpression
        ):
            LOG_INFO("visit function application expression!")
            LOG_INFO("func name:" + node.func_name)

            arguments = OSC2Helper.flat_list(self.visit_children(node))
            line, column = node.get_loc()
            # retrieval_name = para_type_str_sequence(config=self.father_ins.config, arguments=arguments, line=line, column=column, node=node)
            retrieval_name = arguments[0].split(".")[-1]
            method_scope = node.get_scope().resolve(retrieval_name)

            method_name = arguments[0]
            if method_scope is None:
                LOG_ERROR(
                    "Not Find " + method_name + " Method Declaration",
                    token=None,
                    line=line,
                    column=column,
                )
            para_value = None
            if isinstance(method_scope, MethodSymbol):
                method_declaration_node = copy.deepcopy(
                    method_scope.declaration_address
                )
                method_scope = method_declaration_node.get_scope()
                if isinstance(arguments, List):
                    for arg in arguments:
                        if isinstance(arg, Tuple):
                            scope = method_scope.resolve(arg[0])
                            scope.value = arg[1]
                elif isinstance(arguments, Tuple):
                    scope = method_scope.resolve(arguments[0])
                    scope.value = arguments[1]
                para_value = None
                for child in method_declaration_node.get_children():
                    if isinstance(child, ast_node.MethodBody):
                        para_value = self.visit_method_body(child)
                        break
                del method_declaration_node
                if para_value is not None:
                    return para_value
                return para_value
            else:
                pass

        def visit_keep_constraint_declaration(
            self, node: ast_node.KeepConstraintDeclaration
        ):
            arguments = self.visit_children(node)
            retrieval_name = arguments[0]

            # Struct parameter or actor parameter contains '.'
            if "." in retrieval_name:
                layered_names = retrieval_name.split(".")
                prefix = layered_names[0]
                suffix = layered_names[1:]
                if prefix == "it":
                    pass
                elif prefix in self.father_ins.struct_parameters:
                    # param_name is the name of the struct variable, and param_scope is a ParameterSymbol
                    param_scope = node.get_scope().resolve(prefix)
                    self._build_struct_tree(param_scope)
                    self._visit_struct_tree(param_scope, suffix, 0, arguments[1])
            param_scope = node.get_scope().resolve(retrieval_name)
            if param_scope is not None and isinstance(param_scope, ParameterSymbol):
                if arguments[2] == RelationalOperator.EQUALITY.value:
                    param_scope.value = arguments[1]
                elif arguments[2] == RelationalOperator.INEQUALITY.value:
                    pass
                elif arguments[2] == RelationalOperator.LESS_THAN.value:
                    pass
                elif arguments[2] == RelationalOperator.LESS_OR_EQUAL.value:
                    pass
                elif arguments[2] == RelationalOperator.GREATER_THAN.value:
                    pass
                elif arguments[2] == RelationalOperator.GREATER_OR_EQUAL.value:
                    pass
                elif arguments[2] == RelationalOperator.MEMBERSHIP.value:
                    pass

        # For variables of struct type, it is necessary to construct its struct variable tree in the symbol table
        # The struct variable tree is a subtree of the symbol tree
        def _build_struct_tree(self, param_symbol: ParameterSymbol):
            if param_symbol.value is None:
                param_symbol.value = copy.deepcopy(
                    self.father_ins.struct_parameters[param_symbol.name]
                ).get_scope()
            for key in param_symbol.value.symbols:
                child_symbol = param_symbol.value.symbols[key]
                if isinstance(child_symbol, ParameterSymbol):
                    # If the child parameter is of struct type, the current method is called recursively
                    if child_symbol.type in self.father_ins.struct_declaration:
                        self._build_struct_tree(child_symbol)

        # visit struct variable tree and assign value
        def _visit_struct_tree(
            self, root: ParameterSymbol, suffix: list, index: int, value
        ):
            if root.type not in self.father_ins.struct_declaration:
                root.value = value
                return
            if index >= len(suffix):
                return
            to_visit_param = suffix[index]
            child_symbols = root.value.symbols
            if to_visit_param not in child_symbols:
                return
            self._visit_struct_tree(
                child_symbols[to_visit_param], suffix, index + 1, value
            )

    def _create_behavior(self):
        """
        Basic behavior do nothing, i.e. Idle
        """
        behavior_builder = self.BehaviorInit(self)
        behavior_builder.visit(self.ast_tree)

        behavior_tree = behavior_builder.get_behavior_tree()
        self.set_behavior_tree(behavior_tree)

        # py_trees.display.render_dot_tree(behavior_tree)

        return self.behavior

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()

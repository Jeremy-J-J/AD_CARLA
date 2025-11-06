# ---------  以下新增  ---------
import math
import carla
import srunner.osc2_stdlib.misc_object as misc
import srunner.scenariomanager.carla_data_provider as carla_provider
from srunner.tools.osc2_helper import OSC2Helper


class Pedestrian:
    def __init__(self, gender: str) -> None:
        self.gender = gender
        # ===== 追加属性，保持与 Vehicle 对齐 =====
        self.model = "walker.pedestrian.0001"  # CARLA 默认蓝图通配符
        self.rolename = "scenario"
        self.position = misc.Position()
        self.transform = carla.Transform()
        self.speed = 0.0
        self.autopilot = False
        self.random_location = True
        self.color = None
        self.category = "pedestrian"
        self.args = dict()

    # -----------  与 Vehicle 保持一致的一套接口 -----------
    def set_arg(self, kw):
        self.args.update(kw)

    def get_arg(self, key):
        return self.args[key]

    def set_name(self, name: str) -> None:
        self.rolename = name

    def get_name(self) -> str:
        return self.rolename

    def set_model(self, model: str) -> None:
        self.model = model

    def set_position(self, pos: misc.Position) -> None:
        """把 misc.Position 转成 carla.Transform，供 ScenarioManager 使用"""
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
            # 如果 .osc 里用 lane 语法，需要 Path 对象反算 world 坐标，
            # 这里先留空，实际可在 OSC2Helper 里统一做
            print("[pedestrian] LanePosition -> Transform not implemented yet")
        else:
            print("[pedestrian] unsupported position type")

    def get_transform(self) -> carla.Transform:
        """ScenarioManager 在 spawn 时会调这里拿 transform"""
        if OSC2Helper.wait_for_ego and self.rolename == OSC2Helper.ego_name:
            actor = carla_provider.CarlaDataProvider.get_actor_by_name(self.rolename)
            return carla_provider.CarlaDataProvider.get_transform(actor)
        if self.args.get("init_transform"):
            return self.args["init_transform"]
        # 默认返回前面 set_position 计算好的值
        return self.transform

    # -----------  给 .osc 用的“行走”接口 -----------
    def walk_to_position(self, pos: misc.WorldPosition) -> None:
        """OpenSCENARIO 2 里 walk_to_position(...) 的 python 实现"""
        self.set_position(pos)

    def walk_to(self, pos: misc.WorldPosition) -> None:
        """OpenSCENARIO 2 里 walk_to(...) 的 python 实现"""
        self.set_position(pos)


# 你原来的子类保持不变
class Man(Pedestrian):
    def __init__(self) -> None:
        super().__init__("male")


class Women(Pedestrian):
    def __init__(self) -> None:
        super().__init__("female")
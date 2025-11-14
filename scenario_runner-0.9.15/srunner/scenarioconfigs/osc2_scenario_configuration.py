"""
Parse the OSC-2 scenario description file, configure parameters based on type
and keep-constraints, generate relevant type objects from the standard library,
and set parameters.
"""

import sys
from typing import List, Tuple

import carla

import srunner.osc2_stdlib.misc_object as misc
import srunner.osc2_stdlib.variables as variable
import srunner.osc2_stdlib.vehicle as vehicles
import srunner.osc2_stdlib.pedestrian as pedestrians  # 添加行人标准库
from srunner.osc2.ast_manager import ast_node
from srunner.osc2.ast_manager.ast_vistor import ASTVisitor
from srunner.osc2_dm.physical_object import PhysicalObject, UnitObject
from srunner.osc2_dm.physical_types import Physical, Range

# 标准库
from srunner.osc2_stdlib.path import Path

# pylint: disable=line-too-long
from srunner.scenarioconfigs.scenario_configuration import ScenarioConfiguration
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
# pylint: enable=line-too-long

# OSC-2 helper
from srunner.tools.osc2_helper import OSC2Helper

# 车辆类型定义
vehicle_type = ["Car", "Model3", "Mkz2017", "Carlacola", "Rubicon"]

# 行人类型定义 - 基于OSC 1.0行人实现模式
pedestrian_type = ["Pedestrian", "Man", "Woman", "Women", "Child", "Elder"]


def flat_list(list_of_lists):
    """Flatten nested lists produced by AST traversal."""
    if not list_of_lists:
        return list_of_lists
    if isinstance(list_of_lists[0], list):
        return flat_list(list_of_lists[0]) + flat_list(list_of_lists[1:])
    return list_of_lists[:1] + flat_list(list_of_lists[1:])


class OSC2ScenarioConfiguration(ScenarioConfiguration):
    """
    Configuration object produced from an OSC-2 (*.osc) file.
    Extends ScenarioRunner's original ScenarioConfiguration with
    OSC-2 specific parsing logic.
    """

    # ====================================================================== #
    #   Construction & high-level helpers                                    #
    # ====================================================================== #

    def __init__(self, filename: str, client):
        # 1) 先让父类把它期望的全部字段都初始化好
        super().__init__()
        # 2) 本类自己的成员
        self.name = self.filename = filename
        self.client = client

        # 路线（旧版 BasicScenario 依赖单数 “route” 字段）
        # 父类只提供 routes(复数)，因此这里补一个空列表占位
        self.route = []

        # OSC-2 解析相关缓存
        self.ast_tree = OSC2Helper.gen_osc2_ast(self.filename)
        self.path = Path
        self.all_actors: dict = {}
        self.variables: dict = {}
        self.unit_dict: dict = {}
        self.physical_dict: dict = {}

        # 保存 AST 中声明的 struct / scenario
        self.scenario_declaration: dict = {}
        self.struct_declaration: dict = {}

        # 调整缺省天气：白天
        self.weather.sun_azimuth_angle = 45
        self.weather.sun_altitude_angle = 70

        # 解析 *.osc，填充 actor/variable 等
        self._parse_osc2_configuration()

    # ====================================================================== #
    #   Public helpers used by other modules                                 #
    # ====================================================================== #

    def get_car_config(self, car_name: str):
        """
        获取actor配置（支持车辆和行人）
        
        Args:
            car_name: actor名称
            
        Returns:
            车辆或行人的配置对象
        """
        return self.all_actors[car_name]

    def add_ego_vehicles(self, vc):
        """
        添加ego车辆或行人
        
        Args:
            vc: 车辆或行人对象
        """
        self.ego_vehicles.append(vc)
        self.all_actors[vc.get_name()] = vc

    def add_other_actors(self, npc):
        """
        添加其他actor（车辆或行人）
        
        Args:
            npc: 车辆或行人对象
        """
        self.other_actors.append(npc)
        self.all_actors[npc.get_name()] = npc

    def store_variable(self, vary):
        variable.Variable.set_arg(vary)

    # ====================================================================== #
    #   AST 解析器实现                                                       #
    # ====================================================================== #

    class ConfigInit(ASTVisitor):
        """Walk the OSC-2 AST and fill the configuration object."""

        def __init__(self, config_instance) -> None:
            super().__init__()
            self.father_ins: "OSC2ScenarioConfiguration" = config_instance

        # ---------- declarations ------------------------------------------

        def visit_global_parameter_declaration(
            self, node: ast_node.GlobalParameterDeclaration
        ):
            para_name = node.field_name[0]
            arguments = self.visit_children(node)

            if isinstance(arguments, list) and len(arguments) == 2:
                # 形如  speed: km_per_h = 50
                _, para_value = arguments
                self.father_ins.variables[para_name] = para_value
            elif isinstance(arguments, str):
                para_type = arguments
                # 形如  my_car: Car 或 pedestrian1: Man
                if para_type in vehicle_type:
                    vehicle_class = getattr(vehicles, para_type)
                    v_ins = vehicle_class()
                    v_ins.set_name(para_name)
                    if para_name == "ego_vehicle":
                        self.father_ins.add_ego_vehicles(v_ins)
                    else:
                        self.father_ins.add_other_actors(v_ins)
                elif para_type in pedestrian_type:
                    # 处理行人类型 - 参考OSC 1.0行人实现
                    pedestrian_class = getattr(pedestrians, para_type)
                    ped_ins = pedestrian_class()
                    ped_ins.set_name(para_name)
                    # 行人通常作为NPC，不作为ego
                    if para_name == OSC2Helper.ego_name:
                        self.father_ins.add_ego_vehicles(ped_ins)
                    else:
                        self.father_ins.add_other_actors(ped_ins)
                self.father_ins.variables[para_name] = para_type

            self.father_ins.store_variable(self.father_ins.variables)

        def visit_struct_declaration(self, node: ast_node.StructDeclaration):
            self.father_ins.struct_declaration[node.struct_name] = node
            for child in node.get_children():
                if isinstance(child, ast_node.MethodDeclaration):
                    self.visit_method_declaration(child)

        def visit_scenario_declaration(self, node: ast_node.ScenarioDeclaration):
            scenario_name = node.qualified_behavior_name
            self.father_ins.scenario_declaration[scenario_name] = node
            # 移除了 scenario_name != "top" 的限制，以支持任意命名的场景
            # if scenario_name != "top":
            #     return

            for child in node.get_children():
                if isinstance(child, ast_node.ParameterDeclaration):
                    self.visit_parameter_declaration(child)
                elif isinstance(child, ast_node.ModifierInvocation):
                    self.visit_modifier_invocation(child)
                elif isinstance(child, ast_node.VariableDeclaration):
                    self.visit_variable_declaration(child)
                elif isinstance(child, ast_node.EventDeclaration):
                    pass  # 暂不处理
                elif isinstance(child, ast_node.DoDirective):
                    self.visit_do_directive(child)

        def visit_do_directive(self, node: ast_node.DoDirective):
            # 场景设置信息不会出现在行为描述部分
            pass

        # ---------- parameter / variable ----------------------------------

        def visit_parameter_declaration(self, node: ast_node.ParameterDeclaration):
            para_name = node.field_name[0]
            arguments = self.visit_children(node)

            if isinstance(arguments, list) and len(arguments) == 2:
                _, para_value = arguments
                # 若值引用了已有变量，进行替换
                if self.father_ins.variables.get(str(para_value)) is not None:
                    para_value = self.father_ins.variables[str(para_value)]
                self.father_ins.variables[para_name] = para_value
            elif isinstance(arguments, str):
                para_type = arguments
                if para_type in vehicle_type:
                    vehicle_class = getattr(vehicles, para_type)
                    v_ins = vehicle_class()
                    v_ins.set_name(para_name)
                    if para_name == OSC2Helper.ego_name:
                        self.father_ins.add_ego_vehicles(v_ins)
                    else:
                        self.father_ins.add_other_actors(v_ins)
                elif para_type in pedestrian_type:
                    # 处理行人类型 - 参考OSC 1.0行人实现
                    pedestrian_class = getattr(pedestrians, para_type)
                    ped_ins = pedestrian_class()
                    ped_ins.set_name(para_name)
                    if para_name == OSC2Helper.ego_name:
                        self.father_ins.add_ego_vehicles(ped_ins)
                    else:
                        self.father_ins.add_other_actors(ped_ins)
                self.father_ins.variables[para_name] = para_type

            self.father_ins.store_variable(self.father_ins.variables)

        def visit_variable_declaration(self, node: ast_node.VariableDeclaration):
            variable_name = node.field_name[0]
            arguments = self.visit_children(node)

            if isinstance(arguments, list) and len(arguments) == 2:
                _, variable_value = arguments
                if self.father_ins.variables.get(str(variable_value)) is not None:
                    variable_value = self.father_ins.variables[str(variable_value)]
                self.father_ins.variables[variable_name] = variable_value
            elif isinstance(arguments, str):
                self.father_ins.variables[variable_name] = ""

            self.father_ins.store_variable(self.father_ins.variables)

        # ---------- modifier / event --------------------------------------

        def visit_modifier_invocation(self, node: ast_node.ModifierInvocation):
            function_name = node.modifier_name
            actor_name = node.actor
            arguments = self.visit_children(node)

            # 路径函数（在 Path 标准库里）
            if hasattr(self.father_ins.path, function_name):
                path_function = getattr(self.father_ins.path, function_name)
                position_args, keyword_args = self._split_args(arguments)
                path_function(*position_args, **keyword_args)

            # 位置函数（车辆自身）
            if actor_name == "ego_vehicle" and hasattr(vehicles.Vehicle, function_name):
                position_args, keyword_args = self._split_physical_args(arguments)

                # 创建一个 WorldPosition 对象，把值写进去
                pos = misc.WorldPosition(0, 0, 0, 0, 0, 0)
                getattr(pos, "__init__")(*position_args, **keyword_args)

                getattr(self.father_ins.ego_vehicles[0], function_name)(pos)
                self.father_ins.ego_vehicles[0].random_location = False

        # ---------- helper: 将 AST argument 列表拆分为位置 / 关键字 ---------

        def _split_args(self, arguments):
            """把 arguments 拆成 (positional, keyword) 两组."""
            position_args: list = []
            keyword_args: dict = {}
            if isinstance(arguments, List):
                arguments = flat_list(arguments)
                for arg in arguments:
                    if isinstance(arg, Tuple):
                        key, val = arg
                        if self.father_ins.variables.get(key) is not None:
                            keyword_args[key] = self.father_ins.variables[key]
                        else:
                            keyword_args[key] = val
                    else:
                        if self.father_ins.variables.get(arg) is not None:
                            position_args.append(self.father_ins.variables[arg])
                        else:
                            position_args.append(arg)
            else:
                if self.father_ins.variables.get(arguments) is not None:
                    position_args.append(self.father_ins.variables[arguments])
                else:
                    position_args.append(arguments)
            return position_args, keyword_args

        def _split_physical_args(self, arguments):
            """与 _split_args 类似，但转换 Physical → 原始数值."""
            position_args: list = []
            keyword_args: dict = {}
            if isinstance(arguments, List):
                arguments = flat_list(arguments)
                for arg in arguments:
                    if isinstance(arg, Tuple):
                        key, val = arg
                        if isinstance(val, Physical):
                            keyword_args[key] = val.gen_physical_value()
                    else:
                        if isinstance(arg, Physical):
                            position_args.append(arg.gen_physical_value())
            else:
                if isinstance(arguments, Physical):
                    position_args.append(arguments.gen_physical_value())
            return position_args, keyword_args

        # ---------- other AST nodes ---------------------------------------

        def visit_named_argument(self, node: ast_node.NamedArgument):
            return node.argument_name, self.visit_children(node)

        def visit_range_expression(self, node: ast_node.RangeExpression):
            start, end = self.visit_children(node)
            if type(start) != type(end):
                print("[Error] different types between start and end of the range")
                sys.exit(1)

            if isinstance(start, Physical):
                start_num, end_num = start.num, end.num
                if start.unit != end.unit:
                    print("[Error] wrong unit in the range")
                    sys.exit(1)
                unit_name = start.unit
            else:
                start_num, end_num = start, end
                unit_name = None

            if start_num >= end_num:
                print("[Error] wrong start and end in the range")
                sys.exit(1)

            var_range = Range(start_num, end_num)
            return Physical(var_range, unit_name) if unit_name else var_range

        # Literal / identifier visitors
        def visit_physical_literal(self, node: ast_node.PhysicalLiteral):
            return Physical(
                self.visit_children(node), self.father_ins.unit_dict[node.unit_name]
            )

        def visit_integer_literal(self, node: ast_node.IntegerLiteral):
            return int(node.value)

        def visit_float_literal(self, node: ast_node.FloatLiteral):
            return float(node.value)

        def visit_bool_literal(self, node: ast_node.BoolLiteral):
            return node.value

        def visit_string_literal(self, node: ast_node.StringLiteral):
            return node.value

        def visit_identifier(self, node: ast_node.Identifier):
            return node.name

        def visit_identifier_reference(self, node: ast_node.IdentifierReference):
            return node.name

        def visit_type(self, node: ast_node.Type):
            return node.type_name

        def visit_physical_type_declaration(
            self, node: ast_node.PhysicalTypeDeclaration
        ):
            arguments = flat_list(self.visit_children(node))
            si_base_exponent = (
                {arguments[0]: arguments[1]}
                if isinstance(arguments, Tuple)
                else {elem[0]: elem[1] for elem in arguments}
            )
            self.father_ins.physical_dict[node.type_name] = PhysicalObject(
                node.type_name, si_base_exponent
            )

        def visit_unit_declaration(self, node: ast_node.UnitDeclaration):
            arguments = flat_list(self.visit_children(node))
            factor = 1.0
            offset = 0.0
            for elem in arguments:
                if elem[0] == "factor":
                    factor = elem[1]
                elif elem[0] == "offset":
                    offset = elem[1]
            self.father_ins.unit_dict[node.unit_name] = UnitObject(
                node.unit_name,
                self.father_ins.physical_dict[node.physical_name],
                factor,
                offset,
            )

        def visit_si_base_exponent(self, node: ast_node.SIBaseExponent):
            return node.unit_name, self.visit_children(node)

    # ====================================================================== #
    #   Private helpers                                                      #
    # ====================================================================== #

    def _parse_osc2_configuration(self):
        """Parse the given *.osc file and set / validate parameters."""
        conf_visitor = self.ConfigInit(self)
        conf_visitor.visit(self.ast_tree)
        self._set_carla_town()

    def _set_carla_town(self):
        """Ensure the requested CARLA map is loaded and DataProvider updated."""
        self.town = self.path.get_map()

        world = self.client.get_world()
        wmap = world.get_map() if world else None
        if world is None or (wmap and wmap.name.split("/")[-1] != self.town):
            self.client.load_world(self.town)
            world = self.client.get_world()

        # 通知 DataProvider
        CarlaDataProvider.set_world(world)
        if CarlaDataProvider.is_sync_mode():
            world.tick()
        else:
            world.wait_for_tick()
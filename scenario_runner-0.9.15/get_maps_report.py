import carla
import xml.etree.ElementTree as ET
import random

# ==============================================================================
# 【配置区域 - 修改这里来指定地图】
# ==============================================================================

CARLA_HOST = 'localhost'
CARLA_PORT = 2000
# 可以选择的地图
# Available maps:
#  - /Game/Carla/Maps/Town02_Opt
#  - /Game/Carla/Maps/Town04_Opt
#  - /Game/Carla/Maps/Town01
#  - /Game/Carla/Maps/Town04
#  - /Game/Carla/Maps/Town10HD
#  - /Game/Carla/Maps/Town03
#  - /Game/Carla/Maps/Town06
#  - /Game/Carla/Maps/Town03_Opt
#  - /Game/Carla/Maps/Town05_Opt
#  - /Game/Carla/Maps/Town02
#  - /Game/Carla/Maps/Town05
#  - /Game/Carla/Maps/Town10HD_Opt
#  - /Game/Carla/Maps/Town01_Opt
#  - /Game/Carla/Maps/Town07
#  - /Game/Carla/Maps/Town11/Town11
#  - /Game/Carla/Maps/Town12/Town12
#  - /Game/Carla/Maps/Town13/Town13
#  - /Game/Carla/Maps/Town15/Town15
TARGET_MAP = "Town10HD"  # 设为None则分析当前地图

# 车道详细检查参数
INSPECT_SAMPLE_POINTS = 50

# ==============================================================================
# 【核心分析函数】
# ==============================================================================

def get_available_maps(client):
    """获取服务器上所有可用地图列表"""
    try:
        maps = client.get_available_maps()
        return sorted([m.split('/')[-1] for m in maps])
    except Exception as e:
        print(f"无法获取地图列表: {e}")
        return []

def analyze_map_lanes(world, map_name):
    """分析地图车道信息，返回包含关键路段数据的字典"""
    opendrive = world.get_map().to_opendrive()
    root = ET.fromstring(opendrive)
    
    max_lanes_per_section = 0
    max_driving_lanes_per_section = 0
    max_section_info = None
    max_driving_section_info = None
    all_lane_ids = set()
    
    # 遍历所有道路
    for road in root.iter('road'):
        road_id = road.get('id')
        road_name = road.get('name', f'Road_{road_id}')
        road_length = float(road.get('length', 0))
        
        for lane_section in road.iter('laneSection'):
            section_s = float(lane_section.get('s', 0))
            
            # 统计所有车道
            all_lane_ids_in_section = []
            driving_lane_ids_in_section = []
            lane_details = []
            
            # 处理左右车道
            for side in ['left', 'right']:
                side_elem = lane_section.find(side)
                if side_elem is not None:
                    for lane in side_elem.iter('lane'):
                        lane_id = int(lane.get('id'))
                        lane_type = lane.get('type')
                        lane_width = float(lane.find('width').get('a')) if lane.find('width') is not None else 0
                        
                        all_lane_ids_in_section.append(lane_id)
                        all_lane_ids.add(lane_id)
                        
                        lane_details.append({
                            'id': lane_id,
                            'type': lane_type,
                            'width': lane_width
                        })
                        
                        if lane_type == 'driving':
                            driving_lane_ids_in_section.append(lane_id)
            
            # 更新统计最大值
            if len(all_lane_ids_in_section) > max_lanes_per_section:
                max_lanes_per_section = len(all_lane_ids_in_section)
                max_section_info = {
                    'road_id': int(road_id),
                    'road_name': road_name,
                    'section_s': section_s,
                    'road_length': road_length,
                    'lane_ids': sorted(all_lane_ids_in_section),
                    'driving_lane_ids': sorted(driving_lane_ids_in_section),
                    'lane_details': lane_details
                }
            
            if len(driving_lane_ids_in_section) > max_driving_lanes_per_section:
                max_driving_lanes_per_section = len(driving_lane_ids_in_section)
                max_driving_section_info = {
                    'road_id': int(road_id),
                    'road_name': road_name,
                    'section_s': section_s,
                    'road_length': road_length,
                    'driving_lane_ids': sorted(driving_lane_ids_in_section),
                    'all_lane_ids': sorted(all_lane_ids_in_section)
                }
    
    return {
        'map_name': map_name,
        'max_lane_id': max(abs(lid) for lid in all_lane_ids) if all_lane_ids else 0,
        'unique_lane_count': len(all_lane_ids),
        'all_lane_ids': sorted(all_lane_ids),
        'max_section_info': max_section_info,
        'max_driving_lanes_per_section': max_driving_lanes_per_section,
        'max_driving_section_info': max_driving_section_info
    }

def print_report(stats):
    """打印车道统计分析报告"""
    print("\n" + "="*70)
    print(f"CARLA 地图车道统计报告 - {stats['map_name']}")
    print("="*70)
    
    print(f"\n【全局统计】")
    print(f"  最大 |laneId| = {stats['max_lane_id']}")
    print(f"  唯一车道ID总数 = {stats['unique_lane_count']}")
    if stats['all_lane_ids']:
        print(f"  车道ID范围 = {min(stats['all_lane_ids'])} 到 {max(stats['all_lane_ids'])}")
    
    print(f"\n【最宽路段 - 所有车道类型】")
    if stats['max_section_info']:
        info = stats['max_section_info']
        print(f"  所在道路 = {info['road_name']} (ID: {info['road_id']})")
        print(f"  路段位置 = s={info['section_s']:.2f} m")
        print(f"  道路总长度 = {info['road_length']:.2f} m")
        print(f"  车道总数 = {len(info['lane_ids'])}")
        print(f"  车道ID列表 = {info['lane_ids']}")
        print(f"  行车道ID = {info['driving_lane_ids']}")
    
    print(f"\n【最宽路段 - 仅行车道】")
    if stats['max_driving_section_info']:
        info = stats['max_driving_section_info']
        print(f"  所在道路 = {info['road_name']} (ID: {info['road_id']})")
        print(f"  路段位置 = s={info['section_s']:.2f} m")
        print(f"  道路总长度 = {info['road_length']:.2f} m")
        print(f"  行车道数量 = {len(info['driving_lane_ids'])}")
        print(f"  行车道ID = {info['driving_lane_ids']}")
    
    print(f"\n【车道ID说明】")
    print(f"  • 正ID = 右车道（行驶方向）")
    print(f"  • 负ID = 左车道（反向行驶）")
    print(f"  • 0 = 中心参考线")
    print("="*70)

def inspect_critical_road(client, map_name, section_info):
    """详细检查关键路段上所有车道的信息"""
    if not section_info:
        return
    
    print("\n" + "="*70)
    print(f"关键路段车道详细信息 - Road ID {section_info['road_id']}")
    print("="*70)
    
    # 重新加载地图
    world = client.load_world(map_name)
    town_map = world.get_map()
    
    road_id = section_info['road_id']
    lane_ids = section_info['lane_ids']
    
    print(f"\n【路段概览】")
    print(f"  道路名称 = {section_info['road_name']}")
    print(f"  道路总长度 = {section_info['road_length']:.2f} m")
    print(f"  路段起始位置 = s={section_info['section_s']:.2f} m")
    print(f"  包含车道ID = {lane_ids}")
    
    # 在关键路段上采样并检查每个车道
    print(f"\n【车道详细信息】")
    print(f"{'车道ID':>6} {'类型':>10} {'宽度(m)':>8} {'采样点数':>8}")
    print("-"*40)
    
    for lane_detail in section_info['lane_details']:
        lane_id = lane_detail['id']
        lane_type = lane_detail['type']
        lane_width = lane_detail['width']
        
        # 在该车道上采样多个点
        sample_points = 0
        for i in range(INSPECT_SAMPLE_POINTS):
            s = section_info['section_s'] + (i * 0.5)
            try:
                wp = town_map.get_waypoint_xodr(road_id, lane_id, s)
                if wp:
                    sample_points += 1
            except:
                break
        
        print(f"{lane_id:>6} {lane_type:>10} {lane_width:>8.2f} {sample_points:>8}")

# ==============================================================================
# 【主执行流程】
# ==============================================================================

if __name__ == '__main__':
    # 连接CARLA
    print(f"正在连接到 {CARLA_HOST}:{CARLA_PORT}...")
    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(60.0)
    
    world = client.get_world()
    current_map = world.get_map().name
    print(f"✓ 连接成功")
    
    # 显示所有可用地图
    available_maps = get_available_maps(client)
    print(f"\n服务器上共有 {len(available_maps)} 个可用地图:")
    for i, map_name in enumerate(available_maps, 1):
        marker = " →" if map_name == current_map else ""
        print(f"  {i}. {map_name}{marker}")
    
    # 决定分析哪个地图
    map_to_analyze = TARGET_MAP if TARGET_MAP else current_map
    
    if TARGET_MAP and TARGET_MAP != current_map:
        print(f"\n正在加载地图: {map_to_analyze}...")
        try:
            world = client.load_world(map_to_analyze)
            print(f"✓ 地图加载完成")
        except Exception as e:
            print(f"✗ 加载地图失败: {e}")
            map_to_analyze = current_map
    
    print(f"\n{'='*60}")
    print(f"开始分析地图: {map_to_analyze}")
    print(f"{'='*60}\n")
    
    # 分析并输出报告
    stats = analyze_map_lanes(world, map_to_analyze)
    print_report(stats)
    
    # 详细检查关键路段
    if stats['max_section_info']:
        print("\n【准备详细检查关键路段...】")
        inspect_critical_road(client, map_to_analyze, stats['max_section_info'])
    else:
        print("\n未找到有效路段信息，无法详细检查")

    # 详细的地图路段信息
    town_map = world.get_map()
    # 一次性解析 OpenDRIVE 并建立 road_id → 总长度的映射
    opendrive = town_map.to_opendrive()
    root = ET.fromstring(opendrive)
    road_length_map = {int(r.get('id')): float(r.get('length', 0.0))
                        for r in root.iter('road')}

    print("=== 所有 spawn_point 的 road/lane/s/道路总长度 信息（按 road 升序） ===")
    spawn_info = []
    for idx, sp in enumerate(town_map.get_spawn_points()):
        wp = town_map.get_waypoint(sp.location, project_to_road=True,
                                lane_type=carla.LaneType.Any)
        spawn_info.append((wp.road_id, idx, wp.lane_id, wp.s,
                        sp.location.x, sp.location.y))

    # 按 road_id 升序排序后打印，并带上道路总长度
    for road_id, idx, lane_id, s, x, y in sorted(spawn_info, key=lambda x: x[0]):
        total_length = road_length_map.get(road_id, 0.0)
        print(f"road={road_id:3d}, lane={lane_id:3d}, "
            f"s={s:6.1f},  x={x:7.2f}, y={y:7.2f}, 道路总长={total_length:7.2f}m")

    
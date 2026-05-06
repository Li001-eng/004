import streamlit as st
import folium
from streamlit_folium import folium_static, st_folium
from folium import plugins
import random
import time
import math
import copy
from datetime import datetime, timedelta
import pandas as pd

# ==================== 页面配置 ====================
st.set_page_config(page_title="无人机地面站系统 - 左右绕行修复版", layout="wide")

# ==================== 固定坐标 ====================
SCHOOL_CENTER_GCJ = [118.7490, 32.2340]
DEFAULT_A_GCJ = [118.746956, 32.232945]
DEFAULT_B_GCJ = [118.751589, 32.235204]

# 高德地图瓦片地址
GAODE_SATELLITE_URL = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
GAODE_VECTOR_URL = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"

# ==================== 坐标系转换（无修改） ====================
def gcj02_to_wgs84(lng, lat):
    a = 6378245.0
    ee = 0.00669342162296594323
    if out_of_china(lng, lat):
        return lng, lat
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
    mglat = lat + dlat
    mglng = lng + dlng
    return lng * 2 - mglng, lat * 2 - mglat

def wgs84_to_gcj02(lng, lat):
    a = 6378245.0
    ee = 0.00669342162296594323
    if out_of_china(lng, lat):
        return lng, lat
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
    mglat = lat + dlat
    mglng = lng + dlng
    return mglng, mglat

def transform_lat(lng, lat):
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 * math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lat * math.pi) + 40.0 * math.sin(lat / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(lat / 12.0 * math.pi) + 320 * math.sin(lat * math.pi / 30.0)) * 2.0 / 3.0
    return ret

def transform_lng(lng, lat):
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 * math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lng * math.pi) + 40.0 * math.sin(lng / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(lng / 12.0 * math.pi) + 300.0 * math.sin(lng / 30.0 * math.pi)) * 2.0 / 3.0
    return ret

def out_of_china(lng, lat):
    return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)

# ==================== 几何辅助函数（新增左右判断核心逻辑） ====================
def distance(p1, p2):
    """计算两点间经纬度距离（单位：度）"""
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def distance_meter(p1, p2):
    """计算两点间实际距离（单位：米）"""
    return distance(p1, p2) * 111000

def point_to_segment_distance(p, a, b):
    """点到线段的最短距离"""
    ap = (p[0]-a[0], p[1]-a[1])
    ab = (b[0]-a[0], b[1]-a[1])
    t = (ap[0]*ab[0] + ap[1]*ab[1]) / (ab[0]*ab[0] + ab[1]*ab[1] + 1e-9)
    t = max(0.0, min(1.0, t))
    proj = (a[0] + t*ab[0], a[1] + t*ab[1])
    return distance(p, proj)

def segment_to_polygon_min_distance(p1, p2, polygon):
    """线段到多边形的最短距离（返回米）"""
    min_dist = float('inf')
    for pt in polygon:
        d = point_to_segment_distance(pt, p1, p2)
        if d < min_dist:
            min_dist = d
    for i in range(len(polygon)):
        p3 = polygon[i]
        p4 = polygon[(i+1)%len(polygon)]
        steps = 20
        for t in range(steps+1):
            pt = (p3[0] + (p4[0]-p3[0])*t/steps, p3[1] + (p4[1]-p3[1])*t/steps)
            d = point_to_segment_distance(pt, p1, p2)
            if d < min_dist:
                min_dist = d
    return min_dist * 111000

def is_point_in_polygon(p, polygon):
    """判断点是否在多边形内（射线法）"""
    x, y = p
    inside = False
    n = len(polygon)
    for i in range(n):
        j = (i + 1) % n
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)):
            x_intersect = (y - yi) * (xj - xi) / (yj - yi + 1e-9) + xi
            if x < x_intersect:
                inside = not inside
    return inside

# ==================== 新增：向左/向右绕行核心判断函数 ====================
def get_point_side(start, end, point):
    """
    判断点在起点-终点直线的左侧还是右侧
    :param start: 起点 (lng, lat)
    :param end: 终点 (lng, lat)
    :param point: 待判断点 (lng, lat)
    :return: 1=左侧(逆时针), -1=右侧(顺时针), 0=在直线上
    """
    # 向量AB（起点到终点）
    ab_x = end[0] - start[0]
    ab_y = end[1] - start[1]
    # 向量AP（起点到待判断点）
    ap_x = point[0] - start[0]
    ap_y = point[1] - start[1]
    # 叉乘计算
    cross = ab_x * ap_y - ab_y * ap_x
    if cross > 1e-8:
        return 1  # 左侧
    elif cross < -1e-8:
        return -1 # 右侧
    else:
        return 0  # 直线上

def is_point_allowed_direction(start, end, point, direction):
    """
    判断点是否符合指定的绕行方向约束
    :param direction: 'left'=向左绕行, 'right'=向右绕行, None=无约束
    """
    if direction is None:
        return True
    side = get_point_side(start, end, point)
    if direction == 'left':
        # 向左绕行：允许左侧+直线上的点
        return side >= 0
    elif direction == 'right':
        # 向右绕行：允许右侧+直线上的点
        return side <= 0
    return True

# ==================== 核心：飞越/绕行判断规则（无修改） ====================
SAFE_HEIGHT_REDUNDANCY = 2.0  # 安全高度冗余，超过障碍物高度2m即可飞越
def should_avoid_obstacle(obs, flight_height):
    """
    核心规则：
    飞行高度 > 障碍物高度 + 安全冗余 → 不绕行，直接飞越
    飞行高度 ≤ 障碍物高度 + 安全冗余 → 必须绕行
    """
    obstacle_height = obs.get('height', 20)
    return flight_height <= (obstacle_height + SAFE_HEIGHT_REDUNDANCY)

def is_path_safe(p1, p2, obstacles_gcj, safe_radius_m, flight_height):
    """判断路径是否安全，仅对需要绕行的障碍物做校验"""
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        if not coords or len(coords) < 3:
            continue
        # 仅对需要绕行的障碍物做距离校验
        if should_avoid_obstacle(obs, flight_height):
            min_dist = segment_to_polygon_min_distance(p1, p2, coords)
            if min_dist < safe_radius_m - 0.1:
                return False
    return True

def get_avoid_obstacles(obstacles_gcj, flight_height):
    """过滤出当前飞行高度下，需要绕行的障碍物"""
    return [obs for obs in obstacles_gcj if should_avoid_obstacle(obs, flight_height)]

# ==================== 升级：带方向约束的RRT*算法（支持向左/向右绕行） ====================
class RRTStarNode:
    def __init__(self, point):
        self.point = point  # (lng, lat)
        self.parent = None
        self.cost = 0.0  # 从起点到当前节点的代价

def rrt_star_planning(start, end, obstacles_gcj, safe_radius_m, flight_height,
                       max_iter=2500, step_size=0.0002, goal_sample_rate=0.2, search_radius=0.0006, direction=None):
    """
    带方向约束的RRT* 路径规划算法（支持向左/向右强制绕行）
    :param start: 起点 (lng, lat)
    :param end: 终点 (lng, lat)
    :param obstacles_gcj: 全部障碍物
    :param safe_radius_m: 安全半径（米）
    :param flight_height: 飞行高度
    :param max_iter: 最大迭代次数
    :param step_size: 单次扩展步长（经纬度，约22米）
    :param goal_sample_rate: 终点采样概率，加快收敛
    :param search_radius: 重选父节点的搜索半径
    :param direction: 绕行方向约束 None=无约束, 'left'=向左, 'right'=向右
    :return: 平滑后的路径
    """
    # 先过滤出需要绕行的障碍物
    avoid_obs = get_avoid_obstacles(obstacles_gcj, flight_height)
    # 如果没有需要绕行的障碍物，直接返回直线路径
    if not avoid_obs and is_path_safe(start, end, obstacles_gcj, safe_radius_m, flight_height):
        return [start, end]

    # 初始化树
    start_node = RRTStarNode(start)
    end_node = RRTStarNode(end)
    node_list = [start_node]
    best_goal_node = None
    min_goal_cost = float('inf')

    # 计算采样边界（根据方向约束收缩采样范围）
    if direction == 'left':
        # 向左绕行：采样范围向左侧扩展
        min_lng = min(start[0], end[0]) - 0.02
        max_lng = max(start[0], end[0]) + 0.005
        min_lat = min(start[1], end[1]) - 0.005
        max_lat = max(start[1], end[1]) + 0.02
    elif direction == 'right':
        # 向右绕行：采样范围向右侧扩展
        min_lng = min(start[0], end[0]) - 0.005
        max_lng = max(start[0], end[0]) + 0.02
        min_lat = min(start[1], end[1]) - 0.02
        max_lat = max(start[1], end[1]) + 0.005
    else:
        # 无约束：全范围采样
        min_lng = min(start[0], end[0]) - 0.015
        max_lng = max(start[0], end[0]) + 0.015
        min_lat = min(start[1], end[1]) - 0.015
        max_lat = max(start[1], end[1]) + 0.015

    for _ in range(max_iter):
        # 1. 采样随机点（带方向约束）
        if random.random() < goal_sample_rate:
            rnd_point = end
        else:
            # 循环采样，直到找到符合方向约束的点
            while True:
                rnd_point = (random.uniform(min_lng, max_lng), random.uniform(min_lat, max_lat))
                if is_point_allowed_direction(start, end, rnd_point, direction):
                    break

        # 2. 找到最近节点
        nearest_node = min(node_list, key=lambda node: distance(node.point, rnd_point))

        # 3. 扩展新节点
        theta = math.atan2(rnd_point[1] - nearest_node.point[1], rnd_point[0] - nearest_node.point[0])
        new_point = (
            nearest_node.point[0] + step_size * math.cos(theta),
            nearest_node.point[1] + step_size * math.sin(theta)
        )

        # 4. 校验：方向约束 + 路径安全
        if not is_point_allowed_direction(start, end, new_point, direction):
            continue
        if not is_path_safe(nearest_node.point, new_point, obstacles_gcj, safe_radius_m, flight_height):
            continue

        # 5. 创建新节点
        new_node = RRTStarNode(new_point)
        new_node.parent = nearest_node
        new_node.cost = nearest_node.cost + distance(nearest_node.point, new_point)

        # 6. 重选父节点：在搜索半径内找代价最小的父节点（符合方向约束）
        near_nodes = [node for node in node_list if distance(node.point, new_point) < search_radius]
        for near_node in near_nodes:
            if is_path_safe(near_node.point, new_point, obstacles_gcj, safe_radius_m, flight_height):
                temp_cost = near_node.cost + distance(near_node.point, new_point)
                if temp_cost < new_node.cost:
                    new_node.parent = near_node
                    new_node.cost = temp_cost

        # 7. 重布线：优化附近节点的父节点
        for near_node in near_nodes:
            if is_path_safe(new_node.point, near_node.point, obstacles_gcj, safe_radius_m, flight_height):
                temp_cost = new_node.cost + distance(new_node.point, near_node.point)
                if temp_cost < near_node.cost:
                    near_node.parent = new_node
                    near_node.cost = temp_cost

        # 8. 添加新节点到树
        node_list.append(new_node)

        # 9. 检查是否到达终点
        dist_to_goal = distance(new_point, end)
        if dist_to_goal < step_size * 1.5:
            if is_path_safe(new_point, end, obstacles_gcj, safe_radius_m, flight_height):
                total_cost = new_node.cost + dist_to_goal
                if total_cost < min_goal_cost:
                    min_goal_cost = total_cost
                    best_goal_node = RRTStarNode(end)
                    best_goal_node.parent = new_node
                    best_goal_node.cost = total_cost

    # 10. 回溯生成路径
    if best_goal_node is not None:
        path = []
        current_node = best_goal_node
        while current_node is not None:
            path.append(current_node.point)
            current_node = current_node.parent
        path.reverse()

        # 11. 路径简化：去除冗余拐点
        if len(path) > 2:
            simplified_path = [path[0]]
            current_idx = 0
            while current_idx < len(path) - 1:
                next_idx = len(path) - 1
                while next_idx > current_idx:
                    # 简化时也必须符合方向约束
                    mid_point = ((path[current_idx][0]+path[next_idx][0])/2, (path[current_idx][1]+path[next_idx][1])/2)
                    if is_point_allowed_direction(start, end, mid_point, direction) and \
                       is_path_safe(path[current_idx], path[next_idx], obstacles_gcj, safe_radius_m, flight_height):
                        break
                    next_idx -= 1
                simplified_path.append(path[next_idx])
                current_idx = next_idx
            return simplified_path
        return path

    # 迭代完成未找到符合方向的路径，兜底返回无约束最优路径
    if direction is not None:
        st.warning(f"⚠️ {direction}侧无可行路径，已自动切换为最优绕行路径")
        return rrt_star_planning(start, end, obstacles_gcj, safe_radius_m, flight_height, direction=None)
    # 最终兜底返回直线路径
    return [start, end]

# ==================== 原版A*算法（保留，无修改） ====================
def generate_candidate_points(obstacles_gcj, safe_radius_m, flight_height):
    points = []
    deg_per_meter = 1.0 / 111000.0
    offset_deg = safe_radius_m * deg_per_meter
    for obs in obstacles_gcj:
        if not should_avoid_obstacle(obs, flight_height):
            continue
        coords = obs.get('polygon', [])
        if not coords:
            continue
        # 添加原始顶点
        for pt in coords:
            points.append(tuple(pt))
        # 添加向外偏移的点（8个方向）
        for pt in coords:
            for dx, dy in [(offset_deg,0), (-offset_deg,0), (0,offset_deg), (0,-offset_deg),
                           (offset_deg,offset_deg), (offset_deg,-offset_deg), (-offset_deg,offset_deg), (-offset_deg,-offset_deg)]:
                points.append((pt[0]+dx, pt[1]+dy))
        # 添加边的中点偏移
        for i in range(len(coords)):
            p1 = coords[i]
            p2 = coords[(i+1)%len(coords)]
            mid = ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2)
            points.append(mid)
            dx = p2[0]-p1[0]
            dy = p2[1]-p1[1]
            length = math.hypot(dx, dy)
            if length > 0:
                perp_x = -dy / length
                perp_y = dx / length
                for sign in [-1, 1]:
                    off_x = mid[0] + perp_x * offset_deg * sign
                    off_y = mid[1] + perp_y * offset_deg * sign
                    points.append((off_x, off_y))
    return points

def a_star_escape(start, end, obstacles_gcj, safe_radius_m, flight_height):
    # 候选点
    points = [tuple(start), tuple(end)]
    points.extend(generate_candidate_points(obstacles_gcj, safe_radius_m, flight_height))
    # 去重
    unique = []
    for p in points:
        if not any(abs(p[0]-u[0])<1e-6 and abs(p[1]-u[1])<1e-6 for u in unique):
            unique.append(p)
    n = len(unique)
    # 构建图
    graph = {}
    for i in range(n):
        graph[i] = []
        for j in range(n):
            if i == j:
                continue
            if is_path_safe(unique[i], unique[j], obstacles_gcj, safe_radius_m, flight_height):
                graph[i].append((j, distance(unique[i], unique[j])))
    # 找索引
    try:
        start_idx = next(i for i,p in enumerate(unique) if abs(p[0]-start[0])<1e-6 and abs(p[1]-start[1])<1e-6)
        end_idx = next(i for i,p in enumerate(unique) if abs(p[0]-end[0])<1e-6 and abs(p[1]-end[1])<1e-6)
    except StopIteration:
        return [start, end]
    # A*
    import heapq
    open_set = [(0, start_idx)]
    came_from = {}
    g_score = {i: float('inf') for i in range(n)}
    g_score[start_idx] = 0
    f_score = {i: float('inf') for i in range(n)}
    f_score[start_idx] = distance(unique[start_idx], unique[end_idx])
    while open_set:
        current = heapq.heappop(open_set)[1]
        if current == end_idx:
            path = []
            while current in came_from:
                path.append(unique[current])
                current = came_from[current]
            path.append(unique[start_idx])
            path.reverse()
            if len(path) <= 2:
                return path
            simplified = [path[0]]
            for i in range(1, len(path)-1):
                prev = simplified[-1]
                curr = path[i]
                nxt = path[i+1]
                angle1 = math.atan2(curr[1]-prev[1], curr[0]-prev[0])
                angle2 = math.atan2(nxt[1]-curr[1], nxt[0]-curr[0])
                if abs(angle1 - angle2) > 0.01:
                    simplified.append(curr)
            simplified.append(path[-1])
            return simplified
        for neighbor, dist in graph.get(current, []):
            tentative = g_score[current] + dist
            if tentative < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                f_score[neighbor] = tentative + distance(unique[neighbor], unique[end_idx])
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    # 失败返回直线
    return [start, end]

# ==================== 修复：路径规划入口函数（完全实现4种策略分发） ====================
def create_avoidance_path(start, end, obstacles_gcj, flight_height, safe_radius_m, strategy):
    """
    统一路径规划入口，严格按选择的策略执行
    :param strategy: 策略名称
    """
    # 先判断直线路径是否安全
    if is_path_safe(start, end, obstacles_gcj, safe_radius_m, flight_height):
        return [start, end]
    
    # 按策略分发算法
    if strategy == "最佳航线 (A*)":
        return a_star_escape(start, end, obstacles_gcj, safe_radius_m, flight_height)
    elif strategy == "RRT* 最优绕行":
        return rrt_star_planning(start, end, obstacles_gcj, safe_radius_m, flight_height, direction=None)
    elif strategy == "向左绕行":
        return rrt_star_planning(start, end, obstacles_gcj, safe_radius_m, flight_height, direction='left')
    elif strategy == "向右绕行":
        return rrt_star_planning(start, end, obstacles_gcj, safe_radius_m, flight_height, direction='right')
    # 兜底用最优路径
    return rrt_star_planning(start, end, obstacles_gcj, safe_radius_m, flight_height, direction=None)

# ==================== 障碍物缓存功能（无修改） ====================
def save_obstacles_to_cache():
    if 'saved_obstacles' not in st.session_state:
        st.session_state.saved_obstacles = []
    st.session_state.saved_obstacles = copy.deepcopy(st.session_state.obstacles_gcj)
    st.success(f"已保存 {len(st.session_state.obstacles_gcj)} 个障碍物到缓存")

def load_obstacles_from_cache():
    if 'saved_obstacles' not in st.session_state or not st.session_state.saved_obstacles:
        st.warning("缓存中无障碍物，请先保存")
        return False
    st.session_state.obstacles_gcj = st.session_state.saved_obstacles
    st.success(f"已从缓存加载 {len(st.session_state.obstacles_gcj)} 个障碍物")
    return True

# ==================== 心跳包&飞行模拟器（无修改） ====================
class HeartbeatSimulator:
    def __init__(self, start_point_gcj):
        self.history = []
        self.current_pos = start_point_gcj.copy()
        self.path = [start_point_gcj.copy()]
        self.path_index = 0
        self.total_waypoints = 0
        self.simulating = False
        self.paused = False
        self.flight_altitude = 50
        self.speed_m_s = 10  # 飞行速度 m/s
        self.speed_coefficient = 50
        self.progress = 0.0
        self.total_distance_m = 0.0
        self.distance_traveled_m = 0.0
        self.remaining_distance_m = 0.0
        # 新增监控参数
        self.start_time = None
        self.elapsed_time = timedelta(0)
        self.battery_percent = 100.0
        self.battery_consumption_rate = 0.08  # 每秒消耗0.08%电量
        self.communication_status = {
            "GCS": "在线",
            "OBC": "在线",
            "FCU": "在线"
        }

    def set_path(self, path, altitude=50, speed_coefficient=50):
        """设置飞行路径，初始化所有参数"""
        self.path = path
        self.total_waypoints = len(path)
        self.path_index = 0
        self.current_pos = path[0].copy()
        self.flight_altitude = altitude
        self.speed_coefficient = speed_coefficient
        self.speed_m_s = round(speed_coefficient * 0.2, 1)  # 系数转m/s
        self.simulating = True
        self.paused = False
        self.progress = 0.0
        self.distance_traveled_m = 0.0
        # 计算总距离
        self.total_distance_m = 0.0
        for i in range(len(path)-1):
            self.total_distance_m += distance_meter(path[i], path[i+1])
        self.remaining_distance_m = self.total_distance_m
        # 初始化时间和电量
        self.start_time = datetime.now()
        self.elapsed_time = timedelta(0)
        self.battery_percent = 100.0
        self.history = []

    def pause_flight(self):
        self.paused = True

    def resume_flight(self):
        self.paused = False
        self.start_time = datetime.now() - self.elapsed_time

    def stop_flight(self):
        self.simulating = False
        self.paused = False

    def update_and_generate(self):
        """更新飞行状态，生成心跳数据包"""
        current_time = datetime.now()
        # 暂停状态不更新位置
        if self.simulating and not self.paused and self.path_index < len(self.path)-1:
            # 更新已用时间
            self.elapsed_time = current_time - self.start_time
            # 电量消耗
            self.battery_percent = max(0.0, self.battery_percent - self.battery_consumption_rate * 0.2)
            # 位置更新
            target = self.path[self.path_index+1]
            dx = target[0] - self.current_pos[0]
            dy = target[1] - self.current_pos[1]
            dist_to_target_deg = math.hypot(dx, dy)
            dist_to_target_m = dist_to_target_deg * 111000
            # 单步移动距离（根据速度计算）
            step_m = self.speed_m_s * 0.2
            step_deg = step_m / 111000

            if dist_to_target_m < step_m:
                # 到达当前航点，切换到下一个
                self.distance_traveled_m += dist_to_target_m
                self.current_pos = target.copy()
                self.path_index += 1
            else:
                # 向目标航点移动
                ratio = step_deg / dist_to_target_deg
                self.current_pos[0] += dx * ratio
                self.current_pos[1] += dy * ratio
                self.distance_traveled_m += step_m

            # 更新进度和剩余距离
            if self.total_distance_m > 0:
                self.progress = min(1.0, self.distance_traveled_m / self.total_distance_m)
            self.remaining_distance_m = max(0.0, self.total_distance_m - self.distance_traveled_m)

            # 到达终点，结束飞行
            if self.path_index >= len(self.path)-1:
                self.simulating = False
                self.progress = 1.0
                self.remaining_distance_m = 0.0
        else:
            if not self.simulating:
                self.progress = 1.0
                self.remaining_distance_m = 0.0

        # 计算预计到达时间
        eta_seconds = self.remaining_distance_m / self.speed_m_s if self.speed_m_s > 0 and self.simulating else 0
        eta_timedelta = timedelta(seconds=eta_seconds)

        # 生成心跳包
        altitude = self.flight_altitude + random.randint(-2,2) if self.simulating else self.flight_altitude
        speed_display = self.speed_m_s if self.simulating and not self.paused else 0
        hb_data = {
            "timestamp": current_time.strftime("%H:%M:%S"),
            "lng": self.current_pos[0],
            "lat": self.current_pos[1],
            "altitude": altitude,
            "battery": round(self.battery_percent, 1),
            "satellites": random.randint(10,16),
            "speed_m_s": speed_display,
            "progress": self.progress,
            "current_waypoint": self.path_index + 1,
            "total_waypoints": self.total_waypoints,
            "distance_traveled_m": round(self.distance_traveled_m, 1),
            "total_distance_m": round(self.total_distance_m, 1),
            "remaining_distance_m": round(self.remaining_distance_m, 1),
            "elapsed_time": str(self.elapsed_time).split('.')[0],  # 去掉微秒
            "eta": str(eta_timedelta).split('.')[0],
            "simulating": self.simulating,
            "paused": self.paused,
            "communication": self.communication_status
        }
        # 插入历史记录头部
        self.history.insert(0, hb_data)
        if len(self.history) > 100:
            self.history.pop()
        return hb_data

# ==================== 地图可视化（无修改） ====================
def add_safety_buffer(map_obj, obstacles_gcj, safe_radius_m, flight_height):
    """给需要绕行的障碍物添加安全缓冲区可视化"""
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        if not coords:
            continue
        if should_avoid_obstacle(obs, flight_height):
            # 给障碍物每个顶点添加安全半径圆
            for pt in coords:
                folium.Circle(
                    location=[pt[1], pt[0]],
                    radius=safe_radius_m,
                    color='orange',
                    fill=True,
                    fill_opacity=0.15,
                    popup=f"安全缓冲区 (半径 {safe_radius_m}m)"
                ).add_to(map_obj)
            # 给障碍物添加半透明安全轮廓
            folium.Polygon(
                [[c[1], c[0]] for c in coords],
                color="orange",
                weight=2,
                fill=False,
                dash_array='5,5',
                popup=f"障碍物轮廓"
            ).add_to(map_obj)

def create_planning_map(center_gcj, points_gcj, obstacles_gcj, flight_history, planned_path, map_type, straight_blocked, safe_radius_m, flight_height):
    """创建航线规划地图"""
    if map_type == "satellite":
        tiles = GAODE_SATELLITE_URL
        attr = "高德卫星地图"
    else:
        tiles = GAODE_VECTOR_URL
        attr = "高德矢量地图"

    m = folium.Map(
        location=[center_gcj[1], center_gcj[0]],
        zoom_start=17,
        tiles=tiles,
        attr=attr,
        zoom_control=True
    )

    # 绘制工具
    draw = plugins.Draw(
        export=True,
        position='topleft',
        draw_options={
            'polygon': {'allowIntersection': False, 'showArea': True, 'color': '#ff0000', 'fillColor': '#ff0000', 'fillOpacity': 0.4},
            'polyline': False, 'rectangle': False, 'circle': False, 'marker': False, 'circlemarker': False
        },
        edit_options={'edit': True, 'remove': True}
    )
    m.add_child(draw)

    # 添加安全缓冲区
    add_safety_buffer(m, obstacles_gcj, safe_radius_m, flight_height)

    # 绘制障碍物
    for i, obs in enumerate(obstacles_gcj):
        coords = obs.get('polygon', [])
        if coords and len(coords) >= 3:
            need_avoid = should_avoid_obstacle(obs, flight_height)
            popup_text = f"🚧 {obs.get('name', f'障碍物{i+1}')}\n高度: {obs.get('height', 20)}m\n状态: {'需绕行' if need_avoid else '可飞越'}"
            fill_color = "#ff0000" if need_avoid else "#888888"
            folium.Polygon(
                [[c[1], c[0]] for c in coords],
                color=fill_color,
                weight=3,
                fill=True,
                fill_color=fill_color,
                fill_opacity=0.4,
                popup=popup_text
            ).add_to(m)

    # 绘制起点、终点
    if points_gcj.get('A'):
        folium.Marker(
            [points_gcj['A'][1], points_gcj['A'][0]],
            popup="🟢 起点",
            icon=folium.Icon(color="green", icon="play", prefix="fa")
        ).add_to(m)
    if points_gcj.get('B'):
        folium.Marker(
            [points_gcj['B'][1], points_gcj['B'][0]],
            popup="🔴 终点",
            icon=folium.Icon(color="red", icon="stop", prefix="fa")
        ).add_to(m)

    # 绘制规划路径
    if planned_path and len(planned_path) > 1:
        path_locations = [[p[1], p[0]] for p in planned_path]
        folium.PolyLine(
            path_locations,
            color="green",
            weight=4,
            opacity=0.9,
            popup="✈️ 规划航线"
        ).add_to(m)
        # 绘制航点
        for i, point in enumerate(planned_path[1:-1]):
            folium.CircleMarker(
                [point[1], point[0]],
                radius=5,
                color="green",
                fill=True,
                fill_color="white",
                fill_opacity=0.9,
                popup=f"航点 {i+1}"
            ).add_to(m)

    # 绘制直线路径
    if points_gcj.get('A') and points_gcj.get('B'):
        if straight_blocked:
            folium.PolyLine(
                [[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]],
                color="gray",
                weight=2,
                opacity=0.4,
                dash_array='5, 5',
                popup="⚠️ 直线被阻挡"
            ).add_to(m)
        else:
            folium.PolyLine(
                [[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]],
                color="blue",
                weight=2,
                opacity=0.5,
                dash_array='5, 5',
                popup="直线航线（安全）"
            ).add_to(m)

    # 绘制历史飞行轨迹
    if flight_history and len(flight_history) > 1:
        trail = [[p[1], p[0]] for p in flight_history if len(p) >= 2]
        if len(trail) > 1:
            folium.PolyLine(
                trail,
                color="orange",
                weight=3,
                opacity=0.7,
                popup="历史飞行轨迹"
            ).add_to(m)

    return m

# ==================== 主程序（仅侧边栏策略提示优化，其他无修改） ====================
def main():
    st.title("🏫 无人机地面站系统 - 左右绕行修复版")
    st.markdown("---")

    # 初始化session_state
    if "points_gcj" not in st.session_state:
        st.session_state.points_gcj = {'A': DEFAULT_A_GCJ.copy(), 'B': DEFAULT_B_GCJ.copy()}
    if "obstacles_gcj" not in st.session_state:
        st.session_state.obstacles_gcj = []
    if "saved_obstacles" not in st.session_state:
        st.session_state.saved_obstacles = []
    if "heartbeat_sim" not in st.session_state:
        st.session_state.heartbeat_sim = HeartbeatSimulator(st.session_state.points_gcj['A'].copy())
    if "last_hb_time" not in st.session_state:
        st.session_state.last_hb_time = time.time()
    if "simulation_running" not in st.session_state:
        st.session_state.simulation_running = False
    if "flight_altitude" not in st.session_state:
        st.session_state.flight_altitude = 50
    if "flight_history" not in st.session_state:
        st.session_state.flight_history = []
    if "planned_path" not in st.session_state:
        st.session_state.planned_path = None
    if "pending_polygon" not in st.session_state:
        st.session_state.pending_polygon = None
    if "pending_height" not in st.session_state:
        st.session_state.pending_height = 20

    # 侧边栏配置
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"])
    map_type_choice = st.sidebar.radio("🗺️ 地图类型", ["卫星影像", "矢量街道"], index=0)
    map_type = "satellite" if map_type_choice == "卫星影像" else "vector"

    st.sidebar.markdown("---")
    st.sidebar.subheader("⚙️ 无人机参数")
    drone_speed_coefficient = st.sidebar.slider("飞行速度系数", min_value=10, max_value=100, value=50, step=5)
    safe_radius = st.sidebar.number_input("安全半径 (米)", min_value=1, max_value=50, value=10, step=1)
    flight_alt = st.sidebar.number_input("飞行高度 (米)", min_value=0, max_value=300, value=st.session_state.flight_altitude, step=5)
    st.session_state.flight_altitude = flight_alt

    st.sidebar.markdown("---")
    st.sidebar.subheader("🔄 绕行策略")
    strategy = st.sidebar.selectbox("选择避障方式", ["RRT* 最优绕行", "最佳航线 (A*)", "向左绕行", "向右绕行"], index=0)
    # 策略提示
    if strategy == "向左绕行":
        st.sidebar.info("✅ 已启用【向左绕行】：沿起点-终点直线左侧逆时针绕障")
    elif strategy == "向右绕行":
        st.sidebar.info("✅ 已启用【向右绕行】：沿起点-终点直线右侧顺时针绕障")

    st.sidebar.markdown("---")
    obs_count = len(st.session_state.obstacles_gcj)
    avoid_obs_count = len(get_avoid_obstacles(st.session_state.obstacles_gcj, st.session_state.flight_altitude))
    straight_safe = is_path_safe(
        st.session_state.points_gcj['A'],
        st.session_state.points_gcj['B'],
        st.session_state.obstacles_gcj,
        safe_radius,
        st.session_state.flight_altitude
    )
    st.sidebar.info(
        f"🏫 校园区域\n"
        f"🚧 总障碍物: {obs_count}\n"
        f"⚠️ 需绕行: {avoid_obs_count}\n"
        f"📌 直线: {'🚫 不安全' if not straight_safe else '✅ 安全'}"
    )

    if st.sidebar.button("🔄 刷新路径规划", use_container_width=True):
        with st.spinner(f"正在按【{strategy}】规划路径..."):
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'],
                st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj,
                st.session_state.flight_altitude,
                safe_radius,
                strategy
            )
        st.rerun()

    # ==================== 航线规划页面 ====================
    if page == "🗺️ 航线规划":
        st.header("🗺️ 航线规划 - 飞越/绕行双模式")
        if not straight_safe:
            st.warning(f"⚠️ 直线航线被需绕行的障碍物阻挡！已按【{strategy}】规划避障航线（绿色）")
        else:
            st.success(f"✅ 直线航线安全，可直接飞越所有障碍物")
        st.info("📝 操作说明：点击地图左上角📐图标 → 选择多边形 → 围绕建筑物绘制 → 双击完成 → 设置障碍物高度 → 点击「添加障碍物」")

        col1, col2 = st.columns([1, 1.5])
        with col1:
            st.subheader("🎮 控制面板")
            # 起点设置
            st.markdown("#### 🟢 起点 A")
            a_lat = st.number_input("纬度", value=st.session_state.points_gcj['A'][1], format="%.6f", key="a_lat")
            a_lng = st.number_input("经度", value=st.session_state.points_gcj['A'][0], format="%.6f", key="a_lng")
            if st.button("📍 设置 A 点", use_container_width=True):
                st.session_state.points_gcj['A'] = [a_lng, a_lat]
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj,
                    st.session_state.flight_altitude,
                    safe_radius,
                    strategy
                )
                st.rerun()

            # 终点设置
            st.markdown("#### 🔴 终点 B")
            b_lat = st.number_input("纬度", value=st.session_state.points_gcj['B'][1], format="%.6f", key="b_lat")
            b_lng = st.number_input("经度", value=st.session_state.points_gcj['B'][0], format="%.6f", key="b_lng")
            if st.button("📍 设置 B 点", use_container_width=True):
                st.session_state.points_gcj['B'] = [b_lng, b_lat]
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj,
                    st.session_state.flight_altitude,
                    safe_radius,
                    strategy
                )
                st.rerun()

            # 障碍物设置
            st.markdown("#### 🏗️ 障碍物设置")
            new_obs_name = st.text_input("障碍物名称", value=f"建筑物{len(st.session_state.obstacles_gcj)+1}")
            new_obs_height = st.number_input("障碍物高度 (米)", min_value=1, max_value=200, value=st.session_state.pending_height, step=5)
            st.session_state.pending_height = new_obs_height

            if st.button("➕ 添加障碍物", use_container_width=True):
                if st.session_state.pending_polygon and len(st.session_state.pending_polygon) >= 3:
                    st.session_state.obstacles_gcj.append({
                        "name": new_obs_name,
                        "polygon": st.session_state.pending_polygon,
                        "height": st.session_state.pending_height
                    })
                    st.success(f"已添加障碍物「{new_obs_name}」，高度{st.session_state.pending_height}m")
                    st.session_state.pending_polygon = None
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'],
                        st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj,
                        st.session_state.flight_altitude,
                        safe_radius,
                        strategy
                    )
                    st.rerun()
                else:
                    st.warning("请先在地图上绘制多边形障碍物")

            if st.button("🔄 重新规划路径", use_container_width=True):
                with st.spinner(f"正在按【{strategy}】规划路径..."):
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'],
                        st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj,
                        st.session_state.flight_altitude,
                        safe_radius,
                        strategy
                    )
                if st.session_state.planned_path:
                    st.success(f"已规划完成，共 {len(st.session_state.planned_path)} 个航点")
                st.rerun()

            # 飞行控制
            st.markdown("#### ✈️ 飞行控制")
            col_btn1, col_btn2 = st.columns(2)
            with col_btn1:
                if st.button("▶️ 开始飞行", use_container_width=True):
                    path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
                    st.session_state.heartbeat_sim.set_path(path, st.session_state.flight_altitude, drone_speed_coefficient)
                    st.session_state.simulation_running = True
                    st.session_state.flight_history = []
                    st.success("🚁 飞行任务已开始！请切换到「飞行监控」页面查看实时数据")
            with col_btn2:
                if st.button("⏹️ 停止飞行", use_container_width=True):
                    st.session_state.simulation_running = False
                    st.session_state.heartbeat_sim.stop_flight()
                    st.info("飞行任务已停止")

            # 航点信息
            st.markdown("### 📍 航点信息")
            st.write(f"🟢 起点A: ({st.session_state.points_gcj['A'][0]:.6f}, {st.session_state.points_gcj['A'][1]:.6f})")
            st.write(f"🔴 终点B: ({st.session_state.points_gcj['B'][0]:.6f}, {st.session_state.points_gcj['B'][1]:.6f})")
            straight_dist = distance_meter(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'])
            st.caption(f"📏 直线距离: {straight_dist:.0f} 米")
            if st.session_state.planned_path and len(st.session_state.planned_path) > 2:
                path_dist = 0
                for i in range(len(st.session_state.planned_path)-1):
                    path_dist += distance_meter(st.session_state.planned_path[i], st.session_state.planned_path[i+1])
                st.caption(f"🔄 避障路径: {path_dist:.0f} 米")
                st.caption(f"📍 总航点数: {len(st.session_state.planned_path)} 个")

        with col2:
            st.subheader("🗺️ 规划地图")
            flight_trail = [[hb['lng'], hb['lat']] for hb in st.session_state.heartbeat_sim.history[:20]]
            center = st.session_state.points_gcj['A'] or SCHOOL_CENTER_GCJ

            # 初始化路径
            if st.session_state.planned_path is None:
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj,
                    st.session_state.flight_altitude,
                    safe_radius,
                    strategy
                )

            # 渲染地图
            m = create_planning_map(
                center,
                st.session_state.points_gcj,
                st.session_state.obstacles_gcj,
                flight_trail,
                st.session_state.planned_path,
                map_type,
                not straight_safe,
                safe_radius,
                st.session_state.flight_altitude
            )
            output = st_folium(m, width=800, height=600, returned_objects=["last_active_drawing"])

            # 捕获绘制的多边形
            if output and output.get("last_active_drawing"):
                last = output["last_active_drawing"]
                if last and last.get("geometry") and last["geometry"].get("type") == "Polygon":
                    coords = last["geometry"].get("coordinates", [])
                    if coords and len(coords) > 0:
                        poly = [[p[0], p[1]] for p in coords[0]]
                        if len(poly) >= 3:
                            st.session_state.pending_polygon = poly
                            st.success("✅ 已捕获多边形障碍物，请设置名称和高度后点击添加")
            st.caption("📌 图例：🟢 绿色=规划航线 | 🔴 红色=需绕行障碍物 | 🔘 灰色=可飞越障碍物 | 🟠 橙色=安全缓冲区")

    # ==================== 飞行监控页面（无修改） ====================
    elif page == "📡 飞行监控":
        st.header("📡 飞行实时画面 - 任务执行监控")
        current_time = time.time()

        # 自动刷新心跳数据
        if st.session_state.simulation_running:
            if current_time - st.session_state.last_hb_time >= 0.2:
                new_hb = st.session_state.heartbeat_sim.update_and_generate()
                st.session_state.last_hb_time = current_time
                st.session_state.flight_history.append([new_hb['lng'], new_hb['lat']])
                if len(st.session_state.flight_history) > 200:
                    st.session_state.flight_history.pop(0)
                if not st.session_state.heartbeat_sim.simulating:
                    st.session_state.simulation_running = False
                st.rerun()
        else:
            st.session_state.last_hb_time = current_time

        # 任务控制栏
        st.markdown("### 任务控制")
        col_ctrl1, col_ctrl2, col_ctrl3, col_ctrl4 = st.columns(4)
        with col_ctrl1:
            if st.button("▶️ 开始任务", use_container_width=True, disabled=st.session_state.simulation_running):
                path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
                st.session_state.heartbeat_sim.set_path(path, st.session_state.flight_altitude, drone_speed_coefficient)
                st.session_state.simulation_running = True
                st.session_state.flight_history = []
                st.rerun()
        with col_ctrl2:
            if st.button("⏸️ 暂停", use_container_width=True, disabled=not st.session_state.simulation_running or st.session_state.heartbeat_sim.paused):
                st.session_state.heartbeat_sim.pause_flight()
                st.rerun()
        with col_ctrl3:
            if st.button("▶️ 继续", use_container_width=True, disabled=not st.session_state.heartbeat_sim.paused):
                st.session_state.heartbeat_sim.resume_flight()
                st.rerun()
        with col_ctrl4:
            if st.button("⏹️ 停止", use_container_width=True, disabled=not st.session_state.simulation_running and not st.session_state.heartbeat_sim.paused):
                st.session_state.simulation_running = False
                st.session_state.heartbeat_sim.stop_flight()
                st.rerun()

        # 核心状态监控栏
        st.markdown("---")
        if st.session_state.heartbeat_sim.history:
            latest = st.session_state.heartbeat_sim.history[0]
            st.markdown("### 核心飞行状态")
            col1, col2, col3, col4, col5, col6, col7 = st.columns(7)
            col1.metric("当前航点", f"{latest['current_waypoint']}/{latest['total_waypoints']}")
            col2.metric("飞行速度", f"{latest['speed_m_s']} m/s")
            col3.metric("已用时间", latest['elapsed_time'])
            col4.metric("剩余距离", f"{latest['remaining_distance_m']} m")
            col5.metric("预计到达", latest['eta'])
            col6.metric("电量模拟", f"{latest['battery']} %")
            col7.metric("任务进度", f"{round(latest['progress']*100, 1)} %")

            # 进度条
            st.progress(latest['progress'], text=f"✈️ 飞行任务进度: {round(latest['progress']*100, 1)}%")
            st.caption(f"📏 总航程: {latest['total_distance_m']} 米 | 已飞行: {latest['distance_traveled_m']} 米 | 卫星数: {latest['satellites']} 颗")

            # 实时地图+通信状态面板
            st.markdown("---")
            map_col, comm_col = st.columns([2, 1])
            with map_col:
                st.subheader("实时飞行地图")
                tiles = GAODE_SATELLITE_URL if map_type == "satellite" else GAODE_VECTOR_URL
                monitor_map = folium.Map(
                    location=[latest['lat'], latest['lng']],
                    zoom_start=17,
                    tiles=tiles,
                    attr="高德地图"
                )
                # 添加安全缓冲区和障碍物
                add_safety_buffer(monitor_map, st.session_state.obstacles_gcj, safe_radius, st.session_state.flight_altitude)
                for obs in st.session_state.obstacles_gcj:
                    coords = obs.get('polygon', [])
                    if coords and len(coords) >= 3:
                        need_avoid = should_avoid_obstacle(obs, st.session_state.flight_altitude)
                        fill_color = "#ff0000" if need_avoid else "#888888"
                        folium.Polygon(
                            [[c[1], c[0]] for c in coords],
                            color=fill_color,
                            weight=2,
                            fill=True,
                            fill_opacity=0.3,
                            popup=f"高度: {obs.get('height',20)}m"
                        ).add_to(monitor_map)
                # 绘制规划航线
                if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
                    folium.PolyLine(
                        [[p[1], p[0]] for p in st.session_state.planned_path],
                        color="green",
                        weight=4,
                        opacity=0.8,
                        popup="规划航线"
                    ).add_to(monitor_map)
                # 绘制历史轨迹
                trail = [[hb['lat'], hb['lng']] for hb in st.session_state.heartbeat_sim.history[:50] if hb.get('lat') and hb.get('lng')]
                if len(trail) > 1:
                    folium.PolyLine(
                        trail,
                        color="orange",
                        weight=3,
                        opacity=0.8,
                        popup="飞行轨迹"
                    ).add_to(monitor_map)
                # 绘制当前位置
                folium.Marker(
                    [latest['lat'], latest['lng']],
                    popup=f"📍 当前位置\n高度: {latest['altitude']}m\n速度: {latest['speed_m_s']}m/s",
                    icon=folium.Icon(color='red', icon='plane', prefix='fa')
                ).add_to(monitor_map)
                # 绘制起点终点
                folium.Marker(
                    [st.session_state.points_gcj['A'][1], st.session_state.points_gcj['A'][0]],
                    popup="🟢 起点",
                    icon=folium.Icon(color='green', icon='play', prefix='fa')
                ).add_to(monitor_map)
                folium.Marker(
                    [st.session_state.points_gcj['B'][1], st.session_state.points_gcj['B'][0]],
                    popup="🔴 终点",
                    icon=folium.Icon(color='red', icon='stop', prefix='fa')
                ).add_to(monitor_map)
                folium_static(monitor_map, width=800, height=500)

            with comm_col:
                st.subheader("通信链路拓扑与数据流")
                comm_status = latest['communication']
                # 通信状态卡片
                col_gcs, col_obc, col_fcu = st.columns(3)
                col_gcs.metric("GCS", comm_status['GCS'])
                col_obc.metric("OBC", comm_status['OBC'])
                col_fcu.metric("FCU", comm_status['FCU'])
                st.markdown("---")
                # 数据链路状态
                st.info("✅ 数据链路正常\n📶 信号强度: 96%\n🔄 数据包延迟: 12ms")
                st.markdown("---")
                # 无人机状态
                st.subheader("无人机实时状态")
                st.write(f"📍 纬度: {latest['lat']:.6f}")
                st.write(f"📍 经度: {latest['lng']:.6f}")
                st.write(f"📊 相对高度: {latest['altitude']} m")
                st.write(f"🔋 电池电压: {round(11.2 + latest['battery']*0.016, 1)} V")
                st.write(f"🛰️ 搜星数量: {latest['satellites']} 颗")

            # 数据图表
            st.markdown("---")
            st.subheader("📈 飞行数据趋势")
            chart_col1, chart_col2 = st.columns(2)
            with chart_col1:
                if len(st.session_state.heartbeat_sim.history) > 1:
                    alt_data = []
                    for idx, hb in enumerate(reversed(st.session_state.heartbeat_sim.history[:20])):
                        alt_data.append({"序号": idx, "高度(m)": hb["altitude"]})
                    alt_df = pd.DataFrame(alt_data)
                    st.line_chart(alt_df, x="序号", y="高度(m)", color="#1f77b4")
                    st.caption("📊 飞行高度变化趋势")
            with chart_col2:
                if len(st.session_state.heartbeat_sim.history) > 1:
                    speed_data = []
                    for idx, hb in enumerate(reversed(st.session_state.heartbeat_sim.history[:20])):
                        speed_data.append({"序号": idx, "速度(m/s)": hb["speed_m_s"]})
                    speed_df = pd.DataFrame(speed_data)
                    st.line_chart(speed_df, x="序号", y="速度(m/s)", color="#2ca02c")
                    st.caption("📊 飞行速度变化趋势")

            # 历史心跳记录
            st.markdown("---")
            st.subheader("📋 历史心跳记录")
            st.dataframe(pd.DataFrame(st.session_state.heartbeat_sim.history[:10]), use_container_width=True)

        else:
            st.info("⏳ 暂无飞行数据，请在「航线规划」页面点击「开始飞行」启动任务")

    # ==================== 障碍物管理页面（无修改） ====================
    elif page == "🚧 障碍物管理":
        st.header("🚧 障碍物管理")
        st.info(f"当前共 **{len(st.session_state.obstacles_gcj)}** 个障碍物，其中需绕行 **{len(get_avoid_obstacles(st.session_state.obstacles_gcj, st.session_state.flight_altitude))}** 个")

        col1, col2 = st.columns([1, 1.5])
        with col1:
            if st.session_state.obst

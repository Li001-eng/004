import streamlit as st
import folium
from streamlit_folium import folium_static, st_folium
from folium import plugins
import random
import time
import math
import json
import os
from datetime import datetime
import pandas as pd
from typing import List, Dict, Optional, Tuple, Any
from dataclasses import dataclass, field

# ==================== 配置常量 ====================
@dataclass
class Config:
    SCHOOL_CENTER_GCJ: List[float] = field(default_factory=lambda: [118.7490, 32.2340])
    DEFAULT_A_GCJ: List[float] = field(default_factory=lambda: [118.746956, 32.232945])
    DEFAULT_B_GCJ: List[float] = field(default_factory=lambda: [118.751589, 32.235204])
    CONFIG_FILE: str = "obstacle_config.json"
    BACKUP_DIR: str = "backups"
    DEFAULT_SAFETY_RADIUS_METERS: int = 5
    HEARTBEAT_INTERVAL: float = 0.2
    BASE_SPEED_MPS: float = 5.0
    GAODE_SATELLITE_URL: str = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
    GAODE_VECTOR_URL: str = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"

config = Config()
os.makedirs(config.BACKUP_DIR, exist_ok=True)

# ==================== 几何函数 ====================
def point_in_polygon(point, polygon):
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i+1)%n]
        if ((y1 > y) != (y2 > y)) and (x < (x2-x1)*(y-y1)/(y2-y1)+x1):
            inside = not inside
    return inside

def on_segment(p, q, r):
    return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
            min(p[1], r[1]) <= q[1] <= max(p[1], r[1]))

def orientation(p, q, r):
    val = (q[1]-p[1])*(r[0]-q[0]) - (q[0]-p[0])*(r[1]-q[1])
    if abs(val) < 1e-10: return 0
    return 1 if val > 0 else 2

def segments_intersect(p1, p2, p3, p4):
    o1 = orientation(p1, p2, p3)
    o2 = orientation(p1, p2, p4)
    o3 = orientation(p3, p4, p1)
    o4 = orientation(p3, p4, p2)
    if o1 != o2 and o3 != o4: return True
    if o1 == 0 and on_segment(p1, p3, p2): return True
    if o2 == 0 and on_segment(p1, p4, p2): return True
    if o3 == 0 and on_segment(p3, p1, p4): return True
    if o4 == 0 and on_segment(p3, p2, p4): return True
    return False

def line_intersects_polygon(p1, p2, polygon):
    if point_in_polygon(p1, polygon) or point_in_polygon(p2, polygon):
        return True
    n = len(polygon)
    for i in range(n):
        p3 = polygon[i]
        p4 = polygon[(i+1)%n]
        if segments_intersect(p1, p2, p3, p4):
            return True
    return False

def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def meters_to_deg(meters, lat=32.23):
    lat_deg = meters / 111000
    lng_deg = meters / (111000 * math.cos(math.radians(lat)))
    return lng_deg, lat_deg

def point_to_segment_distance_deg(point, seg_start, seg_end):
    px, py = point
    x1, y1 = seg_start
    x2, y2 = seg_end
    dx = x2 - x1
    dy = y2 - y1
    len_sq = dx*dx + dy*dy
    if len_sq == 0:
        return math.sqrt((px-x1)**2 + (py-y1)**2)
    t = ((px-x1)*dx + (py-y1)*dy) / len_sq
    t = max(0, min(1, t))
    proj_x = x1 + t*dx
    proj_y = y1 + t*dy
    return math.sqrt((px-proj_x)**2 + (py-proj_y)**2)

def check_safety_radius(drone_pos, obstacles_gcj, flight_altitude, safety_radius):
    if not drone_pos:
        return True, None, None
    min_dist = float('inf')
    danger_name = None
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        obs_height = obs.get('height', 30)
        if obs_height <= flight_altitude:
            continue
        if coords and len(coords) >= 3:
            for i in range(len(coords)):
                p1 = coords[i]
                p2 = coords[(i+1)%len(coords)]
                dist_m = point_to_segment_distance_deg(drone_pos, p1, p2) * 111000
                if dist_m < min_dist:
                    min_dist = dist_m
                    danger_name = obs.get('name', '障碍物')
    if min_dist < safety_radius:
        return False, min_dist, danger_name
    return True, min_dist if min_dist!=float('inf') else None, None

# ==================== 绕行算法 ====================
def get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude):
    blocking = []
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(start, end, coords):
                blocking.append(obs)
    return blocking

def find_left_path(start, end, obstacles_gcj, flight_altitude, safety_radius=5):
    blocking = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    if not blocking:
        return [start, end]
    max_lng = -float('inf')
    max_lat = -float('inf')
    min_lat = float('inf')
    for obs in blocking:
        for pt in obs.get('polygon', []):
            max_lng = max(max_lng, pt[0])
            max_lat = max(max_lat, pt[1])
            min_lat = min(min_lat, pt[1])
    if max_lng == -float('inf'):
        return [start, end]
    safe_lng, safe_lat = meters_to_deg(safety_radius*3)
    obstacle_height = max_lat - min_lat
    point1 = [start[0] + 0.0012, max_lat + obstacle_height*3 + safe_lat*5 + 0.0002]
    point2 = [max_lng + obstacle_height*2 + safe_lng*3, point1[1]]
    return [start, point1, point2, end]

def find_right_path(start, end, obstacles_gcj, flight_altitude, safety_radius=5):
    blocking = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    if not blocking:
        return [start, end]
    mid_x = (start[0]+end[0])/2
    mid_y = (start[1]+end[1])/2
    dx = end[0]-start[0]
    dy = end[1]-start[1]
    length = math.hypot(dx, dy)
    if length == 0:
        return [start, end]
    perp_x = dy/length
    perp_y = -dx/length
    offset_dist = safety_radius * 10
    lat_rad = math.radians(mid_y)
    lng_scale = 111000 * math.cos(lat_rad)
    lat_scale = 111000
    offset_x = perp_x * offset_dist / lng_scale
    offset_y = perp_y * offset_dist / lat_scale
    waypoint = [mid_x + offset_x, mid_y + offset_y]
    return [start, waypoint, end]

def calculate_path_length(path):
    return sum(distance(path[i], path[i+1]) for i in range(len(path)-1))

def find_best_path(start, end, obstacles_gcj, flight_altitude, safety_radius=5):
    left = find_left_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    right = find_right_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    return left if calculate_path_length(left) < calculate_path_length(right) else right

def create_avoidance_path(start, end, obstacles_gcj, flight_altitude, direction, safety_radius=5):
    if direction == "向左绕行":
        return find_left_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    elif direction == "向右绕行":
        return find_right_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    else:
        return find_best_path(start, end, obstacles_gcj, flight_altitude, safety_radius)

# ==================== 心跳包模拟器 ====================
@dataclass
class HeartbeatData:
    timestamp: str
    flight_time: float
    lat: float
    lng: float
    altitude: float
    voltage: float
    satellites: int
    speed: float
    progress: float
    arrived: bool
    safety_violation: bool
    remaining_distance: float
    current_waypoint: int = 0
    total_waypoints: int = 0

class HeartbeatSimulator:
    def __init__(self, start_point_gcj):
        self.history = []
        self.current_pos = start_point_gcj.copy()
        self.path = [start_point_gcj.copy()]
        self.path_index = 0
        self.simulating = False
        self.flight_altitude = 50
        self.speed = 50
        self.progress = 0.0
        self.total_distance = 0.0
        self.distance_traveled = 0.0
        self.safety_radius = config.DEFAULT_SAFETY_RADIUS_METERS
        self.safety_violation = False
        self.start_time = None
        self.flight_log = []
        self.last_update_time = None

    def set_path(self, path, altitude, speed, safety_radius):
        self.path = path
        self.path_index = 0
        self.current_pos = path[0].copy()
        self.flight_altitude = altitude
        self.speed = speed
        self.safety_radius = safety_radius
        self.simulating = True
        self.progress = 0.0
        self.distance_traveled = 0.0
        self.safety_violation = False
        self.start_time = datetime.now()
        self.last_update_time = None
        self.total_distance = sum(distance(path[i], path[i+1]) for i in range(len(path)-1))

    def update_and_generate(self, obstacles_gcj):
        if not self.simulating or self.path_index >= len(self.path)-1:
            if self.simulating:
                self.simulating = False
            return None
        current_time = time.time()
        if self.last_update_time is None:
            delta_time = config.HEARTBEAT_INTERVAL
        else:
            delta_time = min(0.5, current_time - self.last_update_time)
        self.last_update_time = current_time
        start = self.path[self.path_index]
        end = self.path[self.path_index+1]
        segment_distance = distance(start, end)
        speed_mps = config.BASE_SPEED_MPS * (self.speed / 100)
        move_distance = speed_mps * delta_time
        self.distance_traveled += move_distance
        if self.total_distance > 0:
            completed = 0.0
            for i in range(self.path_index):
                completed += distance(self.path[i], self.path[i+1])
            if segment_distance > 0:
                segment_progress = min(1.0, self.distance_traveled / segment_distance)
                completed += segment_distance * segment_progress
            self.progress = min(1.0, completed / self.total_distance)
        if self.distance_traveled >= segment_distance and self.distance_traveled > 0:
            self.path_index += 1
            self.distance_traveled = 0
            if self.path_index < len(self.path):
                self.current_pos = self.path[self.path_index].copy()
            else:
                self.simulating = False
                return self._generate_heartbeat(True, obstacles_gcj)
        else:
            if segment_distance > 0:
                t = min(1.0, max(0.0, self.distance_traveled / segment_distance))
                lng = start[0] + (end[0]-start[0])*t
                lat = start[1] + (end[1]-start[1])*t
                self.current_pos = [lng, lat]
        safe, _, _ = check_safety_radius(self.current_pos, obstacles_gcj, self.flight_altitude, self.safety_radius)
        if not safe:
            self.safety_violation = True
        return self._generate_heartbeat(False, obstacles_gcj)

    def _generate_heartbeat(self, arrived, obstacles_gcj):
        flight_time = (datetime.now()-self.start_time).total_seconds() if self.start_time else 0
        speed = round(config.BASE_SPEED_MPS * (self.speed / 100), 1) if self.simulating else 0
        # 剩余距离计算（米）
        if arrived:
            remaining_dist = 0.0
        else:
            remaining = 0.0
            if self.path_index < len(self.path)-1:
                remaining += distance(self.current_pos, self.path[self.path_index+1])
                for i in range(self.path_index+1, len(self.path)-1):
                    remaining += distance(self.path[i], self.path[i+1])
            remaining_dist = remaining * 111000
        # 计算当前航点
        total_waypoints = len(self.path)
        if arrived:
            current_wp = total_waypoints
        else:
            if self.progress >= 1.0:
                current_wp = total_waypoints
            else:
                segment_index = int(self.progress * (total_waypoints - 1))
                current_wp = segment_index + 1
                current_wp = min(current_wp, total_waypoints)
        # 预计到达时间
        if arrived:
            eta_str = "00:00"
        elif speed > 0 and remaining_dist > 0:
            eta_sec = remaining_dist / speed
            if eta_sec < 60:
                eta_str = f"{eta_sec:.0f}秒"
            elif eta_sec < 3600:
                mins = int(eta_sec // 60)
                secs = int(eta_sec % 60)
                eta_str = f"{mins:02d}:{secs:02d}"
            else:
                hours = int(eta_sec // 3600)
                mins = int((eta_sec % 3600)//60)
                eta_str = f"{hours:02d}:{mins:02d}"
        else:
            eta_str = "计算中..."
        # 电量模拟
        max_flight_time = 1800
        battery = max(0, 100 - int((flight_time / max_flight_time)*100))
        # 电压随机波动
        voltage = round(22.2 + random.uniform(-0.5, 0.5), 1)
        heartbeat = HeartbeatData(
            timestamp=datetime.now().strftime("%H:%M:%S"),
            flight_time=flight_time,
            lat=self.current_pos[1],
            lng=self.current_pos[0],
            altitude=self.flight_altitude,
            voltage=voltage,
            satellites=random.randint(8,14),
            speed=speed,
            progress=self.progress,
            arrived=arrived,
            safety_violation=self.safety_violation,
            remaining_distance=remaining_dist,
            current_waypoint=current_wp,
            total_waypoints=total_waypoints
        )
        self.history.insert(0, heartbeat)
        if len(self.history) > 100:
            self.history.pop()
        self.flight_log.append(heartbeat)
        if len(self.flight_log) > 1000:
            self.flight_log.pop(0)
        return heartbeat

    def export_flight_data(self):
        if not self.flight_log:
            return pd.DataFrame()
        data = [{
            'timestamp': h.timestamp,
            'flight_time': h.flight_time,
            'lat': h.lat,
            'lng': h.lng,
            'altitude': h.altitude,
            'voltage': h.voltage,
            'satellites': h.satellites,
            'speed': h.speed,
            'progress': h.progress,
            'arrived': h.arrived,
            'safety_violation': h.safety_violation,
            'remaining_distance': h.remaining_distance,
            'current_waypoint': h.current_waypoint,
            'total_waypoints': h.total_waypoints
        } for h in self.flight_log]
        return pd.DataFrame(data)

# ==================== 障碍物管理 ====================
def load_obstacles():
    if os.path.exists(config.CONFIG_FILE):
        try:
            with open(config.CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                obstacles = data.get('obstacles', [])
                for obs in obstacles:
                    if 'height' not in obs: obs['height'] = 30
                    if 'selected' not in obs: obs['selected'] = False
                return obstacles
        except:
            return []
    return []

def save_obstacles(obstacles):
    data = {
        'obstacles': obstacles,
        'count': len(obstacles),
        'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        'version': 'v13.2'
    }
    try:
        with open(config.CONFIG_FILE, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        return True
    except:
        return False

def backup_config():
    if os.path.exists(config.CONFIG_FILE):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        backup_name = f"{config.BACKUP_DIR}/{config.CONFIG_FILE}.{timestamp}.bak"
        try:
            import shutil
            shutil.copy(config.CONFIG_FILE, backup_name)
            return backup_name
        except:
            pass
    return None

def restore_from_backup(backup_path):
    try:
        with open(backup_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
            obstacles = data.get('obstacles', [])
            save_obstacles(obstacles)
            return True
    except:
        return False

def get_latest_backup():
    try:
        files = [f for f in os.listdir(config.BACKUP_DIR) if f.startswith(config.CONFIG_FILE) and f.endswith('.bak')]
        if files:
            files.sort(reverse=True)
            return os.path.join(config.BACKUP_DIR, files[0])
    except:
        pass
    return None

# ==================== 地图创建 ====================
def create_planning_map(center_gcj, points_gcj, obstacles_gcj, flight_history, planned_path, map_type, straight_blocked, flight_altitude, drone_pos, direction, safety_radius):
    tiles = config.GAODE_SATELLITE_URL if map_type == "satellite" else config.GAODE_VECTOR_URL
    m = folium.Map(location=[center_gcj[1], center_gcj[0]], zoom_start=16, tiles=tiles, attr="高德地图")
    draw = plugins.Draw(
        export=True, position='topleft',
        draw_options={
            'polygon': {'allowIntersection': False, 'showArea': True, 'color': '#ff0000', 'fillColor': '#ff0000', 'fillOpacity': 0.4},
            'polyline': False, 'rectangle': False, 'circle': False, 'marker': False, 'circlemarker': False
        },
        edit_options={'edit': True, 'remove': True}
    )
    m.add_child(draw)
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        height = obs.get('height', 30)
        if coords and len(coords) >= 3:
            color = "red" if height > flight_altitude else "orange"
            folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=3, fill=True, fill_color=color, fill_opacity=0.4,
                           popup=f"🚧 {obs.get('name')}\n高度: {height}m").add_to(m)
    if points_gcj.get('A'):
        folium.Marker([points_gcj['A'][1], points_gcj['A'][0]], popup="🟢 起点", icon=folium.Icon(color="green", icon="play", prefix="fa")).add_to(m)
    if points_gcj.get('B'):
        folium.Marker([points_gcj['B'][1], points_gcj['B'][0]], popup="🔴 终点", icon=folium.Icon(color="red", icon="stop", prefix="fa")).add_to(m)
    if planned_path and len(planned_path) > 1:
        line_color = "purple" if "向左" in direction else ("orange" if "向右" in direction else "green")
        folium.PolyLine([[p[1], p[0]] for p in planned_path], color=line_color, weight=5, opacity=0.9, popup=f"✈️ {direction}").add_to(m)
        for i, point in enumerate(planned_path[1:-1]):
            folium.CircleMarker([point[1], point[0]], radius=4, color=line_color, fill=True, fill_color="white", fill_opacity=0.8, popup=f"航点 {i+1}").add_to(m)
    if points_gcj.get('A') and points_gcj.get('B'):
        if straight_blocked:
            folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]], color="gray", weight=2, dash_array='5,5', popup="⚠️ 直线被阻挡").add_to(m)
        else:
            folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]], color="blue", weight=2, dash_array='5,5', popup="直线航线").add_to(m)
    if drone_pos:
        folium.Circle(radius=safety_radius, location=[drone_pos[1], drone_pos[0]], color="blue", weight=2, fill=True, fill_color="blue", fill_opacity=0.2, popup=f"🛡️ 安全半径: {safety_radius}米").add_to(m)
    if flight_history and len(flight_history) > 1:
        trail = [[p[1], p[0]] for p in flight_history if len(p)==2]
        if len(trail) > 1:
            folium.PolyLine(trail, color="orange", weight=2, opacity=0.6, popup="历史轨迹").add_to(m)
    return m

def create_monitor_map(center_lat, center_lng, obstacles_gcj, planned_path, flight_history, current_pos, safety_radius, flight_altitude, direction, waypoints):
    tiles = config.GAODE_SATELLITE_URL
    m = folium.Map(location=[center_lat, center_lng], zoom_start=18, tiles=tiles, attr="高德地图")
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        height = obs.get('height', 30)
        if coords and len(coords) >= 3:
            color = "red" if height > flight_altitude else "orange"
            folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=2, fill=True, fill_opacity=0.3,
                           popup=f"🚧 {obs.get('name')}\n高度: {height}m").add_to(m)
    if planned_path and len(planned_path) > 1:
        line_color = "purple" if "向左" in direction else ("orange" if "向右" in direction else "green")
        folium.PolyLine([[p[1], p[0]] for p in planned_path], color=line_color, weight=3, opacity=0.7, popup="规划航线").add_to(m)
    if flight_history and len(flight_history) > 1:
        trail = [[h[1], h[0]] for h in flight_history[-50:] if len(h)==2]
        if len(trail) > 1:
            folium.PolyLine(trail, color="orange", weight=2, opacity=0.6, popup="历史轨迹").add_to(m)
    if current_pos:
        folium.Circle(radius=safety_radius, location=[current_pos[1], current_pos[0]], color="blue", weight=2, fill=True, fill_color="blue", fill_opacity=0.2, popup=f"🛡️ 安全半径: {safety_radius}米").add_to(m)
        folium.Marker([current_pos[1], current_pos[0]], popup=f"📍 当前位置", icon=folium.Icon(color='red', icon='plane', prefix='fa')).add_to(m)
    for i, wp in enumerate(waypoints):
        color = 'green' if i==0 else ('red' if i==len(waypoints)-1 else 'blue')
        folium.Marker([wp[1], wp[0]], popup=f"航点{i+1}", icon=folium.Icon(color=color)).add_to(m)
    return m

# ==================== 辅助UI函数 ====================
def init_session_state():
    defaults = {
        'points_gcj': {'A': config.DEFAULT_A_GCJ.copy(), 'B': config.DEFAULT_B_GCJ.copy()},
        'obstacles_gcj': load_obstacles(),
        'heartbeat_sim': HeartbeatSimulator(config.DEFAULT_A_GCJ.copy()),
        'last_hb_time': time.time(),
        'simulation_running': False,
        'flight_history': [],
        'planned_path': None,
        'last_flight_altitude': 50,
        'pending_obstacle': None,
        'current_direction': "最佳航线",
        'safety_radius': config.DEFAULT_SAFETY_RADIUS_METERS,
        'auto_backup': True,
        'show_rename_dialog': False,
        'waiting_for_start_point': False,
        'waiting_for_end_point': False,
        'temp_click_point': None
    }
    for k, v in defaults.items():
        if k not in st.session_state:
            st.session_state[k] = v

def render_sidebar():
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"])
    map_type_choice = st.sidebar.radio("🗺️ 地图类型", ["卫星影像", "矢量街道"], index=0)
    map_type = "satellite" if map_type_choice == "卫星影像" else "vector"
    st.sidebar.markdown("---")
    st.sidebar.subheader("⚡ 无人机速度设置")
    drone_speed = st.sidebar.slider("飞行速度系数", 10, 100, 50, 5)
    st.sidebar.markdown("---")
    st.sidebar.subheader("✈️ 无人机飞行高度")
    flight_alt = st.sidebar.slider("飞行高度 (m)", 10, 200, 50, 5)
    st.sidebar.markdown("---")
    st.sidebar.subheader("🛡️ 安全半径设置")
    safety_radius = st.sidebar.slider("安全半径 (米)", 1, 20, st.session_state.safety_radius, 1)
    st.sidebar.markdown("---")
    st.sidebar.subheader("💾 自动保存")
    auto_save = st.sidebar.checkbox("自动保存障碍物", value=st.session_state.auto_backup)
    return page, map_type, drone_speed, flight_alt, auto_save, safety_radius

# ==================== 航线规划页面 ====================
def render_planning_page(map_type, drone_speed, flight_alt, auto_save, safety_radius):
    st.header("🗺️ 航线规划 - 智能避障")
    # 检查直线是否被阻挡
    straight_blocked = any(
        obs.get('height',30) > flight_alt and line_intersects_polygon(
            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'], obs.get('polygon',[])
        ) for obs in st.session_state.obstacles_gcj
    )
    if straight_blocked:
        st.warning(f"⚠️ 存在高于飞行高度({flight_alt}m)的障碍物阻挡航线，已启用 {st.session_state.current_direction}")
    else:
        st.success("✅ 直线航线畅通无阻（所有障碍物高度 ≤ 飞行高度）")
    st.info("📝 点击地图左上角📐图标 → 选择多边形 → 围绕建筑物绘制 → 双击完成 → 输入高度并保存")
    col1, col2 = st.columns([1, 1.5])
    with col1:
        render_planning_controls(flight_alt, drone_speed, auto_save, safety_radius)
    with col2:
        render_planning_map_view(map_type, flight_alt, straight_blocked, safety_radius)

def render_planning_controls(flight_alt, drone_speed, auto_save, safety_radius):
    st.subheader("🎮 控制面板")
    with st.expander("📍 起点/终点设置", expanded=True):
        # 坐标输入方式
        st.markdown("#### 🟢 起点 A")
        col_a1, col_a2 = st.columns(2)
        with col_a1:
            a_lat = st.number_input("纬度", value=st.session_state.points_gcj['A'][1], format="%.6f", key="a_lat")
        with col_a2:
            a_lng = st.number_input("经度", value=st.session_state.points_gcj['A'][0], format="%.6f", key="a_lng")
        if st.button("📍 设置 A 点", use_container_width=True):
            st.session_state.points_gcj['A'] = [a_lng, a_lat]
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, safety_radius
            )
            st.success(f"✅ 起点已设置")
            st.rerun()
        st.markdown("#### 🔴 终点 B")
        col_b1, col_b2 = st.columns(2)
        with col_b1:
            b_lat = st.number_input("纬度", value=st.session_state.points_gcj['B'][1], format="%.6f", key="b_lat")
        with col_b2:
            b_lng = st.number_input("经度", value=st.session_state.points_gcj['B'][0], format="%.6f", key="b_lng")
        if st.button("📍 设置 B 点", use_container_width=True):
            st.session_state.points_gcj['B'] = [b_lng, b_lat]
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, safety_radius
            )
            st.success(f"✅ 终点已设置")
            st.rerun()
    with st.expander("🤖 路径规划策略", expanded=True):
        col_dir1, col_dir2, col_dir3 = st.columns(3)
        with col_dir1:
            if st.button("🔄 最佳航线", use_container_width=True, type="primary" if st.session_state.current_direction=="最佳航线" else "secondary"):
                st.session_state.current_direction = "最佳航线"
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj, flight_alt, "最佳航线", safety_radius
                )
                st.rerun()
        with col_dir2:
            if st.button("⬅️ 向左绕行", use_container_width=True, type="primary" if st.session_state.current_direction=="向左绕行" else "secondary"):
                st.session_state.current_direction = "向左绕行"
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj, flight_alt, "向左绕行", safety_radius
                )
                st.rerun()
        with col_dir3:
            if st.button("➡️ 向右绕行", use_container_width=True, type="primary" if st.session_state.current_direction=="向右绕行" else "secondary"):
                st.session_state.current_direction = "向右绕行"
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj, flight_alt, "向右绕行", safety_radius
                )
                st.rerun()
        if st.button("🔄 重新规划路径", use_container_width=True):
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, safety_radius
            )
            st.rerun()
    with st.expander("✈️ 飞行控制", expanded=True):
        col_met1, col_met2 = st.columns(2)
        with col_met1:
            st.metric("当前飞行高度", f"{flight_alt} m")
        with col_met2:
            st.metric("速度系数", f"{drone_speed}%")
        if st.button("▶️ 开始飞行", use_container_width=True, type="primary"):
            if st.session_state.points_gcj['A'] and st.session_state.points_gcj['B']:
                path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
                st.session_state.heartbeat_sim.set_path(path, flight_alt, drone_speed, safety_radius)
                st.session_state.simulation_running = True
                st.session_state.flight_history = []
                st.success("🚁 飞行已开始！请切换到「飞行监控」页面")
                st.rerun()
            else:
                st.error("请先设置起点和终点")
        if st.button("⏹️ 停止飞行", use_container_width=True):
            st.session_state.simulation_running = False
            st.session_state.heartbeat_sim.simulating = False
            st.info("飞行已停止")
    st.markdown("### 📍 当前坐标")
    st.write(f"🟢 A: ({st.session_state.points_gcj['A'][0]:.6f}, {st.session_state.points_gcj['A'][1]:.6f})")
    st.write(f"🔴 B: ({st.session_state.points_gcj['B'][0]:.6f}, {st.session_state.points_gcj['B'][1]:.6f})")
    dist = math.hypot(st.session_state.points_gcj['B'][0]-st.session_state.points_gcj['A'][0],
                      st.session_state.points_gcj['B'][1]-st.session_state.points_gcj['A'][1]) * 111000
    st.caption(f"📏 直线距离: {dist:.0f} 米")
    if st.session_state.planned_path:
        path_len = calculate_path_length(st.session_state.planned_path) * 111000
        st.caption(f"🔄 规划路径长: {path_len:.0f} 米")

def render_planning_map_view(map_type, flight_alt, straight_blocked, safety_radius):
    st.subheader("🗺️ 规划地图")
    flight_trail = [[hb.lng, hb.lat] for hb in st.session_state.heartbeat_sim.history[:20]] if st.session_state.heartbeat_sim.history else []
    center = st.session_state.points_gcj['A'] or config.SCHOOL_CENTER_GCJ
    if st.session_state.planned_path is None:
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, safety_radius
        )
    drone_pos = st.session_state.heartbeat_sim.current_pos if st.session_state.heartbeat_sim.simulating else None
    m = create_planning_map(
        center, st.session_state.points_gcj, st.session_state.obstacles_gcj, flight_trail,
        st.session_state.planned_path, map_type, straight_blocked, flight_alt,
        drone_pos, st.session_state.current_direction, safety_radius
    )
    output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing"])
    if output and output.get("last_active_drawing"):
        last = output["last_active_drawing"]
        if last and last.get("geometry") and last["geometry"].get("type") == "Polygon":
            coords = last["geometry"]["coordinates"][0]
            if len(coords) >= 3:
                st.session_state.pending_obstacle = [[p[0], p[1]] for p in coords]
                st.rerun()
    if st.session_state.pending_obstacle:
        st.markdown("---")
        st.subheader("📝 添加新障碍物")
        new_name = st.text_input("障碍物名称", f"建筑物{len(st.session_state.obstacles_gcj)+1}")
        new_height = st.number_input("障碍物高度 (米)", 1, 200, 30, 5)
        col_ok, col_cancel = st.columns(2)
        if col_ok.button("✅ 确认添加"):
            st.session_state.obstacles_gcj.append({
                "name": new_name, "polygon": st.session_state.pending_obstacle,
                "height": new_height, "selected": False
            })
            if auto_save:
                save_obstacles(st.session_state.obstacles_gcj)
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, safety_radius
            )
            st.session_state.pending_obstacle = None
            st.rerun()
        if col_cancel.button("❌ 取消"):
            st.session_state.pending_obstacle = None
            st.rerun()

# ==================== 飞行监控页面（优化版） ====================
def render_flight_monitoring_page(map_type, flight_alt, drone_speed, safety_radius):
    st.header("📡 飞行实时画面 - 任务执行监控")

    # 控制按钮
    col_btn = st.columns(4)
    with col_btn[0]:
        if st.button("▶️ 开始任务", use_container_width=True):
            if not st.session_state.simulation_running:
                if st.session_state.planned_path is None:
                    st.warning("请先在规划页面刷新规划路径")
                else:
                    st.session_state.heartbeat_sim.set_path(
                        st.session_state.planned_path, flight_alt, drone_speed, safety_radius
                    )
                    st.session_state.simulation_running = True
                    st.session_state.flight_history = []
                    st.rerun()
            else:
                # 如果已开始，恢复（如果处于暂停状态）
                st.session_state.heartbeat_sim.simulating = True
                st.rerun()
    with col_btn[1]:
        if st.button("⏸️ 暂停", use_container_width=True):
            if st.session_state.simulation_running:
                st.session_state.heartbeat_sim.simulating = False
                st.rerun()
            else:
                st.warning("当前没有飞行任务")
    with col_btn[2]:
        if st.button("⏹️ 停止", use_container_width=True):
            st.session_state.simulation_running = False
            st.session_state.heartbeat_sim.simulating = False
            st.rerun()
    with col_btn[3]:
        if st.button("🔄 重置", use_container_width=True):
            st.session_state.simulation_running = False
            st.session_state.heartbeat_sim = HeartbeatSimulator(st.session_state.points_gcj['A'][:])
            st.session_state.flight_history = []
            st.rerun()

    st.markdown("---")

    # 实时心跳更新（自动刷新）
    if st.session_state.simulation_running:
        current_time = time.time()
        if current_time - st.session_state.last_hb_time >= config.HEARTBEAT_INTERVAL:
            new_hb = st.session_state.heartbeat_sim.update_and_generate(st.session_state.obstacles_gcj)
            if new_hb:
                st.session_state.last_hb_time = current_time
                st.session_state.flight_history.append([new_hb.lng, new_hb.lat])
                if len(st.session_state.flight_history) > 200:
                    st.session_state.flight_history.pop(0)
                if not st.session_state.heartbeat_sim.simulating:
                    st.session_state.simulation_running = False
            st.rerun()
    else:
        st.session_state.last_hb_time = time.time()

    # 获取最新心跳数据
    if st.session_state.heartbeat_sim.history:
        d = st.session_state.heartbeat_sim.history[0]
    else:
        d = HeartbeatData(
            timestamp="--:--:--", flight_time=0, lat=0, lng=0, altitude=flight_alt,
            voltage=0, satellites=0, speed=0, progress=0, arrived=False,
            safety_violation=False, remaining_distance=0,
            current_waypoint=0, total_waypoints=0
        )

    # 飞行进度条
    progress_val = d.progress if not d.arrived else 1.0
    st.progress(progress_val, text=f"✈️ 飞行进度: {int(progress_val*100)}%")

    # 主要指标卡片
    st.markdown("### 📊 实时飞行数据")
    col1, col2, col3 = st.columns(3)
    with col1:
        st.metric("🎯 当前航点", f"{d.current_waypoint} / {d.total_waypoints}")
    with col2:
        st.metric("💨 飞行速度", f"{d.speed:.1f} m/s")
    with col3:
        minutes = int(d.flight_time // 60)
        seconds = int(d.flight_time % 60)
        st.metric("⏰ 已用时间", f"{minutes:02d}:{seconds:02d}")

    col4, col5, col6 = st.columns(3)
    with col4:
        remaining = max(0, d.remaining_distance)
        if remaining >= 1000:
            dist_text = f"{remaining/1000:.2f} km"
        else:
            dist_text = f"{remaining:.0f} m"
        st.metric("📏 剩余距离", dist_text)
    with col5:
        # 预计到达时间
        if d.arrived:
            eta = "00:00"
        elif d.speed > 0 and remaining > 0:
            eta_sec = remaining / d.speed
            mins = int(eta_sec // 60)
            secs = int(eta_sec % 60)
            eta = f"{mins:02d}:{secs:02d}"
        else:
            eta = "计算中..."
        st.metric("🕐 预计到达", eta)
    with col6:
        battery = max(0, 100 - int((d.flight_time / 1800)*100))
        st.metric("🔋 电量模拟", f"{battery}%")

    st.markdown("---")

    # 设备状态与通信拓扑（模拟）
    col_status, col_top = st.columns(2)
    with col_status:
        st.subheader("📡 设备状态")
        online = st.session_state.simulation_running
        st.markdown(f"- **GCS**：{'✅ 在线' if online else '❌ 离线'}")
        st.markdown(f"- **OBC**：{'✅ 在线' if online else '❌ 离线'}")
        st.markdown(f"- **FCU**：{'✅ 在线' if online else '❌ 离线'}")
    with col_top:
        st.subheader("🔗 通信链路拓扑与数据流")
        delay = random.randint(10,50) if online else 0
        loss = random.uniform(0,0.2) if online else 0
        st.markdown(f"""
        - **GCS** ↔ **OBC**：延迟 {delay} ms
        - **GCS** ↔ **FCU**：延迟 {delay+5} ms
        - **OBC** ↔ **FCU**：延迟 ~{max(0,delay-2)} ms
        - **丢包率**：{loss:.1f}%
        """)
        st.code("GCS → OBC → FCU → UAV")
        st.caption("数据流：遥控指令 → 飞控 → 执行器 | 遥测数据 ← 飞控 ← 传感器")

    st.markdown("---")

    # 实时飞行地图
    st.subheader("🗺️ 实时飞行地图")
    if st.session_state.heartbeat_sim.history:
        latest = st.session_state.heartbeat_sim.history[0]
        center = (latest.lat, latest.lng)
    else:
        center = (st.session_state.points_gcj['A'][1], st.session_state.points_gcj['A'][0])
    m = create_monitor_map(
        center[0], center[1], st.session_state.obstacles_gcj,
        st.session_state.planned_path, st.session_state.flight_history,
        [latest.lng, latest.lat] if st.session_state.heartbeat_sim.history else None,
        safety_radius, flight_alt, st.session_state.current_direction,
        st.session_state.waypoints if 'waypoints' in st.session_state else [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
    )
    folium_static(m, width=1000, height=500)

    # 飞行数据图表
    if len(st.session_state.heartbeat_sim.history) > 1:
        st.markdown("---")
        st.subheader("📈 飞行数据图表")
        hist_data = st.session_state.heartbeat_sim.history[:30]
        times = [i*config.HEARTBEAT_INTERVAL for i in range(len(hist_data))]
        speed_vals = [h.speed for h in hist_data]
        remain_vals = [max(0, h.remaining_distance) for h in hist_data]
        df_speed = pd.DataFrame({"时间(s)": times, "速度(m/s)": speed_vals})
        df_remain = pd.DataFrame({"时间(s)": times, "剩余距离(m)": remain_vals})
        col_ch1, col_ch2 = st.columns(2)
        with col_ch1:
            st.line_chart(df_speed, x="时间(s)", y="速度(m/s)")
        with col_ch2:
            st.line_chart(df_remain, x="时间(s)", y="剩余距离(m)")

    # 飞行日志表格
    if st.session_state.heartbeat_sim.flight_log:
        st.markdown("---")
        st.subheader("📋 飞行日志")
        log_df = st.session_state.heartbeat_sim.export_flight_data().head(10)
        log_df = log_df[['timestamp', 'flight_time', 'lat', 'lng', 'altitude', 'speed', 'remaining_distance', 'current_waypoint']]
        log_df.columns = ['时间', '飞行时间(s)', '纬度', '经度', '高度(m)', '速度(m/s)', '剩余距离(m)', '当前航点']
        st.dataframe(log_df, use_container_width=True)

# ==================== 障碍物管理页面 ====================
def render_obstacle_management_page(flight_alt, safety_radius):
    st.header("🚧 障碍物管理")
    st.info(f"当前共 {len(st.session_state.obstacles_gcj)} 个障碍物")
    col1, col2 = st.columns([1, 1.5])
    with col1:
        for i, obs in enumerate(st.session_state.obstacles_gcj):
            with st.container(border=True):
                col_name, col_btn = st.columns([3,1])
                col_name.write(f"🚧 {obs.get('name', f'障碍物{i+1}')} (高度: {obs.get('height',30)}m)")
                if col_btn.button("删除", key=f"del_{i}"):
                    st.session_state.obstacles_gcj.pop(i)
                    if st.session_state.auto_backup:
                        save_obstacles(st.session_state.obstacles_gcj)
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj, flight_alt,
                        st.session_state.current_direction, safety_radius
                    )
                    st.rerun()
        if st.button("💾 保存到缓存", use_container_width=True):
            save_obstacles(st.session_state.obstacles_gcj)
            st.success("已保存")
        if st.button("📂 从缓存加载", use_container_width=True):
            loaded = load_obstacles()
            if loaded:
                st.session_state.obstacles_gcj = loaded
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj, flight_alt,
                    st.session_state.current_direction, safety_radius
                )
                st.rerun()
        if st.button("🗑️ 全部清除", use_container_width=True):
            st.session_state.obstacles_gcj = []
            save_obstacles([])
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt,
                st.session_state.current_direction, safety_radius
            )
            st.rerun()
    with col2:
        tiles = config.GAODE_SATELLITE_URL
        m = folium.Map(location=[config.SCHOOL_CENTER_GCJ[1], config.SCHOOL_CENTER_GCJ[0]], zoom_start=16, tiles=tiles, attr="高德地图")
        for obs in st.session_state.obstacles_gcj:
            coords = obs.get('polygon', [])
            if coords and len(coords) >= 3:
                folium.Polygon([[c[1], c[0]] for c in coords], color="red", weight=3, fill=True, fill_opacity=0.5).add_to(m)
        folium.Marker([config.DEFAULT_A_GCJ[1], config.DEFAULT_A_GCJ[0]], popup="起点", icon=folium.Icon(color='green')).add_to(m)
        folium.Marker([config.DEFAULT_B_GCJ[1], config.DEFAULT_B_GCJ[0]], popup="终点", icon=folium.Icon(color='red')).add_to(m)
        folium_static(m, width=700, height=500)

# ==================== 主程序 ====================
def main():
    st.set_page_config(page_title="无人机地面站系统", layout="wide")
    init_session_state()

    page, map_type, drone_speed, flight_alt, auto_save, safety_radius = render_sidebar()
    st.session_state.auto_backup = auto_save
    st.session_state.safety_radius = safety_radius

    if flight_alt != st.session_state.last_flight_altitude:
        st.session_state.last_flight_altitude = flight_alt
        if st.session_state.planned_path is not None:
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt,
                st.session_state.current_direction, safety_radius
            )
            st.rerun()

    if page == "🗺️ 航线规划":
        render_planning_page(map_type, drone_speed, flight_alt, auto_save, safety_radius)
    elif page == "📡 飞行监控":
        render_flight_monitoring_page(map_type, flight_alt, drone_speed, safety_radius)
    elif page == "🚧 障碍物管理":
        render_obstacle_management_page(flight_alt, safety_radius)

if __name__ == "__main__":
    main()

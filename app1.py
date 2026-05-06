import streamlit as st
import folium
from streamlit_folium import folium_static, st_folium
from folium import plugins
import random
import time
import math
import copy
from datetime import datetime
import pandas as pd

# ==================== 页面配置 ====================
st.set_page_config(page_title="无人机地面站系统 - 飞行监控增强版", layout="wide")

# ==================== 坐标 ====================
SCHOOL_CENTER_GCJ = [118.7490, 32.2340]
DEFAULT_A_GCJ = [118.746956, 32.232945]
DEFAULT_B_GCJ = [118.751589, 32.235204]

GAODE_SATELLITE_URL = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
GAODE_VECTOR_URL = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"

# ==================== 坐标系转换 ====================
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

# ==================== 几何辅助函数 ====================
def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def point_to_segment_distance(p, a, b):
    ap = (p[0]-a[0], p[1]-a[1])
    ab = (b[0]-a[0], b[1]-a[1])
    t = (ap[0]*ab[0] + ap[1]*ab[1]) / (ab[0]*ab[0] + ab[1]*ab[1] + 1e-9)
    t = max(0.0, min(1.0, t))
    proj = (a[0] + t*ab[0], a[1] + t*ab[1])
    return distance(p, proj)

def segment_to_polygon_min_distance(p1, p2, polygon):
    min_dist = float('inf')
    for pt in polygon:
        d = point_to_segment_distance(pt, p1, p2)
        if d < min_dist:
            min_dist = d
    for i in range(len(polygon)):
        p3 = polygon[i]
        p4 = polygon[(i+1)%len(polygon)]
        steps = 10
        for t in range(steps+1):
            pt = (p3[0] + (p4[0]-p3[0])*t/steps, p3[1] + (p4[1]-p3[1])*t/steps)
            d = point_to_segment_distance(pt, p1, p2)
            if d < min_dist:
                min_dist = d
    return min_dist * 111000

def line_intersects_polygon(p1, p2, polygon):
    if point_in_polygon(p1, polygon) or point_in_polygon(p2, polygon):
        return True
    for i in range(len(polygon)):
        p3 = polygon[i]
        p4 = polygon[(i+1)%len(polygon)]
        if segments_intersect(p1, p2, p3, p4):
            return True
    return False

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

def segments_intersect(p1, p2, p3, p4):
    def ccw(A,B,C):
        return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])
    return (ccw(p1,p3,p4) != ccw(p2,p3,p4)) and (ccw(p1,p2,p3) != ccw(p1,p2,p4))

def should_avoid_obstacle(obs, flight_height):
    return flight_height <= obs.get('height', 20)

def is_segment_safe(p1, p2, obstacles, safe_radius_m, flight_height):
    for obs in obstacles:
        if not should_avoid_obstacle(obs, flight_height):
            continue
        poly = obs.get('polygon', [])
        if not poly:
            continue
        if line_intersects_polygon(p1, p2, poly):
            return False
        min_dist = segment_to_polygon_min_distance(p1, p2, poly)
        if min_dist < safe_radius_m - 0.1:
            return False
    return True

# ==================== 强制绕行核心算法 ====================
def generate_bypass_path(start, end, obstacles, safe_radius_m, flight_height, side='left'):
    avoid_obs = [obs for obs in obstacles if should_avoid_obstacle(obs, flight_height)]
    if not avoid_obs:
        return [start, end]
    
    mid_x = (start[0] + end[0]) / 2
    mid_y = (start[1] + end[1]) / 2
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return [start, end]
    ux = dx / length
    uy = dy / length
    perp_x = -uy
    perp_y = ux
    if side == 'right':
        perp_x = uy
        perp_y = -ux
    
    deg_per_meter = 1.0 / 111000.0
    for attempt in range(1, 31):
        offset_m = safe_radius_m * 2 * attempt
        offset_deg = offset_m * deg_per_meter
        waypoint = (mid_x + perp_x * offset_deg, mid_y + perp_y * offset_deg)
        if is_segment_safe(start, waypoint, avoid_obs, safe_radius_m, flight_height) and \
           is_segment_safe(waypoint, end, avoid_obs, safe_radius_m, flight_height):
            return [start, waypoint, end]
    
    all_pts = []
    for obs in avoid_obs:
        all_pts.extend(obs.get('polygon', []))
    if not all_pts:
        return [start, end]
    cx = sum(p[0] for p in all_pts) / len(all_pts)
    cy = sum(p[1] for p in all_pts) / len(all_pts)
    max_dist = 0
    far_pt = all_pts[0]
    for pt in all_pts:
        d = distance((cx, cy), pt)
        if d > max_dist:
            max_dist = d
            far_pt = pt
    dir_x = far_pt[0] - cx
    dir_y = far_pt[1] - cy
    nd = math.hypot(dir_x, dir_y)
    if nd > 0:
        dir_x /= nd
        dir_y /= nd
    else:
        dir_x, dir_y = 1, 0
    ext_m = safe_radius_m * 15
    ext_deg = ext_m * deg_per_meter
    external_point = (far_pt[0] + dir_x * ext_deg, far_pt[1] + dir_y * ext_deg)
    return [start, external_point, end]

def create_avoidance_path(start, end, obstacles, flight_height, safe_radius_m, strategy):
    avoid_obs = [obs for obs in obstacles if should_avoid_obstacle(obs, flight_height)]
    straight_safe = True
    for obs in avoid_obs:
        poly = obs.get('polygon', [])
        if poly and line_intersects_polygon(start, end, poly):
            straight_safe = False
            break
    if straight_safe:
        return [start, end]
    if strategy == 'left':
        return generate_bypass_path(start, end, obstacles, safe_radius_m, flight_height, 'left')
    elif strategy == 'right':
        return generate_bypass_path(start, end, obstacles, safe_radius_m, flight_height, 'right')
    else:
        left_path = generate_bypass_path(start, end, obstacles, safe_radius_m, flight_height, 'left')
        right_path = generate_bypass_path(start, end, obstacles, safe_radius_m, flight_height, 'right')
        if left_path and right_path:
            def path_len(p):
                return sum(distance(p[i], p[i+1]) for i in range(len(p)-1))
            return left_path if path_len(left_path) <= path_len(right_path) else right_path
        return left_path or right_path or [start, end]

# ==================== 障碍物管理（内存缓存） ====================
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

# ==================== 心跳包模拟器（增强版） ====================
class HeartbeatSimulator:
    def __init__(self, start_point_gcj):
        self.history = []
        self.current_pos = start_point_gcj.copy()
        self.path = [start_point_gcj.copy()]
        self.path_index = 0
        self.simulating = False
        self.paused = False
        self.flight_altitude = 50
        self.speed = 50
        self.progress = 0.0
        self.total_distance = 0.0
        self.distance_traveled = 0.0
        self.start_time = None
        self.last_update_time = None
        
    def set_path(self, path, altitude=50, speed=50):
        self.path = path
        self.path_index = 0
        self.current_pos = path[0].copy()
        self.flight_altitude = altitude
        self.speed = speed
        self.simulating = True
        self.paused = False
        self.progress = 0.0
        self.distance_traveled = 0.0
        self.total_distance = 0.0
        for i in range(len(path)-1):
            self.total_distance += distance(path[i], path[i+1])
        self.start_time = time.time()
        self.last_update_time = time.time()
        
    def pause(self):
        self.paused = True
        
    def resume(self):
        self.paused = False
        self.last_update_time = time.time()
        
    def stop(self):
        self.simulating = False
        self.paused = False
        
    def update_and_generate(self):
        if not self.simulating or self.paused:
            return self._generate_heartbeat()
        
        if self.path_index < len(self.path)-1:
            target = self.path[self.path_index+1]
            dx = target[0] - self.current_pos[0]
            dy = target[1] - self.current_pos[1]
            dist_to_target = math.hypot(dx, dy)
            speed_mps = 0.5 + (self.speed / 100) * 4.5
            dt = 0.2
            step_m = speed_mps * dt
            step_deg = step_m / 111000.0
            if dist_to_target < step_deg:
                self.distance_traveled += dist_to_target
                self.current_pos = target.copy()
                self.path_index += 1
            else:
                ratio = step_deg / dist_to_target
                self.current_pos[0] += dx * ratio
                self.current_pos[1] += dy * ratio
                self.distance_traveled += step_deg
            if self.total_distance > 0:
                self.progress = min(1.0, self.distance_traveled / self.total_distance)
            if self.path_index >= len(self.path)-1:
                self.simulating = False
                self.progress = 1.0
        else:
            self.simulating = False
            self.progress = 1.0
        self.last_update_time = time.time()
        return self._generate_heartbeat()
    
    def _generate_heartbeat(self):
        altitude = self.flight_altitude + random.randint(-5,5) if self.simulating else random.randint(0,10)
        speed_display = round(0.5 + (self.speed / 100) * 4.5, 1) if self.simulating and not self.paused else 0
        battery = max(0, 100 - int(self.progress * 100))
        remaining_dist = max(0, self.total_distance - self.distance_traveled)
        remaining_time_sec = remaining_dist / (speed_display + 0.01) / 111000.0 * 3600 if speed_display > 0 else 0
        remaining_time_str = f"{int(remaining_time_sec//60):02d}:{int(remaining_time_sec%60):02d}"
        return {
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "lng": self.current_pos[0],
            "lat": self.current_pos[1],
            "altitude": altitude,
            "voltage": round(random.uniform(11.5,12.8),1),
            "satellites": random.randint(8,14),
            "speed": speed_display,
            "progress": self.progress,
            "distance_traveled": self.distance_traveled,
            "total_distance": self.total_distance,
            "simulating": self.simulating,
            "paused": self.paused,
            "current_waypoint": f"{self.path_index+1}/{len(self.path)}",
            "remaining_time": remaining_time_str,
            "battery": battery
        }

# ==================== 安全半径可视化 ====================
def add_safety_buffer(map_obj, obstacles_gcj, safe_radius_m, flight_height):
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        if not coords:
            continue
        if should_avoid_obstacle(obs, flight_height):
            for pt in coords:
                folium.Circle(
                    location=[pt[1], pt[0]],
                    radius=safe_radius_m,
                    color='orange',
                    fill=True,
                    fill_opacity=0.2,
                    popup=f"安全区域 (半径 {safe_radius_m}m)"
                ).add_to(map_obj)

def create_planning_map(center_gcj, points_gcj, obstacles_gcj, flight_history, planned_path, map_type, straight_blocked, safe_radius_m, flight_height):
    if map_type == "satellite":
        tiles = GAODE_SATELLITE_URL
        attr = "高德卫星地图"
    else:
        tiles = GAODE_VECTOR_URL
        attr = "高德矢量地图"
    m = folium.Map(location=[center_gcj[1], center_gcj[0]], zoom_start=16, tiles=tiles, attr=attr)
    draw = plugins.Draw(
        export=True, position='topleft',
        draw_options={'polygon': {'allowIntersection': False, 'showArea': True, 'color': '#ff0000', 'fillColor': '#ff0000', 'fillOpacity': 0.4},
                      'polyline': False, 'rectangle': False, 'circle': False, 'marker': False, 'circlemarker': False},
        edit_options={'edit': True, 'remove': True}
    )
    m.add_child(draw)
    add_safety_buffer(m, obstacles_gcj, safe_radius_m, flight_height)
    for i, obs in enumerate(obstacles_gcj):
        coords = obs.get('polygon', [])
        if coords and len(coords) >= 3:
            popup_text = f"🚧 {obs.get('name', f'障碍物{i+1}')}\n高度: {obs.get('height', 20)}m"
            folium.Polygon([[c[1], c[0]] for c in coords], color="red", weight=3, fill=True, fill_color="red", fill_opacity=0.4, popup=popup_text).add_to(m)
    if points_gcj.get('A'):
        folium.Marker([points_gcj['A'][1], points_gcj['A'][0]], popup="🟢 起点", icon=folium.Icon(color="green", icon="play", prefix="fa")).add_to(m)
    if points_gcj.get('B'):
        folium.Marker([points_gcj['B'][1], points_gcj['B'][0]], popup="🔴 终点", icon=folium.Icon(color="red", icon="stop", prefix="fa")).add_to(m)
    if planned_path and len(planned_path) > 1:
        path_locations = [[p[1], p[0]] for p in planned_path]
        folium.PolyLine(path_locations, color="green", weight=5, opacity=0.9, popup="✈️ 避障航线（安全距离保持）").add_to(m)
        for i, point in enumerate(planned_path[1:-1]):
            folium.CircleMarker([point[1], point[0]], radius=4, color="green", fill=True, fill_color="white", fill_opacity=0.8, popup=f"航点 {i+1}").add_to(m)
    if points_gcj.get('A') and points_gcj.get('B'):
        if straight_blocked:
            folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]], color="gray", weight=2, opacity=0.4, dash_array='5, 5', popup="⚠️ 直线被阻挡").add_to(m)
        else:
            folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]], color="blue", weight=2, opacity=0.5, dash_array='5, 5', popup="直线航线（安全）").add_to(m)
    if flight_history and len(flight_history) > 1:
        trail = [[p[1], p[0]] for p in flight_history if len(p) >= 2]
        if len(trail) > 1:
            folium.PolyLine(trail, color="orange", weight=2, opacity=0.6, popup="历史轨迹").add_to(m)
    return m

# ==================== 主程序 ====================
def main():
    st.title("🏫 无人机地面站系统 - 飞行监控增强版")
    st.markdown("---")
    
    # 初始化状态
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
    
    # 侧边栏
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"])
    map_type_choice = st.sidebar.radio("🗺️ 地图类型", ["卫星影像", "矢量街道"], index=0)
    map_type = "satellite" if map_type_choice == "卫星影像" else "vector"
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("⚙️ 无人机参数")
    drone_speed = st.sidebar.slider("飞行速度系数", min_value=10, max_value=100, value=50, step=5)
    safe_radius = st.sidebar.number_input("安全半径 (米)", min_value=1, max_value=30, value=5, step=1)
    flight_alt = st.sidebar.number_input("飞行高度 (米)", min_value=0, max_value=200, value=st.session_state.flight_altitude, step=5)
    st.session_state.flight_altitude = flight_alt
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("🔄 绕行策略")
    strategy = st.sidebar.radio("选择避障方式", ["最佳航线 (A*)", "向左绕行", "向右绕行"], index=0)
    strategy_map = {"最佳航线 (A*)": "best", "向左绕行": "left", "向右绕行": "right"}
    selected_strategy = strategy_map[strategy]
    
    st.sidebar.markdown("---")
    obs_count = len(st.session_state.obstacles_gcj)
    avoid_obs = [obs for obs in st.session_state.obstacles_gcj if should_avoid_obstacle(obs, st.session_state.flight_altitude)]
    straight_blocked = False
    for obs in avoid_obs:
        poly = obs.get('polygon', [])
        if poly and line_intersects_polygon(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'], poly):
            straight_blocked = True
            break
    st.sidebar.info(f"🏫 校园区域\n🚧 障碍物: {obs_count}（需避让: {len(avoid_obs)}）\n📌 直线: {'🚫 被阻挡' if straight_blocked else '✅ 安全'}")
    
    if st.sidebar.button("🔄 刷新数据", use_container_width=True):
        with st.spinner("正在规划强制绕行路径（保证安全距离）..."):
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'],
                st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj,
                st.session_state.flight_altitude,
                safe_radius,
                selected_strategy
            )
        st.rerun()
    
    # ==================== 航线规划页面 ====================
    if page == "🗺️ 航线规划":
        st.header("🗺️ 航线规划 - 强制绕行")
        if straight_blocked:
            st.warning(f"⚠️ 直线航线被需要避让的建筑物阻挡！已自动规划 {strategy} 避障航线（绿色折线）")
        else:
            st.success(f"✅ 直线航线安全（可飞越所有障碍物或保持安全距离）")
        st.info("📝 点击地图左上角📐图标 → 选择多边形 → 围绕建筑物绘制 → 双击完成，然后在侧边栏设置该障碍物高度，最后点击「添加障碍物」保存")
        
        col1, col2 = st.columns([1, 1.5])
        with col1:
            st.subheader("🎮 控制面板")
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
                    selected_strategy
                )
                st.rerun()
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
                    selected_strategy
                )
                st.rerun()
            st.markdown("#### 🏗️ 新障碍物高度")
            new_obs_height = st.number_input("高度 (米)", min_value=1, max_value=200, value=st.session_state.pending_height, step=5)
            st.session_state.pending_height = new_obs_height
            if st.button("➕ 添加障碍物（从当前圈选）", use_container_width=True):
                if st.session_state.pending_polygon and len(st.session_state.pending_polygon) >= 3:
                    st.session_state.obstacles_gcj.append({
                        "name": f"建筑物{len(st.session_state.obstacles_gcj)+1}",
                        "polygon": st.session_state.pending_polygon,
                        "height": st.session_state.pending_height
                    })
                    st.success(f"已添加障碍物（高度{st.session_state.pending_height}m），当前共 {len(st.session_state.obstacles_gcj)} 个")
                    st.session_state.pending_polygon = None
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'],
                        st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj,
                        st.session_state.flight_altitude,
                        safe_radius,
                        selected_strategy
                    )
                    st.rerun()
                else:
                    st.warning("请先在地图上绘制一个多边形")
            if st.button("🔄 重新规划路径（应用当前策略）", use_container_width=True):
                with st.spinner("重新规划中..."):
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'],
                        st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj,
                        st.session_state.flight_altitude,
                        safe_radius,
                        selected_strategy
                    )
                if st.session_state.planned_path:
                    st.success(f"已规划 {len(st.session_state.planned_path)} 个航点，策略：{strategy}")
                st.rerun()
            st.markdown("#### ✈️ 飞行控制")
            col_btn1, col_btn2 = st.columns(2)
            with col_btn1:
                if st.button("▶️ 开始飞行", use_container_width=True):
                    path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
                    st.session_state.heartbeat_sim.set_path(path, st.session_state.flight_altitude, drone_speed)
                    st.session_state.simulation_running = True
                    st.session_state.flight_history = []
                    st.success("🚁 飞行已开始！请切换到「飞行监控」页面")
            with col_btn2:
                if st.button("⏹️ 停止飞行", use_container_width=True):
                    st.session_state.simulation_running = False
                    st.session_state.heartbeat_sim.stop()
                    st.info("飞行已停止")
            st.markdown("### 📍 当前航点")
            st.write(f"🟢 A点: ({st.session_state.points_gcj['A'][0]:.6f}, {st.session_state.points_gcj['A'][1]:.6f})")
            st.write(f"🔴 B点: ({st.session_state.points_gcj['B'][0]:.6f}, {st.session_state.points_gcj['B'][1]:.6f})")
            a, b = st.session_state.points_gcj['A'], st.session_state.points_gcj['B']
            dist = math.hypot(b[0]-a[0], b[1]-a[1]) * 111000
            st.caption(f"📏 直线距离: {dist:.0f} 米")
            if st.session_state.planned_path and len(st.session_state.planned_path) > 2:
                path_dist = 0
                for i in range(len(st.session_state.planned_path)-1):
                    p1, p2 = st.session_state.planned_path[i], st.session_state.planned_path[i+1]
                    path_dist += math.hypot(p2[0]-p1[0], p2[1]-p1[1]) * 111000
                st.caption(f"🔄 避障路径: {path_dist:.0f} 米")
        
        with col2:
            st.subheader("🗺️ 规划地图")
            flight_trail = [[hb['lng'], hb['lat']] for hb in st.session_state.heartbeat_sim.history[:20]]
            center = st.session_state.points_gcj['A'] or SCHOOL_CENTER_GCJ
            if st.session_state.planned_path is None:
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj,
                    st.session_state.flight_altitude,
                    safe_radius,
                    selected_strategy
                )
            m = create_planning_map(center, st.session_state.points_gcj, st.session_state.obstacles_gcj, 
                                    flight_trail, st.session_state.planned_path, map_type, 
                                    straight_blocked, safe_radius, st.session_state.flight_altitude)
            output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing"])
            if output and output.get("last_active_drawing"):
                last = output["last_active_drawing"]
                if last and last.get("geometry") and last["geometry"].get("type") == "Polygon":
                    coords = last["geometry"].get("coordinates", [])
                    if coords and len(coords) > 0:
                        poly = [[p[0], p[1]] for p in coords[0]]
                        if len(poly) >= 3:
                            st.session_state.pending_polygon = poly
                            st.success("已捕获多边形，请设置高度并点击「添加障碍物」保存")
            st.caption("📌 **图例**：🟢 绿色=避障航线（强制绕行且保持安全距离） | 🔴 红色=障碍物 | 🟠 橙色圆=安全缓冲区域（仅对需避让障碍物）")
    
    # ==================== 飞行监控页面（增强版） ====================
    elif page == "📡 飞行监控":
        st.header("📡 飞行实时画面 - 任务执行监控")
        
        # 控制按钮行
        col_btn1, col_btn2, col_btn3 = st.columns(3)
        with col_btn1:
            if st.button("▶️ 开始任务", use_container_width=True):
                if not st.session_state.simulation_running:
                    path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
                    st.session_state.heartbeat_sim.set_path(path, st.session_state.flight_altitude, drone_speed)
                    st.session_state.simulation_running = True
                    st.rerun()
                else:
                    st.session_state.heartbeat_sim.resume()
                    st.rerun()
        with col_btn2:
            if st.button("⏸️ 暂停", use_container_width=True):
                if st.session_state.simulation_running:
                    st.session_state.heartbeat_sim.pause()
                    st.rerun()
        with col_btn3:
            if st.button("⏹️ 停止", use_container_width=True):
                st.session_state.simulation_running = False
                st.session_state.heartbeat_sim.stop()
                st.rerun()
        
        st.markdown("---")
        
        # 监控指标表格
        col1, col2, col3, col4, col5 = st.columns(5)
        # 获取最新心跳数据
        current_time = time.time()
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
        
        if st.session_state.heartbeat_sim.history:
            latest = st.session_state.heartbeat_sim.history[0]
            with col1:
                st.metric("当前航点", latest.get('current_waypoint', '0/0'))
            with col2:
                st.metric("飞行速度", f"{latest.get('speed', 0)} m/s")
            with col3:
                st.metric("剩余距离", f"{max(0, (latest.get('total_distance',0)-latest.get('distance_traveled',0))*111000):.0f} m")
            with col4:
                st.metric("预计到达", latest.get('remaining_time', '00:00'))
            with col5:
                st.metric("电量模拟", f"{latest.get('battery', 0)}%")
        
        # 任务进度条
        if st.session_state.heartbeat_sim.history:
            progress_val = st.session_state.heartbeat_sim.history[0].get('progress', 0)
            st.progress(progress_val, text=f"✈️ 任务进度: {progress_val*100:.1f}%")
        else:
            st.progress(0, text="✈️ 任务进度: 0%")
        
        st.markdown("---")
        
        # 状态指示与通信拓扑
        col_status, col_topology = st.columns(2)
        with col_status:
            st.subheader("📡 设备状态")
            gcs_online = True
            obc_online = st.session_state.simulation_running
            fcu_online = st.session_state.simulation_running
            st.markdown(f"- **GCS**：{'✅ 在线' if gcs_online else '❌ 离线'}")
            st.markdown(f"- **OBC**：{'✅ 在线' if obc_online else '❌ 离线'}")
            st.markdown(f"- **FCU**：{'✅ 在线' if fcu_online else '❌ 离线'}")
        
        with col_topology:
            st.subheader("🔗 通信链路拓扑")
            st.markdown("""GCS (地面站) → OBC (机载计算机) → FCU (飞控) → UAV""")
            st.caption("数据流：遥控指令 → 飞控 → 执行器 | 遥测数据 ← 飞控 ← 传感器")

st.markdown("---")

# 实时飞行地图
st.subheader("🗺️ 实时飞行地图")
if st.session_state.heartbeat_sim.history:
    latest = st.session_state.heartbeat_sim.history[0]
    tiles = GAODE_SATELLITE_URL if map_type == "satellite" else GAODE_VECTOR_URL
    monitor_map = folium.Map(location=[latest['lat'], latest['lng']], zoom_start=17, tiles=tiles, attr="高德地图")
    add_safety_buffer(monitor_map, st.session_state.obstacles_gcj, safe_radius, st.session_state.flight_altitude)
    for obs in st.session_state.obstacles_gcj:
        coords = obs.get('polygon', [])
        if coords and len(coords) >= 3:
            popup_text = f"高度: {obs.get('height',20)}m"
            folium.Polygon([[c[1], c[0]] for c in coords], color="red", weight=2, fill=True, fill_opacity=0.3, popup=popup_text).add_to(monitor_map)
    if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
        folium.PolyLine([[p[1], p[0]] for p in st.session_state.planned_path], color="green", weight=3, opacity=0.7, popup="避障航线").add_to(monitor_map)
    trail = [[hb['lat'], hb['lng']] for hb in st.session_state.heartbeat_sim.history[:30] if hb.get('lat') and hb.get('lng')]
    if len(trail) > 1:
        folium.PolyLine(trail, color="orange", weight=2, opacity=0.7, popup="历史轨迹").add_to(monitor_map)
    folium.Marker([latest['lat'], latest['lng']], popup=f"📍 当前位置\n高度: {latest['altitude']}m\n速度: {latest.get('speed',0)} m/s", 
                 icon=folium.Icon(color='red', icon='plane', prefix='fa')).add_to(monitor_map)
    if st.session_state.points_gcj['A']:
        folium.Marker([st.session_state.points_gcj['A'][1], st.session_state.points_gcj['A'][0]], 
                     popup="🟢 起点", icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(monitor_map)
    if st.session_state.points_gcj['B']:
        folium.Marker([st.session_state.points_gcj['B'][1], st.session_state.points_gcj['B'][0]], 
                     popup="🔴 终点", icon=folium.Icon(color='red', icon='stop', prefix='fa')).add_to(monitor_map)
    folium_static(monitor_map, width=1000, height=500)
else:
    st.info("⏳ 等待飞行任务开始... 请在「航线规划」页面规划路径并点击「开始飞行」")

# ==================== 障碍物管理页面 ====================
elif page == "🚧 障碍物管理":
    st.header("🚧 障碍物管理")
    st.info(f"当前共 **{len(st.session_state.obstacles_gcj)}** 个障碍物（含高度信息）")
    col1, col2 = st.columns([1, 1.5])
    with col1:
        if st.session_state.obstacles_gcj:
            for i, obs in enumerate(st.session_state.obstacles_gcj):
                col_name, col_height, col_btn = st.columns([2,1,1])
                col_name.write(f"🚧 {obs.get('name', f'障碍物{i+1}')}")
                col_height.write(f"高度: {obs.get('height',20)}m")
                if col_btn.button("删除", key=f"del_{i}"):
                    st.session_state.obstacles_gcj.pop(i)
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'],
                        st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj,
                        st.session_state.flight_altitude,
                        safe_radius,
                        selected_strategy
                    )
                    st.rerun()
        else:
            st.write("暂无障碍物")
        st.markdown("---")
        col_save, col_load = st.columns(2)
        if col_save.button("💾 保存到缓存", use_container_width=True):
            save_obstacles_to_cache()
        if col_load.button("📂 从缓存加载", use_container_width=True):
            if load_obstacles_from_cache():
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj,
                    st.session_state.flight_altitude,
                    safe_radius,
                    selected_strategy
                )
                st.rerun()
        col_clear_all, col_download = st.columns(2)
        if col_clear_all.button("🗑️ 全部清除", use_container_width=True):
            st.session_state.obstacles_gcj = []
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'],
                st.session_state.points_gcj['B'],
                [],
                st.session_state.flight_altitude,
                safe_radius,
                selected_strategy
            )
            st.rerun()
    with col2:
        tiles = GAODE_SATELLITE_URL if map_type == "satellite" else GAODE_VECTOR_URL
        obs_map = folium.Map(location=[SCHOOL_CENTER_GCJ[1], SCHOOL_CENTER_GCJ[0]], zoom_start=16, tiles=tiles, attr="高德地图")
        for obs in st.session_state.obstacles_gcj:
            coords = obs.get('polygon', [])
            if coords and len(coords) >= 3:
                popup_text = f"高度: {obs.get('height',20)}m"
                folium.Polygon([[c[1], c[0]] for c in coords], color="red", weight=3, fill=True, fill_opacity=0.5, popup=popup_text).add_to(obs_map)
        folium.Marker([DEFAULT_A_GCJ[1], DEFAULT_A_GCJ[0]], popup="🟢 起点", icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(obs_map)
        folium.Marker([DEFAULT_B_GCJ[1], DEFAULT_B_GCJ[0]], popup="🔴 终点", icon=folium.Icon(color='red', icon='stop', prefix='fa')).add_to(obs_map)
        folium_static(obs_map, width=700, height=500)

if __name__ == "__main__":
    main()

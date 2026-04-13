import streamlit as st
import folium
from streamlit_folium import st_folium
import numpy as np
import json
import os
from datetime import datetime
from folium.plugins import Draw

# ==================== GCJ-02 与 WGS84 转换 ====================
a = 6378245.0
ee = 0.00669342162296594323

def _transform_lat(x, y):
    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * np.sqrt(abs(x))
    ret += (20.0 * np.sin(6.0 * x * np.pi) + 20.0 * np.sin(2.0 * x * np.pi)) * 2.0 / 3.0
    ret += (20.0 * np.sin(y * np.pi) + 40.0 * np.sin(y / 3.0 * np.pi)) * 2.0 / 3.0
    ret += (160.0 * np.sin(y / 12.0 * np.pi) + 320 * np.sin(y * np.pi / 30.0)) * 2.0 / 3.0
    return ret

def _transform_lng(x, y):
    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * np.sqrt(abs(x))
    ret += (20.0 * np.sin(6.0 * x * np.pi) + 20.0 * np.sin(2.0 * x * np.pi)) * 2.0 / 3.0
    ret += (20.0 * np.sin(x * np.pi) + 40.0 * np.sin(x / 3.0 * np.pi)) * 2.0 / 3.0
    ret += (150.0 * np.sin(x / 12.0 * np.pi) + 300.0 * np.sin(x / 30.0 * np.pi)) * 2.0 / 3.0
    return ret

def wgs84_to_gcj02(lng, lat):
    dlat = _transform_lat(lng - 105.0, lat - 35.0)
    dlng = _transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * np.pi
    magic = np.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = np.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * np.pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * np.cos(radlat) * np.pi)
    return lng + dlng, lat + dlat

def gcj02_to_wgs84(lng, lat):
    dlat = _transform_lat(lng - 105.0, lat - 35.0)
    dlng = _transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * np.pi
    magic = np.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = np.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * np.pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * np.cos(radlat) * np.pi)
    return lng - dlng, lat - dlat

# ==================== 模拟心跳包 ====================
def get_next_heartbeat():
    if 'drone_pos_gcj' not in st.session_state:
        st.session_state.drone_pos_gcj = (118.7492, 32.2328)
    if 'heartbeat_history' not in st.session_state:
        st.session_state.heartbeat_history = []
    if st.session_state.get('B_gcj'):
        target_lng, target_lat = st.session_state.B_gcj
        cur_lng, cur_lat = st.session_state.drone_pos_gcj
        dx = (target_lng - cur_lng) * 0.1
        dy = (target_lat - cur_lat) * 0.1
        if abs(dx) > 0.00001 or abs(dy) > 0.00001:
            st.session_state.drone_pos_gcj = (cur_lng + dx, cur_lat + dy)
    heartbeat = {
        "timestamp": datetime.now().strftime("%H:%M:%S"),
        "lng": st.session_state.drone_pos_gcj[0],
        "lat": st.session_state.drone_pos_gcj[1],
        "alt": st.session_state.get('flight_height', 50)
    }
    st.session_state.heartbeat_history.insert(0, heartbeat)
    st.session_state.heartbeat_history = st.session_state.heartbeat_history[:5]
    return heartbeat

# ==================== 障碍物持久化 ====================
CONFIG_FILE = "obstacle_config.json"
def save_polygons_to_file():
    data = {"version": "v12.2", "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"), "polygons": st.session_state.polygons}
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    st.success(f"已保存 {len(st.session_state.polygons)} 个障碍物")
def load_polygons_from_file():
    if not os.path.exists(CONFIG_FILE):
        st.error("配置文件不存在")
        return
    with open(CONFIG_FILE, "r", encoding="utf-8") as f:
        data = json.load(f)
    st.session_state.polygons = data.get("polygons", [])
    st.success(f"已加载 {len(st.session_state.polygons)} 个障碍物")
def clear_all_obstacles():
    st.session_state.polygons = []
    st.success("已清除所有障碍物")
def one_click_deploy():
    st.session_state.polygons = [
        [[118.7485, 32.2325], [118.7490, 32.2327], [118.7488, 32.2330]],
        [[118.7495, 32.2335], [118.7500, 32.2332], [118.7498, 32.2338]],
        [[118.7480, 32.2340], [118.7485, 32.2342], [118.7482, 32.2345]]
    ]
    st.success("已部署预设障碍物")

# ==================== 主界面 ====================
st.set_page_config(layout="wide", page_title="无人机障碍物规划 - 点击设置+手动输入")
st.title("✈️ 校园无人机飞行规划与实时监控")
st.markdown("**卫星地图 + GCJ-02坐标** | **点击地图设置起点/终点（或手动输入）** | **绘制多边形添加障碍物**")

# 初始化
for key in ['polygons', 'A_gcj', 'B_gcj', 'flight_height', 'drone_pos_gcj', 'heartbeat_history', 'set_mode']:
    if key not in st.session_state:
        if key == 'polygons': st.session_state.polygons = []
        elif key == 'A_gcj': st.session_state.A_gcj = None
        elif key == 'B_gcj': st.session_state.B_gcj = None
        elif key == 'flight_height': st.session_state.flight_height = 50
        elif key == 'drone_pos_gcj': st.session_state.drone_pos_gcj = (118.7492, 32.2328)
        elif key == 'heartbeat_history': st.session_state.heartbeat_history = []
        elif key == 'set_mode': st.session_state.set_mode = "起点A"

# ==================== 侧边栏 ====================
with st.sidebar:
    st.header("🎮 控制面板")
    st.subheader("📍 地图点击模式")
    mode = st.radio("点击地图设置:", ["起点A", "终点B"], index=0 if st.session_state.set_mode=="起点A" else 1)
    st.session_state.set_mode = mode
    
    # 手动输入坐标作为备选（100%可靠）
    st.subheader("✏️ 手动设置坐标 (GCJ-02)")
    col1, col2 = st.columns(2)
    with col1:
        manual_lat = st.number_input("纬度", value=32.2323, format="%.6f", key="manual_lat")
    with col2:
        manual_lng = st.number_input("经度", value=118.7490, format="%.6f", key="manual_lng")
    if st.button("设置选中的点"):
        if st.session_state.set_mode == "起点A":
            st.session_state.A_gcj = (manual_lng, manual_lat)
            st.success(f"起点A已手动设置: ({manual_lng}, {manual_lat})")
        else:
            st.session_state.B_gcj = (manual_lng, manual_lat)
            st.success(f"终点B已手动设置: ({manual_lng}, {manual_lat})")
    
    # 显示当前坐标
    st.markdown("---")
    st.subheader("📍 当前坐标 (GCJ-02)")
    if st.session_state.A_gcj:
        st.info(f"起点A: {st.session_state.A_gcj[0]:.6f}, {st.session_state.A_gcj[1]:.6f}")
    if st.session_state.B_gcj:
        st.info(f"终点B: {st.session_state.B_gcj[0]:.6f}, {st.session_state.B_gcj[1]:.6f}")
    
    st.markdown("---")
    st.subheader("🚁 飞行参数")
    st.session_state.flight_height = st.number_input("设定飞行高度 (m)", value=st.session_state.flight_height, step=5)
    
    st.markdown("---")
    st.subheader("💓 心跳包")
    if st.button("📡 获取最新心跳"):
        get_next_heartbeat()
        st.rerun()
    if st.session_state.heartbeat_history:
        cur = st.session_state.heartbeat_history[0]
        st.metric("无人机位置", f"{cur['lng']:.5f}, {cur['lat']:.5f}")
        st.caption(f"高度: {cur['alt']}m | {cur['timestamp']}")
    
    st.markdown("---")
    st.subheader("🛑 障碍物配置")
    col1, col2 = st.columns(2)
    with col1:
        if st.button("💾 保存到文件"): save_polygons_to_file()
    with col2:
        if st.button("📂 从文件加载"): load_polygons_from_file()
    col3, col4 = st.columns(2)
    with col3:
        if st.button("🗑️ 清除全部"): clear_all_obstacles()
    with col4:
        if st.button("🚀 一键部署"): one_click_deploy()
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "rb") as f:
            st.download_button("📥 下载配置", data=f, file_name="obstacle_config.json", mime="application/json")
    st.info(f"当前障碍物数量: {len(st.session_state.polygons)}")
    st.caption("提示：如果地图点击无效，请使用手动输入。")

# ==================== 构建地图 ====================
# 计算中心点（WGS84）
if st.session_state.A_gcj and st.session_state.B_gcj:
    center_gcj = ((st.session_state.A_gcj[0]+st.session_state.B_gcj[0])/2, (st.session_state.A_gcj[1]+st.session_state.B_gcj[1])/2)
elif st.session_state.A_gcj:
    center_gcj = st.session_state.A_gcj
elif st.session_state.B_gcj:
    center_gcj = st.session_state.B_gcj
else:
    center_gcj = (118.7492, 32.2332)
center_wgs = gcj02_to_wgs84(center_gcj[0], center_gcj[1])

m = folium.Map(location=[center_wgs[1], center_wgs[0]], zoom_start=17,
               tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
               attr='Esri World Imagery')
folium.TileLayer('openstreetmap', opacity=0.5).add_to(m)

# 添加 A/B/无人机/航线/障碍物
if st.session_state.A_gcj:
    a_wgs = gcj02_to_wgs84(st.session_state.A_gcj[0], st.session_state.A_gcj[1])
    folium.Marker([a_wgs[1], a_wgs[0]], popup='起点 A', icon=folium.Icon(color='green')).add_to(m)
if st.session_state.B_gcj:
    b_wgs = gcj02_to_wgs84(st.session_state.B_gcj[0], st.session_state.B_gcj[1])
    folium.Marker([b_wgs[1], b_wgs[0]], popup='终点 B', icon=folium.Icon(color='red')).add_to(m)
if st.session_state.heartbeat_history:
    drone_gcj = (st.session_state.heartbeat_history[0]['lng'], st.session_state.heartbeat_history[0]['lat'])
    drone_wgs = gcj02_to_wgs84(drone_gcj[0], drone_gcj[1])
    folium.Marker([drone_wgs[1], drone_wgs[0]], popup='无人机', icon=folium.Icon(color='blue', icon='plane', prefix='fa')).add_to(m)
if st.session_state.A_gcj and st.session_state.B_gcj:
    a_wgs = gcj02_to_wgs84(st.session_state.A_gcj[0], st.session_state.A_gcj[1])
    b_wgs = gcj02_to_wgs84(st.session_state.B_gcj[0], st.session_state.B_gcj[1])
    folium.PolyLine([[a_wgs[1], a_wgs[0]], [b_wgs[1], b_wgs[0]]], color='blue', weight=3).add_to(m)
for poly_gcj in st.session_state.polygons:
    wgs_poly = [gcj02_to_wgs84(lng, lat)[::-1] for lng, lat in poly_gcj]  # 转换为 [lat, lng]
    folium.Polygon(wgs_poly, color='red', fill=True, fill_opacity=0.4, weight=2).add_to(m)

# 添加绘图控件
draw = Draw(draw_options={'polygon': {'allowIntersection': False, 'showArea': True}, 'polyline': False, 'rectangle': False, 'circle': False, 'marker': False, 'circlemarker': False},
            edit_options={'edit': True, 'remove': True})
draw.add_to(m)

# 显示地图并捕获交互
output = st_folium(m, width=1200, height=600, key="map_with_draw", returned_objects=["last_click", "last_draw"])

# ==================== 处理点击事件（调试版） ====================
if output and output.get("last_click"):
    click = output["last_click"]
    st.sidebar.write("🔍 调试：点击返回数据:", click)  # 显示原始数据，帮助诊断
    if click and "lng" in click and "lat" in click:
        lng_wgs = click["lng"]
        lat_wgs = click["lat"]
        gcj_lng, gcj_lat = wgs84_to_gcj02(lng_wgs, lat_wgs)
        if st.session_state.set_mode == "起点A":
            st.session_state.A_gcj = (gcj_lng, gcj_lat)
            st.sidebar.success(f"起点A已设置 (GCJ-02): {gcj_lng:.6f}, {gcj_lat:.6f}")
        else:
            st.session_state.B_gcj = (gcj_lng, gcj_lat)
            st.sidebar.success(f"终点B已设置 (GCJ-02): {gcj_lng:.6f}, {gcj_lat:.6f}")
        st.rerun()
    else:
        st.sidebar.warning("点击坐标数据不完整，请重试")

# 处理多边形绘制
if output and output.get("last_draw"):
    draw_data = output["last_draw"]
    if draw_data and draw_data.get("geometry", {}).get("type") == "Polygon":
        coords_wgs = draw_data["geometry"]["coordinates"][0]
        poly_gcj = [wgs84_to_gcj02(lng, lat) for lng, lat in coords_wgs]
        if len(poly_gcj) >= 3:
            st.session_state.polygons.append(poly_gcj)
            st.sidebar.success("新障碍物已添加！")
            st.rerun()

st.caption("✅ 使用说明：\n1. 左侧选择起点A/终点B，然后在地图上单击（若无效，请使用手动输入）。\n2. 使用多边形工具绘制障碍物。\n3. 心跳包点击获取最新位置。")

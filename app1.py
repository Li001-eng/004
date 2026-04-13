import streamlit as st
import folium
from streamlit_folium import folium_static
import numpy as np
import json
import os
from datetime import datetime

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
def update_heartbeat():
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

def save_polygons():
    data = {
        "version": "v12.2",
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "polygons": st.session_state.polygons
    }
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    st.success(f"已保存 {len(st.session_state.polygons)} 个障碍物到 {os.path.abspath(CONFIG_FILE)}")

def load_polygons():
    if not os.path.exists(CONFIG_FILE):
        st.error("配置文件不存在，请先保存")
        return
    with open(CONFIG_FILE, "r", encoding="utf-8") as f:
        data = json.load(f)
    st.session_state.polygons = data.get("polygons", [])
    st.success(f"已加载 {len(st.session_state.polygons)} 个障碍物")

def clear_polygons():
    st.session_state.polygons = []
    st.success("已清除所有障碍物")

# ==================== 主界面 ====================
st.set_page_config(layout="wide", page_title="无人机障碍物规划 - 稳定手动版")
st.title("✈️ 校园无人机飞行规划与实时监控")
st.markdown("**卫星地图 + GCJ-02坐标** | **手动输入障碍物多边形（100%可靠）**")

# 初始化状态
if 'polygons' not in st.session_state:
    st.session_state.polygons = []
if 'A_gcj' not in st.session_state:
    st.session_state.A_gcj = None
if 'B_gcj' not in st.session_state:
    st.session_state.B_gcj = None
if 'flight_height' not in st.session_state:
    st.session_state.flight_height = 50
if 'drone_pos_gcj' not in st.session_state:
    st.session_state.drone_pos_gcj = (118.7492, 32.2328)
if 'heartbeat_history' not in st.session_state:
    st.session_state.heartbeat_history = []

# ==================== 侧边栏 ====================
with st.sidebar:
    st.header("🎮 控制面板")
    
    # 起点A手动输入
    st.subheader("📍 起点A (GCJ-02)")
    col1, col2 = st.columns(2)
    with col1:
        a_lat = st.number_input("纬度", value=32.2323, format="%.6f", key="a_lat")
    with col2:
        a_lng = st.number_input("经度", value=118.7490, format="%.6f", key="a_lng")
    if st.button("设置A点"):
        st.session_state.A_gcj = (a_lng, a_lat)
        st.success(f"A点: ({a_lng}, {a_lat})")
    
    # 终点B手动输入
    st.subheader("🏁 终点B (GCJ-02)")
    col3, col4 = st.columns(2)
    with col3:
        b_lat = st.number_input("纬度", value=32.2344, format="%.6f", key="b_lat")
    with col4:
        b_lng = st.number_input("经度", value=118.7490, format="%.6f", key="b_lng")
    if st.button("设置B点"):
        st.session_state.B_gcj = (b_lng, b_lat)
        st.success(f"B点: ({b_lng}, {b_lat})")
    
    # 显示当前坐标
    st.markdown("---")
    if st.session_state.A_gcj:
        st.info(f"起点A: {st.session_state.A_gcj[0]:.6f}, {st.session_state.A_gcj[1]:.6f}")
    if st.session_state.B_gcj:
        st.info(f"终点B: {st.session_state.B_gcj[0]:.6f}, {st.session_state.B_gcj[1]:.6f}")
    
    # 飞行参数
    st.subheader("🚁 飞行参数")
    st.session_state.flight_height = st.number_input("设定飞行高度 (m)", value=st.session_state.flight_height, step=5)
    
    # 心跳包
    st.subheader("💓 心跳包")
    if st.button("📡 获取最新心跳"):
        update_heartbeat()
        st.rerun()
    if st.session_state.heartbeat_history:
        cur = st.session_state.heartbeat_history[0]
        st.metric("无人机位置 (GCJ-02)", f"{cur['lng']:.5f}, {cur['lat']:.5f}")
        st.caption(f"高度: {cur['alt']}m | {cur['timestamp']}")
    
    # 障碍物配置
    st.subheader("🛑 障碍物配置")
    
    # 手动添加障碍物多边形
    st.markdown("**添加多边形（GCJ-02坐标）**")
    st.markdown("每行一个顶点：经度,纬度（至少3行）")
    poly_input = st.text_area("多边形顶点", height=150, 
                              placeholder="118.7485,32.2325\n118.7490,32.2327\n118.7488,32.2330")
    if st.button("➕ 添加障碍物"):
        try:
            coords = []
            for line in poly_input.strip().split('\n'):
                if line.strip():
                    lng, lat = map(float, line.split(','))
                    coords.append([lng, lat])
            if len(coords) >= 3:
                st.session_state.polygons.append(coords)
                st.success(f"已添加障碍物，当前总数: {len(st.session_state.polygons)}")
            else:
                st.error("至少需要3个顶点")
        except Exception as e:
            st.error(f"格式错误: {e}")
    
    # 障碍物列表显示
    if st.session_state.polygons:
        st.write(f"当前障碍物数量: {len(st.session_state.polygons)}")
        for i, poly in enumerate(st.session_state.polygons):
            st.caption(f"多边形 {i+1}: {len(poly)} 个顶点")
    
    col_btn1, col_btn2 = st.columns(2)
    with col_btn1:
        if st.button("💾 保存到文件"):
            save_polygons()
    with col_btn2:
        if st.button("📂 从文件加载"):
            load_polygons()
    col_btn3, col_btn4 = st.columns(2)
    with col_btn3:
        if st.button("🗑️ 清除全部"):
            clear_polygons()
    with col_btn4:
        if st.button("🚀 一键部署"):
            st.session_state.polygons = [
                [[118.7485, 32.2325], [118.7490, 32.2327], [118.7488, 32.2330]],
                [[118.7495, 32.2335], [118.7500, 32.2332], [118.7498, 32.2338]]
            ]
            st.success("已部署预设障碍物")
    
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "rb") as f:
            st.download_button("📥 下载配置文件", data=f, file_name="obstacle_config.json", mime="application/json")

# ==================== 地图显示 ====================
# 计算中心点
if st.session_state.A_gcj and st.session_state.B_gcj:
    center_gcj = ((st.session_state.A_gcj[0]+st.session_state.B_gcj[0])/2,
                  (st.session_state.A_gcj[1]+st.session_state.B_gcj[1])/2)
elif st.session_state.A_gcj:
    center_gcj = st.session_state.A_gcj
elif st.session_state.B_gcj:
    center_gcj = st.session_state.B_gcj
else:
    center_gcj = (118.7492, 32.2332)

center_wgs = gcj02_to_wgs84(center_gcj[0], center_gcj[1])

m = folium.Map(location=[center_wgs[1], center_wgs[0]], zoom_start=17,
               tiles='https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
               attr='Google Satellite')
folium.TileLayer('openstreetmap', opacity=0.5).add_to(m)

# 添加A点
if st.session_state.A_gcj:
    a_wgs = gcj02_to_wgs84(st.session_state.A_gcj[0], st.session_state.A_gcj[1])
    folium.Marker([a_wgs[1], a_wgs[0]], popup='起点 A', icon=folium.Icon(color='green')).add_to(m)
# 添加B点
if st.session_state.B_gcj:
    b_wgs = gcj02_to_wgs84(st.session_state.B_gcj[0], st.session_state.B_gcj[1])
    folium.Marker([b_wgs[1], b_wgs[0]], popup='终点 B', icon=folium.Icon(color='red')).add_to(m)
# 添加无人机
if st.session_state.heartbeat_history:
    drone_gcj = (st.session_state.heartbeat_history[0]['lng'], st.session_state.heartbeat_history[0]['lat'])
    drone_wgs = gcj02_to_wgs84(drone_gcj[0], drone_gcj[1])
    folium.Marker([drone_wgs[1], drone_wgs[0]], popup='无人机', icon=folium.Icon(color='blue', icon='plane', prefix='fa')).add_to(m)
# 添加航线
if st.session_state.A_gcj and st.session_state.B_gcj:
    a_wgs = gcj02_to_wgs84(st.session_state.A_gcj[0], st.session_state.A_gcj[1])
    b_wgs = gcj02_to_wgs84(st.session_state.B_gcj[0], st.session_state.B_gcj[1])
    folium.PolyLine([[a_wgs[1], a_wgs[0]], [b_wgs[1], b_wgs[0]]], color='blue', weight=3).add_to(m)
# 添加所有障碍物
for poly_gcj in st.session_state.polygons:
    wgs_poly = []
    for lng, lat in poly_gcj:
        wlng, wlat = gcj02_to_wgs84(lng, lat)
        wgs_poly.append([wlat, wlng])
    folium.Polygon(wgs_poly, color='red', fill=True, fill_opacity=0.4, weight=2).add_to(m)

folium_static(m, width=1200, height=600)

st.caption("✅ 使用说明：\n"
           "1. **设置起点/终点**：在侧边栏手动输入GCJ-02经纬度，点击对应按钮。\n"
           "2. **添加障碍物**：在侧边栏文本框内输入多边形顶点（每行 经度,纬度），至少3行，点击「添加障碍物」。\n"
           "3. **保存/加载**：点击「保存到文件」生成JSON，下次启动点击「从文件加载」恢复。\n"
           "4. **心跳包**：点击「获取最新心跳」模拟无人机向B点移动。\n"
           "5. 地图显示自动将GCJ-02坐标转换为WGS84，确保位置准确。")

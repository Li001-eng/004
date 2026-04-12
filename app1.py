import streamlit as st
import folium
from streamlit_folium import folium_static
import numpy as np
import json
import os
from datetime import datetime

# ==================== GCJ-02 转 WGS84 ====================
def gcj02_to_wgs84(lng, lat):
    a = 6378245.0
    ee = 0.00669342162296594323
    def transform_lat(x, y):
        ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * np.sqrt(abs(x))
        ret += (20.0 * np.sin(6.0 * x * np.pi) + 20.0 * np.sin(2.0 * x * np.pi)) * 2.0 / 3.0
        ret += (20.0 * np.sin(y * np.pi) + 40.0 * np.sin(y / 3.0 * np.pi)) * 2.0 / 3.0
        ret += (160.0 * np.sin(y / 12.0 * np.pi) + 320 * np.sin(y * np.pi / 30.0)) * 2.0 / 3.0
        return ret
    def transform_lng(x, y):
        ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * np.sqrt(abs(x))
        ret += (20.0 * np.sin(6.0 * x * np.pi) + 20.0 * np.sin(2.0 * x * np.pi)) * 2.0 / 3.0
        ret += (20.0 * np.sin(x * np.pi) + 40.0 * np.sin(x / 3.0 * np.pi)) * 2.0 / 3.0
        ret += (150.0 * np.sin(x / 12.0 * np.pi) + 300.0 * np.sin(x / 30.0 * np.pi)) * 2.0 / 3.0
        return ret
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * np.pi
    magic = np.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = np.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * np.pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * np.cos(radlat) * np.pi)
    wgs_lat = lat - dlat
    wgs_lng = lng - dlng
    return wgs_lng, wgs_lat

# ==================== 模拟心跳包 ====================
def get_heartbeat():
    if 'drone_pos_gcj' not in st.session_state:
        st.session_state.drone_pos_gcj = (118.7492, 32.2328)
    # 简单向B点移动（若存在）
    if st.session_state.get('B'):
        target_lng, target_lat = st.session_state.B
        cur_lng, cur_lat = st.session_state.drone_pos_gcj
        dx = (target_lng - cur_lng) * 0.02
        dy = (target_lat - cur_lat) * 0.02
        if abs(dx) > 0.00001 or abs(dy) > 0.00001:
            st.session_state.drone_pos_gcj = (cur_lng + dx, cur_lat + dy)
    return {"lng": st.session_state.drone_pos_gcj[0],
            "lat": st.session_state.drone_pos_gcj[1],
            "alt": st.session_state.get('flight_height', 50)}

# ==================== 配置文件路径 ====================
CONFIG_FILE = "obstacle_config.json"

def save_polygons_to_file():
    """保存当前障碍物列表到JSON文件"""
    data = {
        "version": "v12.2",
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "polygons": st.session_state.polygons
    }
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    st.session_state.last_save_time = data["timestamp"]
    st.success(f"已保存到 {os.path.abspath(CONFIG_FILE)}")

def load_polygons_from_file():
    """从JSON文件加载障碍物列表"""
    if not os.path.exists(CONFIG_FILE):
        st.error("配置文件不存在，请先保存")
        return
    with open(CONFIG_FILE, "r", encoding="utf-8") as f:
        data = json.load(f)
    st.session_state.polygons = data.get("polygons", [])
    st.session_state.last_save_time = data.get("timestamp", "未知")
    st.success(f"已加载 {len(st.session_state.polygons)} 个障碍物，版本 {data.get('version', '未知')}")

def clear_all_obstacles():
    st.session_state.polygons = []
    st.success("已清除所有障碍物")

def one_click_deploy():
    """一键部署：加载预设示例障碍物（校园内典型区域）"""
    example_polygons = [
        [[118.7485, 32.2325], [118.7490, 32.2327], [118.7488, 32.2330]],
        [[118.7495, 32.2335], [118.7500, 32.2332], [118.7498, 32.2338]],
        [[118.7480, 32.2340], [118.7485, 32.2342], [118.7482, 32.2345]]
    ]
    st.session_state.polygons = example_polygons
    st.success("已部署预设障碍物（3个示例多边形）")

# ==================== Streamlit 主界面 ====================
st.set_page_config(layout="wide", page_title="无人机障碍物规划系统")
st.title("✈️ 校园无人机飞行规划与实时监控")
st.markdown("卫星地图 | GCJ-02坐标自动转换 | 障碍物持久化（JSON文件）")

# ---------- 初始化 session_state ----------
if 'polygons' not in st.session_state:
    st.session_state.polygons = []
if 'A' not in st.session_state:
    st.session_state.A = None
if 'B' not in st.session_state:
    st.session_state.B = None
if 'flight_height' not in st.session_state:
    st.session_state.flight_height = 50
if 'last_save_time' not in st.session_state:
    st.session_state.last_save_time = None

# ---------- 侧边栏：飞行参数与障碍物管理 ----------
with st.sidebar:
    st.header("🎮 控制面板")
    
    # 起点A
    st.subheader("📍 起点A (GCJ-02)")
    col1, col2 = st.columns(2)
    with col1:
        lat_a = st.number_input("纬度", value=32.2323, format="%.6f", key="lat_a")
    with col2:
        lng_a = st.number_input("经度", value=118.7490, format="%.6f", key="lng_a")
    if st.button("设置A点"):
        st.session_state.A = (lng_a, lat_a)
        st.success(f"A点: ({lng_a}, {lat_a})")
    
    # 终点B
    st.subheader("🏁 终点B (GCJ-02)")
    col3, col4 = st.columns(2)
    with col3:
        lat_b = st.number_input("纬度", value=32.2344, format="%.6f", key="lat_b")
    with col4:
        lng_b = st.number_input("经度", value=118.7490, format="%.6f", key="lng_b")
    if st.button("设置B点"):
        st.session_state.B = (lng_b, lat_b)
        st.success(f"B点: ({lng_b}, {lat_b})")
    
    # 飞行高度
    st.subheader("🚁 飞行参数")
    height = st.number_input("设定飞行高度 (m)", value=st.session_state.flight_height, step=5)
    st.session_state.flight_height = height
    
    st.markdown("---")
    
    # 障碍物管理区域（持久化功能）
    st.subheader("🛑 障碍物配置持久化")
    st.caption(f"配置文件: `{os.path.abspath(CONFIG_FILE)}` | 版本: v12.2 障碍物持久化版")
    
    col_btn1, col_btn2 = st.columns(2)
    with col_btn1:
        if st.button("💾 保存到文件"):
            save_polygons_to_file()
    with col_btn2:
        if st.button("📂 从文件加载"):
            load_polygons_from_file()
    
    col_btn3, col_btn4 = st.columns(2)
    with col_btn3:
        if st.button("🗑️ 清除全部"):
            clear_all_obstacles()
    with col_btn4:
        if st.button("🚀 一键部署"):
            one_click_deploy()
    
    st.markdown("---")
    st.subheader("📥 下载配置文件到本地")
    # 提供下载按钮（如果文件存在）
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "rb") as f:
            file_bytes = f.read()
        st.download_button(
            label="⬇️ 下载 obstacle_config.json",
            data=file_bytes,
            file_name="obstacle_config.json",
            mime="application/json"
        )
    else:
        st.info("尚未保存配置文件，请先点击「保存到文件」")
    
    # 显示文件状态
    st.markdown("---")
    st.subheader("📄 文件状态")
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        num = len(data.get("polygons", []))
        save_time = data.get("timestamp", "未知")
        version = data.get("version", "未知")
        st.success(f"共 {num} 个障碍物 | 保存时间: {save_time} | 版本: {version}")
    else:
        st.warning("暂无配置文件，保存后将显示状态")
    
    # 手动添加障碍物（多边形顶点输入）
    st.markdown("---")
    st.subheader("✏️ 手动添加障碍物")
    st.markdown("每行一个顶点：经度,纬度 (GCJ-02)")
    polygon_input = st.text_area("多边形顶点", height=150,
                                 placeholder="118.7485,32.2325\n118.7490,32.2327\n118.7488,32.2330")
    if st.button("➕ 添加障碍物"):
        try:
            coords = []
            for line in polygon_input.strip().split('\n'):
                if line.strip():
                    lng, lat = map(float, line.split(','))
                    coords.append([lng, lat])
            if len(coords) >= 3:
                st.session_state.polygons.append(coords)
                st.success("障碍物已添加")
            else:
                st.error("至少需要3个点")
        except Exception as e:
            st.error(f"格式错误: {e}")

# ---------- 心跳包数据 ----------
heartbeat = get_heartbeat()
drone_pos_gcj = (heartbeat["lng"], heartbeat["lat"])

# 侧边栏显示心跳信息
st.sidebar.metric("最新心跳", f"经度 {heartbeat['lng']:.5f} | 纬度 {heartbeat['lat']:.5f} | 高度 {heartbeat['alt']}m")

# ---------- 确定地图中心点 ----------
if st.session_state.A and st.session_state.B:
    center_gcj = ((st.session_state.A[0] + st.session_state.B[0]) / 2,
                  (st.session_state.A[1] + st.session_state.B[1]) / 2)
elif st.session_state.A:
    center_gcj = st.session_state.A
elif st.session_state.B:
    center_gcj = st.session_state.B
else:
    center_gcj = (118.7492, 32.2332)  # 校园默认中心

center_wgs = gcj02_to_wgs84(center_gcj[0], center_gcj[1])

# ---------- 创建 Folium 地图 ----------
m = folium.Map(
    location=[center_wgs[1], center_wgs[0]],
    zoom_start=17,
    tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
    attr='Esri World Imagery'
)
# 叠加半透明 OpenStreetMap 标注层
folium.TileLayer('openstreetmap', opacity=0.5).add_to(m)

# 添加 A/B 点
if st.session_state.A:
    a_wgs = gcj02_to_wgs84(st.session_state.A[0], st.session_state.A[1])
    folium.Marker([a_wgs[1], a_wgs[0]], popup='起点 A', icon=folium.Icon(color='green')).add_to(m)
if st.session_state.B:
    b_wgs = gcj02_to_wgs84(st.session_state.B[0], st.session_state.B[1])
    folium.Marker([b_wgs[1], b_wgs[0]], popup='终点 B', icon=folium.Icon(color='red')).add_to(m)

# 添加无人机位置
drone_wgs = gcj02_to_wgs84(drone_pos_gcj[0], drone_pos_gcj[1])
folium.Marker([drone_wgs[1], drone_wgs[0]], popup='无人机', icon=folium.Icon(color='blue', icon='plane', prefix='fa')).add_to(m)

# 添加航线
if st.session_state.A and st.session_state.B:
    a_wgs = gcj02_to_wgs84(st.session_state.A[0], st.session_state.A[1])
    b_wgs = gcj02_to_wgs84(st.session_state.B[0], st.session_state.B[1])
    folium.PolyLine([[a_wgs[1], a_wgs[0]], [b_wgs[1], b_wgs[0]]], color='blue', weight=3).add_to(m)

# 添加所有障碍物（多边形）
for poly in st.session_state.polygons:
    # poly 格式: [[lng, lat], ...] GCJ-02
    wgs_poly = []
    for lng, lat in poly:
        wgs_lng, wgs_lat = gcj02_to_wgs84(lng, lat)
        wgs_poly.append([wgs_lat, wgs_lng])
    folium.Polygon(wgs_poly, color='red', fill=True, fill_opacity=0.4, weight=2).add_to(m)

# ---------- 显示地图 ----------
folium_static(m, width=1200, height=600)

# 底部提示
st.caption("Leaflet | Esri World Imagery | 坐标系统: GCJ-02 输入自动转 WGS84 显示")

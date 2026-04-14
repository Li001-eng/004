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
        "alt": st.session_state.flight_height
    }
    st.session_state.heartbeat_history.insert(0, heartbeat)
    st.session_state.heartbeat_history = st.session_state.heartbeat_history[:5]
    return heartbeat

# ==================== 障碍物持久化 ====================
CONFIG_FILE = "obstacle_config.json"

def save_polygons():
    """保存障碍物到文件（直接存储 GCJ-02 坐标列表）"""
    data = {
        "version": "v12.2",
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "polygons": st.session_state.polygons
    }
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    st.success(f"已保存 {len(st.session_state.polygons)} 个障碍物")

def load_polygons():
    """从文件加载障碍物"""
    if not os.path.exists(CONFIG_FILE):
        st.error("配置文件不存在，请先保存")
        return
    with open(CONFIG_FILE, "r", encoding="utf-8") as f:
        data = json.load(f)
    st.session_state.polygons = data.get("polygons", [])
    st.success(f"已加载 {len(st.session_state.polygons)} 个障碍物")

def clear_polygons():
    st.session_state.polygons = []
    save_polygons()  # 清空后也保存到文件
    st.success("已清除所有障碍物并保存")

# ==================== 主界面 ====================
st.set_page_config(layout="wide", page_title="无人机障碍物规划系统 - 自动保存")
st.title("✈️ 校园无人机飞行规划与实时监控")
st.markdown("**卫星地图 | GCJ-02坐标自动转换 | 障碍物持久化 | 心跳包手动刷新**")
st.markdown("🎯 **地图圈选障碍物后自动保存** | 也可手动输入坐标")

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
if 'manual_polygon_text' not in st.session_state:
    st.session_state.manual_polygon_text = ""

# ==================== 侧边栏布局 ====================
with st.sidebar:
    st.header("控制面板")
    
    # ----- 获取最新心跳按钮 -----
    if st.button("📡 获取最新心跳", key="heartbeat", use_container_width=True):
        update_heartbeat()
        st.rerun()
    
    # ----- 最新心跳包卡片（仪表盘）-----
    st.subheader("最新心跳包")
    if st.session_state.heartbeat_history:
        cur = st.session_state.heartbeat_history[0]
        col1, col2 = st.columns(2)
        col1.metric("经度 (GCJ-02)", f"{cur['lng']:.6f}")
        col2.metric("纬度 (GCJ-02)", f"{cur['lat']:.6f}")
        col3, col4 = st.columns(2)
        col3.metric("飞行高度 (m)", f"{cur['alt']}")
        col4.metric("时间戳", cur['timestamp'])
    else:
        st.info("暂无心跳数据，请点击「获取最新心跳」")
    
    # ----- 起点A -----
    st.subheader("起点A (GCJ-02)")
    col1, col2 = st.columns(2)
    with col1:
        a_lat = st.number_input("纬度", value=32.2323, format="%.6f", key="a_lat")
    with col2:
        a_lng = st.number_input("经度", value=118.7490, format="%.6f", key="a_lng")
    if st.button("设置A点", key="set_A", use_container_width=True):
        st.session_state.A_gcj = (a_lng, a_lat)
        st.rerun()
    
    # ----- 终点B -----
    st.subheader("终点B (GCJ-02)")
    col1, col2 = st.columns(2)
    with col1:
        b_lat = st.number_input("纬度", value=32.2344, format="%.6f", key="b_lat")
    with col2:
        b_lng = st.number_input("经度", value=118.7490, format="%.6f", key="b_lng")
    if st.button("设置B点", key="set_B", use_container_width=True):
        st.session_state.B_gcj = (b_lng, b_lat)
        st.rerun()
    
    # ----- 飞行参数 -----
    st.subheader("飞行参数")
    st.session_state.flight_height = st.number_input("设定飞行高度 (m)", value=st.session_state.flight_height, step=5, key="flight_height_input")
    
    # ----- 心跳包历史 -----
    st.subheader("心跳包历史（最近5次）")
    if st.session_state.heartbeat_history:
        for hb in st.session_state.heartbeat_history:
            st.caption(f"{hb['timestamp']}  Lng:{hb['lng']:.5f}  Lat:{hb['lat']:.5f}  Alt:{hb['alt']}m")
    else:
        st.caption("暂无历史")
    
    st.markdown("---")
    
    # ----- 障碍物配置持久化 -----
    st.subheader("障碍物配置持久化")
    st.caption(f"配置文件: `{os.path.abspath(CONFIG_FILE)}` | 版本: v12.2")
    col1, col2 = st.columns(2)
    with col1:
        if st.button("💾 保存到文件", key="save", use_container_width=True):
            save_polygons()
    with col2:
        if st.button("📂 从文件加载", key="load", use_container_width=True):
            load_polygons()
    col1, col2 = st.columns(2)
    with col1:
        if st.button("🗑️ 清除全部", key="clear", use_container_width=True):
            clear_polygons()
            st.rerun()
    with col2:
        if st.button("🚀 一键部署", key="deploy", use_container_width=True):
            st.session_state.polygons = [
                [[118.7485, 32.2325], [118.7490, 32.2327], [118.7488, 32.2330]],
                [[118.7495, 32.2335], [118.7500, 32.2332], [118.7498, 32.2338]]
            ]
            save_polygons()
            st.success("已部署预设障碍物并保存")
            st.rerun()
    
    # 下载配置文件
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "rb") as f:
            st.download_button("📥 下载配置文件", data=f, file_name="obstacle_config.json", mime="application/json", key="download", use_container_width=True)
    
    st.markdown("---")
    
    # ----- 手动输入障碍物（备用）-----
    st.subheader("✏️ 手动添加障碍物 (GCJ-02)")
    manual_input = st.text_area("多边形顶点（每行 经度,纬度）", height=120, key="manual_input",
                                value=st.session_state.manual_polygon_text,
                                placeholder="118.7485,32.2325\n118.7490,32.2327\n118.7488,32.2330")
    st.session_state.manual_polygon_text = manual_input
    if st.button("➕ 手动添加并保存", key="add_manual", use_container_width=True):
        try:
            lines = manual_input.strip().split('\n')
            coords = []
            for line in lines:
                if line.strip():
                    lng, lat = map(float, line.split(','))
                    coords.append([lng, lat])
            if len(coords) >= 3:
                st.session_state.polygons.append(coords)
                save_polygons()  # 立即保存
                st.success(f"已添加手动障碍物，当前总数: {len(st.session_state.polygons)}")
                st.session_state.manual_polygon_text = ""
                st.rerun()
            else:
                st.error("至少需要3个顶点")
        except Exception as e:
            st.error(f"格式错误: {e}")
    
    st.info(f"当前障碍物总数: {len(st.session_state.polygons)}")

# ==================== 地图显示 ====================
# 计算地图中心点 (WGS84)
if st.session_state.A_gcj and st.session_state.B_gcj:
    center_gcj = ((st.session_state.A_gcj[0] + st.session_state.B_gcj[0]) / 2,
                  (st.session_state.A_gcj[1] + st.session_state.B_gcj[1]) / 2)
elif st.session_state.A_gcj:
    center_gcj = st.session_state.A_gcj
elif st.session_state.B_gcj:
    center_gcj = st.session_state.B_gcj
else:
    center_gcj = (118.7492, 32.2332)
center_wgs_lng, center_wgs_lat = gcj02_to_wgs84(center_gcj[0], center_gcj[1])
center = [center_wgs_lat, center_wgs_lng]

# 创建底图（Esri 卫星图 + 半透明 OSM 标注）
m = folium.Map(location=center, zoom_start=17,
               tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
               attr='Esri World Imagery')
folium.TileLayer('openstreetmap', opacity=0.5).add_to(m)

# 添加起点A
if st.session_state.A_gcj:
    lng, lat = gcj02_to_wgs84(st.session_state.A_gcj[0], st.session_state.A_gcj[1])
    folium.Marker([lat, lng], popup='起点 A', icon=folium.Icon(color='green')).add_to(m)
# 添加终点B
if st.session_state.B_gcj:
    lng, lat = gcj02_to_wgs84(st.session_state.B_gcj[0], st.session_state.B_gcj[1])
    folium.Marker([lat, lng], popup='终点 B', icon=folium.Icon(color='red')).add_to(m)
# 添加无人机
if st.session_state.heartbeat_history:
    cur = st.session_state.heartbeat_history[0]
    lng, lat = gcj02_to_wgs84(cur['lng'], cur['lat'])
    folium.Marker([lat, lng], popup='无人机', icon=folium.Icon(color='blue', icon='plane', prefix='fa')).add_to(m)
# 添加航线
if st.session_state.A_gcj and st.session_state.B_gcj:
    a_lng, a_lat = gcj02_to_wgs84(st.session_state.A_gcj[0], st.session_state.A_gcj[1])
    b_lng, b_lat = gcj02_to_wgs84(st.session_state.B_gcj[0], st.session_state.B_gcj[1])
    folium.PolyLine([[a_lat, a_lng], [b_lat, b_lng]], color='blue', weight=3).add_to(m)
# 添加所有已保存的障碍物
for poly_gcj in st.session_state.polygons:
    wgs_poly = []
    for lng, lat in poly_gcj:
        wlng, wlat = gcj02_to_wgs84(lng, lat)
        wgs_poly.append([wlat, wlng])
    folium.Polygon(wgs_poly, color='red', fill=True, fill_opacity=0.4, weight=2).add_to(m)

# 添加绘图控件 (Draw)
draw = Draw(
    draw_options={
        'polygon': {'allowIntersection': False, 'showArea': True},
        'polyline': False,
        'rectangle': False,
        'circle': False,
        'marker': False,
        'circlemarker': False
    },
    edit_options={'edit': True, 'remove': True}
)
draw.add_to(m)

# 显示地图并捕获绘制事件
output = st_folium(m, width=1200, height=600, key="main_map", returned_objects=["last_draw"])

# ==================== 自动保存障碍物（核心功能）====================
# 当用户完成多边形绘制时，自动保存到文件并更新地图
if output and output.get("last_draw"):
    draw_data = output["last_draw"]
    if draw_data and draw_data.get("geometry", {}).get("type") == "Polygon":
        coords_wgs = draw_data["geometry"]["coordinates"][0]  # 外环坐标，WGS84
        if len(coords_wgs) >= 3:
            # 转换为 GCJ-02 存储
            coords_gcj = []
            for lng, lat in coords_wgs:
                gcj_lng, gcj_lat = wgs84_to_gcj02(lng, lat)
                coords_gcj.append([gcj_lng, gcj_lat])
            # 添加到 session_state
            st.session_state.polygons.append(coords_gcj)
            # 立即保存到文件
            save_polygons()
            # 显示提示信息
            st.sidebar.success(f"✅ 已自动添加并保存障碍物，当前总数: {len(st.session_state.polygons)}")
            # 刷新页面以显示新障碍物
            st.rerun()
        else:
            st.sidebar.warning("多边形顶点数不足3，未保存")

# 底部说明
st.caption("✅ 操作指南：\n"
           "1. **地图圈选**：使用左上角多边形工具绘制 → **自动保存**，无需额外操作。\n"
           "2. **手动输入**：在侧边栏文本框输入顶点（GCJ-02坐标，每行 经度,纬度）→ 点击「手动添加并保存」。\n"
           "3. 起点/终点通过手动输入经纬度设置。\n"
           "4. 所有障碍物自动保存到本地 JSON 文件，下次打开可「从文件加载」。")

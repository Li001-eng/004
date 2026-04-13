import streamlit as st
import folium
from streamlit_folium import st_folium
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

# 反向转换（WGS84 -> GCJ-02）这里简单近似，实际需要精确转换，但本应用不需要
# 注意：用户在地图上点击获取的是WGS84坐标，我们需要将其存储为GCJ-02
# 为了简化，我们假设校园范围内偏差是固定的，直接存储WGS84作为GCJ-02？不行，会导致偏差。
# 正确做法：将地图点击的WGS84坐标转换为GCJ-02后存储。
# 提供一个简单的反向转换函数（精确转换需要迭代，这里使用近似公式，误差很小）
def wgs84_to_gcj02(lng, lat):
    """WGS84 转 GCJ-02 (简化版，精度足够)"""
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
    return lng + dlng, lat + dlat

# ==================== 心跳包模拟 ====================
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
    data = {
        "version": "v12.2",
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "polygons": st.session_state.polygons  # 存储GCJ-02坐标
    }
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    st.success(f"已保存 {len(st.session_state.polygons)} 个障碍物到 {os.path.abspath(CONFIG_FILE)}")

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

# ==================== 主界面 ====================
st.set_page_config(layout="wide", page_title="无人机障碍物规划 - 交互式地图圈选")
st.title("✈️ 校园无人机飞行规划与实时监控")
st.markdown("**卫星地图** | GCJ-02坐标自动转换 | **点击地图设置A/B点** | **绘制多边形添加障碍物**")

# ---------- 初始化状态 ----------
if 'polygons' not in st.session_state:
    st.session_state.polygons = []  # 每个多边形为 [[lng,lat], ...] GCJ-02
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
if 'set_mode' not in st.session_state:
    st.session_state.set_mode = "起点A"  # 起点A / 终点B

# ---------- 侧边栏控制 ----------
with st.sidebar:
    st.header("🎮 控制面板")
    
    # 设置模式切换
    st.subheader("📍 地图点击模式")
    mode = st.radio("点击地图设置:", ["起点A", "终点B"], index=0 if st.session_state.set_mode=="起点A" else 1)
    st.session_state.set_mode = mode
    
    # 显示当前坐标
    if st.session_state.A_gcj:
        st.info(f"起点A (GCJ-02): {st.session_state.A_gcj[0]:.6f}, {st.session_state.A_gcj[1]:.6f}")
    if st.session_state.B_gcj:
        st.info(f"终点B (GCJ-02): {st.session_state.B_gcj[0]:.6f}, {st.session_state.B_gcj[1]:.6f}")
    
    st.markdown("---")
    
    # 飞行参数
    st.subheader("🚁 飞行参数")
    height = st.number_input("设定飞行高度 (m)", value=st.session_state.flight_height, step=5)
    st.session_state.flight_height = height
    
    st.markdown("---")
    
    # 心跳包控制
    st.subheader("💓 心跳包")
    if st.button("📡 获取最新心跳"):
        new_hb = get_next_heartbeat()
        st.success(f"心跳已更新: {new_hb['timestamp']} 经度={new_hb['lng']:.5f} 纬度={new_hb['lat']:.5f}")
    # 显示当前心跳
    if st.session_state.heartbeat_history:
        cur = st.session_state.heartbeat_history[0]
        st.metric("当前无人机位置 (GCJ-02)", f"{cur['lng']:.5f}, {cur['lat']:.5f}")
        st.caption(f"高度: {cur['alt']}m | 时间: {cur['timestamp']}")
    
    st.markdown("---")
    
    # 障碍物管理
    st.subheader("🛑 障碍物配置")
    col1, col2 = st.columns(2)
    with col1:
        if st.button("💾 保存到文件"):
            save_polygons_to_file()
    with col2:
        if st.button("📂 从文件加载"):
            load_polygons_from_file()
    col3, col4 = st.columns(2)
    with col3:
        if st.button("🗑️ 清除全部"):
            clear_all_obstacles()
    with col4:
        if st.button("📥 下载配置"):
            if os.path.exists(CONFIG_FILE):
                with open(CONFIG_FILE, "rb") as f:
                    st.download_button("点击下载", data=f, file_name="obstacle_config.json", mime="application/json")
            else:
                st.error("配置文件不存在")
    
    st.info(f"当前障碍物数量: {len(st.session_state.polygons)}")
    
    st.markdown("---")
    st.caption("提示：在地图上单击即可根据左侧模式设置起点/终点；使用绘制工具画多边形后自动添加为障碍物。")

# ---------- 创建地图（支持绘图和点击） ----------
# 确定地图中心点（WGS84）
if st.session_state.A_gcj and st.session_state.B_gcj:
    center_gcj = ((st.session_state.A_gcj[0] + st.session_state.B_gcj[0])/2,
                  (st.session_state.A_gcj[1] + st.session_state.B_gcj[1])/2)
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

# 添加现有的 A/B 点标记（WGS84）
if st.session_state.A_gcj:
    a_wgs = gcj02_to_wgs84(st.session_state.A_gcj[0], st.session_state.A_gcj[1])
    folium.Marker([a_wgs[1], a_wgs[0]], popup='起点 A', icon=folium.Icon(color='green')).add_to(m)
if st.session_state.B_gcj:
    b_wgs = gcj02_to_wgs84(st.session_state.B_gcj[0], st.session_state.B_gcj[1])
    folium.Marker([b_wgs[1], b_wgs[0]], popup='终点 B', icon=folium.Icon(color='red')).add_to(m)

# 添加无人机位置（WGS84）
if st.session_state.heartbeat_history:
    drone_gcj = (st.session_state.heartbeat_history[0]['lng'], st.session_state.heartbeat_history[0]['lat'])
    drone_wgs = gcj02_to_wgs84(drone_gcj[0], drone_gcj[1])
    folium.Marker([drone_wgs[1], drone_wgs[0]], popup='无人机', icon=folium.Icon(color='blue', icon='plane', prefix='fa')).add_to(m)

# 添加航线（如果A和B都存在）
if st.session_state.A_gcj and st.session_state.B_gcj:
    a_wgs = gcj02_to_wgs84(st.session_state.A_gcj[0], st.session_state.A_gcj[1])
    b_wgs = gcj02_to_wgs84(st.session_state.B_gcj[0], st.session_state.B_gcj[1])
    folium.PolyLine([[a_wgs[1], a_wgs[0]], [b_wgs[1], b_wgs[0]]], color='blue', weight=3).add_to(m)

# 添加现有障碍物（多边形，GCJ-02坐标需转WGS84显示）
for poly_gcj in st.session_state.polygons:
    wgs_poly = []
    for lng, lat in poly_gcj:
        wlng, wlat = gcj02_to_wgs84(lng, lat)
        wgs_poly.append([wlat, wlng])
    folium.Polygon(wgs_poly, color='red', fill=True, fill_opacity=0.4, weight=2).add_to(m)

# 添加绘图控件（Draw）
from folium.plugins import Draw
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

# 使用 st_folium 捕获地图交互数据
output = st_folium(m, width=1200, height=600, returned_objects=["last_click", "last_draw"])

# ---------- 处理地图交互 ----------
if output and output.get("last_click"):
    click_lat = output["last_click"]["lat"]
    click_lng = output["last_click"]["lng"]
    # 将 WGS84 点击坐标转换为 GCJ-02 存储
    gcj_lng, gcj_lat = wgs84_to_gcj02(click_lng, click_lat)
    if st.session_state.set_mode == "起点A":
        st.session_state.A_gcj = (gcj_lng, gcj_lat)
        st.success(f"起点A已设置 (GCJ-02): {gcj_lng:.6f}, {gcj_lat:.6f}")
    else:
        st.session_state.B_gcj = (gcj_lng, gcj_lat)
        st.success(f"终点B已设置 (GCJ-02): {gcj_lng:.6f}, {gcj_lat:.6f}")
    st.rerun()

if output and output.get("last_draw"):
    # 用户绘制了一个多边形
    draw_data = output["last_draw"]
    if draw_data.get("geometry", {}).get("type") == "Polygon":
        coords_wgs = draw_data["geometry"]["coordinates"][0]  # 外环坐标 [[lng, lat], ...]
        # 转换为 GCJ-02 存储
        poly_gcj = []
        for lng, lat in coords_wgs:
            gcj_lng, gcj_lat = wgs84_to_gcj02(lng, lat)
            poly_gcj.append([gcj_lng, gcj_lat])
        if len(poly_gcj) >= 3:
            st.session_state.polygons.append(poly_gcj)
            st.success("新障碍物已添加！")
            st.rerun()

# 底部说明
st.caption("✅ 操作指南：\n"
           "1. 左侧选择「起点A」或「终点B」，然后在地图上**单击**即可设置点。\n"
           "2. 使用地图左上角的**多边形工具**绘制障碍物区域，绘制完成后自动保存。\n"
           "3. 点击「获取最新心跳」模拟无人机向终点移动。\n"
           "4. 所有障碍物可保存/加载/清除。")

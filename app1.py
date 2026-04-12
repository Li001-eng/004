import streamlit as st
import folium
from streamlit_folium import folium_static
import numpy as np
import json

# ========== GCJ-02 <-> WGS84 转换（简化版，误差<1m） ==========
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

# ========== 模拟心跳包 ==========
def get_heartbeat():
    if 'drone_pos_gcj' not in st.session_state:
        st.session_state.drone_pos_gcj = (118.7492, 32.2328)
    # 简单移动逻辑（略）
    return {"lng": st.session_state.drone_pos_gcj[0], "lat": st.session_state.drone_pos_gcj[1], "alt": 50}

# ========== 主程序 ==========
st.set_page_config(layout="wide")
st.title("✈️ 校园无人机障碍物规划系统")
st.markdown("卫星地图 | GCJ-02 坐标自动转换 | 多边形障碍物记忆")

# 初始化
if 'polygons' not in st.session_state:
    st.session_state.polygons = []   # 每个多边形为 [[lng,lat], ...]
if 'A' not in st.session_state:
    st.session_state.A = None
if 'B' not in st.session_state:
    st.session_state.B = None

# 侧边栏
with st.sidebar:
    st.header("控制面板")
    col1, col2 = st.columns(2)
    with col1:
        lat_a = st.number_input("起点A 纬度", value=32.2322, format="%.6f")
    with col2:
        lng_a = st.number_input("起点A 经度", value=118.7490, format="%.6f")
    if st.button("设置A点"):
        st.session_state.A = (lng_a, lat_a)
    
    col3, col4 = st.columns(2)
    with col3:
        lat_b = st.number_input("终点B 纬度", value=32.2343, format="%.6f")
    with col4:
        lng_b = st.number_input("终点B 经度", value=118.7490, format="%.6f")
    if st.button("设置B点"):
        st.session_state.B = (lng_b, lat_b)
    
    st.subheader("障碍物多边形（GCJ-02坐标）")
    st.markdown("每行一个顶点：经度,纬度（例如：118.7488,32.2328）")
    polygon_input = st.text_area("多边形顶点", height=150, 
                                 placeholder="118.7488,32.2328\n118.7492,32.2330\n118.7485,32.2332")
    if st.button("添加障碍物"):
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
        except:
            st.error("格式错误，请使用 经度,纬度 每行")
    
    if st.button("清除所有障碍物"):
        st.session_state.polygons = []
        st.success("已清除")
    
    st.info(f"当前障碍物数量: {len(st.session_state.polygons)}")

# 心跳数据
heartbeat = get_heartbeat()
drone_pos_gcj = (heartbeat["lng"], heartbeat["lat"])

# 确定地图中心
if st.session_state.A and st.session_state.B:
    center_gcj = ((st.session_state.A[0]+st.session_state.B[0])/2, (st.session_state.A[1]+st.session_state.B[1])/2)
elif st.session_state.A:
    center_gcj = st.session_state.A
elif st.session_state.B:
    center_gcj = st.session_state.B
else:
    center_gcj = (118.7492, 32.2332)

# 转换中心点用于folium（WGS84）
center_wgs = gcj02_to_wgs84(center_gcj[0], center_gcj[1])
m = folium.Map(location=[center_wgs[1], center_wgs[0]], zoom_start=17, 
               tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
               attr='Esri World Imagery')
# 叠加OSM半透明标注层
folium.TileLayer('openstreetmap', opacity=0.5).add_to(m)

# 添加A,B点（转换坐标）
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

# 添加障碍物（转换坐标）
for poly in st.session_state.polygons:
    wgs_poly = [[gcj02_to_wgs84(lng, lat)[1], gcj02_to_wgs84(lng, lat)[0]] for lng, lat in poly]
    folium.Polygon(wgs_poly, color='red', fill=True, fill_opacity=0.4).add_to(m)

# 显示地图
folium_static(m, width=1000, height=600)

# 心跳信息显示
st.sidebar.metric("最新心跳", f"经度 {heartbeat['lng']:.5f} | 纬度 {heartbeat['lat']:.5f} | 高度 {heartbeat['alt']}m")

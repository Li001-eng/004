""")
st.caption("数据流：遥控指令 → 飞控 → 执行器 | 遥测数据 ← 飞控 ← 传感器")

st.markdown("---")

# 实时飞行地图（与原监控地图相同）
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

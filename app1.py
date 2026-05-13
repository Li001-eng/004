import streamlit as st
import folium
from streamlit_folium import st_folium, folium_static
from folium.plugins import Draw
import random, time, math, copy
from datetime import datetime

st.set_page_config(layout="wide")
st.title("🏫 无人机地面站系统 - 航点飞行（手动刷新版）")

# 常量
SCHOOL_CENTER = [118.7490, 32.2340]
A_DFT = [118.746956, 32.232945]
B_DFT = [118.751589, 32.235204]
SAT_URL = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
VEC_URL = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"
ATTR = "高德地图"

# 坐标转换（与之前相同，此处省略完整代码，实际运行时需补齐）
def gcj2wgs(lng, lat):
    # ... 完整实现（略）
    return lng, lat

def wgs2gcj(lng, lat):
    # ...
    return lng, lat

# 几何辅助函数（完整实现，此处省略）
def dist(p1, p2): return math.hypot(p1[0]-p2[0], p1[1]-p2[1])
def point_in_poly(p, poly):
    # ...
    pass
def lines_intersect(a,b,c,d):
    # ...
    pass
def line_cross_poly(p1,p2,poly):
    # ...
    pass
def seg_to_poly_dist(p1, p2, poly):
    # ...
    pass
def should_avoid(obs, h): return h <= obs.get('height',20)
def path_safe(p1,p2,obs,rad_m,h):
    # ...
    pass

# 绕行生成（完整实现，略）
def gen_bypass(A,B,obs,rad_m,h,side='left'):
    # ...
    return [A,B]

def plan_single_segment(A,B,obs,h,rad,strat):
    # ...
    return [A,B]

def plan_full_path(waypoints, obs, h, rad, strat):
    full = []
    for i in range(len(waypoints)-1):
        seg = plan_single_segment(waypoints[i], waypoints[i+1], obs, h, rad, strat)
        if i == 0:
            full.extend(seg)
        else:
            full.extend(seg[1:])
    return full

# ==================== 心跳模拟器（简化版，支持手动步进） ====================
class HeartbeatManual:
    def __init__(self, start):
        self.hist = []
        self.pos = start[:]
        self.path = [start[:]]
        self.idx = 0
        self.sim = False
        self.pause = False
        self.alt = 50
        self.spd = 50
        self.prog = 0
        self.total = 0
        self.trav = 0
        self.start_time = None
        self.elapsed = 0
    def set_path(self, path, alt, spd):
        self.path = path
        self.idx = 0
        self.pos = path[0][:]
        self.alt = alt
        self.spd = spd
        self.sim = True
        self.pause = False
        self.prog = 0
        self.trav = 0
        self.total = sum(dist(path[i], path[i+1]) for i in range(len(path)-1))
        self.start_time = time.time()
        self.elapsed = 0
    def reset(self):
        if self.path:
            self.pos = self.path[0][:]
        self.idx = 0
        self.sim = False
        self.pause = False
        self.prog = 0
        self.trav = 0
        self.start_time = None
        self.elapsed = 0
    def pause(self):
        self.pause = True
    def resume(self):
        self.pause = False
    def stop(self):
        self.sim = False
        self.pause = False
        self.start_time = None
    def step(self):
        """手动前进一步（0.2秒）"""
        if not self.sim or self.pause:
            return self._hb()
        if self.start_time:
            self.elapsed = time.time() - self.start_time
        if self.idx < len(self.path)-1:
            tar = self.path[self.idx+1]
            dx, dy = tar[0]-self.pos[0], tar[1]-self.pos[1]
            d2t = math.hypot(dx, dy)
            speed_mps = 0.5 + (self.spd / 100) * 4.5
            step_m = speed_mps * 0.2
            step_deg = step_m / 111000.0
            if d2t < step_deg:
                self.trav += d2t
                self.pos = tar[:]
                self.idx += 1
            else:
                r = step_deg / d2t
                self.pos[0] += dx * r
                self.pos[1] += dy * r
                self.trav += step_deg
            if self.total > 0:
                self.prog = min(1.0, self.trav / self.total)
            if self.idx >= len(self.path)-1:
                self.sim = False
                self.prog = 1.0
        else:
            self.sim = False
            self.prog = 1.0
        return self._hb()
    def _hb(self):
        speed = round(0.5 + (self.spd / 100) * 4.5, 1) if self.sim and not self.pause else 0
        battery = max(0, 100 - int(self.prog * 100))
        remain_dist = max(0, self.total - self.trav)
        remain_sec = remain_dist / (speed + 0.01) / 111000.0 * 3600 if speed > 0 else 0
        delay = round(random.uniform(10, 50), 1) if self.sim else 0
        loss = round(random.uniform(0, 0.2), 1) if self.sim else 0
        return {
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "lng": self.pos[0], "lat": self.pos[1],
            "altitude": self.alt + random.randint(-5,5) if self.sim else random.randint(0,10),
            "speed": speed,
            "progress": self.prog,
            "total": self.total,
            "traveled": self.trav,
            "current_wp": f"{self.idx+1}/{len(self.path)}",
            "remain": f"{int(remain_sec//60):02d}:{int(remain_sec%60):02d}",
            "battery": battery,
            "elapsed": self.elapsed,
            "delay_ms": delay,
            "loss_percent": loss,
            "simulating": self.sim,
            "paused": self.pause
        }

# 障碍物缓存等函数（与之前相同，略）
def save_cache(): pass
def load_cache(): return False
def add_safety(m, obs, rad, h): pass
def make_map(center, waypoints, obs, hist, full_path, maptype, rad, h):
    # 与之前相同，略
    return folium.Map()

# 主程序
def main():
    # 初始化状态
    if 'waypoints' not in st.session_state:
        st.session_state.waypoints = [A_DFT[:], B_DFT[:]]
    if 'obs' not in st.session_state: st.session_state.obs = []
    if 'hb' not in st.session_state: st.session_state.hb = HeartbeatManual(st.session_state.waypoints[0][:])
    if 'running' not in st.session_state: st.session_state.running = False
    if 'alt' not in st.session_state: st.session_state.alt = 50
    if 'hist' not in st.session_state: st.session_state.hist = []
    if 'full_path' not in st.session_state: st.session_state.full_path = None
    if 'pending_poly' not in st.session_state: st.session_state.pending_poly = None
    if 'pending_h' not in st.session_state: st.session_state.pending_h = 20
    if 'drone_spd' not in st.session_state: st.session_state.drone_spd = 50
    if 'safe_rad' not in st.session_state: st.session_state.safe_rad = 5
    if 'sel_strat' not in st.session_state: st.session_state.sel_strat = 'best'
    if 'new_wp_lng' not in st.session_state: st.session_state.new_wp_lng = A_DFT[0]
    if 'new_wp_lat' not in st.session_state: st.session_state.new_wp_lat = A_DFT[1]

    with st.sidebar:
        st.header("控制面板")
        page = st.radio("模块", ["规划","监控","障碍物"])
        map_type = "satellite" if st.radio("地图", ["卫星影像","矢量街道"]) == "卫星影像" else "vector"
        st.markdown("---")
        st.subheader("无人机参数")
        st.session_state.drone_spd = st.slider("速度系数",10,100,st.session_state.drone_spd)
        st.session_state.safe_rad = st.number_input("安全半径(米)",1,30,st.session_state.safe_rad)
        st.session_state.alt = st.number_input("飞行高度(米)",0,200,st.session_state.alt)
        st.markdown("---")
        st.subheader("绕行策略")
        strat = st.radio("避障方式",["最佳航线","向左绕行","向右绕行"])
        strat_map = {"最佳航线":"best","向左绕行":"left","向右绕行":"right"}
        st.session_state.sel_strat = strat_map[strat]
        st.info(f"障碍物: {len(st.session_state.obs)}")
        if st.session_state.full_path:
            st.success(f"✅ 路径已规划: {len(st.session_state.full_path)}个航段点")
        else:
            st.warning("⚠️ 路径未规划，请点击「刷新规划」")
        if st.button("刷新规划", use_container_width=True):
            with st.spinner("规划全航线中..."):
                st.session_state.full_path = plan_full_path(st.session_state.waypoints,
                                                            st.session_state.obs,
                                                            st.session_state.alt,
                                                            st.session_state.safe_rad,
                                                            st.session_state.sel_strat)
            st.rerun()

    if page == "规划":
        st.header("航线规划 - 多航点避障")
        st.info("📝 点击地图📐画多边形→设置高度→「添加障碍物」；下方可添加/删除航点（起点和终点固定）")
        col1,col2 = st.columns([1,1.5])
        with col1:
            st.markdown("#### 🗺️ 航点管理")
            # 起点
            st.markdown("**起点**")
            col_s = st.columns(2)
            with col_s[0]:
                a_lat = st.number_input("纬度", value=st.session_state.waypoints[0][1], format="%.6f", key="a_lat")
            with col_s[1]:
                a_lng = st.number_input("经度", value=st.session_state.waypoints[0][0], format="%.6f", key="a_lng")
            if st.button("更新起点"):
                st.session_state.waypoints[0] = [a_lng, a_lat]
                st.rerun()
            # 中间航点
            st.markdown("**中间航点**")
            if len(st.session_state.waypoints) > 2:
                for i in range(1, len(st.session_state.waypoints)-1):
                    col_wp = st.columns([3,1])
                    col_wp[0].write(f"航点{i}: ({st.session_state.waypoints[i][0]:.6f}, {st.session_state.waypoints[i][1]:.6f})")
                    if col_wp[1].button("删除", key=f"del_wp_{i}"):
                        st.session_state.waypoints.pop(i)
                        st.rerun()
            else:
                st.write("暂无中间航点")
            # 添加新航点
            st.markdown("**添加新航点**")
            col_add = st.columns(2)
            with col_add[0]:
                new_lng = st.number_input("经度", value=st.session_state.new_wp_lng, format="%.6f", key="new_lng")
            with col_add[1]:
                new_lat = st.number_input("纬度", value=st.session_state.new_wp_lat, format="%.6f", key="new_lat")
            if st.button("➕ 添加航点"):
                st.session_state.waypoints.insert(-1, [new_lng, new_lat])
                st.rerun()
            # 终点
            st.markdown("**终点**")
            col_e = st.columns(2)
            with col_e[0]:
                b_lat = st.number_input("纬度", value=st.session_state.waypoints[-1][1], format="%.6f", key="b_lat")
            with col_e[1]:
                b_lng = st.number_input("经度", value=st.session_state.waypoints[-1][0], format="%.6f", key="b_lng")
            if st.button("更新终点"):
                st.session_state.waypoints[-1] = [b_lng, b_lat]
                st.rerun()
            st.markdown("---")
            # 障碍物添加
            st.markdown("#### 🏗️ 新障碍物高度")
            st.session_state.pending_h = st.number_input("高度(米)",1,200,st.session_state.pending_h)
            if st.button("➕ 添加障碍物"):
                if st.session_state.pending_poly and len(st.session_state.pending_poly)>=3:
                    st.session_state.obs.append({"name":f"建筑物{len(st.session_state.obs)+1}",
                                                 "polygon":st.session_state.pending_poly,
                                                 "height":st.session_state.pending_h})
                    st.success(f"已添加，共{len(st.session_state.obs)}个")
                    st.session_state.pending_poly = None
                    st.session_state.full_path = plan_full_path(st.session_state.waypoints,
                                                                st.session_state.obs,
                                                                st.session_state.alt,
                                                                st.session_state.safe_rad,
                                                                st.session_state.sel_strat)
                    st.rerun()
                else: st.warning("请先在地图上画多边形")
            if st.button("🔄 重新规划路径"):
                with st.spinner("规划全航线中..."):
                    st.session_state.full_path = plan_full_path(st.session_state.waypoints,
                                                                st.session_state.obs,
                                                                st.session_state.alt,
                                                                st.session_state.safe_rad,
                                                                st.session_state.sel_strat)
                st.rerun()
            st.markdown("#### ✈️ 飞行控制")
            if st.button("▶️ 开始飞行"):
                if st.session_state.full_path is None or len(st.session_state.full_path) < 2:
                    st.warning("请先点击「刷新规划」生成完整路径")
                else:
                    st.session_state.hb.set_path(st.session_state.full_path, st.session_state.alt, st.session_state.drone_spd)
                    st.session_state.running = True
                    st.session_state.hist = []
                    st.success("飞行开始，请切换至「监控」页面")
            if st.button("⏹️ 停止飞行"):
                st.session_state.running = False
                st.session_state.hb.stop()
            st.caption(f"航线共{len(st.session_state.waypoints)}个航点")
            if st.session_state.full_path:
                st.caption(f"完整路径含{len(st.session_state.full_path)}个航段点")
        with col2:
            center = st.session_state.waypoints[0] or SCHOOL_CENTER
            if st.session_state.full_path is None:
                st.session_state.full_path = plan_full_path(st.session_state.waypoints,
                                                            st.session_state.obs,
                                                            st.session_state.alt,
                                                            st.session_state.safe_rad,
                                                            st.session_state.sel_strat)
            # 调用 make_map 函数（需完整实现，此处略）
            # m = make_map(...)
            # output = st_folium(m, ...)
            st.info("地图显示区域（需补齐 make_map 函数）")
            # 临时占位，实际应包含完整地图代码
        st.caption("图例：绿色=避障航线 红色=障碍物 橙色=安全区 | 蓝色旗帜=中间航点")

    elif page == "监控":
        st.header("飞行实时画面 - 任务执行监控")
        # 控制按钮
        col_btn = st.columns(4)
        with col_btn[0]:
            if st.button("▶️ 开始任务", use_container_width=True):
                if not st.session_state.running:
                    if st.session_state.full_path is None:
                        st.warning("请先在规划页面刷新规划路径")
                    else:
                        st.session_state.hb.set_path(st.session_state.full_path, st.session_state.alt, st.session_state.drone_spd)
                        st.session_state.running = True
                        st.rerun()
                else:
                    st.session_state.hb.resume()
                    st.rerun()
        with col_btn[1]:
            if st.button("⏸️ 暂停", use_container_width=True):
                if st.session_state.running:
                    st.session_state.hb.pause()
                    st.rerun()
                else:
                    st.warning("当前没有飞行任务")
        with col_btn[2]:
            if st.button("⏹️ 停止", use_container_width=True):
                st.session_state.running = False
                st.session_state.hb.stop()
                st.rerun()
        with col_btn[3]:
            if st.button("🔄 重置", use_container_width=True):
                st.session_state.running = False
                st.session_state.hb.reset()
                st.session_state.hist = []
                st.rerun()
        st.markdown("---")
        # 手动刷新按钮
        if st.button("📡 手动刷新飞行数据", use_container_width=True):
            if st.session_state.running:
                new_hb = st.session_state.hb.step()
                st.session_state.hist.append([new_hb['lng'], new_hb['lat']])
                if len(st.session_state.hist) > 200:
                    st.session_state.hist.pop(0)
                if not st.session_state.hb.sim:
                    st.session_state.running = False
                st.rerun()
            else:
                st.warning("未开始飞行，请先点击「开始任务」")
        st.markdown("---")
        # 显示最新心跳数据
        if st.session_state.hb.hist:
            d = st.session_state.hb.hist[0]
        else:
            d = {"current_wp":"0/0","speed":0,"elapsed":0,"total":0,"traveled":0,"remain":"00:00","battery":0,"progress":0,"delay_ms":0,"loss_percent":0}
        # 指标显示
        row1 = st.columns(4)
        row1[0].metric("当前航点", d.get('current_wp','0/0'))
        row1[1].metric("飞行速度", f"{d.get('speed',0)} m/s")
        elapsed = d.get('elapsed',0)
        row1[2].metric("已用时间", f"{int(elapsed//60):02d}:{int(elapsed%60):02d}")
        remaining = max(0, (d.get('total',0)-d.get('traveled',0))*111000)
        row1[3].metric("剩余距离", f"{remaining:.0f} m")
        row2 = st.columns(2)
        row2[0].metric("预计到达", d.get('remain','00:00'))
        row2[1].metric("电量模拟", f"{d.get('battery',0)}%")
        st.progress(d.get('progress',0), text=f"任务进度: {d.get('progress',0)*100:.1f}%")
        st.markdown("---")
        # 设备状态与通信拓扑
        col_status, col_top = st.columns(2)
        with col_status:
            st.subheader("📡 设备状态")
            online = st.session_state.running
            st.markdown(f"- **GCS**：{'✅ 在线' if online else '❌ 离线'}")
            st.markdown(f"- **OBC**：{'✅ 在线' if online else '❌ 离线'}")
            st.markdown(f"- **FCU**：{'✅ 在线' if online else '❌ 离线'}")
        with col_top:
            st.subheader("🔗 通信链路拓扑与数据流")
            delay = d.get('delay_ms',0)
            loss = d.get('loss_percent',0)
            st.markdown(f"""
            - **GCS** ↔ **OBC**：延迟 {delay} ms  
            - **GCS** ↔ **FCU**：延迟 {delay+5} ms  
            - **OBC** ↔ **FCU**：延迟 ~{max(0,delay-2)} ms  
            - **丢包率**：{loss}%
            """)
            st.code("GCS → OBC → FCU → UAV")
            st.caption("数据流：遥控指令 → 飞控 → 执行器 | 遥测数据 ← 飞控 ← 传感器")
        st.markdown("---")
        # 实时飞行地图
        st.subheader("🗺️ 实时飞行地图")
        if st.session_state.hb.hist:
            latest = st.session_state.hb.hist[0]
            center = [latest['lat'], latest['lng']]
        elif st.session_state.waypoints:
            center = [st.session_state.waypoints[0][1], st.session_state.waypoints[0][0]]
        else:
            center = [SCHOOL_CENTER[1], SCHOOL_CENTER[0]]
        tiles = SAT_URL if map_type=="satellite" else VEC_URL
        m = folium.Map(location=center, zoom_start=17, tiles=tiles, attr=ATTR)
        add_safety(m, st.session_state.obs, st.session_state.safe_rad, st.session_state.alt)
        for o in st.session_state.obs:
            coords = o.get('polygon',[])
            if len(coords)>=3:
                folium.Polygon([[c[1],c[0]] for c in coords], color='red', fill=True, fill_opacity=0.3).add_to(m)
        if st.session_state.full_path:
            folium.PolyLine([[p[1],p[0]] for p in st.session_state.full_path], color='green', weight=3).add_to(m)
        if st.session_state.hb.hist:
            trail = [[h['lat'],h['lng']] for h in st.session_state.hb.hist[:30]]
            if len(trail)>1:
                folium.PolyLine(trail, color='orange', weight=2).add_to(m)
            latest = st.session_state.hb.hist[0]
            folium.Marker([latest['lat'], latest['lng']], popup=f"📍 当前位置\n高度:{latest['altitude']}m",
                          icon=folium.Icon(color='red', icon='plane', prefix='fa')).add_to(m)
        for i,wp in enumerate(st.session_state.waypoints):
            color = 'green' if i==0 else ('red' if i==len(st.session_state.waypoints)-1 else 'blue')
            folium.Marker([wp[1], wp[0]], popup=f"航点{i+1}", icon=folium.Icon(color=color)).add_to(m)
        folium_static(m, width=1000, height=500)

    elif page == "障碍物":
        st.header("障碍物管理")
        st.info(f"共 {len(st.session_state.obs)} 个障碍物")
        col1,col2 = st.columns([1,1.5])
        with col1:
            for i,o in enumerate(st.session_state.obs):
                c1,c2,c3 = st.columns([2,1,1])
                c1.write(f"🚧 {o.get('name',f'障碍物{i+1}')}")
                c2.write(f"高度:{o.get('height',20)}m")
                if c3.button("删除", key=f"del{i}"):
                    st.session_state.obs.pop(i)
                    st.session_state.full_path = plan_full_path(st.session_state.waypoints,
                                                                st.session_state.obs,
                                                                st.session_state.alt,
                                                                st.session_state.safe_rad,
                                                                st.session_state.sel_strat)
                    st.rerun()
            if st.button("💾 保存到缓存"): save_cache()
            if st.button("📂 从缓存加载"):
                if load_cache():
                    st.session_state.full_path = plan_full_path(st.session_state.waypoints,
                                                                st.session_state.obs,
                                                                st.session_state.alt,
                                                                st.session_state.safe_rad,
                                                                st.session_state.sel_strat)
                    st.rerun()
            if st.button("🗑️ 全部清除"):
                st.session_state.obs=[]
                st.session_state.full_path = plan_full_path(st.session_state.waypoints,
                                                            st.session_state.obs,
                                                            st.session_state.alt,
                                                            st.session_state.safe_rad,
                                                            st.session_state.sel_strat)
                st.rerun()
        with col2:
            tiles = SAT_URL if map_type=="satellite" else VEC_URL
            m = folium.Map(location=[SCHOOL_CENTER[1],SCHOOL_CENTER[0]], zoom_start=16, tiles=tiles, attr=ATTR)
            for o in st.session_state.obs:
                coords=o.get('polygon',[])
                if len(coords)>=3:
                    folium.Polygon([[c[1],c[0]] for c in coords], color='red', weight=3, fill=True, fill_opacity=0.5, popup=f"高度:{o.get('height',20)}m").add_to(m)
            folium.Marker([A_DFT[1],A_DFT[0]], popup="起点", icon=folium.Icon(color='green')).add_to(m)
            folium.Marker([B_DFT[1],B_DFT[0]], popup="终点", icon=folium.Icon(color='red')).add_to(m)
            folium_static(m, width=700, height=500)

if __name__ == "__main__":
    main()

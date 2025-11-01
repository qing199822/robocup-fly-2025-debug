import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.patheffects as pe
from matplotlib import font_manager
import numpy as np
import json
import os
import urllib.request
import glob

# --- 全局变量与配置 ---
WORLD_DIRECTORY = 'world'  # 指定存放 .world 文件的目录

# 【修改点 1】: 定义无人机巡航高度的映射表
# -------------------------------------------------------------------
# 使用字典来为每架无人机指定巡航高度。
# 如果无人机ID不在此字典中，则使用默认高度 Z_ALTITUDE_DEFAULT。
# -------------------------------------------------------------------
Z_ALTITUDE_MAPPING = {
    1: 4.5,
    2: 4.0,
    3: 3.5,
    4: 4.25
    # 0号机和5号机将使用下面的默认值
}
Z_ALTITUDE_DEFAULT = 5.0 # 为未在上面指定高度的无人机设置一个默认巡航高度

# 字体配置
FONT_FILENAME = "SimHei.ttf"
FONT_URL = "https://github.com/adobe-fonts/source-han-sans/raw/release/OTF/SimplifiedChinese/SourceHanSansSC-Regular.otf"

# --- 动态状态变量 ---
WORLD_FILES = []              # 存储所有找到的world文件路径
CURRENT_WORLD_INDEX = 0       # 当前显示的世界文件索引

INITIAL_POSITIONS = {
    0: (-17, -3), 1: (-14, -3), 2: (-17, 0),
    3: (-14, 0), 4: (-17, 3), 5: (-14, 3)
}
road_data = []
lamp_post_data = []
waypoints_data = {}           # {world_file: {vehicle_id: [(x,y), ...]}, ...}
current_vehicle_id = None
colors = ['red', 'blue', 'green', 'orange', 'purple', 'cyan']

# --- Matplotlib 设置 ---
fig, ax = plt.subplots(figsize=(16, 13))

def setup_font():
    """检查、下载并设置中文字体"""
    if os.path.exists(FONT_FILENAME):
        print(f"字体文件 '{FONT_FILENAME}' 已存在，直接加载。")
    else:
        print(f"字体文件 '{FONT_FILENAME}' 不存在，正在尝试从网络下载...")
        try:
            urllib.request.urlretrieve(FONT_URL, FONT_FILENAME)
            print("字体下载成功！")
        except Exception as e:
            print(f"字体自动下载失败: {e}")
            print("请手动下载字体文件，并将其命名为 'SimHei.ttf' 放置于脚本相同目录下。")
            return False

    font_manager.fontManager.addfont(FONT_FILENAME)
    plt.rcParams['font.sans-serif'] = [os.path.splitext(FONT_FILENAME)[0]]
    plt.rcParams['axes.unicode_minus'] = False
    print("中文字体设置成功！")
    return True

def parse_world_file(file_path):
    """解析单个 world 文件，提取道路和路灯信息。"""
    if not os.path.exists(file_path):
        print(f"错误: 文件 '{file_path}' 不存在。")
        return [], []
    print(f"\n--- 正在解析地图文件: {os.path.basename(file_path)} ---")
    try:
        with open(file_path, 'r', encoding='utf-8') as f: content = f.read()
        root = ET.fromstring(content)
        
        roads = []
        for road_elem in root.findall('.//road'):
            width = float(road_elem.find('width').text)
            points = [(float(c.split()[0]), float(c.split()[1])) for c in [p.text for p in road_elem.findall('point')]]
            if len(points) >= 2:
                for i in range(len(points) - 1): roads.append((points[i], points[i+1], width))
        print(f"地图解析完成，找到 {len(roads)} 条道路。")

        lamp_posts = []
        for model_elem in root.findall('.//model'):
            if 'lamp_post' in model_elem.get('name', ''):
                pose_elem = model_elem.find('pose')
                if pose_elem is not None and pose_elem.text:
                    pose_parts = pose_elem.text.split()
                    if len(pose_parts) >= 2:
                        try:
                            x, y = float(pose_parts[0]), float(pose_parts[1])
                            lamp_posts.append((x, y))
                        except (ValueError, IndexError):
                            print(f"警告：解析模型 '{model_elem.get('name')}' 的pose时格式不正确。")
        print(f"找到 {len(lamp_posts)} 个路灯。")
        return roads, lamp_posts
    except Exception as e:
        print(f"解析XML文件时发生错误: {e}"); return [], []

def plot_base_map():
    ax.set_facecolor('darkgray')
    for p1, p2, width in road_data:
        p1, p2 = np.array(p1), np.array(p2)
        direction = p2 - p1
        if np.linalg.norm(direction) < 1e-6: continue
        normal = np.array([-direction[1], direction[0]]) / np.linalg.norm(direction)
        v = [p1 + normal * width / 2, p2 + normal * width / 2, p2 - normal * width / 2, p1 - normal * width / 2]
        ax.add_patch(patches.Polygon(v, closed=True, facecolor='dimgray', edgecolor='black', linewidth=0.5))

    if lamp_post_data:
        lamp_x, lamp_y = zip(*lamp_post_data)
        ax.plot(lamp_x, lamp_y, 'ko', markersize=3, zorder=6, label='路灯') 
    
    for vehicle_num, pos in INITIAL_POSITIONS.items():
        color = colors[vehicle_num]
        ax.plot(pos[0], pos[1], '*', color=color, markersize=18, 
                markeredgecolor='white', label=f'无人机 {vehicle_num} 起点', zorder=10)
        ax.text(pos[0], pos[1], str(vehicle_num), color='white', fontsize=10, weight='bold',
                ha='center', va='center', path_effects=[pe.withStroke(linewidth=2, foreground='black')], zorder=12)

    ax.set_xlabel("X 轴坐标 (m)")
    ax.set_ylabel("Y 轴坐标 (m)")
    ax.grid(True, linestyle='--', color='gray', alpha=0.5)
    ax.set_aspect('equal', adjustable='box')

def update_title_and_legend():
    world_name = os.path.basename(WORLD_FILES[CURRENT_WORLD_INDEX])
    map_info = f"地图: {world_name} ({CURRENT_WORLD_INDEX + 1}/{len(WORLD_FILES)})"
    
    if current_vehicle_id is not None:
        color_name = colors[current_vehicle_id]
        # 在标题中显示当前无人机的巡航高度
        altitude = Z_ALTITUDE_MAPPING.get(current_vehicle_id, Z_ALTITUDE_DEFAULT)
        title = f"{map_info} | 模式: 设置 {current_vehicle_id}号机 ({color_name}, 高度: {altitude}m) | [左键]添加 | [右键]撤销 | [E]完成"
    else:
        title = f"{map_info} | [0-5]选机 | [N]下一张图 | [P]上一张图 | [Q]退出并保存所有"
    ax.set_title(title, fontsize=12)
    ax.legend(loc='best')
    
def redraw_canvas():
    ax.cla()
    plot_base_map()
    current_world_file = WORLD_FILES[CURRENT_WORLD_INDEX]
    
    if current_world_file in waypoints_data:
        vehicle_waypoints = waypoints_data[current_world_file]
        for vehicle_num, points in sorted(vehicle_waypoints.items()):
            if not points: continue
            color = colors[vehicle_num]
            x_coords, y_coords = [p[0] for p in points], [p[1] for p in points]
            ax.plot(x_coords, y_coords, '--o', color=color, markersize=5, 
                    markeredgecolor='black', label=f'无人机 {vehicle_num} 路径', zorder=5)
            for i, (x, y) in enumerate(points):
                ax.text(x + 0.5, y + 0.5, str(i + 1), color='white', fontsize=8, ha='center', va='center',
                        path_effects=[pe.withStroke(linewidth=1.5, foreground='black')], zorder=11)
    update_title_and_legend()
    fig.canvas.draw_idle()

def load_world(world_index):
    """加载并显示指定索引的世界地图"""
    global road_data, lamp_post_data, current_vehicle_id
    current_vehicle_id = None # 切换地图时，重置当前选中的无人机
    
    world_path = WORLD_FILES[world_index]
    road_data, lamp_post_data = parse_world_file(world_path)
    if not road_data:
        print(f"警告: 地图 '{os.path.basename(world_path)}' 为空或解析失败。")
    
    redraw_canvas()

def on_key_press(event):
    global current_vehicle_id, CURRENT_WORLD_INDEX
    key = event.key.lower()

    if key in [str(i) for i in range(0, 6)]:
        if current_vehicle_id is not None: 
            print(f"提示: 请先按 'e' 键结束对 {current_vehicle_id} 号机的设置。")
            return
        current_vehicle_id = int(key)
        
        # 为当前世界和无人机初始化路径点列表
        current_world_file = WORLD_FILES[CURRENT_WORLD_INDEX]
        if current_world_file not in waypoints_data:
            waypoints_data[current_world_file] = {}
        if current_vehicle_id not in waypoints_data[current_world_file]:
            waypoints_data[current_world_file][current_vehicle_id] = []
            
        altitude = Z_ALTITUDE_MAPPING.get(current_vehicle_id, Z_ALTITUDE_DEFAULT)
        print(f"--- 模式: 设置 {current_vehicle_id} 号机 (地图: {os.path.basename(current_world_file)}, 巡航高度: {altitude}m) ---")
        redraw_canvas()
    
    elif key == 'e':
        if current_vehicle_id is not None:
            print(f"--- {current_vehicle_id} 号机路径点设置完毕 ---")
            current_vehicle_id = None
            redraw_canvas()
            print("请选择下一架无人机, 或按 'N'/'P' 切换地图。")
        else: 
            print("提示: 请先按 0-5 选择无人机。")
    
    elif key == 'n' or key == 'p': # Next or Previous
        if current_vehicle_id is not None:
             print(f"提示: 请先按 'e' 键完成对 {current_vehicle_id} 号机的设置，再切换地图。")
             return
        
        generate_mission_json_for_world(WORLD_FILES[CURRENT_WORLD_INDEX])
        
        if key == 'n':
            CURRENT_WORLD_INDEX = (CURRENT_WORLD_INDEX + 1) % len(WORLD_FILES)
        else: # key == 'p'
            CURRENT_WORLD_INDEX = (CURRENT_WORLD_INDEX - 1 + len(WORLD_FILES)) % len(WORLD_FILES)
        
        load_world(CURRENT_WORLD_INDEX)

    elif key == 'q': 
        plt.close(fig)

def on_click(event):
    if not event.inaxes or current_vehicle_id is None:
        if not event.inaxes: return
        print("提示: 请先按 0-5 选择要设置的无人机。")
        return
        
    current_world_file = WORLD_FILES[CURRENT_WORLD_INDEX]
    waypoints = waypoints_data[current_world_file][current_vehicle_id]
    
    if event.button == 1:
        x, y = round(event.xdata, 2), round(event.ydata, 2)
        waypoints.append((x, y))
        print(f"为 {current_vehicle_id} 号机添加航点 {len(waypoints)}: (x={x}, y={y})")
    elif event.button == 3 and waypoints:
        waypoints.pop()
        print(f"为 {current_vehicle_id} 号机撤销上一个航点。")
    redraw_canvas()

def on_scroll(event):
    if not event.inaxes: return
    scale_factor = 1.1 if event.button == 'up' else 1 / 1.1
    cur_xlim, cur_ylim = ax.get_xlim(), ax.get_ylim()
    xdata, ydata = event.xdata, event.ydata
    new_width = (cur_xlim[1] - cur_xlim[0]) * scale_factor
    new_height = (cur_ylim[1] - cur_ylim[0]) * scale_factor
    relx = (cur_xlim[1] - xdata) / (cur_xlim[1] - cur_xlim[0])
    rely = (cur_ylim[1] - ydata) / (cur_ylim[1] - cur_ylim[0])
    ax.set_xlim([xdata - new_width * (1 - relx), xdata + new_width * relx])
    ax.set_ylim([ydata - new_height * (1 - rely), ydata + new_height * rely])
    fig.canvas.draw_idle()

def generate_mission_json_for_world(world_file_path):
    """为指定的world文件生成对应的mission.json"""
    if world_file_path not in waypoints_data or not any(waypoints_data[world_file_path].values()):
        return # 如果没有为这个世界设置任何航点，则不生成文件

    base_name = os.path.basename(world_file_path)
    name_without_ext, _ = os.path.splitext(base_name)
    output_filename = f"{name_without_ext}_mission.json"

    mission_list = []
    vehicle_waypoints = waypoints_data[world_file_path]

    for vehicle_num in sorted(vehicle_waypoints.keys()):
        points = vehicle_waypoints[vehicle_num]
        if not points: continue
        
        # 【修改点 2】: 在生成航点时，动态获取巡航高度
        # -------------------------------------------------------------------
        # 使用 .get() 方法：如果 vehicle_num 存在于 Z_ALTITUDE_MAPPING 中，
        # 则返回其对应的高度；否则，返回 Z_ALTITUDE_DEFAULT。
        # -------------------------------------------------------------------
        altitude = Z_ALTITUDE_MAPPING.get(vehicle_num, Z_ALTITUDE_DEFAULT)
        
        mission_list.append({
            "vehicle_id": f"typhoon_h480_{vehicle_num}",
            "waypoints": [{"x": p[0], "y": p[1], "z": altitude} for p in points]
        })
    
    if not mission_list: return

    try:
        with open(output_filename, 'w', encoding='utf-8') as f:
            json.dump(mission_list, f, indent=2, ensure_ascii=False)
        print(f"\n--- 已为地图 '{base_name}' 生成任务文件: {output_filename} ---")
    except Exception as e: 
        print(f"为 '{base_name}' 写入 JSON 文件时出错: {e}")

# --- 主程序 ---
if __name__ == "__main__":
    if setup_font():
        world_pattern = os.path.join(WORLD_DIRECTORY, '*.world')
        WORLD_FILES = sorted(glob.glob(world_pattern))

        if not WORLD_FILES:
            print(f"错误: 在 '{WORLD_DIRECTORY}/' 目录下未找到任何 .world 文件。")
            print("请确保该目录存在，并且包含至少一个 .world 地图文件。")
        else:
            print(f"在 '{WORLD_DIRECTORY}/' 目录中找到 {len(WORLD_FILES)} 个世界地图。")
            
            fig.canvas.mpl_connect('key_press_event', on_key_press)
            fig.canvas.mpl_connect('button_press_event', on_click)
            fig.canvas.mpl_connect('scroll_event', on_scroll)
            
            # 加载第一个世界
            load_world(CURRENT_WORLD_INDEX)
            
            print("\n欢迎使用无人机航点设置工具！")
            print("="*60)
            print("操作指南:")
            print("1. 按键盘数字 [0-5] 选择要设置航点的无人机。")
            print("2. 在地图上 [鼠标左键] 点击以添加航点，[右键] 撤销。")
            print("3. 完成当前无人机设置后，按 [E] 键确认。")
            print("4. 按 [N] 键切换到下一个地图，按 [P] 键切换到上一个。")
            print("   (切换地图会自动保存当前地图的航点)")
            print("5. [鼠标滚轮] 缩放地图视野。")
            print("6. 按 [Q] 键或关闭窗口，将自动保存所有已设置的航点并退出。")
            print("="*60)
            plt.show()
            
            # 退出时，为所有设置过航点的地图生成文件
            print("\n程序退出，正在保存所有未保存的航点数据...")
            for world_file in waypoints_data:
                generate_mission_json_for_world(world_file)
            print("所有数据已保存。再见！")

import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.patheffects as pe
from matplotlib import font_manager
import numpy as np
import json
import os
import urllib.request


# --- 全局变量与配置 ---
# 修改：将 WORLD_FILE 指向您的新文件
WORLD_FILE = 'robocup.world' 
OUTPUT_JSON_FILE = 'mission.json'
Z_ALTITUDE = 5.0

# 字体配置
FONT_FILENAME = "SimHei.ttf"
FONT_URL = "https://github.com/adobe-fonts/source-han-sans/raw/release/OTF/SimplifiedChinese/SourceHanSansSC-Regular.otf"

INITIAL_POSITIONS = {
    0: (0, -3), 1: (3, -3), 2: (0, 0),
    3: (3, 0), 4: (0, 3), 5: (3, 3)
}
road_data = []
lamp_post_data = []
waypoints_data = {}
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
            print(f"下载地址: {FONT_URL}")
            return False

    font_manager.fontManager.addfont(FONT_FILENAME)
    plt.rcParams['font.sans-serif'] = [os.path.splitext(FONT_FILENAME)[0]]
    plt.rcParams['axes.unicode_minus'] = False
    print("中文字体设置成功！")
    return True

def parse_world_file(file_path):
    """
    解析 world 文件，提取道路和路灯信息。
    """
    if not os.path.exists(file_path):
        print(f"错误: 文件 '{file_path}' 不存在。")
        return [], []
    print(f"正在解析地图文件: {file_path}...")
    try:
        with open(file_path, 'r', encoding='utf-8') as f: content = f.read()
        root = ET.fromstring(content)
        
        # 1. 解析道路
        roads = []
        for road_elem in root.findall('.//road'):
            width_elem = road_elem.find('width')
            if width_elem is None: continue
            width = float(width_elem.text)
            points = [(float(c.split()[0]), float(c.split()[1])) for c in [p.text for p in road_elem.findall('point')]]
            if len(points) >= 2:
                for i in range(len(points) - 1): roads.append((points[i], points[i+1], width))
        print(f"地图解析完成，找到 {len(roads)} 条道路。")

        # 2. 解析路灯 (*** 此处是核心修改 ***)
        lamp_posts = []
        # 修改查找逻辑：从查找 <include> 变为查找 <model>
        for model_elem in root.findall('.//model'):
            # 检查模型的 'name' 属性是否包含 'lamp_post'
            model_name = model_elem.get('name', '')
            if 'lamp_post' in model_name:
                # pose 元素是 model 的直接子元素
                pose_elem = model_elem.find('pose')
                if pose_elem is not None and pose_elem.text:
                    pose_parts = pose_elem.text.split()
                    if len(pose_parts) >= 2:
                        try:
                            x, y = float(pose_parts[0]), float(pose_parts[1])
                            lamp_posts.append((x, y))
                        except (ValueError, IndexError):
                            print(f"警告：解析模型 '{model_name}' 的pose时格式不正确。")

        print(f"找到 {len(lamp_posts)} 个路灯。")

        return roads, lamp_posts
    except Exception as e:
        print(f"解析XML文件时发生错误: {e}"); return [], []


def plot_base_map():
    ax.set_facecolor('darkgray')
    
    # 绘制道路
    for p1, p2, width in road_data:
        p1, p2 = np.array(p1), np.array(p2)
        direction = p2 - p1
        if np.linalg.norm(direction) < 1e-6: continue
        normal = np.array([-direction[1], direction[0]]) / np.linalg.norm(direction)
        v = [p1 + normal * width / 2, p2 + normal * width / 2, p2 - normal * width / 2, p1 - normal * width / 2]
        ax.add_patch(patches.Polygon(v, closed=True, facecolor='dimgray', edgecolor='black', linewidth=0.5))

    # 绘制路灯
    if lamp_post_data:
        lamp_x, lamp_y = zip(*lamp_post_data)
        ax.plot(lamp_x, lamp_y, 'ko', markersize=3, zorder=6, label='路灯') 
    
    # 绘制无人机起点
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
    if current_vehicle_id is not None:
        color_name = colors[current_vehicle_id]
        title = f"模式: 设置 {current_vehicle_id}号机 ({color_name}) | [左键]添加 | [右键]撤销 | [E]完成 | [Q]退出 | [滚轮]缩放"
    else:
        title = "按 [0-5] 选择无人机 | [Q] 退出并保存 | [滚轮] 缩放"
    ax.set_title(title, fontsize=12)
    ax.legend(loc='best')
    
def redraw_canvas():
    ax.cla()
    plot_base_map()
    for vehicle_num, points in sorted(waypoints_data.items()):
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

def on_key_press(event):
    global current_vehicle_id
    key = event.key.lower()
    if key in [str(i) for i in range(0, 6)]:
        vehicle_num = int(key)
        if current_vehicle_id is not None: 
            print(f"警告: 请先按 'e' 键结束对 {current_vehicle_id} 号机的设置。")
            return
        current_vehicle_id = vehicle_num
        if current_vehicle_id not in waypoints_data: 
            waypoints_data[current_vehicle_id] = []
        print(f"\n--- 模式: 设置 {current_vehicle_id} 号机 (颜色: {colors[current_vehicle_id]}) ---")
        redraw_canvas()
    elif key == 'e':
        if current_vehicle_id is not None:
            print(f"--- {current_vehicle_id} 号机路径点设置完毕 ---")
            
            # 新增：在结束当前无人机设置时自动保存
            if waypoints_data[current_vehicle_id]:
                generate_mission_json()
                print(f"已自动保存 {current_vehicle_id} 号机的航点数据")
            
            current_vehicle_id = None
            redraw_canvas()
            print("\n请选择下一架无人机或按 'q' 退出。")
        else: 
            print("提示: 请先按 0-5 选择无人机。")
    elif key == 'q': 
        # 新增：退出时也保存一次
        generate_mission_json()
        plt.close(fig)

def on_click(event):
    if not event.inaxes: return
    if current_vehicle_id is None:
        print("提示: 请先按 0-5 选择要设置的无人机。")
        return
        
    waypoints = waypoints_data[current_vehicle_id]
    if event.button == 1:
        x, y = round(event.xdata, 2), round(event.ydata, 2)
        waypoints.append((x, y))
        print(f"为 {current_vehicle_id} 号机添加航点 {len(waypoints)}: (x={x}, y={y})")
    elif event.button == 3:
        if waypoints: 
            waypoints.pop()
            print(f"为 {current_vehicle_id} 号机撤销上一个航点。")
        else: 
            print(f"{current_vehicle_id} 号机没有可撤销的航点。")
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

def generate_mission_json():
    mission_list = []
    for vehicle_num in sorted(waypoints_data.keys()):
        points = waypoints_data[vehicle_num]
        if not points: continue
        mission_list.append({
            "vehicle_id": f"typhoon_h480_{vehicle_num}",
            "waypoints": [{"x": p[0], "y": p[1], "z": Z_ALTITUDE} for p in points]
        })
    if not mission_list: 
        print("\n未记录任何航点，不生成 mission.json。")
        return
    try:
        with open(OUTPUT_JSON_FILE, 'w', encoding='utf-8') as f:
            json.dump(mission_list, f, indent=2, ensure_ascii=False)
        print(f"\n成功生成 mission.json 文件！包含 {len(mission_list)} 架无人机的任务。")
    except Exception as e: 
        print(f"写入 JSON 文件时出错: {e}")

# --- 主程序 ---
if __name__ == "__main__":
    if setup_font():
        road_data, lamp_post_data = parse_world_file(WORLD_FILE)
        if road_data:
            fig.canvas.mpl_connect('key_press_event', on_key_press)
            fig.canvas.mpl_connect('button_press_event', on_click)
            fig.canvas.mpl_connect('scroll_event', on_scroll)
            redraw_canvas()
            print("\n欢迎使用无人机航点设置工具！")
            print("="*60)
            print("操作指南:")
            print("1. 按键盘数字 [0-5] 来选择要设置航点的无人机。")
            print("2. 在地图上 [鼠标左键] 点击以添加航点。")
            print("3. [鼠标右键] 点击以撤销上一个添加的航点。")
            print("4. [鼠标滚轮] 缩放地图视野。")
            print("5. 完成当前无人机的设置后，按 [E] 键确认并自动保存。")
            print("6. 设置完所有无人机后，按 [Q] 键或关闭窗口，程序将自动保存 mission.json 文件。")
            print("="*60)
            plt.show()
            generate_mission_json()
        else:
            print("未能成功解析地图数据，程序退出。")

import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.patheffects as pe
from matplotlib import font_manager
import numpy as np
import os
import urllib.request
import glob

# --- 全局变量与配置 ---
WORLD_DIRECTORY = 'world'         # 存放 .world 文件的源目录
OUTPUT_PNG_DIR = 'png'            # 存放输出 .png 文件的目标目录
DPI_RESOLUTION = 150              # 输出图片的清晰度 (dots per inch)

# 字体配置
FONT_FILENAME = "SimHei.ttf"
FONT_URL = "https://github.com/adobe-fonts/source-han-sans/raw/release/OTF/SimplifiedChinese/SourceHanSansSC-Regular.otf"

# 静态绘图元素
INITIAL_POSITIONS = {
    0: (0, -3), 1: (3, -3), 2: (0, 0),
    3: (3, 0), 4: (0, 3), 5: (3, 3)
}
colors = ['red', 'blue', 'green', 'orange', 'purple', 'cyan']

# --- Matplotlib 设置 (在非交互模式下使用 Agg 后端) ---
plt.switch_backend('Agg')
fig, ax = plt.subplots(figsize=(16, 13))

def setup_font():
    """检查、下载并设置中文字体"""
    if os.path.exists(FONT_FILENAME):
        print(f"字体文件 '{FONT_FILENAME}' 已存在。")
    else:
        print(f"字体文件 '{FONT_FILENAME}' 不存在，正在下载...")
        try:
            urllib.request.urlretrieve(FONT_URL, FONT_FILENAME)
            print("字体下载成功！")
        except Exception as e:
            print(f"字体自动下载失败: {e}。将使用默认字体。")
            return False

    font_manager.fontManager.addfont(FONT_FILENAME)
    plt.rcParams['font.sans-serif'] = [os.path.splitext(FONT_FILENAME)[0]]
    plt.rcParams['axes.unicode_minus'] = False
    print("中文字体设置成功！")
    return True

def parse_world_file(file_path):
    """解析 world 文件，提取道路和路灯信息 (逻辑不变)"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f: content = f.read()
        root = ET.fromstring(content)
        
        roads = []
        for road_elem in root.findall('.//road'):
            width = float(road_elem.find('width').text)
            points = [(float(c.split()[0]), float(c.split()[1])) for c in [p.text for p in road_elem.findall('point')]]
            if len(points) >= 2:
                for i in range(len(points) - 1): roads.append((points[i], points[i+1], width))

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
                            pass
        return roads, lamp_posts
    except Exception as e:
        print(f"  解析文件 '{os.path.basename(file_path)}' 时出错: {e}"); return [], []

def plot_map_to_axis(ax, road_data, lamp_post_data):
    """将解析出的地图数据绘制到给定的 Matplotlib Axis 上"""
    ax.clear()  # 清除上一次绘图的内容
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
    ax.legend(loc='best')

# --- 主程序 ---
if __name__ == "__main__":
    print("--- 开始批量生成世界地图预览图 ---")
    setup_font()

    # 1. 查找所有 .world 文件
    world_pattern = os.path.join(WORLD_DIRECTORY, '*.world')
    world_files = sorted(glob.glob(world_pattern))

    if not world_files:
        print(f"\n错误: 在 '{WORLD_DIRECTORY}/' 目录下未找到任何 .world 文件。")
        print("请确保该目录存在，并且包含至少一个 .world 地图文件。")
    else:
        # 2. 确保输出目录存在
        os.makedirs(OUTPUT_PNG_DIR, exist_ok=True)
        print(f"\n找到 {len(world_files)} 个世界地图，将开始处理...")
        print("-" * 40)

        # 3. 循环处理每个文件
        for world_path in world_files:
            world_filename = os.path.basename(world_path)
            print(f"正在处理: {world_filename} ...")

            # 解析文件
            roads, lamps = parse_world_file(world_path)
            
            if not roads:
                print(f"  -> 跳过: 未能从 {world_filename} 中解析出道路数据。")
                continue

            # 绘制地图
            plot_map_to_axis(ax, roads, lamps)
            ax.set_title(f"地图预览: {world_filename}", fontsize=16)

            # 构建输出路径
            name_without_ext = os.path.splitext(world_filename)[0]
            output_path = os.path.join(OUTPUT_PNG_DIR, f"{name_without_ext}.png")

            # 保存图像
            try:
                # 使用 bbox_inches='tight' 来自动裁剪掉多余的白边
                fig.savefig(output_path, dpi=DPI_RESOLUTION, bbox_inches='tight')
                print(f"  -> 成功保存至: {output_path}")
            except Exception as e:
                print(f"  -> 保存图片时出错: {e}")
        
        print("-" * 40)
        print("所有文件处理完毕！")

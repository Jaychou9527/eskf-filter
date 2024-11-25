import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path

def plot_3d_curve(filename, output_image):
    # 初始化列表存储 x, y, z 坐标
    x_data, y_data, z_data = [], [], []

    # 打开文件并读取数据
    with open(filename, 'r') as file:
        for line in file:
            # 将一行的三个数值分割并转换为浮点数
            x, y, z = map(float, line.split())
            x_data.append(x)
            y_data.append(y)
            z_data.append(z)

    # 创建 3D 图形
    fig = plt.figure()
    plt.title('position of uav', fontsize=16)
    ax = fig.add_subplot(111, projection='3d')

    # 绘制曲线
    ax.plot(x_data, y_data, z_data, label='3D Trajectory', color='blue', linewidth=2)

    # 设置图注、标题和轴标签
    ax.legend()
    ax.set_title('3D Curve Plot')
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')

    # 保存图像到文件
    plt.savefig(output_image, dpi=648, bbox_inches='tight')

    # 显示图形
    plt.show()

# 获取当前文件所在路径
current_file_path = Path(__file__).resolve()

# 获取上级路径
parent_path = current_file_path.parent
# 再次获取上一级路径
grandparent_path = parent_path.parent

# 读取文件数据
data_file = grandparent_path / 'output.txt'
with open(data_file, 'r') as file:
    data = file.readlines()
# 调用函数，传入生成的 txt 文件名和输出图片文件名
plot_3d_curve(data_file, 'trajectory_plot.png')


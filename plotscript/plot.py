import matplotlib.pyplot as plt
from pathlib import Path

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

# 转换数据为浮点数
data = [float(line.strip()) for line in data]

# 创建绘图
plt.figure(figsize=(15, 9))
plt.plot(data, marker='o', markersize=3, linestyle='-', color='b', label='height Curve')
plt.title('height of drone', fontsize=16)
plt.xlabel('steps', fontsize=14)
plt.ylabel('height/m', fontsize=14)
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend(fontsize=12)

# 保存绘图
output_image = 'data_curve.png'
plt.savefig(output_image, dpi=648, bbox_inches='tight')
plt.show()

print(f"Plot saved as {output_image}")

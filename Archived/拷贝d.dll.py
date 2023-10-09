import os
import shutil

# 源文件夹路径
# src_dir = r'E:\BaseMode\PCL 1.12.1\3rdParty\VTK\bin'
src_dir = r'E:\BaseMode\PCL 1.12.1_\bin'
# 目标文件夹路径
dst_dir = os.path.join(os.path.expanduser('~'), 'Desktop', 'build')

# 如果目标文件夹不存在，则创建它
if not os.path.exists(dst_dir):
    os.makedirs(dst_dir)

# 遍历源文件夹中的所有文件
for filename in os.listdir(src_dir):
    # 如果文件名以 *.dll 结尾
    if filename.endswith('d.dll'):
        # 构造源文件和目标文件的完整路径
        src_file = os.path.join(src_dir, filename)
        dst_file = os.path.join(dst_dir, filename)
        # 拷贝文件
        shutil.copy(src_file, dst_file)

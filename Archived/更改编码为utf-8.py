import os
import chardet

def convert_to_utf8(folder_path):
    for filename in os.listdir(folder_path):
        file_path = os.path.join(folder_path, filename)
        if os.path.isdir(file_path):
            convert_to_utf8(file_path)
        elif filename.endswith(('.h', '.cpp', '.hpp')):
            with open(file_path, 'rb') as f:
                data = f.read()
                encoding = chardet.detect(data)['encoding']
                text = data.decode(encoding)
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(text)

folder_path = 'E:/Baiusc/20230628/VolumeCalc'  # 文件夹路径
convert_to_utf8(folder_path)

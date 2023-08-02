# PointFitPlane 

### 选点拟合平面/预览，输入点，输出点所在的平面。基于PCL。
---
# 一、Git 拉代码
```bash
# github http
git clone https://github.com/Baiusc/PointFitPlane.git
# github ssh
git clone git@github.com:Baiusc/PointFitPlane.git
# gitee http
git clone https://gitee.com/Baiusc/point-fit-plane.git
# gitee ssh
git clone git@gitee.com/Baiusc/point-fit-plane.git
```
# 二、CMake build 项目
### 方式1 
先在`.gitignore`所在的工程目录建`build`文件夹，进入`build`内，用命令行 `cmake ..`
### 方式2
用`cmake-gui`，可建立vs解决方案。注意安装pcl库时如果用的是`vs 2019`，则用`cmake-gui`建立的解决方案也应选`vs 2019`

# 三、多平台联合推送
修改.git目录下的config文件：
```bash
[remote "origin"]
	url = git@github.com:Baiusc/PointFitPlane.git
	url = git@gitee.com:Baiusc/point-fit-plane.git
	fetch = +refs/heads/*:refs/remotes/origin/*
[branch "main"]
	remote = origin
	merge = refs/heads/main
```




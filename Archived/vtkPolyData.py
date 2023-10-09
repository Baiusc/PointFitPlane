import vtk
import random
import math


def mkVtkIdList(it):
    '''创建一个vtkIdList对象。vtkIdList是VTK中用于存储和管理点、单元、边等标识符的数据结构。
    函数mkVtkIdList接受一个可迭代的对象it作为参数，并遍历其中的每个元素i。
    对每个元素进行类型转换为整数，并使用vtkIdList的InsertNextId方法将其插入到vtkIdList对象中。
    '''
    vil = vtk.vtkIdList()
    for i in it:
        vil.InsertNextId(int(i))
    return vil

# 绘制通用方法
def myShow(cube):
    # Now we'll look at it.
    cubeMapper = vtk.vtkPolyDataMapper()
    if vtk.VTK_MAJOR_VERSION <= 5:
        cubeMapper.SetInput(cube)
    else:
        cubeMapper.SetInputData(cube)
        cubeMapper.SetScalarRange(0, 7)
        cubeActor = vtk.vtkActor()
        cubeActor.SetMapper(cubeMapper)
        cubeActor.GetProperty().SetColor(1.0, 0.0, 0.0) #所有立方体的初始颜色

    # The usual rendering stuff.
    camera = vtk.vtkCamera()
    camera.SetPosition(1, 1, 1)
    camera.SetFocalPoint(0, 0, 0)

    renderer = vtk.vtkRenderer()
    axes = vtk.vtkAxesActor()  # 创建坐标轴
    renderer.AddActor(axes)
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(renderer)

    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)

    renderer.AddActor(cubeActor)
    renderer.SetActiveCamera(camera)
    renderer.ResetCamera()
    renderer.SetBackground(0, 0, 0)

    renWin.SetSize(300, 300)

    # interact with data
    renWin.Render()
    iren.Start()
    del cubeMapper
    del cubeActor
    del camera
    del renderer
    del renWin
    del iren

def createRandomCubes():
    # 8个三维值代表长方体的8个顶点
    base_vertex = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (0.0, 1.0, 0.0),
                 (0.0, 0.0, 1.0), (1.0, 0.0, 1.0), (1.0, 1.0, 1.0), (0.0, 1.0, 1.0)]
    # 点的编号，每个面由4个点组成
    base_indices = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4),
           (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    num_cubes = 10*10000
    offset = 1.2
    cubes = []
    # 计算正方形边长
    side_length = int(math.ceil(math.sqrt(num_cubes)))

    for i in range(num_cubes):
        x_offset = (i % side_length) * offset
        y_offset = (i // side_length) * offset

        vertex = [(point[0] + x_offset,
                point[1] + y_offset,
                point[2]) for point in base_vertex]

        indices = [(point[0] + 8 * i,
                    point[1] + 8 * i,
                    point[2] + 8 * i,
                    point[3] + 8 * i) for point in base_indices]

        height = random.uniform(2, 20)  # 随机生成高度
        cubes.append((vertex, indices, height))

        
    polyData = vtk.vtkPolyData() # 多边形
    points = vtk.vtkPoints() # 点
    polys = vtk.vtkCellArray() # 点索引
    scalars = vtk.vtkFloatArray() # 点的标量
    # 填入点
    for index, cube in enumerate(cubes):
        vertex, indices, height = cube
        for point in vertex:
            points.InsertNextPoint(point) 
        for plane in indices:
            polys.InsertNextCell(mkVtkIdList(plane)) 
        for i in range(8):
            scalars.InsertNextValue(height)

    polyData.SetPoints(points)
    del points
    polyData.SetPolys(polys)
    del polys
    polyData.GetPointData().SetScalars(scalars)
    del scalars

    myShow(polyData)
    # Clean up
    del polyData



def create_cylinders(num_cylinders = 10000, side_length=1):
    # 创建 vtkPolyData 数据结构
    poly_data = vtk.vtkPolyData()
    points = vtk.vtkPoints()
    cells = vtk.vtkCellArray()
    scalars = vtk.vtkFloatArray()

    # 构建立方柱
    for i in range(num_cylinders):
        height = i  # 设置不同的高度

        # 添加立方柱的顶点坐标
        points.InsertNextPoint(-side_length / 2, -side_length / 2, height)
        points.InsertNextPoint(side_length / 2, -side_length / 2, height)
        points.InsertNextPoint(side_length / 2, side_length / 2, height)
        points.InsertNextPoint(-side_length / 2, side_length / 2, height)
        points.InsertNextPoint(-side_length / 2, -side_length / 2, height + 1)
        points.InsertNextPoint(side_length / 2, -side_length / 2, height + 1)
        points.InsertNextPoint(side_length / 2, side_length / 2, height + 1)
        points.InsertNextPoint(-side_length / 2, side_length / 2, height + 1)

        # 添加立方柱的面
        hexahedron = vtk.vtkHexahedron()
        hexahedron.GetPointIds().SetId(0, i * 8)
        hexahedron.GetPointIds().SetId(1, i * 8 + 1)
        hexahedron.GetPointIds().SetId(2, i * 8 + 2)
        hexahedron.GetPointIds().SetId(3, i * 8 + 3)
        hexahedron.GetPointIds().SetId(4, i * 8 + 4)
        hexahedron.GetPointIds().SetId(5, i * 8 + 5)
        hexahedron.GetPointIds().SetId(6, i * 8 + 6)
        hexahedron.GetPointIds().SetId(7, i * 8 + 7)

        # 添加面到单元连接信息
        
        cells.InsertNextCell(hexahedron)

        # 添加标量值
        for j in range(8):
            scalars.InsertNextValue(height)

    # 将数据赋值给 poly_data
    poly_data.SetPoints(points)
    poly_data.SetPolys(cells) # 修改这里，使用 SetPolys 方法
    poly_data.GetPointData().SetScalars(scalars)

    return poly_data

data = create_cylinders()
myShow(data)



import vtk
import random

def myShow(data):
    '''显示vtkDataSet类数据结构
    '''
    mapper = vtk.vtkDataSetMapper()
    mapper.SetInputData(data)
    mapper.SetScalarRange(0, 7)

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(1.0, 0.0, 0.0)

    camera = vtk.vtkCamera()
    camera.SetPosition(1, 1, 1)
    camera.SetFocalPoint(0, 0, 0)

    renderer = vtk.vtkRenderer()
    axes = vtk.vtkAxesActor()
    axes.SetTotalLength(10, 10, 10)
    renderer.AddActor(axes)

    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(renderer)

    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)

    renderer.AddActor(actor)
    renderer.SetActiveCamera(camera)
    renderer.ResetCamera()
    renderer.SetBackground(0, 0, 0)

    renWin.SetSize(800, 600)
    renWin.Render()
    iren.Start()
def createUnstructuredGrid(x_dim, y_dim, spacing, height, z_bottom:int = 0):
    '''用UnstructuredGrid创建立方体群
    '''
    # 创建vtkUnstructuredGrid数据结构
    unstructuredGrid = vtk.vtkUnstructuredGrid()

    # 创建点数据
    points = vtk.vtkPoints()
    index = 0
    for y in range(y_dim):
        for x in range(x_dim):
            points.InsertNextPoint(x * spacing[0], y * spacing[1], z_bottom)
    for y in range(y_dim):
        for x in range(x_dim):
            points.InsertNextPoint(x * spacing[0], y * spacing[1], height[index] + z_bottom)
            index += 1
    unstructuredGrid.SetPoints(points)

    # 创建单元格数据
    cells = vtk.vtkCellArray()
    for y in range(y_dim - 1):
        for x in range(x_dim - 1):
            index0 = (y + 0) * x_dim + (x + 0)
            index1 = (y + 0) * x_dim + (x + 1)
            index2 = (y + 1) * x_dim + (x + 1)
            index3 = (y + 1) * x_dim + (x + 0)
            index4 = (y + 0) * x_dim + (x + 0) + x_dim * y_dim
            index5 = (y + 0) * x_dim + (x + 1) + x_dim * y_dim
            index6 = (y + 1) * x_dim + (x + 1) + x_dim * y_dim
            index7 = (y + 1) * x_dim + (x + 0) + x_dim * y_dim
            print("Index 0:", index0)
            print("Index 1:", index1)
            print("Index 2:", index2)
            print("Index 3:", index3)
            print("Index 4:", index4)
            print("Index 5:", index5)
            print("Index 6:", index6)
            print("Index 7:", index7)
            hexahedron = vtk.vtkHexahedron()
            hexahedron.GetPointIds().SetId(0, index0)
            hexahedron.GetPointIds().SetId(1, index1)
            hexahedron.GetPointIds().SetId(2, index2)
            hexahedron.GetPointIds().SetId(3, index3)
            hexahedron.GetPointIds().SetId(4, index4)
            hexahedron.GetPointIds().SetId(5, index5)
            hexahedron.GetPointIds().SetId(6, index6)
            hexahedron.GetPointIds().SetId(7, index7)
            cells.InsertNextCell(hexahedron)
    unstructuredGrid.SetCells(vtk.VTK_HEXAHEDRON, cells)

    return unstructuredGrid

def createUnstructuredGrid2(x_dim, y_dim, spacing, height):
    '''用UnstructuredGrid创建立方体群
    '''
    # 创建vtkUnstructuredGrid数据结构
    unstructuredGrid = vtk.vtkUnstructuredGrid()

    # 创建点数据
    points = vtk.vtkPoints()
    index = 0
    for y in range(y_dim):
        for x in range(x_dim):
            z = height[index]
            index += 1
            points.InsertNextPoint(x * spacing[0], y * spacing[1], z)
    unstructuredGrid.SetPoints(points)

    # 创建单元格数据
    cells = vtk.vtkCellArray()
    for y in range(y_dim - 1):
        for x in range(x_dim - 1):
            hexahedron = vtk.vtkHexahedron()
            index0 = (y + 0) * x_dim + (x + 0)
            index1 = (y + 0) * x_dim + (x + 1)
            index2 = (y + 1) * x_dim + (x + 1)
            index3 = (y + 1) * x_dim + (x + 0)
            index4 = (y + 0) * x_dim + (x + 0) + x_dim * y_dim
            index5 = (y + 0) * x_dim + (x + 1) + x_dim * y_dim
            index6 = (y + 1) * x_dim + (x + 1) + x_dim * y_dim
            index7 = (y + 1) * x_dim + (x + 0) + x_dim * y_dim
            cells.InsertNextCell(hexahedron)

            print("Index 0:", index0)
            print("Index 1:", index1)
            print("Index 2:", index2)
            print("Index 3:", index3)
            print("Index 4:", index4)
            print("Index 5:", index5)
            print("Index 6:", index6)
            print("Index 7:", index7)
    unstructuredGrid.SetCells(vtk.VTK_HEXAHEDRON, cells)

    return unstructuredGrid
def printPoints(points):
    for i in range(points.GetNumberOfPoints()):
        p = points.GetPoint(i)
        print(f"Point {i}: ({p[0]}, {p[1]}, {p[2]})")

# 设置参数
x_dim = 3
y_dim = 3
spacing = [1, 1]

# 指定随机数范围
min = 0
max = 10

# 创建高度数组
height = [min + (max - min) * random.random() for _ in range(150)]

# 创建vtkUnstructuredGrid对象
data = createUnstructuredGrid(x_dim, y_dim, spacing, height)
# 获取点数据
points = data.GetPoints()

# 打印点数据
printPoints(points)


myShow(data)
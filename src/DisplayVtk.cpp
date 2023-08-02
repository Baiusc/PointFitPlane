
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkAxesActor.h>
#include <vtkProperty.h>

#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkImageData.h>
#include <vtkStructuredGrid.h>
#include <vtkUnstructuredGrid.h>
#include <vtkRectilinearGrid.h>
#include <vtkDataSetMapper.h>

#include <array>



#pragma region 渲染vtk数据结构
/// <summary>
/// vtk通用show方法
/// </summary>
/// <param name="data">vtkDataSet类的指针</param>
static void myShow(vtkDataSet* data) {
    vtkDataSetMapper* mapper = vtkDataSetMapper::New();
    mapper->SetInputData(data);

    mapper->SetScalarRange(0, 7);

    vtkActor* actor = vtkActor::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 0.0, 0.0);

    vtkCamera* camera = vtkCamera::New();
    camera->SetPosition(1, 1, 1);
    camera->SetFocalPoint(0, 0, 0);

    vtkRenderer* renderer = vtkRenderer::New();
    vtkAxesActor* axes = vtkAxesActor::New();
    // 设置x、y和z轴的长度
    axes->SetTotalLength(10, 10, 10);
    renderer->AddActor(axes);

    vtkRenderWindow* renWin = vtkRenderWindow::New();
    renWin->AddRenderer(renderer);

    vtkRenderWindowInteractor* iren = vtkRenderWindowInteractor::New();
    iren->SetRenderWindow(renWin);

    renderer->AddActor(actor);
    renderer->SetActiveCamera(camera);
    renderer->ResetCamera();
    renderer->SetBackground(0, 0, 0);

    renWin->SetSize(800, 600);
    renWin->Render();
    iren->Start();

    mapper->Delete();
    actor->Delete();
    camera->Delete();
    renderer->Delete();
    renWin->Delete();
    iren->Delete();
}

#pragma endregion

#pragma region 构造vtk数据结构
/// <summary>
/// 构建vtkIdList （存储一系列整数的列表）
/// </summary>
/// <param name="ids"></param>
/// <returns></returns>
static vtkIdList* mkVtkIdList(std::initializer_list<int> ids) {
    vtkIdList* vil = vtkIdList::New();
    for (auto id : ids) {
        vil->InsertNextId(id);
    }
    return vil;
}

/// <summary>
/// 测试用 随机生成100万个相邻的立方体
/// </summary>
static void createRandomHugeCubes() {
    // 创建相应的 vtkPolyData 对象以及相关的数据容器，如 vtkPoints、vtkCellArray 和 vtkFloatArray。
    vtkPolyData* polyData = vtkPolyData::New();
    vtkPoints* points = vtkPoints::New();
    vtkCellArray* polys = vtkCellArray::New();
    vtkFloatArray* scalars = vtkFloatArray::New();

    // 定义基准立方体的顶点和顶点索引
    std::vector<std::tuple<std::vector<std::tuple<double, double, double>>, std::vector<std::tuple<int, int, int, int>>, double>> cubes;
    std::vector<std::tuple<double, double, double>> base_vertex = {
        {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}, {1.0, 0.0, 1.0}, {1.0, 1.0, 1.0}, {0.0, 1.0, 1.0}
    };
    std::vector<std::tuple<int, int, int, int>> base_indices = {
        {0, 1, 2, 3}, {4, 5, 6, 7}, {0, 1, 5, 4},
        {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}
    };
    // 定义立方体数量、边长和间距
    int num_cubes = 10 * 10000;
    double offset = 1.2;
    int side_length = std::ceil(std::sqrt(num_cubes));
    // 生成立方体群cubes
    for (int i = 0; i < num_cubes; i++) {
        double x_offset = (i % side_length) * offset;
        double y_offset = (i / side_length) * offset;

        std::vector<std::tuple<double, double, double>> vertex;
        std::vector<std::tuple<int, int, int, int>> indices;
        double height = 2 + static_cast<double>(rand()) / RAND_MAX * (20 - 2);

        for (auto point : base_vertex) {
            vertex.push_back({ std::get<0>(point) + x_offset, std::get<1>(point) + y_offset, std::get<2>(point) });
        }

        for (auto plane : base_indices) {
            indices.push_back({ std::get<0>(plane) + 8 * i, std::get<1>(plane) + 8 * i, std::get<2>(plane) + 8 * i, std::get<3>(plane) + 8 * i });
        }

        cubes.push_back({ vertex, indices, height });
    }
    // 构建 polyData
    for (int i = 0; i < cubes.size(); i++) {
        std::vector<std::tuple<double, double, double>> vertex;
        std::vector<std::tuple<int, int, int, int>> indices;
        double height;
        std::tie(vertex, indices, height) = cubes[i];
        for (auto point : vertex) {
            points->InsertNextPoint(std::get<0>(point), std::get<1>(point), std::get<2>(point));
        }
        for (auto plane : indices) {
            polys->InsertNextCell(mkVtkIdList({ std::get<0>(plane), std::get<1>(plane), std::get<2>(plane), std::get<3>(plane) }));
        }
        for (int j = 0; j < 8; j++) {
            scalars->InsertNextValue(height); // 添加标量值
        }
    }
    // 将数据设置到 vtkPolyData 中
    polyData->SetPoints(points);
    points->Delete();

    polyData->SetPolys(polys);
    polys->Delete();

    polyData->GetPointData()->SetScalars(scalars);
    scalars->Delete();
    // 调用 myShow 函数来显示 polyData。
    myShow(polyData);
    polyData->Delete();
}
#pragma endregion
#pragma region 常用的vtk数据结构的构造
// 构造vtkImageData
static vtkSmartPointer<vtkImageData> CreateRandomImageData(int width, int height, int depth, int numComponents) {
    vtkSmartPointer<vtkImageData> imageData = vtkSmartPointer<vtkImageData>::New();

    imageData->SetDimensions(width, height, depth);
    imageData->SetSpacing(1.0, 1.0, 1.0);
    imageData->SetOrigin(0.0, 0.0, 0.0);

    imageData->AllocateScalars(VTK_DOUBLE, numComponents);

    vtkSmartPointer<vtkMath> random = vtkSmartPointer<vtkMath>::New();

    for (int z = 0; z < depth; ++z) {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                double* pixel = static_cast<double*>(imageData->GetScalarPointer(x, y, z));

                for (int c = 0; c < numComponents; ++c) {
                    pixel[c] = random->Random();
                }
            }
        }
    }

    return imageData;
}

#include <vtkImageData.h>
#include <vtkSmartPointer.h>
#include <array>
// 构造vtkImageData
vtkSmartPointer<vtkImageData> createDice(int dimX, int dimY, int dimZ, int spacing) {
    vtkSmartPointer<vtkImageData> imageData = vtkSmartPointer<vtkImageData>::New();
    imageData->SetDimensions(dimX + 2 * spacing, dimY + 2 * spacing, dimZ + 2 * spacing);
    imageData->AllocateScalars(VTK_UNSIGNED_CHAR, 3);

    std::array<std::array<int, 3>, 6> colors = { {
        {255, 0, 0}, // 红色
        {255, 255, 0}, // 黄色
        {0, 255, 0}, // 绿色
        {0, 255, 255}, // 青色
        {0, 0, 255}, // 蓝色
        {255, 0, 255} // 紫色
    } };
    std::array<int, 3> gray = { {234,237,246} }; // 灰色

    unsigned char* pixel = static_cast<unsigned char*>(imageData->GetScalarPointer(0, 0, 0));
    for (int z = 0; z < dimZ + 2 * spacing; z++) {
        for (int y = 0; y < dimY + 2 * spacing; y++) {
            for (int x = 0; x < dimX + 2 * spacing; x++) {
                if (x >= spacing && x < dimX + spacing && y >= spacing && y < dimY + spacing && z >= spacing && z < dimZ + spacing) {
                    pixel[0] = gray[0];
                    pixel[1] = gray[1];
                    pixel[2] = gray[2];
                }
                else if (x < spacing) {
                    if ((y / spacing) % 2 == (z / spacing) % 2) {
                        pixel[0] = colors[0][0];
                        pixel[1] = colors[0][1];
                        pixel[2] = colors[0][2];
                    }
                    else {
                        pixel[0] = gray[0];
                        pixel[1] = gray[1];
                        pixel[2] = gray[2];
                    }
                }
                else if (x >= dimX + spacing) {
                    if ((y / spacing) % 2 == (z / spacing) % 2) {
                        pixel[0] = colors[1][0];
                        pixel[1] = colors[1][1];
                        pixel[2] = colors[1][2];
                    }
                    else {
                        pixel[0] = gray[0];
                        pixel[1] = gray[1];
                        pixel[2] = gray[2];
                    }
                }
                else if (y < spacing) {
                    if ((x / spacing) % 2 == (z / spacing) % 2) {
                        pixel[0] = colors[2][0];
                        pixel[1] = colors[2][1];
                        pixel[2] = colors[2][2];
                    }
                    else {
                        pixel[0] = gray[0];
                        pixel[1] = gray[1];
                        pixel[2] = gray[2];
                    }
                }
                else if (y >= dimY + spacing) {
                    if ((x / spacing) % 2 == (z / spacing) % 2) {
                        pixel[0] = colors[3][0];
                        pixel[1] = colors[3][1];
                        pixel[2] = colors[3][2];
                    }
                    else {
                        pixel[0] = gray[0];
                        pixel[1] = gray[1];
                        pixel[2] = gray[2];
                    }
                }
                else if (z < spacing) {
                    if ((x / spacing) % 2 == (y / spacing) % 2) {
                        pixel[0] = colors[4][0];
                        pixel[1] = colors[4][1];
                        pixel[2] = colors[4][2];
                    }
                    else {
                        pixel[0] = gray[0];
                        pixel[1] = gray[1];
                        pixel[2] = gray[2];
                    }
                }
                else {
                    if ((x / spacing) % 2 == (y / spacing) % 2) {
                        pixel[0] = colors[5][0];
                        pixel[1] = colors[5][1];
                        pixel[2] = colors[5][2];
                    }
                    else {
                        pixel[0] = gray[0];
                        pixel[1] = gray[1];
                        pixel[2] = gray[2];
                    }
                }
                pixel += 3;
            }
        }
    }

    return imageData;
}

#include <vtkStructuredGrid.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
// 构造vtkStructuredGrid
vtkSmartPointer<vtkStructuredGrid> createStructuredGrid(int x_dim, int y_dim, double spacing[3], double height[])
{
    // 创建vtkStructuredGrid数据结构
    vtkSmartPointer<vtkStructuredGrid> structuredGrid = vtkSmartPointer<vtkStructuredGrid>::New();
    structuredGrid->SetDimensions(x_dim, y_dim, 1);

    // 创建点数据
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    int index = 0;
    for (int y = 0; y < y_dim; ++y)
    {
        for (int x = 0; x < x_dim; ++x)
        {
            double z = height[index++];
            points->InsertNextPoint(x * spacing[0], y * spacing[1], z);
        }
    }
    structuredGrid->SetPoints(points);

    return structuredGrid;
}
void printPoints(vtkSmartPointer<vtkPoints> points)
{
    for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i)
    {
        double p[3];
        points->GetPoint(i, p);
        std::cout << "Point " << i << ": (" << p[0] << ", " << p[1] << ", " << p[2] << ")" << std::endl;
    }
}
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>
#include <vtkHexahedron.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>

vtkSmartPointer<vtkUnstructuredGrid> createUnstructuredGrid(int x_dim, int y_dim, double spacing[3], double height[])
{
    // 创建vtkUnstructuredGrid数据结构
    vtkSmartPointer<vtkUnstructuredGrid> unstructuredGrid = vtkSmartPointer<vtkUnstructuredGrid>::New();

    // 创建点数据
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    int index = 0;
    for (int y = 0; y < y_dim; ++y)
    {
        for (int x = 0; x < x_dim; ++x)
        {
            double z = height[index++];
            points->InsertNextPoint(x * spacing[0], y * spacing[1], z);
        }
    }
    unstructuredGrid->SetPoints(points);

    // 创建单元格数据
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    for (int y = 0; y < y_dim - 1; ++y)
    {
        for (int x = 0; x < x_dim - 1; ++x)
        {
            vtkSmartPointer<vtkHexahedron> hexahedron = vtkSmartPointer<vtkHexahedron>::New();
            hexahedron->GetPointIds()->SetId(0, (y + 0) * x_dim + (x + 0));
            hexahedron->GetPointIds()->SetId(1, (y + 0) * x_dim + (x + 1));
            hexahedron->GetPointIds()->SetId(2, (y + 1) * x_dim + (x + 1));
            hexahedron->GetPointIds()->SetId(3, (y + 1) * x_dim + (x + 0));
            hexahedron->GetPointIds()->SetId(4, (y + 0) * x_dim + (x + 0) + x_dim * y_dim);
            hexahedron->GetPointIds()->SetId(5, (y + 0) * x_dim + (x + 1) + x_dim * y_dim);
            hexahedron->GetPointIds()->SetId(6, (y + 1) * x_dim + (x + 1) + x_dim * y_dim);
            hexahedron->GetPointIds()->SetId(7, (y + 1) * x_dim + (x + 0) + x_dim * y_dim);
            cells->InsertNextCell(hexahedron);
        }
    }
    unstructuredGrid->SetCells(VTK_HEXAHEDRON, cells);

    return unstructuredGrid;
}



#pragma endregion





int maine(int argc, char** argv)
{
    createRandomHugeCubes(); // 随机创建立方体群并调用vtk显示

    //// 设置参数
    //int x_dim = 10;
    //int y_dim = 10;
    //double spacing[3] = { 2.0, 2.0, 2.0 };

    //// 初始化随机数生成器
    //std::srand(std::time(nullptr));

    //// 指定随机数范围
    //double min = 0;
    //double max = 10;

    //// 创建高度数组
    //double height[150];
    //for (int i = 0; i < 150; ++i)
    //{
    //    height[i] = min + (max - min) * std::rand() / static_cast<double>(RAND_MAX);
    //}
    //// 创建vtkUnstructuredGrid对象
    //vtkSmartPointer<vtkUnstructuredGrid> data = createUnstructuredGrid(x_dim, y_dim, spacing, height);
    //
    //myShow(data);
    return 0;
}


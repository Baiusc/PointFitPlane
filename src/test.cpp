#include "Test.h"
//#include <mutex>
//#include "vtkAbstractTransform.h"

namespace Test
{
    __declspec(dllexport)
    // 
    int testFunc()
    {

        std::cout << "C++" << std::endl;

        return 3;
    }
}
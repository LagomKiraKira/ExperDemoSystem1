#include "ExperDemoSystem.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    // Disable IPP
    cv::setUseOptimized(false);

    // ======== 新增这行：禁用 Qt 的自动粗暴缩放，改用物理像素 ========
    qputenv("QT_ENABLE_HIGHDPI_SCALING", "0");
    // ==========================================================

    QApplication a(argc, argv);
    
    try {
        ExperDemoSystem w;
        w.show();
        return a.exec();
    }
    catch (const std::runtime_error& e) {
        std::cout << "Exception caught:" << e.what();
        return EXIT_FAILURE;
    }
    
}

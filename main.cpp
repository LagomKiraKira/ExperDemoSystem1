#include "ExperDemoSystem.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    // Disable IPP
    cv::setUseOptimized(false);

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

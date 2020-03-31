#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/SoDB.h>
#include <QApplication>
#include <QFileDialog>
#include <QMessageBox>
#include <QWidget>
#include <stdexcept>
#include "rl_build/include/sg/so/Scene.h"

int main(int argc, char** argv) {
  try {
    // SoDB::init();	//compile error

    QWidget* widget = SoQt::init(argc, argv, argv[0]);
    widget->resize(800, 600);

    QString filename =
        "/home/derek/XR1_WS/src/src/RoboticsLib/rl_build/xml_examples/rlsg/"
        "unimation-puma560_boxes.convex.xml";

    rl::sg::so::Scene scene;

    if (!filename.isEmpty()) {
      scene.load(filename.toStdString());
    }

    SoQtExaminerViewer viewer(widget, nullptr, true,
                              SoQtFullViewer::BUILD_POPUP);
    viewer.setSceneGraph(scene.root);
    viewer.setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
    viewer.show();

    widget->setWindowTitle(filename + (filename.isEmpty() ? "" : " - ") +
                           "rlViewDemo");

    SoQt::show(widget);
    SoQt::mainLoop();

    return EXIT_SUCCESS;
  } catch (const std::exception& e) {
    QApplication application(argc, argv);
    QMessageBox::critical(nullptr, "Error", e.what());
    return EXIT_FAILURE;
  }
}

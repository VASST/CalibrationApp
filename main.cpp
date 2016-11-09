// Main app file
#include "mainWidget.h"

// VTK
#include <vtk_glew.h>

// OpenCV
#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

#include <QApplication>

int main(int argc, char* argv[])
{

  QApplication app(argc, argv);
  app.setOrganizationName("Robarts Research Institute, Canada");
  app.setApplicationName("Phantom-Less Calibration");

  mainWidget mainWin;
  mainWin.show();
  return app.exec();
}


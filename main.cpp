// VTK
#include <vtk_glew.h>
#include <vtkImageActor.h>
#include <vtkImageData.h>
#include <vtkImageImport.h>
#include <vtkImageMapper.h>
#include <vtkInteractorStyleImage.h>
#include <vtkPNGWriter.h>
#include <vtkPlaneSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTexture.h>
#include <vtkTextureMapToPlane.h>

// Main app file
#include "mainWidget.h"

// VTK OpenVR
#include <vtkOpenVRCamera.h>
#include <vtkOpenVRRenderer.h>
#include <vtkOpenVRRenderWindow.h>
#include <vtkOpenVRRenderWindowInteractor.h>

// OvrvisionPro includes
#include <ovrvision_pro.h>
#include <ovrvision_setting.h>

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


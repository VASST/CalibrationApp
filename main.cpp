#include <I:/d/VTK/ThirdParty/glew/vtkglew/include/GL/glew.h>
// Main app file
#include "mainWidget.h"

// VTK
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTexture.h>
#include <vtkImageImport.h>
#include <vtkImageMapper.h>
#include <vtkImageActor.h>
#include <vtkImageData.h>
#include <vtkInteractorStyleImage.h>
#include <vtkPNGWriter.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkTextureMapToPlane.h>
#include <vtkPlaneSource.h>

// VTK OpenVR
#include <vtkOpenVRCamera.h>
#include <vtkOpenVRRenderer.h>
#include <vtkOpenVRRenderWindow.h>
#include <vtkOpenVRRenderWindowInteractor.h>

#include <ovrvision_pro.h>
#include <../src/lib_src/ovrvision_setting.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>

#include <QApplication>

int main(int argc, char *argv[])
{
	CoInitialize(nullptr);

	QApplication app(argc, argv);
	app.setOrganizationName("Robarts Research Institute, Canada");
	app.setApplicationName("Phantom-Less Calibration");

	mainWidget mainWin;
	mainWin.show();
	return app.exec();
}


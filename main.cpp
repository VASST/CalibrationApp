#include <I:/d/VTK/ThirdParty/glew/vtkglew/include/GL/glew.h>

// Main app file
#include "mainWidget.h"

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


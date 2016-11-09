/*=========================================================================

Program:   Phantom-Less Calibration GUI
Module:    $RCSfile: mainWidget.cpp,v $
Creator:   Elvis C. S. Chen <chene@robarts.ca>
Language:  C++
Author:    $Author: Elvis Chen $
Date:      $Date: 2013/05/03 15:45:30 $
Version:   $Revision: 0.99 $

==========================================================================

Copyright (c) Elvis C. S. Chen, elvis.chen@gmail.com

Use, modification and redistribution of the software, in source or
binary forms, are permitted provided that the following terms and
conditions are met:

1) Redistribution of the source code, in verbatim or modified
form, must retain the above copyright notice, this license,
the following disclaimer, and any notices that refer to this
license and/or the following disclaimer.

2) Redistribution in binary form must include the above copyright
notice, a copy of this license and the following disclaimer
in the documentation or with other materials provided with the
distribution.

3) Modified copies of the source code must be clearly marked as such,
and must not be misrepresented as verbatim copies of the source code.

THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE SOFTWARE "AS IS"
WITHOUT EXPRESSED OR IMPLIED WARRANTY INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE.  IN NO EVENT SHALL ANY COPYRIGHT HOLDER OR OTHER PARTY WHO MAY
MODIFY AND/OR REDISTRIBUTE THE SOFTWARE UNDER THE TERMS OF THIS LICENSE
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, LOSS OF DATA OR DATA BECOMING INACCURATE
OR LOSS OF PROFIT OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF
THE USE OR INABILITY TO USE THE SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGES.


=========================================================================*/

// Must come first, as other includes load gl.h
#include <vtk_glew.h>

// local includes
#include "mainWidget.h"
#include "eccTrackerWidget.h"

// C++ includes
#include <vector>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

// QT includes
#include <QAction>
#include <QDockWidget>
#include <QFrame>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QStatusBar>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QtGui>

// VTK includes
#include <QVTKWidget.h>
#include <vtkNDITracker.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkTrackerTool.h>
#include <vtkTransform.h>

// VTK
#include <vtkImageData.h>
#include <vtkImageImport.h>
#include <vtkImageMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkSmartPointer.h>
#include <vtkTexture.h>
#include <vtkImageImport.h>
#include <vtkImageMapper.h>
#include <vtkImageData.h>
#include <vtkTextureMapToPlane.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataReader.h>
#include <vtkActor.h>
#include <vtkSTLReader.h>
#include <vtksys/SystemTools.hxx>
#include <vtkXMLPolyDataReader.h>
#include <vtkProperty.h>
#include <vtkOBJReader.h>
#include <vtkPLYReader.h>
#include <vtkSphereSource.h>

// VTK OpenVR
#include <vtkOpenVRCamera.h>
#include <vtkOpenVRRenderer.h>
#include <vtkOpenVRRenderWindow.h>
#include <vtkOpenVRRenderWindowInteractor.h>
#include <ovrvision_pro.h>
#include <../src/lib_src/ovrvision_setting.h>

// OpenCV
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// Elvis Math
#include "matrix.h"
#include "mathUtils.h"

// WinRT includes
#include <ppltasks.h>

using namespace cv;
using namespace std;
using namespace echen;

template< class PReader > vtkPolyData *readAnPolyData(const char *fname) {
	vtkSmartPointer< PReader > reader =
		vtkSmartPointer< PReader >::New();
	reader->SetFileName(fname);
	reader->Update();
	reader->GetOutput()->Register(reader);
	return(vtkPolyData::SafeDownCast(reader->GetOutput()));
}

mainWidget::mainWidget(QWidget* parent)
  : QMainWindow(parent)
{
  /*!
  * use QVTKWidget as the centralwidget of the mainwidget.
  */
  qvtk = new QVTKWidget();
  qvtk->setMinimumSize(480, 480);
  setCentralWidget(qvtk);
	
  //
  // VTK related objects
  //
  createVTKObjects();
  setupVTKPipeline();
	setupARRendering();

  /*!
  * create the rest of the QT/GUI.
  */
  controlDock = 0;
  createActions();
  createMenus();
  createStatusBar();
  createToolInformation();
}

mainWidget::~mainWidget()
{
  myTracker->StopTracking();   /*!< Make sure the tracker is stopped when the program exits. */
  this->destroyVTKObjects();   /*!< VTK cleanup. */

	if (ovrvisionProHandle.isOpen()) /*!< Close ovrvision device if still open upon exit. */
  {
		ovrvisionProHandle.Close();
  }
}

void mainWidget::ovrvisionUpdate()
{
	ovrvisionProHandle.SetCameraExposure(22500);

  // Query the SDK for the latest frames
	ovrvisionProHandle.PreStoreCamData(OVR::OV_CAMQT_DMSRMP);

  // Grab Left and Right Images
	cv::Mat matLeft(ovrvisionProHandle.GetCamHeight(), ovrvisionProHandle.GetCamWidth(), CV_8UC4, ovrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_LEFT));
	cv::Mat matRight(ovrvisionProHandle.GetCamHeight(), ovrvisionProHandle.GetCamWidth(), CV_8UC4, ovrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_RIGHT));
}

Point pt1;
void mainWidget::onMouse(int event, int x, int y, int flags, void* param)
{
  switch (event)
  {
  case CV_EVENT_LBUTTONDOWN:
  {
    pt1.x = x;
    pt1.y = y;
    break;
  }
  case CV_EVENT_LBUTTONUP:
  {
    break;
  }

  }
}

void mainWidget::calculateProjectionError()
{
	// Query the SDK for the latest frames
	ovrvisionProHandle.PreStoreCamData(OVR::OV_CAMQT_DMSRMP);

	// Grab image
	cv::Mat matLeft(ovrvisionProHandle.GetCamHeight(), ovrvisionProHandle.GetCamWidth(), CV_8UC4, ovrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_LEFT));
	
	// Undistort image
	cv::Mat undistorted;
	undistort(matLeft, undistorted, intrinsic, distortion);

	// Set p2l hand-eye calibration
	vtkSmartPointer<vtkTransform> p2lTransform = vtkSmartPointer<vtkTransform>::New();
	vtkSmartPointer<vtkMatrix4x4> p2lMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	p2lMatrix->SetElement(0, 0, 0.0677294484076749);
	p2lMatrix->SetElement(0, 1, -0.992989179048552); 
	p2lMatrix->SetElement(0, 2, -0.0968773044158076);
	p2lMatrix->SetElement(0, 3, -61.6285338697333); 

	p2lMatrix->SetElement(1, 0, 0.737296753348973); 
	p2lMatrix->SetElement(1, 1, 0.11523277439025); 
	p2lMatrix->SetElement(1, 2, -0.665668765383645); 
	p2lMatrix->SetElement(1, 3, -14.6388968911687); 

	p2lMatrix->SetElement(2, 0, 0.672165321419851); 
	p2lMatrix->SetElement(2, 1, -0.0263419437173227); 
	p2lMatrix->SetElement(2, 2, 0.739932350071099); 
	p2lMatrix->SetElement(2, 3, -4.60575695614759); 

	p2lMatrix->SetElement(3, 0, 0);
	p2lMatrix->SetElement(3, 1, 0);
	p2lMatrix->SetElement(3, 2, 0);
	p2lMatrix->SetElement(3, 3, 1);

	p2lTransform->PostMultiply();
	p2lTransform->Identity();
	p2lTransform->Concatenate(referenceCoil->GetTransform());
	p2lTransform->Concatenate(oculusHMD->GetTransform()->GetLinearInverse());
	p2lTransform->Concatenate(p2lMatrix);
	double posePosition[3];

	p2lTransform->GetPosition(posePosition);

	//Set Tsai calibration
	vtkSmartPointer<vtkTransform> tsaiTransform = vtkSmartPointer<vtkTransform>::New();
	vtkSmartPointer<vtkMatrix4x4> tsaiMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	tsaiMatrix->SetElement(0, 0, 0.06308);
	tsaiMatrix->SetElement(0, 1, -0.99365);
	tsaiMatrix->SetElement(0, 2, -0.093137);
	tsaiMatrix->SetElement(0, 3, -58.705);

	tsaiMatrix->SetElement(1, 0, 0.7493);
	tsaiMatrix->SetElement(1, 1, 0.1088);
	tsaiMatrix->SetElement(1, 2, -0.65324);
	tsaiMatrix->SetElement(1, 3, -20.885);

	tsaiMatrix->SetElement(2, 0, 0.65922);
	tsaiMatrix->SetElement(2, 1, -0.028581);
	tsaiMatrix->SetElement(2, 2, 0.7514);
	tsaiMatrix->SetElement(2, 3, 0.98286);

	tsaiMatrix->SetElement(3, 0, 0);
	tsaiMatrix->SetElement(3, 1, 0);
	tsaiMatrix->SetElement(3, 2, 0);
	tsaiMatrix->SetElement(3, 3, 1);

	tsaiTransform->PostMultiply();
	tsaiTransform->Identity();
	tsaiTransform->Concatenate(referenceCoil->GetTransform());
	tsaiTransform->Concatenate(oculusHMD->GetTransform()->GetLinearInverse());
	tsaiTransform->Concatenate(tsaiMatrix);
	double tsaiPosition[3];
	tsaiTransform->GetPosition(tsaiPosition);

	// Set Navy calibration
	vtkSmartPointer<vtkTransform> navyTransform = vtkSmartPointer<vtkTransform>::New();
	vtkSmartPointer<vtkMatrix4x4> navyMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	navyMatrix->SetElement(0, 0, 0.066094);
	navyMatrix->SetElement(0, 1, -0.9942);
	navyMatrix->SetElement(0, 2, -0.084801);
	navyMatrix->SetElement(0, 3, -59.594);

	navyMatrix->SetElement(1, 0, 0.75048);
	navyMatrix->SetElement(1, 1, 0.10554);
	navyMatrix->SetElement(1, 2, -0.65242);
	navyMatrix->SetElement(1, 3, -21.016);

	navyMatrix->SetElement(2, 0, 0.65759);
	navyMatrix->SetElement(2, 1, -0.02052);
	navyMatrix->SetElement(2, 2, 0.7531);
	navyMatrix->SetElement(2, 3, 1.9691);

	navyMatrix->SetElement(3, 0, 0);
	navyMatrix->SetElement(3, 1, 0);
	navyMatrix->SetElement(3, 2, 0);
	navyMatrix->SetElement(3, 3, 1);

	navyTransform->PostMultiply();
	navyTransform->Identity();
	navyTransform->Concatenate(referenceCoil->GetTransform());
	navyTransform->Concatenate(oculusHMD->GetTransform()->GetLinearInverse());
	navyTransform->Concatenate(navyMatrix);
	double navyPosition[3];
	navyTransform->GetPosition(navyPosition);

	// Set inria calibration
	vtkSmartPointer<vtkTransform> inriaTransform = vtkSmartPointer<vtkTransform>::New();
	vtkSmartPointer<vtkMatrix4x4> inriaMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	inriaMatrix->SetElement(0, 0, 0.066176);
	inriaMatrix->SetElement(0, 1, -0.9942);
	inriaMatrix->SetElement(0, 2, -0.084783);
	inriaMatrix->SetElement(0, 3, -59.6);

	inriaMatrix->SetElement(1, 0, 0.75052);
	inriaMatrix->SetElement(1, 1, 0.10559);
	inriaMatrix->SetElement(1, 2, -0.65235);
	inriaMatrix->SetElement(1, 3, -21.022);

	inriaMatrix->SetElement(2, 0, 0.65752);
	inriaMatrix->SetElement(2, 1, -0.020461);
	inriaMatrix->SetElement(2, 2, 0.75316);
	inriaMatrix->SetElement(2, 3, 1.9791);

	inriaMatrix->SetElement(3, 0, 0);
	inriaMatrix->SetElement(3, 1, 0);
	inriaMatrix->SetElement(3, 2, 0);
	inriaMatrix->SetElement(3, 3, 1);

	inriaTransform->PostMultiply();
	inriaTransform->Identity();
	inriaTransform->Concatenate(referenceCoil->GetTransform());
	inriaTransform->Concatenate(oculusHMD->GetTransform()->GetLinearInverse());
	inriaTransform->Concatenate(inriaMatrix);
	double inriaPosition[3];
	inriaTransform->GetPosition(inriaPosition);

	// Set Dual calibration
	vtkSmartPointer<vtkTransform> dualTransform = vtkSmartPointer<vtkTransform>::New();
	vtkSmartPointer<vtkMatrix4x4> dualMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	dualMatrix->SetElement(0, 0, 0.076184);
	dualMatrix->SetElement(0, 1, -0.99475);
	dualMatrix->SetElement(0, 2, -0.068354);
	dualMatrix->SetElement(0, 3, -63.642);

	dualMatrix->SetElement(1, 0, 0.76375);
	dualMatrix->SetElement(1, 1, 0.10229);
	dualMatrix->SetElement(1, 2, -0.63736);
	dualMatrix->SetElement(1, 3, -24.856);

	dualMatrix->SetElement(2, 0, 0.641);
	dualMatrix->SetElement(2, 1, -0.0036492);
	dualMatrix->SetElement(2, 2, 0.76753);
	dualMatrix->SetElement(2, 3, 5.0178);

	dualMatrix->SetElement(3, 0, 0);
	dualMatrix->SetElement(3, 1, 0);
	dualMatrix->SetElement(3, 2, 0);
	dualMatrix->SetElement(3, 3, 1);

	dualTransform->PostMultiply();
	dualTransform->Identity();
	dualTransform->Concatenate(referenceCoil->GetTransform());
	dualTransform->Concatenate(oculusHMD->GetTransform()->GetLinearInverse());
	dualTransform->Concatenate(dualMatrix);
	double dualPosition[3];
	dualTransform->GetPosition(dualPosition);

	// Set branch calibration
	vtkSmartPointer<vtkTransform> branchTransform = vtkSmartPointer<vtkTransform>::New();
	vtkSmartPointer<vtkMatrix4x4> branchMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	branchMatrix->SetElement(0, 0, 0.0793);
	branchMatrix->SetElement(0, 1, -0.9917);
	branchMatrix->SetElement(0, 2, -0.1013);
	branchMatrix->SetElement(0, 3, -54.1888);

	branchMatrix->SetElement(1, 0, 0.7845);
	branchMatrix->SetElement(1, 1, 0.1248);
	branchMatrix->SetElement(1, 2, -0.6075);
	branchMatrix->SetElement(1, 3, -34.1630);

	branchMatrix->SetElement(2, 0, 0.6151);
	branchMatrix->SetElement(2, 1, -0.0313);
	branchMatrix->SetElement(2, 2, 0.7878);
	branchMatrix->SetElement(2, 3, -1.5138);

	branchMatrix->SetElement(3, 0, 0);
	branchMatrix->SetElement(3, 1, 0);
	branchMatrix->SetElement(3, 2, 0);
	branchMatrix->SetElement(3, 3, 1);

	branchTransform->PostMultiply();
	branchTransform->Identity();
	branchTransform->Concatenate(referenceCoil->GetTransform());
	branchTransform->Concatenate(oculusHMD->GetTransform()->GetLinearInverse());
	branchTransform->Concatenate(branchMatrix);
	double branchPosition[3];
	branchTransform->GetPosition(branchPosition);

	// Find circle in image
	cv::Mat hsv;
	cv::Mat threshold;

	// Convert BGRA image to HSV image
	cv::cvtColor(undistorted, hsv, COLOR_BGR2HSV);
	cv::Mat drawing;

	cv::cvtColor(undistorted, drawing, COLOR_BGR2RGB);
	drawing.setTo(cv::Scalar(0, 0, 0));

	// Filter everything except red - (0, 70, 50) -> (10, 255, 255) & (160, 70, 50) -> (179, 255, 255)
	cv::inRange(hsv, cv::Scalar(HMinLower->value(), SMinLower->value(), VMinLower->value()), cv::Scalar(HMaxLower->value(), SMaxLower->value(), VMaxLower->value()), thresholdFinal);
	cv::inRange(hsv, cv::Scalar(HMinUpper->value(), SMinUpper->value(), VMinUpper->value()), cv::Scalar(HMaxUpper->value(), SMaxUpper->value(), VMaxUpper->value()), threshold);

	cv::Mat mask;
	cv::addWeighted(thresholdFinal, 1.0, threshold, 1.0, 0.0, mask);

	// Create a Gaussian & median Blur Filter
	medianBlur(mask, mask, 5);
	GaussianBlur(mask, mask, Size(9, 9), 2, 2);
	vector<Vec3f> circles;

	// Draw the circles detected
	Mat canny_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	// Apply the Hough Transform to find the circles
	HoughCircles(mask, circles, CV_HOUGH_GRADIENT, 2, mask.rows / 16, 255, 30);

	vector<Point2f> finalCenter(1);
	// Outline circle and centroid
	if (circles.size() > 0)
	{
		Point center(circles[0][0], circles[0][1]);
		int radius = circles[0][2];

		// Draw circle on image
		circle(drawing, center, radius, Scalar(100, 100, 100), -1, 8, 0);

		int thresh = 100;
		int max_thresh = 255;
		RNG rng(12345);

		// Detect edges using canny
		Canny(drawing, canny_output, thresh, thresh * 2, 3);

		// Find contours
		findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		/// Approximate contours to polygons + get bounding rects and circles
		vector<vector<Point> > contours_poly(contours.size());
		vector<Point2f>centerTwo(contours.size());
		vector<float>radiusTwo(contours.size());

		for (int i = 0; i < contours.size(); i++)
		{
			approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true); // Finds polygon
			minEnclosingCircle((Mat)contours_poly[i], centerTwo[i], radiusTwo[i]); // Finds circle

			Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		}
		finalCenter[0].x = centerTwo[0].x;
		finalCenter[0].y = centerTwo[0].y;
	}

	// If Hough Circles fails to find the circle
	else if (circles.size() == 0)
	{
		int thresh = 100;
		int max_thresh = 255;
		RNG rng(12345);

		medianBlur(mask, mask, 3);

		// Detect edges using canny
		Canny(mask, canny_output, thresh, thresh * 2, 3);

		// Find contours
		findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		/// Approximate contours to polygons + get bounding rects and circles
		vector<vector<Point> > contours_poly(contours.size());
		vector<Rect> boundRect(contours.size());
		vector<Point2f>center(contours.size());
		vector<float>radius(contours.size());

		for (int i = 0; i < contours.size(); i++)
		{
			approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true); // Finds polygon
			boundRect[i] = boundingRect(Mat(contours_poly[i]));		   // Finds rectangle
			minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]); // Finds circle
		}

		/// Draw circle
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

		finalCenter[0].x = center[0].x;
		finalCenter[0].y = center[0].y;
	}

	// Calculate position of centroid for each hand-eye calibration
	double xPrime = posePosition[0] / posePosition[2];
	double yPrime = posePosition[1] / posePosition[2];

	double u = (leftIntrinsicParam[0][0] * xPrime) + leftIntrinsicParam[0][2];
	double v = (leftIntrinsicParam[1][1] * yPrime) + leftIntrinsicParam[1][2];

	double xPrime3 = tsaiPosition[0] / tsaiPosition[2];
	double yPrime3 = tsaiPosition[1] / tsaiPosition[2];

	double u3 = (leftIntrinsicParam[0][0] * xPrime3) + leftIntrinsicParam[0][2];
	double v3 = (leftIntrinsicParam[1][1] * yPrime3) + leftIntrinsicParam[1][2];

	double xPrime4 = navyPosition[0] / navyPosition[2];
	double yPrime4 = navyPosition[1] / navyPosition[2];

	double u4 = (leftIntrinsicParam[0][0] * xPrime4) + leftIntrinsicParam[0][2];
	double v4 = (leftIntrinsicParam[1][1] * yPrime4) + leftIntrinsicParam[1][2];

	double xPrime5 = inriaPosition[0] / inriaPosition[2];
	double yPrime5 = inriaPosition[1] / inriaPosition[2];

	double u5 = (leftIntrinsicParam[0][0] * xPrime5) + leftIntrinsicParam[0][2];
	double v5 = (leftIntrinsicParam[1][1] * yPrime5) + leftIntrinsicParam[1][2];

	double xPrime6 = dualPosition[0] / dualPosition[2];
	double yPrime6 = dualPosition[1] / dualPosition[2];

	double u6 = (leftIntrinsicParam[0][0] * xPrime6) + leftIntrinsicParam[0][2];
	double v6 = (leftIntrinsicParam[1][1] * yPrime6) + leftIntrinsicParam[1][2];

	double xPrime7 = branchPosition[0] / branchPosition[2];
	double yPrime7 = branchPosition[1] / branchPosition[2];

	double u7 = (leftIntrinsicParam[0][0] * xPrime7) + leftIntrinsicParam[0][2];
	double v7 = (leftIntrinsicParam[1][1] * yPrime7) + leftIntrinsicParam[1][2];

	// Calculate distance between points
	double x1 = finalCenter[0].x;
	double y1 = finalCenter[0].y;
	double x2 = u;
	double y2 = v;
	double x3 = u3;
	double y3 = v3;
	double x4 = u4;
	double y4 = v4;
	double x5 = u5;
	double y5 = v5;
	double x6 = u6;
	double y6 = v6;
	double x7 = u7;
	double y7 = v7;

	// Calculate distance from ground truth
	double distance = sqrt((x2 - x1)*(x2 - x1) + (y2 - y1) * (y2 - y1));
	double distanceTsai = sqrt((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
	double distanceNavy = sqrt((x4 - x1) * (x4 - x1) + (y4 - y1) * (y4 - y1));
	double distanceInria = sqrt((x5 - x1) * (x5 - x1) + (y5 - y1) * (y5 - y1));
	double distanceDual = sqrt((x6 - x1) * (x6 - x1) + (y6 - y1) * (y6 - y1));
	double distanceBranch = sqrt((x7 - x1) * (x7 - x1) + (y7 - y1) * (y7 - y1));

	// Set centroid to variables for drawing
	vector<Point2f> p2lCenter(1);
	p2lCenter[0].x = u;
	p2lCenter[0].y = v;

	vector<Point2f> tsaiCenter(1);
	tsaiCenter[0].x = u3;
	tsaiCenter[0].y = v3;

	vector<Point2f> navyCenter(1);
	navyCenter[0].x = u4;
	navyCenter[0].y = v4;

	vector<Point2f> inriaCenter(1);
	inriaCenter[0].x = u5;
	inriaCenter[0].y = v5;

	vector<Point2f> dualCenter(1);
	dualCenter[0].x = u6;
	dualCenter[0].y = v6;

	vector<Point2f> branchCenter(1);
	branchCenter[0].x = u7;
	branchCenter[0].y = v7;

	// Draw circles
	circle(undistorted, finalCenter[0], 3, Scalar(100, 100, 100), -1, 8, 0);
	circle(undistorted, p2lCenter[0], 3, Scalar(255, 255, 0), -1, 8, 0);// Blue
	circle(undistorted, tsaiCenter[0], 3, Scalar(0, 255, 0), -1, 8, 0); // Lime
	circle(undistorted, navyCenter[0], 3, Scalar(255, 255, 0), -1, 8, 0); // Blue
	circle(undistorted, inriaCenter[0], 3, Scalar(255, 0, 255), -1, 8, 0); // Pink
	circle(undistorted, dualCenter[0], 3, Scalar(0, 255, 255), -1, 8, 0); // Yellow
	circle(undistorted, branchCenter[0], 3, Scalar(255, 255, 255), -1, 8, 0); // White

	// Save position and projection error data to file
	ofstream dataFile("I:/DataCollection/ColNov5.csv");
	dataFile << "Hand Eye Calibration\n";
	dataFile << point2Line->GetElement(0, 0) << "," << point2Line->GetElement(0, 1) << "," << point2Line->GetElement(0, 2) << "," << point2Line->GetElement(0, 3) << "\n";
	dataFile << point2Line->GetElement(1, 0) << "," << point2Line->GetElement(1, 1) << "," << point2Line->GetElement(1, 2) << "," << point2Line->GetElement(1, 3) << "\n";
	dataFile << point2Line->GetElement(2, 0) << "," << point2Line->GetElement(2, 1) << "," << point2Line->GetElement(2, 2) << "," << point2Line->GetElement(2, 3) << "\n";
	dataFile << point2Line->GetElement(3, 0) << "," << point2Line->GetElement(3, 1) << "," << point2Line->GetElement(3, 2) << "," << point2Line->GetElement(3, 3) << "\n";
	dataFile << "Circle Location" << "," << x1 << "," << y1 << "\n";
	dataFile << "Point 2 Line" << "," << u << "," << v << "," << posePosition[2] << "," << distance << "\n";
	dataFile << "Tsai" << "," << u3 << "," << v3 << "," << tsaiPosition[2] << "," << distanceTsai << "\n";
	dataFile << "Navy" << "," << u4 << "," << v4 << "," << navyPosition[2] << "," << distanceNavy << "\n";
	dataFile << "Inria" << "," << u5 << "," << v5 << "," << inriaPosition[2] << "," << distanceInria << "\n";
	dataFile << "Dual" << "," << u6 << "," << v6 << "," << dualPosition[2] << "," << distanceDual << "\n";
	dataFile << "Branch" << "," << u7 << "," << v7 << "," << branchPosition[2] << "," << distanceBranch << "\n";

	imwrite("I:/DataCollection/collectionNov5.png", undistorted);

	// Show Blur result
	namedWindow("Results", CV_WINDOW_AUTOSIZE);
	imshow("Results", undistorted);
	cv::waitKey(0);

}

void mainWidget::collectPose()
{
	// Grab left image
	cv::Mat matLeft(ovrvisionProHandle.GetCamHeight(), ovrvisionProHandle.GetCamWidth(), CV_8UC4, ovrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_LEFT));
	
	// Undistort image
	cv::Mat undistorted;
	undistort(matLeft, undistorted, intrinsic, distortion);

	// Get transform matrix of tracked camera
	vtkSmartPointer<vtkMatrix4x4> transform = vtkSmartPointer<vtkMatrix4x4>::New();
	oculusHMD->GetTransform()->GetMatrix(transform);

	// Save data
	ofstream myfile("I:/ImagePoses/pose.csv");
	myfile << transform->GetElement(0, 0) << "," << transform->GetElement(0, 1) << "," << transform->GetElement(0, 2) << "," << transform->GetElement(0, 3)
		<< "," << transform->GetElement(1, 0) << "," << transform->GetElement(1, 1) << "," << transform->GetElement(1, 2) << "," << transform->GetElement(1, 3)
		<< "," << transform->GetElement(2, 0) << "," << transform->GetElement(2, 1) << "," << transform->GetElement(2, 2) << "," << transform->GetElement(2, 3)
		<< "," << transform->GetElement(3, 0) << "," << transform->GetElement(3, 1) << "," << transform->GetElement(3, 2) << "," << transform->GetElement(3, 3);
	imwrite("I:/ImagePoses/pose.png", undistorted);
}

// on accept the next pose is calculated
void mainWidget::nextPose(bool checked)
{
  if (checked)
  {
		// Update OVRvision device
    ovrvisionUpdate();

		// Initialize variables
		cv::Mat hsv;
		cv::Mat threshold;

		// Query the SDK for the latest frames
		ovrvisionProHandle.PreStoreCamData(OVR::OV_CAMQT_DMSRMP);

		// Grab Left and Right Images
		cv::Mat matLeft(ovrvisionProHandle.GetCamHeight(), ovrvisionProHandle.GetCamWidth(), CV_8UC4, ovrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_LEFT));
		cv::Mat matRight(ovrvisionProHandle.GetCamHeight(), ovrvisionProHandle.GetCamWidth(), CV_8UC4, ovrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_RIGHT));

		// Undistort images
		cv::Mat undistorted;
		undistort(matLeft, undistorted, intrinsic, distortion);

		// Show image to user
		namedWindow("BGRA", CV_WINDOW_AUTOSIZE);
		imshow("BGRA", undistorted);
		cv::waitKey(0);

		// Convert BGRA image to HSV image
		cv::cvtColor(undistorted, hsv, COLOR_BGR2HSV);
		cv::Mat drawing;

		cv::cvtColor(undistorted, drawing, COLOR_BGR2RGB);
		drawing.setTo(cv::Scalar(0, 0, 0));

		// Filter everything except red - (0, 70, 50) -> (10, 255, 255) & (160, 70, 50) -> (179, 255, 255)
		cv::inRange(hsv, cv::Scalar(HMinLower->value(), SMinLower->value(), VMinLower->value()), cv::Scalar(HMaxLower->value(), SMaxLower->value(), VMaxLower->value()), thresholdFinal);
		cv::inRange(hsv, cv::Scalar(HMinUpper->value(), SMinUpper->value(), VMinUpper->value()), cv::Scalar(HMaxUpper->value(), SMaxUpper->value(), VMaxUpper->value()), threshold);

		cv::Mat mask;
		cv::addWeighted(thresholdFinal, 1.0, threshold, 1.0, 0.0, mask);

		// Show color threshold to user
		namedWindow("Color Threshold", CV_WINDOW_AUTOSIZE);
		imshow("Color Threshold", mask);
		cv::waitKey(0);

		// Create a Gaussian & median Blur Filter
		medianBlur(mask, mask, 5);
		GaussianBlur(mask, mask, Size(9, 9), 2, 2);
		vector<Vec3f> circles;

		// Draw the circles detected
		cv::Mat canny_output;
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		// Apply the Hough Transform to find the circles
		HoughCircles(mask, circles, CV_HOUGH_GRADIENT, 2, mask.rows / 16, 255, 30);

		// Outline circle and centroid
		if (circles.size() > 0)
		{
			Point2f center(circles[0][0], circles[0][1]);
			int radius = circles[0][2];

			// Draw detected circle
			circle(drawing, center, radius, Scalar(100, 100, 100), -1, 8, 0);

			// Show detected circle
			namedWindow("Circle", CV_WINDOW_AUTOSIZE);
			imshow("Circle", drawing);
			cv::waitKey(0);

			// Set thresholds for contour detection
			int thresh = 100;
			int max_thresh = 255;
			RNG rng(12345);

			// Detect edges using canny
			Canny(drawing, canny_output, thresh, thresh * 2, 3);

			// Find contours
			findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

			/// Approximate contours to polygons + get bounding rects and circles
			vector<vector<Point> > contours_poly(contours.size());
			vector<Point2f>centerTwo(contours.size());
			vector<float>radiusTwo(contours.size());

			for (int i = 0; i < contours.size(); i++)
			{
				approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true); // Finds polygon
				minEnclosingCircle((Mat)contours_poly[i], centerTwo[i], radiusTwo[i]); // Finds circle

				Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

				// Draw circle
				circle(drawing, centerTwo[i], (int)radiusTwo[0], color, 2, 8, 0);

				// Draw circle center
				circle(drawing, centerTwo[i], 3, color, 2, 8, 0);
			}

			// Show final result
			namedWindow("Final Detection", CV_WINDOW_AUTOSIZE);
			imshow("Final Detection", drawing);
			cv::waitKey(0);

			// Save circle position data
			poseCenters.push_back(centerTwo[0]);
			int numRows = dataTable->verticalHeader()->count();

			// Write to table in GUI
			dataTable->insertRow(numRows);
			dataTable->setItem(numRows, 0, new QTableWidgetItem("Circle"));
			dataTable->setItem(numRows, 1, new QTableWidgetItem(QString::number(centerTwo[0].x)));
			dataTable->setItem(numRows, 2, new QTableWidgetItem(QString::number(centerTwo[0].y)));
			dataTable->setItem(numRows, 3, new QTableWidgetItem(QString::number(radiusTwo[0])));
		}

		// If Hough Circles fails to find circle
		else if (circles.size() == 0)
		{
			int thresh = 100;
			int max_thresh = 255;
			RNG rng(12345);

			medianBlur(mask, mask, 3);

			// Detect edges using canny
			Canny(mask, canny_output, thresh, thresh * 2, 3);

			// Find contours
			findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

			/// Approximate contours to polygons + get bounding rects and circles
			vector<vector<Point> > contours_poly(contours.size());
			vector<Rect> boundRect(contours.size());
			vector<Point2f>center(contours.size());
			vector<float>radius(contours.size());

			for (int i = 0; i < contours.size(); i++)
			{
				approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true); // Finds polygon
				boundRect[i] = boundingRect(Mat(contours_poly[i]));		   // Finds rectangle
				minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]); // Finds circle
			}

			/// Draw circle
			Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

			// Draw circle
			circle(mask, center[0], (int)radius[0], color, 2, 8, 0);

			// Draw circle center
			circle(mask, center[0], 3, color, 2, 8, 0);

			// Save circle data for point-line calibration
			poseCenters.push_back(center[0]);

			// Write data to table in GUI
			int numRows = dataTable->verticalHeader()->count();
			dataTable->insertRow(numRows);
			dataTable->setItem(numRows, 0, new QTableWidgetItem("Circle"));
			dataTable->setItem(numRows, 1, new QTableWidgetItem(QString::number(center[0].x)));
			dataTable->setItem(numRows, 2, new QTableWidgetItem(QString::number(center[0].y)));
			dataTable->setItem(numRows, 3, new QTableWidgetItem(QString::number(radius[0])));
		}

		// Get transform
		getTransform();
  }

  // Uncheck next Pose button
  trackerWidget->nextPoseButton->setChecked(false);
}

// On manual select
void mainWidget::manualSelection(bool checked)
{
  if (checked)
  {
		// Update OVRvision device
    ovrvisionUpdate();

    // if GetTransform is clicked
    if (trackerWidget->calibrationButton->isChecked() == true)
    {
      // Initialize Variables
      cv::Mat hsv;
      cv::Mat threshold;

      // Query the SDK for the latest frames
			ovrvisionProHandle.PreStoreCamData(OVR::OV_CAMQT_DMSRMP);

      // Grab Left and Right Images
			cv::Mat matLeft(ovrvisionProHandle.GetCamHeight(), ovrvisionProHandle.GetCamWidth(), CV_8UC4, ovrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_LEFT));
			cv::Mat matRight(ovrvisionProHandle.GetCamHeight(), ovrvisionProHandle.GetCamWidth(), CV_8UC4, ovrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_RIGHT));

			// Undistort image
			cv::Mat undistorted;
			undistort(matLeft, undistorted, intrinsic, distortion);

			// Show image to user
      namedWindow("BGRA", CV_WINDOW_AUTOSIZE);
			imshow("BGRA", undistorted);
      cv::waitKey(0);

      // Convert BGRA image to HSV image
			cv::cvtColor(undistorted, hsv, COLOR_BGR2HSV);

      // Filter everything except red - (0, 70, 50) -> (10, 255, 255) & (160, 70, 50) -> (179, 255, 255)
      cv::inRange(hsv, cv::Scalar(HMinLower->value(), SMinLower->value(), VMinLower->value()), cv::Scalar(HMaxLower->value(), SMaxLower->value(), VMaxLower->value()), thresholdFinal);
      cv::inRange(hsv, cv::Scalar(HMinUpper->value(), SMinUpper->value(), VMinUpper->value()), cv::Scalar(HMaxUpper->value(), SMaxUpper->value(), VMaxUpper->value()), threshold);

      cv::Mat mask;
      cv::addWeighted(thresholdFinal, 1.0, threshold, 1.0, 0.0, mask);

      // Create a Gaussian & median Blur Filter
      medianBlur(mask, mask, 5);
      GaussianBlur(mask, mask, Size(9, 9), 2, 2);

      // Select points
      namedWindow("Select Points", CV_WINDOW_AUTOSIZE);

      cv::setMouseCallback("Select Points", onMouse, 0);
      imshow("Select Points", mask);
      waitKey(0);

			// Draw selected point 
      circle(mask, pt1, 3, (0, 100, 100), -1, 8, 0);

			/// Show user results
      namedWindow("Manual Selection", CV_WINDOW_AUTOSIZE);
      imshow("Manual Selection", mask);
      cv::waitKey(0);

      // Delete the last circle centroid coordinates and replace with new
      if (poseCenters.size() != 0)
      {
        poseCenters.pop_back();
        int numRows = dataTable->verticalHeader()->count();
        dataTable->removeRow(numRows);
        dataTable->removeRow(numRows - 1);
      }
      poseCenters.push_back(pt1);

      getTransform();
    }
  }

  // Uncheck manual button
  trackerWidget->manualButton->setChecked(false);
}

/*!
* Centralized place to create all vtk objects.
*/
void mainWidget::createVTKObjects()
{

  myTracker = vtkSmartPointer< vtkNDITracker >::New();
}

/*!
* A centralized place to delete all vtk objects.
*/
void mainWidget::destroyVTKObjects()
{
}

/*!
* A centralized place to setup all vtk pipelines.
*/
void mainWidget::setupVTKPipeline()
{

  ren->SetBackground(.1, .2, .4);
  qvtk->GetRenderWindow()->AddRenderer(ren);

	isTrackerInit = false;

}

void mainWidget::setupARRendering()
{
	// Requested capture format
	OVR::Camprop RequestedFormat(OVR::OV_CAMVR_FULL); // 960x950
	bool CameraSync(true);
	ovrvisionProHandle.SetCameraExposure(22500);

	// Init the ovrvision using the nvidia card as a compute device
	auto vendor = "NVIDIA Corporation";
	if (!ovrvisionProHandle.isOpen())
	{
		if (!ovrvisionProHandle.Open(0, RequestedFormat, vendor)) // We don't need to share it with OpenGL/D3D, but in the future we could access the images in GPU memory
		{
			printf("Unable to connect to OvrvisionPro device.");
			exit(1);
		}
	}
	ovrvisionProHandle.SetCameraSyncMode(CameraSync);

	// Set intrinsic calibration
	leftIntrinsicParam = Matrix<double>(3, 3);
	leftIntrinsicParam[0][0] = intrinsic.at<double>(0, 0) = 4.4308778509658629e+02;
	leftIntrinsicParam[0][1] = intrinsic.at<double>(0, 1) = 0;
	leftIntrinsicParam[0][2] = intrinsic.at<double>(0, 2) = 4.9137630327079307e+02;
	leftIntrinsicParam[1][0] = intrinsic.at<double>(1, 0) = 0;
	leftIntrinsicParam[1][1] = intrinsic.at<double>(1, 1) = 4.4088255151097923e+02;
	leftIntrinsicParam[1][2] = intrinsic.at<double>(1, 2) = 4.7733731041974312e+02;
	leftIntrinsicParam[2][0] = intrinsic.at<double>(2, 0) = 0;
	leftIntrinsicParam[2][1] = intrinsic.at<double>(2, 1) = 0;
	leftIntrinsicParam[2][2] = intrinsic.at<double>(2, 2) = 1;

	// Distortion Parameters
	distortion.at<double>(0, 0) = -3.4217579502885637e-01;
	distortion.at<double>(0, 1) = 1.5322858206254297e-01;
	distortion.at<double>(0, 2) = 7.0265221404534526e-04;
	distortion.at<double>(0, 3) = -1.0123352757817517e-03;

	ovrvisionProHandle.SetCameraExposure(22500);

	// Grab Left and Right Images
	matLeft = cv::Mat(ovrvisionProHandle.GetCamHeight(), ovrvisionProHandle.GetCamWidth(), CV_8UC4, ovrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_LEFT));
	matRight = cv::Mat(ovrvisionProHandle.GetCamHeight(), ovrvisionProHandle.GetCamWidth(), CV_8UC4, ovrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_RIGHT));

	// Undistort image
	cv::Mat undistorted;
	undistort(matLeft, undistorted, intrinsic, distortion);

	// Convert From BGRA to RGB
	cv::cvtColor(undistorted, rgbMatLeft, CV_BGRA2RGB); 
	cv::cvtColor(matRight, rgbMatRight, CV_BGRA2RGB);

	// Flip Orientation for VTK
	cv::flip(rgbMatLeft, finalMatLeft, 0);
	cv::flip(rgbMatRight, finalMatRight, 0);

	// Convert image from opencv to vtk
	imageImportLeft->SetDataSpacing(1, 1, 1);
	imageImportLeft->SetDataOrigin(0, 0, 0);
	imageImportLeft->SetWholeExtent(0, finalMatLeft.size().width - 1, 0, finalMatLeft.size().height - 1, 0, 0);
	imageImportLeft->SetDataExtentToWholeExtent();
	imageImportLeft->SetDataScalarTypeToUnsignedChar();
	imageImportLeft->SetNumberOfScalarComponents(finalMatLeft.channels());
	imageImportLeft->SetImportVoidPointer(finalMatLeft.data);
	imageImportLeft->Update();

	imageImportRight->SetDataSpacing(1, 1, 1);
	imageImportRight->SetDataOrigin(0, 0, 0);
	imageImportRight->SetWholeExtent(0, finalMatRight.size().width - 1, 0, finalMatRight.size().height - 1, 0, 0);
	imageImportRight->SetDataExtentToWholeExtent();
	imageImportRight->SetDataScalarTypeToUnsignedChar();
	imageImportRight->SetNumberOfScalarComponents(finalMatRight.channels());
	imageImportRight->SetImportVoidPointer(finalMatRight.data);
	imageImportRight->Update();

	textureLeft->SetInputConnection(imageImportLeft->GetOutputPort());
	textureRight->SetInputConnection(imageImportRight->GetOutputPort());

	// Phantom calibration
	double m[16] = { -0.0245,    0.9997,   -0.0077,  -58.3509,
		-0.1436,   -0.0112,   -0.9896,  -31.2855,
		-0.9893,   -0.0231,    0.1438 , 137.54,
		0,         0,         0,    1.0 };

	phantomToPhysicalTransform->GetMatrix()->DeepCopy(m);

	// left eye point to line
	double n[16] = { 0.076504987514104, -0.992849764678156, -0.0916314993001692, -65.9036612287519,
		0.738991468675458, 0.118158036072677, -0.663272408393668, -17.5314838935303,
		0.66935685259087, -0.0169712489221592, 0.742747184848025, -3.65616773796918,
		0, 0, 0, 1 };

	transformP2L->GetMatrix()->DeepCopy(n);

	myTracker->LoadVirtualSROM(4, "I:/Stylus_Update7.rom"); // reference rom in port 4 Stylus_PRevision3 Stylus_Update7.rom
  referenceCoil = myTracker->GetTool(4);
	myTracker->LoadVirtualSROM(5, "I:/HMDUpdate.rom"); // oculusHMD rom in port 5
  oculusHMD = myTracker->GetTool(5);
	myTracker->LoadVirtualSROM(6, "I:/Projects/VRApp/8700449.rom"); // I: / Projects / VRApp / 8700449.rom
	phantomTool = myTracker->GetTool(6);

	vtkMatrix4x4 *hmdPose;
	hmdPose = vtkMatrix4x4::New();
	hmdPose->DeepCopy(oculusHMD->GetTransform()->GetMatrix());

	ren->SetActiveCamera(vrCamera);
	vtkSmartPointer<vtkOpenVRRenderer> vrRen = vtkSmartPointer<vtkOpenVRRenderer>::New();
	vrCamera->SetHMDPose(hmdPose);

	phantomTransform->PostMultiply();
	phantomTransform->Identity();
	phantomTransform->Concatenate(phantomToPhysicalTransform);
	auto phantomToTracker = phantomTool->GetTransform();
	phantomTransform->Concatenate(phantomToTracker);
	phantomTransform->Concatenate(oculusHMD->GetTransform()->GetLinearInverse());
	phantomTransform->Scale(1.f, -1.f, -1.f);
	phantomTransform->Modified();

	sphereTransform->PostMultiply();
	sphereTransform->Identity();
	sphereTransform->Concatenate(phantomTool->GetTransform());
	sphereTransform->Concatenate(oculusHMD->GetTransform()->GetLinearInverse());

	vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
	sphere->SetRadius(2.5);
	vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	sphereMapper->SetInputConnection(sphere->GetOutputPort());
	sphereActor->SetMapper(sphereMapper);
	sphereActor->SetUserTransform(sphereTransform);

	vtkSmartPointer< vtkPolyDataReader > reader =
		vtkSmartPointer< vtkPolyDataReader >::New();
	reader->SetFileName("L2.vtk");
	vtkSmartPointer< vtkPolyDataMapper > phantomMapper =
		vtkSmartPointer< vtkPolyDataMapper >::New();
	phantomMapper->SetInputConnection(reader->GetOutputPort());
	phantomActor->SetMapper(phantomMapper);
	phantomActor->SetUserTransform(phantomTransform);

	ren->AddActor(phantomActor);
	ren->AddActor(sphereActor);

	double center_x = (matLeft.cols - intrinsic.at<double>(0, 2)) / ((matLeft.cols - 1) / 2.0) - 1.0;
	double center_y = intrinsic.at<double>(1, 2) / ((matLeft.rows - 1.0) / 2.0) - 1.0;
	double viewAngle = 2.0 * atan((matLeft.rows / 2.0) / intrinsic.at<double>(1, 1)) * 45.0 / atan(1.0);

  ren->ResetCamera();
	ren->GetActiveCamera()->SetViewAngle(viewAngle);
	ren->GetActiveCamera()->SetPosition(0, 0, 0);
	ren->GetActiveCamera()->SetViewUp(0, -1, 0);
	ren->GetActiveCamera()->SetFocalPoint(0, 0, intrinsic.at<double>(1, 1));
	ren->GetActiveCamera()->SetWindowCenter(center_x, center_y);
	ren->GetActiveCamera()->SetClippingRange(0.01, 1000.01);
	ren->GetActiveCamera()->SetParallelProjection(0);
	ren->GetActiveCamera()->Modified();

	// reset the camera according to visible actors
  ren->ResetCameraClippingRange();
  qvtk->GetRenderWindow()->Render();

	//	qvtk->GetRenderWindow()->Render();
	vrWindow = dynamic_cast<vtkOpenVRRenderWindow*>(qvtk->GetRenderWindow());
	vrWindow->SetTexturedBackground(true);
	vrWindow->SetLeftBackgroundTexture(textureLeft);
	vrWindow->AddRenderer(ren);

	// reset the camera according to visible actors
	ren->ResetCameraClippingRange();
	qvtk->GetRenderWindow()->Render();

	renderAR = true;

}

//
// a slot for updating tracker information and rendering
//
/*
* This is the function the real-time timer is calling
*
* The objective of this function is to:
*
* - update the tracking information
* - update the colours of the widgets (buttons) to reflect the status of the tracked tools
*
*/
void mainWidget::updateTrackerInfo()
{
  if (isTrackerInit)     /*!< Make sure the tracker is initialized. */
  {
    myTracker->Update();   /*!< Update the tracking information */
    ren->ResetCameraClippingRange();
    qvtk->GetRenderWindow()->Render();

    if (!referenceCoil->IsMissing() &&
        !referenceCoil->IsOutOfView() &&
        !referenceCoil->IsOutOfVolume())
    {
      /*!< update the US probe button */
      trackerWidget->lightWidgets[0]->GreenOn();
    }
    else
    {
      trackerWidget->lightWidgets[0]->RedOn();
    }

    if (!oculusHMD->IsMissing() &&
        !oculusHMD->IsOutOfView() &&
        !oculusHMD->IsOutOfVolume())
    {
      /*!< update the US probe button */
      trackerWidget->lightWidgets[1]->GreenOn();
    }
    else
    {
      trackerWidget->lightWidgets[1]->RedOn();
    }
		if (!phantomTool->IsMissing() &&
			!phantomTool->IsOutOfView() &&
			!phantomTool->IsOutOfVolume())
		{  /*!< update the US probe button */
			trackerWidget->lightWidgets[2]->GreenOn();
		}
		else
		{
			trackerWidget->lightWidgets[2]->RedOn();
		}

		// Get position of circle centroid
		double posePosition[3];
		posMatrix->GetPosition(posePosition);
		phantomTransform->Modified();

		cv::Mat objectPoints(1, 3, CV_64FC1);
		objectPoints.at<double>(0, 0) = posePosition[0];
		objectPoints.at<double>(0, 1) = posePosition[1];
		objectPoints.at<double>(0, 2) = posePosition[2];

		// Project point
		vector<Point2d> projectedPoints;
		double xPrime = objectPoints.at<double>(0, 0) / objectPoints.at<double>(0, 2);
		double yPrime = objectPoints.at<double>(0, 1) / objectPoints.at<double>(0, 2);

		double u = (leftIntrinsicParam[0][0] * xPrime) + leftIntrinsicParam[0][2];
		double v = (leftIntrinsicParam[1][1] * yPrime) + leftIntrinsicParam[1][2];
		
		vector<Point2f> center(1);
		center[0].x = u;
		center[0].y = v;
		ovrvisionProHandle.SetCameraExposure(22500);

		// Query the SDK for the latest frames
		ovrvisionProHandle.PreStoreCamData(OVR::OV_CAMQT_DMSRMP);

		// Set HMD pose in reference to Spectra
		vtkMatrix4x4 *hmdPose;
		hmdPose = vtkMatrix4x4::New();
		hmdPose->DeepCopy(oculusHMD->GetTransform()->GetLinearInverse()->GetMatrix());
		vrCamera->SetHMDPose(hmdPose);
			
		// Grab Left and Right Images;
		cv::Mat matLeft(ovrvisionProHandle.GetCamHeight(), ovrvisionProHandle.GetCamWidth(), CV_8UC4, ovrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_LEFT));

		cv::Mat undistorted;
		undistort(matLeft, undistorted, intrinsic, distortion);

		// circle center
		circle(undistorted, center[0], 3, (0, 100, 100), -1, 8, 0);
		
		// circle outline
		circle(undistorted, center[0], 14, Scalar(100, 100, 100), 3, 8, 0);

		// Convert From BGRA to RGB
		cv::cvtColor(undistorted, rgbMatLeft, CV_BGRA2RGB);

		// Flip Orientation for VTK
		cv::flip(rgbMatLeft, finalMatLeft, 0);
		cv::flip(rgbMatRight, finalMatRight, 0);

		// Update 
		imageImportLeft->Modified();
		imageImportLeft->Update();
		imageImportRight->Modified();
		imageImportRight->Update();

		textureLeft->Modified();
		textureRight->Modified();
		textureLeft->Update();
		textureRight->Update();

		// Render
		ren->ResetCameraClippingRange();
		qvtk->GetRenderWindow()->Render();
		vrWindow->Render();
  }
}

/*!
* A QT slot to start the QTimer for acquire tracking data.
*
* If the tracker is NOT initialized, this QT slot will initialize
* the tracker by probing all the available ports.
*/
void mainWidget::startTrackerSlot(bool checked)
{
  if (checked)
  {
    /*!
    * Check if the tracker is initialized.  If not, initialize it here.
    */
    if (!isTrackerInit)
    {
      myTracker->SetBaudRate(115200); /*!< Set the baud rate sufficiently high. */

      statusBar()->showMessage(tr("Tracking system NOT initialized."), 5000);
      if (myTracker->Probe())  /*!< Find the tracker. */
      {
        statusBar()->showMessage("Tracker Initialized", 5000);
        isTrackerInit = true;
      }
      else
      {
        statusBar()->showMessage("Tracker Initialization FAILED", 5000);
        isTrackerInit = false;
        trackerButton->setChecked(false);
        //        trackerButton->toggle();
      }
    }

    /*!
    * If tracker is initialized, start tracking.
    */
    if (isTrackerInit)
    {

      statusBar()->showMessage(tr("Tracking started."), 5000);
      myTracker->StartTracking();

      checkToolPorts();

      //trackerTimer->start( 0 ); /*!< Update the tracker as quickly as we can. */
      trackerTimer->start(35); /*!< The vtk pipeline takes about 15msec, so this is roughly 20 FPS. */
    }
  }
  else
  {
    /*! Bottom is un-toggled. */
    if (isTrackerInit)
    {
      trackerTimer->stop();
      myTracker->StopTracking();

      /*!
      * Turn all the light widgets to blue.
      */
      for (int i = 0; i < 4; i++)
      {
        lightWidgets[i]->BlueOn();
      }
      statusBar()->showMessage(tr("stopping tracker"), 5000);
    }
  }
}

void mainWidget::createActions()
{
  quitAct = new QAction(tr("&Quit"), this);
  quitAct->setShortcuts(QKeySequence::Quit);
  quitAct->setStatusTip(tr("Quit the application"));
  connect(quitAct, SIGNAL(triggered()), this, SLOT(close()));

  aboutAct = new QAction(tr("&About"), this);
  aboutAct->setStatusTip(tr("About this application"));
  connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));

  controlAct = new QAction(tr("&Tracker controls"), this);
  controlAct->setStatusTip(tr(""));
  connect(controlAct, SIGNAL(triggered()), this, SLOT(createControlDock()));

  aboutRobartsAct = new QAction(tr("About &Robarts"), this);
  aboutRobartsAct->setStatusTip(tr("About Robarts Research Institute"));
  connect(aboutRobartsAct, SIGNAL(triggered()), this, SLOT(aboutRobarts()));

}

void mainWidget::createMenus()
{
  fileMenu = menuBar()->addMenu(tr("&File"));
  fileMenu->addSeparator();
  fileMenu->addAction(quitAct);

  calibMenu = menuBar()->addMenu(tr("&Calibration"));

  controlMenu = menuBar()->addMenu(tr("&Control"));
  controlMenu->addAction(controlAct);

  helpMenu = menuBar()->addMenu(tr("&Help"));
  helpMenu->addSeparator();
  helpMenu->addAction(aboutAct);
  helpMenu->addAction(aboutRobartsAct);
}

void mainWidget::createToolInformation()
{
	/*!
	* Create a dock widget for the tool information
	*/
	toolInfo = new QDockWidget(tr("Tool Information"), this);
	toolInfo->setAllowedAreas(Qt::RightDockWidgetArea);
	toolInfo->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable);
	addDockWidget(Qt::RightDockWidgetArea, toolInfo);
	toolInfo->setMinimumWidth(406);

	/*!
	* Setup layout and frame
	*/
	QFrame *mainFrame = new QFrame;
	mainFrame->setFrameStyle(QFrame::WinPanel | QFrame::Sunken);
	mainFrame->setLineWidth(2);

	QVBoxLayout *vlayout = new QVBoxLayout;
	vlayout->setMargin(0);
	vlayout->setSpacing(10);
	vlayout->setAlignment(Qt::AlignTop);
	mainFrame->setLayout(vlayout);

	/*!
	* Create table to hold tool information
	*/
	dataTable = new QTableWidget();
	dataTable->setRowCount(3);
	dataTable->setColumnCount(4);
	dataTable->setItem(0, 0, new QTableWidgetItem("Tracked Object"));

	dataTable->setItem(0, 1, new QTableWidgetItem("x"));
	dataTable->setItem(0, 2, new QTableWidgetItem("y"));
	dataTable->setItem(0, 3, new QTableWidgetItem("z"));

	dataTable->setShowGrid(true);
	dataTable->horizontalHeader()->hide();
	dataTable->verticalHeader()->hide();

	toolInfo->setWidget(mainFrame);
	vlayout->addWidget(dataTable);

}

void mainWidget::pivotCalibration(bool checked)
{
  if (checked)
  {
    std::cerr << "checked" << std::endl;

    double m[16] = { 1, 0, 0, 0,
                     0, 1, 0, 0,
                     0, 0, 1, 0,
                     0, 0, 0, 1
                   };

    /*!
    * Reset the calibration matrix associated with the DRB each time.
    */
    vtkSmartPointer< vtkMatrix4x4 > matrix =
      vtkSmartPointer< vtkMatrix4x4 >::New();
    matrix->DeepCopy(m);
    referenceCoil->SetCalibrationMatrix(matrix);
    referenceCoil->InitializeToolTipCalibration();
    referenceCoil->SetCollectToolTipCalibrationData(1);
  }
  else
  {
    /*!
    * Compute the calibration after the toggle button is being released,
    * and update the GUI to report the RMS.
    */
    std::cerr << "not checked" << std::endl;
    referenceCoil->SetCollectToolTipCalibrationData(0);
    QString tempString;
    double value = referenceCoil->DoToolTipCalibration();
    tempString.setNum(referenceCoil->DoToolTipCalibration());
    stylusTipRMS->setText(tempString);
  }
}

/*!
* Create a dock window for controlling NDI tracker
*/
void mainWidget::createControlDock()
{
  if (controlDock)
  {
    controlDock->show();
  }
  else
  {
    /*!
    * create a timer here for the tracker
    */
    trackerTimer = new QTimer(this);
    connect(trackerTimer, SIGNAL(timeout()), this, SLOT(updateTrackerInfo()));

    controlDock = new QDockWidget(tr("Tracker Control"), this);
    controlDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    controlDock->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable);
    addDockWidget(Qt::LeftDockWidgetArea, controlDock);
    controlDock->setMinimumWidth(180);

    QFrame* mainFrame = new QFrame;
    mainFrame->setFrameStyle(QFrame::WinPanel | QFrame::Sunken);
    mainFrame->setLineWidth(2);

    QGridLayout* controlsLayout = new QGridLayout;
    controlsLayout->setMargin(0);
    controlsLayout->setSpacing(10);
    controlsLayout->setAlignment(Qt::AlignTop);
    mainFrame->setLayout(controlsLayout);

    controlDock->setWidget(mainFrame);

    // Setup Slider bar for HSV range
    HMinLower = new QSpinBox(this);
    HMaxLower = new QSpinBox(this);
    SMinLower = new QSpinBox(this);
    SMaxLower = new QSpinBox(this);
    VMinLower = new QSpinBox(this);
    VMaxLower = new QSpinBox(this);
    HMinUpper = new QSpinBox(this);
    HMaxUpper = new QSpinBox(this);
    SMinUpper = new QSpinBox(this);
    SMaxUpper = new QSpinBox(this);
    VMinUpper = new QSpinBox(this);
    VMaxUpper = new QSpinBox(this);

    HMinLower->setRange(0, 180);
    HMaxLower->setRange(0, 180);
    SMinLower->setRange(0, 255);
    SMaxLower->setRange(0, 255);
    VMinLower->setRange(0, 255);
		VMaxLower->setRange(0, 255);

    HMinUpper->setRange(0, 180);
    HMaxUpper->setRange(0, 180);
    SMinUpper->setRange(0, 255);
    SMaxUpper->setRange(0, 255);
    VMinUpper->setRange(0, 255);
    VMaxUpper->setRange(0, 255);

    // Set Values to find Red
    HMinLower->setValue(0);
    HMaxLower->setValue(10);
    SMinLower->setValue(70);
    SMaxLower->setValue(255);
    VMinLower->setValue(50);
		VMaxLower->setValue(255);

    HMinUpper->setValue(160);
    HMaxUpper->setValue(179);
    SMinUpper->setValue(70);
    SMaxUpper->setValue(255);
    VMinUpper->setValue(50);
    VMaxUpper->setValue(255);

    QLabel* HMinLowerLabel = new QLabel(tr("Lower Hue Minimum: "));
    QLabel* HMaxLowerLabel = new QLabel(tr("Lower Hue Maximum: "));
    QLabel* SMinLowerLabel = new QLabel(tr("Lower Saturation Minimum: "));
    QLabel* SMaxLowerLabel = new QLabel(tr("Lower Saturation Maximum: "));
    QLabel* VMinLowerLabel = new QLabel(tr("Lower Value Minimum: "));
    QLabel* VMaxLowerLabel = new QLabel(tr("Lower Value Maximum: "));

    QLabel* HMinUpperLabel = new QLabel(tr("Upper Hue Minimum: "));
    QLabel* HMaxUpperLabel = new QLabel(tr("Upper Hue Maximum: "));
    QLabel* SMinUpperLabel = new QLabel(tr("Upper Saturation Minimum: "));
    QLabel* SMaxUpperLabel = new QLabel(tr("Upper Saturation Maximum: "));
    QLabel* VMinUpperLabel = new QLabel(tr("Upper Value Minimum: "));
    QLabel* VMaxUpperLabel = new QLabel(tr("Upper Value Maximum: "));

    // Add HSV Controls Wdiget
    controlsLayout->addWidget(HMinLowerLabel, 0, 0);
    controlsLayout->addWidget(HMinLower, 0, 1);
    controlsLayout->addWidget(HMaxLowerLabel, 1, 0);
    controlsLayout->addWidget(HMaxLower, 1, 1);
    controlsLayout->addWidget(SMinLowerLabel, 2, 0);
    controlsLayout->addWidget(SMinLower, 2, 1);
    controlsLayout->addWidget(SMaxLowerLabel, 3, 0);
    controlsLayout->addWidget(SMaxLower, 3, 1);
    controlsLayout->addWidget(VMinLowerLabel, 4, 0);
    controlsLayout->addWidget(VMinLower, 4, 1);
    controlsLayout->addWidget(VMaxLowerLabel, 5, 0);
    controlsLayout->addWidget(VMaxLower, 5, 1);

    controlsLayout->addWidget(HMinUpperLabel, 6, 0);
		controlsLayout->addWidget(HMinUpper, 6, 1);
    controlsLayout->addWidget(HMaxUpperLabel, 7, 0);
    controlsLayout->addWidget(HMaxUpper, 7, 1);
    controlsLayout->addWidget(SMinUpperLabel, 8, 0);
    controlsLayout->addWidget(SMinUpper, 8, 1);
    controlsLayout->addWidget(SMaxUpperLabel, 9, 0);
    controlsLayout->addWidget(SMaxUpper, 9, 1);
    controlsLayout->addWidget(VMinUpperLabel, 10, 0);
    controlsLayout->addWidget(VMinUpper, 10, 1);
    controlsLayout->addWidget(VMaxUpperLabel, 11, 0);
    controlsLayout->addWidget(VMaxUpper, 11, 1);

    // Add Tracking Widget
    trackerWidget = new eccTrackerWidget();
    controlsLayout->addWidget(trackerWidget);

    trackerWidget->setLabel(0, tr("(Port 0)"));
    trackerWidget->setLabel(1, tr("(Port 1)"));
    trackerWidget->setLabel(2, tr("(Port 2)"));
    trackerWidget->setLabel(3, tr("(Port 3)"));

    connect(trackerWidget->trackerButton, SIGNAL(toggled(bool)),
            this, SLOT(startTrackerSlot(bool)));

    connect(trackerWidget->calibrationButton, SIGNAL(toggled(bool)),
            this, SLOT(startCalibration(bool)));

    connect(trackerWidget->nextPoseButton, SIGNAL(toggled(bool)),
            this, SLOT(nextPose(bool)));

    connect(trackerWidget->manualButton, SIGNAL(toggled(bool)),
            this, SLOT(manualSelection(bool)));

		connect(trackerWidget->collectPoses, SIGNAL(clicked()), this, SLOT(collectPose()));

		connect(trackerWidget->projectionError, SIGNAL(clicked()), this, SLOT(calculateProjectionError()));

    // Calibration widget
    QDockWidget* stylusDock = new QDockWidget(tr("Stylus tip calibration"), this);
    stylusDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    stylusDock->setFeatures(QDockWidget::AllDockWidgetFeatures);
    stylusDock->setMinimumWidth(180);
    addDockWidget(Qt::RightDockWidgetArea, stylusDock);

    QFrame* frame = new QFrame;
    frame->setFrameStyle(QFrame::WinPanel | QFrame::Sunken);
    frame->setLineWidth(2);

    QVBoxLayout* vl = new QVBoxLayout;
    vl->setMargin(0);
    vl->setSpacing(10);
    vl->setAlignment(Qt::AlignTop);
    frame->setLayout(vl);
    stylusDock->setWidget(frame);

    QPushButton* pivotButton = new QPushButton(tr("Pivot"), this);
    pivotButton->setPalette(QPalette(Qt::blue));
    pivotButton->setAutoFillBackground(true);
    pivotButton->setCheckable(true);
    vl->addWidget(pivotButton);

    /*!
    * Connect the button to the actual function call to do the
    * math.
    */
    connect(pivotButton, SIGNAL(toggled(bool)),
            this, SLOT(pivotCalibration(bool)));

    QString tempString;
    tempString.setNum(0.0);

    QLabel* rms = new QLabel(tr("RMS"), this);
    stylusTipRMS = new QLineEdit;
    stylusTipRMS->setText(tempString);
    vl->addWidget(rms);
    vl->addWidget(stylusTipRMS);
    vl->setAlignment(Qt::AlignTop);

  }
}

void mainWidget::getTransform()
{
	if (poseCenters.size() <= 15)
  {
    // perform quaternion averaging here
    bool averaged = false;
    int nSamples = 100;
    Matrix<double> DRB(7, nSamples);
    double pos[3], ori[4];
    Matrix<double> q(4, 1), aa(4, 1), q7;

    vtkSmartPointer< vtkTransform > Transform =
      vtkSmartPointer< vtkTransform >::New();
    Transform->PostMultiply();

    for (int j = 0; j < nSamples; j++)
    {
      isProbeVisible = isOculusVisible = false;
      while (!(isProbeVisible && isOculusVisible))
      {
        myTracker->Update();
        if (!referenceCoil->IsMissing() && !referenceCoil->IsOutOfView() && !referenceCoil->IsOutOfVolume())
        {
          isProbeVisible = true;
        }
        if (!oculusHMD->IsMissing() && !oculusHMD->IsOutOfView() && !oculusHMD->IsOutOfVolume())
        {
          isOculusVisible = true;
        }
      }
      Transform->Identity();
      Transform->Concatenate(referenceCoil->GetTransform());
      Transform->Concatenate(oculusHMD->GetTransform()->GetLinearInverse());
      Transform->Modified();

      Transform->GetOrientationWXYZ(ori);
      Transform->GetPosition(pos);

      for (int k = 0; k < 4; k++)
      {
        aa[k][0] = ori[k];
      }

      // vtk the angle is in degree
      aa[0][0] = aa[0][0] / 45.0 * atan(1.0);

      aa2q(aa, q);
      DRB[0][j] = q[0][0];
      DRB[1][j] = q[1][0];
      DRB[2][j] = q[2][0];
      DRB[3][j] = q[3][0];
      DRB[4][j] = pos[0];
      DRB[5][j] = pos[1];
      DRB[6][j] = pos[2];
    }

    q7avg(DRB, q7); // now q7 is the averaged quaternion
    for (int j = 0; j < 4; j++)
    {
      q[j][0] = q7[j][0];
    }

    q2aa(q, aa);
    aa[0][0] = aa[0][0] * 45.0 / atan(1.0);

    Transform->Identity();
    Transform->PostMultiply();
    Transform->RotateWXYZ(aa[0][0], aa[1][0], aa[2][0], aa[3][0]);
    Transform->Translate(q7[4][0], q7[5][0], q7[6][0]);
    Transform->Modified();
    averaged = true;

    double position[3];
    Transform->GetPosition(position);
    X[0][poseCenters.size() - 1] = position[0];
    X[1][poseCenters.size() - 1] = position[1];
    X[2][poseCenters.size() - 1] = position[2];

    // Origin matrix - always 0,0,0
    origin[0][poseCenters.size() - 1] = 0;
    origin[1][poseCenters.size() - 1] = 0;
    origin[2][poseCenters.size() - 1] = 0;

    Matrix<double> pixel(3, 1);
    pixel[0][0] = poseCenters[poseCenters.size() - 1].x;
    pixel[1][0] = poseCenters[poseCenters.size() - 1].y;
    pixel[2][0] = 1;

    // Find the inverse of the camera intrinsic param matrix
    Matrix<double> leftIntrinsicInv(3, 3);
    invm3x3(leftIntrinsicParam, leftIntrinsicInv);

    // Calculate D matrix by multiplying the inverse of the
    // intrinsic param matrix by the pixel matrix
    Matrix<double> dMatrix(3, 1);
    dMatrix = leftIntrinsicInv * pixel;

		// Multiply by inverse of distortion coefficients
		vector<Point2d> pointCoords(1);
		vector<Point2d> undistortedPoints;
		pointCoords[0].x = poseCenters[poseCenters.size() - 1].x;
		pointCoords[0].y = poseCenters[poseCenters.size() - 1].y;

    // Normalize the D matrix
    double sum1;
    sum1 = (dMatrix[0][0] * dMatrix[0][0]) + (dMatrix[1][0] * dMatrix[1][0]) + (dMatrix[2][0] * dMatrix[2][0]);
    dNormalized[0][poseCenters.size() - 1] = dMatrix[0][0] / sqrt(sum1);
    dNormalized[1][poseCenters.size() - 1] = dMatrix[1][0] / sqrt(sum1);
    dNormalized[2][poseCenters.size() - 1] = dMatrix[2][0] / sqrt(sum1);

    // Send data to table
    int numRows = dataTable->verticalHeader()->count();
    dataTable->insertRow(numRows);
    dataTable->setItem(numRows, 0, new QTableWidgetItem("X"));
    dataTable->setItem(numRows, 1, new QTableWidgetItem(QString::number(X[0][poseCenters.size() - 1])));
    dataTable->setItem(numRows, 2, new QTableWidgetItem(QString::number(X[1][poseCenters.size() - 1])));
    dataTable->setItem(numRows, 3, new QTableWidgetItem(QString::number(X[2][poseCenters.size() - 1])));

    dataTable->insertRow(numRows + 1);
    dataTable->setItem(numRows + 1, 0, new QTableWidgetItem("D"));
    dataTable->setItem(numRows + 1, 1, new QTableWidgetItem(QString::number(dNormalized[0][poseCenters.size() - 1])));
    dataTable->setItem(numRows + 1, 2, new QTableWidgetItem(QString::number(dNormalized[1][poseCenters.size() - 1])));
    dataTable->setItem(numRows + 1, 3, new QTableWidgetItem(QString::number(dNormalized[2][poseCenters.size() - 1])));
  }

  if (poseCenters.size() == 15)
  {
    Matrix<double> rotation;
    Matrix<double> translation;
    double tol = 1e-9;
    double error = 0;

    // Calculate point to line
    p2l(X, origin, dNormalized, tol, rotation, translation, error);

    double x = translation[0][0];
    double y = translation[1][0];
    double z = translation[2][0];

    double row1col1 = rotation[0][0];
    double row1col2 = rotation[0][1];
    double row1col3 = rotation[0][2];

    double row2col1 = rotation[1][0];
    double row2col2 = rotation[1][1];
    double row2col3 = rotation[1][2];

    double row3col1 = rotation[2][0];
    double row3col2 = rotation[2][1];
    double row3col3 = rotation[2][2];

    point2Line->SetElement(0, 0, row1col1);
    point2Line->SetElement(0, 1, row1col2);
    point2Line->SetElement(0, 2, row1col3);

    point2Line->SetElement(1, 0, row2col1);
    point2Line->SetElement(1, 1, row2col2);
    point2Line->SetElement(1, 2, row2col3);

    point2Line->SetElement(2, 0, row3col1);
    point2Line->SetElement(2, 1, row3col2);
    point2Line->SetElement(2, 2, row3col3);

    point2Line->SetElement(0, 3, x);
    point2Line->SetElement(1, 3, y);
    point2Line->SetElement(2, 3, z);

    point2Line->SetElement(3, 0, 0);
    point2Line->SetElement(3, 1, 0);
    point2Line->SetElement(3, 2, 0);
    point2Line->SetElement(3, 3, 1);

    // tool transform = referenceCoil->GetTransform()
    // HMD transform inverse = oculusHMD->GetTransform()->GetLinearInverse()
    posMatrix->PostMultiply();
    posMatrix->Identity();
    posMatrix->Concatenate(referenceCoil->GetTransform());
    posMatrix->Concatenate(oculusHMD->GetTransform()->GetLinearInverse());
    posMatrix->Concatenate(point2Line);
    double posePosition[3];
    Matrix<double> posePositionM(3, 1);
    Matrix<double> result(3, 1);
    posMatrix->GetPosition(posePosition);

    cv::Mat objectPoints(1, 3, CV_64FC1);
    objectPoints.at<double>(0, 0) = posePosition[0];
    objectPoints.at<double>(0, 1) = posePosition[1];
    objectPoints.at<double>(0, 2) = posePosition[2];

    cv::Mat rvec(3, 1, CV_64FC1);
    rvec.at<double>(0, 0) = 0.0;
    rvec.at<double>(1, 0) = 0.0;
    rvec.at<double>(2, 0) = 0.0;

    cv::Mat tvec(3, 1, CV_64FC1);
    tvec.at<double>(0, 0) = 0.0;
    tvec.at<double>(1, 0) = 0.0;
    tvec.at<double>(2, 0) = 0.0;

    cv::Mat intrinsic(3, 3, CV_64FC1);
    intrinsic.at<double>(0, 0) = leftIntrinsicParam[0][0];
    intrinsic.at<double>(0, 1) = leftIntrinsicParam[0][1];
    intrinsic.at<double>(0, 2) = leftIntrinsicParam[0][2];
    intrinsic.at<double>(1, 0) = leftIntrinsicParam[1][0];
    intrinsic.at<double>(1, 1) = leftIntrinsicParam[1][1];
    intrinsic.at<double>(1, 2) = leftIntrinsicParam[1][2];
    intrinsic.at<double>(2, 0) = leftIntrinsicParam[2][0];
    intrinsic.at<double>(2, 1) = leftIntrinsicParam[2][1];
    intrinsic.at<double>(2, 2) = leftIntrinsicParam[2][2];

    cv::Mat distortion(1, 8, CV_64FC1);
		distortion.at<double>(0, 0) = 0.0;
		distortion.at<double>(0, 1) = 0.0;
		distortion.at<double>(0, 2) = 0.0;
		distortion.at<double>(0, 3) = 0.0;
		distortion.at<double>(0, 4) = 0.0;
		distortion.at<double>(0, 5) = 0.0;
		distortion.at<double>(0, 6) = 0.0;
		distortion.at<double>(0, 7) = 0.0;

    vector<Point2d> projectedPoints;
		double xPrime = objectPoints.at<double>(0, 0) / objectPoints.at<double>(0, 2);
		double yPrime = objectPoints.at<double>(0, 1) / objectPoints.at<double>(0, 2);

		double u = (leftIntrinsicParam[0][0] * xPrime) + leftIntrinsicParam[0][2];
		double v = (leftIntrinsicParam[1][1] * yPrime) + leftIntrinsicParam[1][2];

    vector<Point2f> center(1);

		center[0].x = u;
		center[0].y = v;

		ovrvisionProHandle.SetCameraExposure(22500);

    // Query the SDK for the latest frames
		ovrvisionProHandle.PreStoreCamData(OVR::OV_CAMQT_DMSRMP);

    // Grab Left and Right Images
		cv::Mat matLeft(ovrvisionProHandle.GetCamHeight(), ovrvisionProHandle.GetCamWidth(), CV_8UC4, ovrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_LEFT));
		cv::Mat matRight(ovrvisionProHandle.GetCamHeight(), ovrvisionProHandle.GetCamWidth(), CV_8UC4, ovrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_RIGHT));

		cv::Mat undistorted;
		undistort(matLeft, undistorted, intrinsic, distortion);

    // circle center
		circle(undistorted, center[0], 3, (0, 100, 100), -1, 8, 0);
    // circle outline
		circle(undistorted, center[0], 14, Scalar(100, 100, 100), 3, 8, 0);

    /// Show your results
    cv::namedWindow("Circle Found", CV_WINDOW_AUTOSIZE);
		cv::imshow("Circle Found", undistorted);
    cv::waitKey(0);

    cv::Mat hsv;
    cv::Mat threshold;

    // Convert BGRA image to HSV image
		cv::cvtColor(undistorted, hsv, COLOR_BGR2HSV);

    // Filter everything except red - (0, 70, 50) -> (10, 255, 255) & (160, 70, 50) -> (179, 255, 255)
    cv::inRange(hsv, cv::Scalar(HMinLower->value(), SMinLower->value(), VMinLower->value()), cv::Scalar(HMaxLower->value(), SMaxLower->value(), VMaxLower->value()), thresholdFinal);
    cv::inRange(hsv, cv::Scalar(HMinUpper->value(), SMinUpper->value(), VMinUpper->value()), cv::Scalar(HMaxUpper->value(), SMaxUpper->value(), VMaxUpper->value()), threshold);

    cv::Mat mask;
    cv::addWeighted(thresholdFinal, 1.0, threshold, 1.0, 0.0, mask);

    namedWindow("Color Threshold", CV_WINDOW_AUTOSIZE);
    imshow("Color Threshold", mask);
    cv::waitKey(0);

    // Create a Gaussian & median Blur Filter
    medianBlur(mask, mask, 5);
    GaussianBlur(mask, mask, Size(9, 9), 2, 2);

    // circle center
    circle(mask, center[0], 3, (0, 100, 100), -1, 8, 0);
    // circle outline
    circle(mask, center[0], 14, Scalar(100, 100, 100), 3, 8, 0);

    // Show Blur result
		namedWindow("P2L Result", CV_WINDOW_AUTOSIZE);
		imshow("P2L Result", mask);
    cv::waitKey(0);
  }
}

void mainWidget::startCalibration(bool checked)
{
  if (checked)
  {
    // Update Ovrvision image
    ovrvisionUpdate();

    if (poseCenters.size() != 0)
    {
      poseCenters.resize(0);
    }

    // Refresh table
    for (int i = dataTable->verticalHeader()->count(); i >= 0; i--)
    {
      dataTable->removeRow(i);
    }

    // Initialize matrices
    X.newsize(3, 15);
    origin.newsize(3, 15);
    dNormalized.newsize(3, 15);
  }
}

void mainWidget::createStatusBar()
{
  statusBar()->showMessage(tr("Ready"), 5000);
}

/*!
* A QT slot to display information about this application.
*/
void mainWidget::about()
{
  QMessageBox::about(this, tr("About Phantom-Less Calibration"),
                     tr("This is a US Calibration GUI using the phantom-less technique\n\n"
                        "By: \n\n"
                        "Elvis C.S. Chen\t\t"
                        "chene@robarts.ca"));
}

/*
* A QT slot to display information about Robarts Research Institution.
*/
void mainWidget::aboutRobarts()
{
  QMessageBox::about(this, tr("About Robarts Research Institute"),
                     tr("This program is developed at\n\n"
                        "Imaging Laboratories,\n"
                        "Robarts Research Institute.\n\n"
                        "London, Ontario\n"
                        "Canada, N6A5K8"));
}

/*!
* Check the vtkTrackerTool flags to determine
* what tools are connected.
*/
void mainWidget::checkToolPorts()
{
  myTracker->Update();
}
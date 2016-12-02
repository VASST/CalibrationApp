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
//#include <QVTKWidget.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
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

// Plus
#include <vtkPlusNDITracker.h>
#include "PlusTrackedFrame.h"
#include "PlusConfigure.h"
#include "vtkCommand.h"
#include "vtkCallbackCommand.h"
#include "vtkPlusDataCollector.h"
#include "vtkPlusChannel.h"
#include "vtkPlusDataSource.h"
#include "vtkPlusDevice.h"
#include "vtkPlusRfProcessor.h"
#include "vtkPlusSavedDataSource.h"
#include "vtkPlusVirtualMixer.h"

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
  * VTK related objects
  */
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
  this->destroyVTKObjects();	/*!< VTK cleanup. */
  dataCollector->Stop();		/*!< Stop Data Collection*/
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

void mainWidget::viewScene(bool checked)
{
	if (checked)
	{
		if (!isTrackerInit)
		{
			trackerChannel->GetTrackedFrame(trackedFrame);

			// Get left and right frames
			leftMixer->GetChannel()->GetTrackedFrame(leftMixerFrame);
			rightMixer->GetChannel()->GetTrackedFrame(rightMixerFrame);

			// Get left and right images
			vtkImageData *leftImage = leftMixerFrame.GetImageData()->GetImage();
			vtkImageData *rightImage = rightMixerFrame.GetImageData()->GetImage();
			int leftDims[3];
			int rightDims[3];
			leftImage->GetDimensions(leftDims);
			rightImage->GetDimensions(rightDims);

			// Copy vtkImage to cv::Mat
			finalMatLeft = cv::Mat(leftDims[1], leftDims[0], CV_8UC3, leftImage->GetScalarPointer(0, 0, 0));
			finalMatRight = cv::Mat(rightDims[1], rightDims[0], CV_8UC3, rightImage->GetScalarPointer(0, 0, 0));

			// Undistort images
			//undistort(matLeft, finalMatLeft, intrinsicLeft, distortionLeft);
			//undistort(matRight, finalMatRight, intrinsicRight, distortionRight);

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

			// left eye point to line
			double p2l[16] = { 0.0677294484076749, -0.992989179048552, -0.0968773044158076, -61.6285338697333,
				0.737296753348973, 0.11523277439025, -0.665668765383645, -14.6388968911687,
				0.672165321419851, -0.0263419437173227, 0.739932350071099, -4.60575695614759, 0, 0, 0, 1 };

			vrWindow = dynamic_cast<vtkOpenVRRenderWindow*>(renWindow.Get());
			vrWindow->SetTexturedBackground(true);
			vrWindow->SetLeftBackgroundTexture(textureLeft);
			vrWindow->SetRightBackgroundTexture(textureRight);
			vrWindow->AddRenderer(ren);

			//	ren->ResetCameraClippingRange();
			vrWindow->Render();

			bool isLMatrixValid(false);
			repository->SetTransforms(leftMixerFrame);
			if (repository->GetTransform(PlusTransformName("PointerTip", "LeftImagePlane"), tTip2ImageL, &isLMatrixValid) == PLUS_SUCCESS && isLMatrixValid)
			{
				posMatrixLeft->PostMultiply();
				posMatrixLeft->Identity();
				posMatrixLeft->Concatenate(tTip2ImageL);
			}

			bool isRMatrixValid(false);
			repository->SetTransforms(rightMixerFrame);
			if (repository->GetTransform(PlusTransformName("PointerTip", "RightImagePlane"), tTip2ImageR, &isRMatrixValid) == PLUS_SUCCESS && isRMatrixValid)
			{
				posMatrixRight->PostMultiply();
				posMatrixRight->Identity();
				posMatrixRight->Concatenate(tTip2ImageR);
			}
		}
		isViewScene = true;	
	}
	else
	{
		isViewScene = false;
	}
}

void mainWidget::collectPose()
{
	trackerChannel->GetTrackedFrame(trackedFrame);

	// Get left and right frames
	leftMixer->GetChannel()->GetTrackedFrame(leftMixerFrame);
	rightMixer->GetChannel()->GetTrackedFrame(rightMixerFrame);

	// Get left and right images
	vtkImageData *leftImage = leftMixerFrame.GetImageData()->GetImage();
	vtkImageData *rightImage = rightMixerFrame.GetImageData()->GetImage();
	int leftDims[3];
	int rightDims[3];
	leftImage->GetDimensions(leftDims);
	rightImage->GetDimensions(rightDims);

	// Copy vtkImage to cv::Mat
	matLeft = cv::Mat(leftDims[1], leftDims[0], CV_8UC3, leftImage->GetScalarPointer(0, 0, 0));
	matRight = cv::Mat(rightDims[1], rightDims[0], CV_8UC3, rightImage->GetScalarPointer(0, 0, 0));

	// Undistort images
	undistort(matLeft, finalMatLeft, intrinsicLeft, distortionLeft);
	undistort(matRight, finalMatRight, intrinsicRight, distortionRight);

	bool isLMatrixValid(false);
	if(repository->GetTransform(PlusTransformName("PointerTip", "LeftImagePlane"), tTip2ImageL, &isLMatrixValid) == PLUS_SUCCESS && isLMatrixValid)
	{
		// Save data
		ofstream myfile("./Results/poseL.csv");
		myfile << tTip2ImageL->GetElement(0, 0) << "," << tTip2ImageL->GetElement(0, 1) << "," << tTip2ImageL->GetElement(0, 2) << "," << tTip2ImageL->GetElement(0, 3)
			<< "," << tTip2ImageL->GetElement(1, 0) << "," << tTip2ImageL->GetElement(1, 1) << "," << tTip2ImageL->GetElement(1, 2) << "," << tTip2ImageL->GetElement(1, 3)
			<< "," << tTip2ImageL->GetElement(2, 0) << "," << tTip2ImageL->GetElement(2, 1) << "," << tTip2ImageL->GetElement(2, 2) << "," << tTip2ImageL->GetElement(2, 3)
			<< "," << tTip2ImageL->GetElement(3, 0) << "," << tTip2ImageL->GetElement(3, 1) << "," << tTip2ImageL->GetElement(3, 2) << "," << tTip2ImageL->GetElement(3, 3);
	}
	
	bool isRMatrixValid(false);
	if (repository->GetTransform(PlusTransformName("PointerTip", "RightImagePlane"), tTip2ImageR, &isRMatrixValid) == PLUS_SUCCESS && isRMatrixValid)
	{
		// Save data
		ofstream myfile("./Results/poseR.csv");
		myfile << tTip2ImageR->GetElement(0, 0) << "," << tTip2ImageR->GetElement(0, 1) << "," << tTip2ImageR->GetElement(0, 2) << "," << tTip2ImageR->GetElement(0, 3)
			<< "," << tTip2ImageR->GetElement(1, 0) << "," << tTip2ImageR->GetElement(1, 1) << "," << tTip2ImageR->GetElement(1, 2) << "," << tTip2ImageR->GetElement(1, 3)
			<< "," << tTip2ImageR->GetElement(2, 0) << "," << tTip2ImageR->GetElement(2, 1) << "," << tTip2ImageR->GetElement(2, 2) << "," << tTip2ImageR->GetElement(2, 3)
			<< "," << tTip2ImageR->GetElement(3, 0) << "," << tTip2ImageR->GetElement(3, 1) << "," << tTip2ImageR->GetElement(3, 2) << "," << tTip2ImageR->GetElement(3, 3);
	}
	string number = to_string(imageCount);
	string name = "./Results/poseR" + number;
	string fullName = name +".png";

	// Save left and right images
	imwrite("./Results/poseL.png", finalMatLeft);
	imwrite(fullName, matRight);
	imageCount++;
}

// on accept the next pose is calculated
void mainWidget::nextPose(bool checked)
{
  if (checked)
  {
		// Initialize variables
		cv::Mat hsvLeft;
		cv::Mat hsvRight;
		cv::Mat thresholdLeft;
		cv::Mat thresholdRight;
		cv::Mat thresholdFinalLeft;
		cv::Mat thresholdFinalRight;

		leftMixer->GetChannel()->GetTrackedFrame(leftMixerFrame);
		rightMixer->GetChannel()->GetTrackedFrame(rightMixerFrame);

		vtkImageData *leftImage = leftMixerFrame.GetImageData()->GetImage();
		vtkImageData *rightImage = rightMixerFrame.GetImageData()->GetImage();
		int leftDims[3];
		int rightDims[3];
		leftImage->GetDimensions(leftDims);
		rightImage->GetDimensions(rightDims);

		// Copy vtkImage to cv::Mat
		matLeft = cv::Mat(leftDims[1], leftDims[0], CV_8UC3, leftImage->GetScalarPointer(0, 0, 0));
		matRight = cv::Mat(rightDims[1], rightDims[0], CV_8UC3, rightImage->GetScalarPointer(0, 0, 0));

		undistortedLeft = cv::Mat(leftDims[1], leftDims[0], CV_8UC3);
		undistortedRight = cv::Mat(rightDims[1], rightDims[0], CV_8UC3);

		undistort(matLeft, undistortedLeft, intrinsicLeft, distortionLeft);
		undistort(matRight, undistortedRight, intrinsicRight, distortionRight);

		cv::flip(undistortedLeft, undistortedLeft, 0);
		cv::flip(undistortedRight, undistortedRight, 0);

		// Convert RGB image to HSV image
		cv::cvtColor(undistortedLeft, hsvLeft, COLOR_RGB2HSV);
		cv::cvtColor(undistortedRight, hsvRight, COLOR_RGB2HSV);

		cv::Mat drawingLeft;
		cv::Mat drawingRight;

		cv::cvtColor(undistortedLeft, drawingLeft, COLOR_RGB2BGR);
		cv::cvtColor(undistortedRight, drawingRight, COLOR_RGB2BGR);
		drawingLeft.setTo(cv::Scalar(0, 0, 0));
		drawingRight.setTo(cv::Scalar(0, 0, 0));

		namedWindow("RGB", CV_WINDOW_AUTOSIZE);
		imshow("RGB", undistortedRight);
		cv::waitKey(0);

		// Filter everything except red - (0, 70, 50) -> (10, 255, 255) & (160, 70, 50) -> (179, 255, 255)
		cv::inRange(hsvLeft, cv::Scalar(HMinLower->value(), SMinLower->value(), VMinLower->value()), cv::Scalar(HMaxLower->value(), SMaxLower->value(), VMaxLower->value()), thresholdFinalLeft);
		cv::inRange(hsvLeft, cv::Scalar(HMinUpper->value(), SMinUpper->value(), VMinUpper->value()), cv::Scalar(HMaxUpper->value(), SMaxUpper->value(), VMaxUpper->value()), thresholdLeft);
		cv::inRange(hsvRight, cv::Scalar(HMinLower->value(), SMinLower->value(), VMinLower->value()), cv::Scalar(HMaxLower->value(), SMaxLower->value(), VMaxLower->value()), thresholdFinalRight);
		cv::inRange(hsvRight, cv::Scalar(HMinUpper->value(), SMinUpper->value(), VMinUpper->value()), cv::Scalar(HMaxUpper->value(), SMaxUpper->value(), VMaxUpper->value()), thresholdRight);

		cv::Mat maskLeft;
		cv::Mat maskRight;
		cv::addWeighted(thresholdFinalLeft, 1.0, thresholdLeft, 1.0, 0.0, maskLeft);
		cv::addWeighted(thresholdFinalRight, 1.0, thresholdRight, 1.0, 0.0, maskRight);

		// Create a Gaussian & median Blur Filter
		medianBlur(maskLeft, maskLeft, 5);
		GaussianBlur(maskLeft, maskLeft, Size(9, 9), 2, 2);
		medianBlur(maskRight, maskRight, 5);
		GaussianBlur(maskRight, maskRight, Size(9, 9), 2, 2);

		namedWindow("Blur", CV_WINDOW_AUTOSIZE);
		imshow("Blur", maskRight);
		cv::waitKey(0);

		vector<Vec3f> circlesLeft;
		vector<Vec3f> circlesRight;

		// Draw the circles detected
		cv::Mat cannyOutputLeft;
		cv::Mat cannyOutputRight;
		vector<vector<Point> > contoursLeft;
		vector<vector<Point> > contoursRight;
		vector<Vec4i> hierarchyLeft;
		vector<Vec4i> hierarchyRight;

		// Apply the Hough Transform to find the circles
		HoughCircles(maskLeft, circlesLeft, CV_HOUGH_GRADIENT, 2, maskLeft.rows / 16, 255, 30);
		HoughCircles(maskRight, circlesRight, CV_HOUGH_GRADIENT, 2, maskRight.rows / 16, 255, 30);

		// Outline circle and centroid in left image
		if (circlesLeft.size() > 0)
		{
			Point2f center(circlesLeft[0][0], circlesLeft[0][1]);
			int radius = circlesLeft[0][2];

			// Draw detected circle
			circle(drawingLeft, center, radius, Scalar(100, 100, 100), -1, 8, 0);

			// Show detected circle
			//namedWindow("Circle", CV_WINDOW_AUTOSIZE);
			//imshow("Circle", drawingLeft);
			//cv::waitKey(0);

			// Set thresholds for contour detection
			int thresh = 100;
			int max_thresh = 255;
			RNG rng(12345);

			// Detect edges using canny
			Canny(drawingLeft, cannyOutputLeft, thresh, thresh * 2, 3);

			// Find contours
			findContours(cannyOutputLeft, contoursLeft, hierarchyLeft, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

			/// Approximate contours to polygons + get bounding rects and circles
			vector<vector<Point> > contours_poly(contoursLeft.size());
			vector<Point2f>centerTwo(contoursLeft.size());
			vector<float>radiusTwo(contoursLeft.size());

			for (int i = 0; i < contoursLeft.size(); i++)
			{
				approxPolyDP(Mat(contoursLeft[i]), contours_poly[i], 3, true); // Finds polygon
				minEnclosingCircle((Mat)contours_poly[i], centerTwo[i], radiusTwo[i]); // Finds circle

				Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

				// Draw circle
				circle(drawingLeft, centerTwo[i], (int)radiusTwo[0], color, 2, 8, 0);

				// Draw circle center
				circle(drawingLeft, centerTwo[i], 3, color, 2, 8, 0);
			}

			// Show final result
			//namedWindow("Final Detection", CV_WINDOW_AUTOSIZE);
			//imshow("Final Detection", drawingLeft);
			//cv::waitKey(0);

			// Save circle position data
			poseCentersLeft.push_back(centerTwo[0]);
			int numRows = dataTable->verticalHeader()->count();

			// Write to table in GUI
			dataTable->insertRow(numRows);
			dataTable->setItem(numRows, 0, new QTableWidgetItem("Circle"));
			dataTable->setItem(numRows, 1, new QTableWidgetItem(QString::number(centerTwo[0].x)));
			dataTable->setItem(numRows, 2, new QTableWidgetItem(QString::number(centerTwo[0].y)));
			dataTable->setItem(numRows, 3, new QTableWidgetItem(QString::number(radiusTwo[0])));
		}

		// If Hough Circles fails to find circle
		else if (circlesLeft.size() == 0)
		{
			int thresh = 100;
			int max_thresh = 255;
			RNG rng(12345);

			medianBlur(maskLeft, maskLeft, 3);

			// Detect edges using canny
			Canny(maskLeft, cannyOutputLeft, thresh, thresh * 2, 3);

			// Find contours
			findContours(cannyOutputLeft, contoursLeft, hierarchyLeft, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

			/// Approximate contours to polygons + get bounding rects and circles
			vector<vector<Point> > contours_poly(contoursLeft.size());
			vector<Rect> boundRect(contoursLeft.size());
			vector<Point2f>center(contoursLeft.size());
			vector<float>radius(contoursLeft.size());

			for (int i = 0; i < contoursLeft.size(); i++)
			{
				approxPolyDP(Mat(contoursLeft[i]), contours_poly[i], 3, true); // Finds polygon
				boundRect[i] = boundingRect(Mat(contours_poly[i]));		   // Finds rectangle
				minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]); // Finds circle
			}

			/// Draw circle
			Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

			// Draw circle
			circle(maskLeft, center[0], (int)radius[0], color, 2, 8, 0);

			// Draw circle center
			circle(maskLeft, center[0], 3, color, 2, 8, 0);

			// Save circle data for point-line calibration
			poseCentersLeft.push_back(center[0]);

			// Write data to table in GUI
			int numRows = dataTable->verticalHeader()->count();
			dataTable->insertRow(numRows);
			dataTable->setItem(numRows, 0, new QTableWidgetItem("Circle"));
			dataTable->setItem(numRows, 1, new QTableWidgetItem(QString::number(center[0].x)));
			dataTable->setItem(numRows, 2, new QTableWidgetItem(QString::number(center[0].y)));
			dataTable->setItem(numRows, 3, new QTableWidgetItem(QString::number(radius[0])));
		}

		// Get left transform
		getLeftTransform();

		// Outline circle and centroid in right image
		if (circlesRight.size() > 0)
		{
			Point2f center(circlesRight[0][0], circlesRight[0][1]);
			int radius = circlesRight[0][2];

			// Draw detected circle
			circle(drawingRight, center, radius, Scalar(100, 100, 100), -1, 8, 0);

			// Show detected circle
			namedWindow("Circle", CV_WINDOW_AUTOSIZE);
			imshow("Circle", drawingRight);
			cv::waitKey(0);

			// Set thresholds for contour detection
			int thresh = 100;
			int max_thresh = 255;
			RNG rng(12345);

			// Detect edges using canny
			Canny(drawingRight, cannyOutputRight, thresh, thresh * 2, 3);

			// Find contours
			findContours(cannyOutputRight, contoursRight, hierarchyRight, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

			/// Approximate contours to polygons + get bounding rects and circles
			vector<vector<Point> > contours_poly(contoursRight.size());
			vector<Point2f>centerTwo(contoursRight.size());
			vector<float>radiusTwo(contoursRight.size());

			for (int i = 0; i < contoursRight.size(); i++)
			{
				approxPolyDP(Mat(contoursRight[i]), contours_poly[i], 3, true); // Finds polygon
				minEnclosingCircle((Mat)contours_poly[i], centerTwo[i], radiusTwo[i]); // Finds circle

				Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

				// Draw circle
				circle(drawingRight, centerTwo[i], (int)radiusTwo[0], color, 2, 8, 0);

				// Draw circle center
				circle(drawingRight, centerTwo[i], 3, color, 2, 8, 0);
			}

			// Show final result
			namedWindow("Final Detection", CV_WINDOW_AUTOSIZE);
			imshow("Final Detection", drawingRight);
			cv::waitKey(0);

			// Save circle position data
			poseCentersRight.push_back(centerTwo[0]);
			int numRows = dataTable->verticalHeader()->count();

			// Write to table in GUI
			dataTable->insertRow(numRows);
			dataTable->setItem(numRows, 0, new QTableWidgetItem("Circle"));
			dataTable->setItem(numRows, 1, new QTableWidgetItem(QString::number(centerTwo[0].x)));
			dataTable->setItem(numRows, 2, new QTableWidgetItem(QString::number(centerTwo[0].y)));
			dataTable->setItem(numRows, 3, new QTableWidgetItem(QString::number(radiusTwo[0])));
		}

		// If Hough Circles fails to find circle
		else if (circlesRight.size() == 0)
		{
			int thresh = 100;
			int max_thresh = 255;
			RNG rng(12345);

			medianBlur(maskRight, maskRight, 3);

			// Detect edges using canny
			Canny(maskRight, cannyOutputRight, thresh, thresh * 2, 3);

			// Find contours
			findContours(cannyOutputRight, contoursRight, hierarchyRight, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

			/// Approximate contours to polygons + get bounding rects and circles
			vector<vector<Point> > contours_poly(contoursRight.size());
			vector<Rect> boundRect(contoursRight.size());
			vector<Point2f>center(contoursRight.size());
			vector<float>radius(contoursRight.size());

			for (int i = 0; i < contoursRight.size(); i++)
			{
				approxPolyDP(Mat(contoursRight[i]), contours_poly[i], 3, true); // Finds polygon
				boundRect[i] = boundingRect(Mat(contours_poly[i]));		   // Finds rectangle
				minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]); // Finds circle
			}

			/// Draw circle
			Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

			// Draw circle
			circle(maskRight, center[0], (int)radius[0], color, 2, 8, 0);

			// Draw circle center
			circle(maskRight, center[0], 3, color, 2, 8, 0);

			// Save circle data for point-line calibration
			poseCentersRight.push_back(center[0]);

			// Write data to table in GUI
			int numRows = dataTable->verticalHeader()->count();
			dataTable->insertRow(numRows);
			dataTable->setItem(numRows, 0, new QTableWidgetItem("Circle"));
			dataTable->setItem(numRows, 1, new QTableWidgetItem(QString::number(center[0].x)));
			dataTable->setItem(numRows, 2, new QTableWidgetItem(QString::number(center[0].y)));
			dataTable->setItem(numRows, 3, new QTableWidgetItem(QString::number(radius[0])));
		}

		// Get right transform
		getRightTransform();
  }

  // Uncheck next Pose button
  trackerWidget->nextPoseButton->setChecked(false);
}

// On manual select
void mainWidget::manualSelection(bool checked)
{
  if (checked)
  {
    // if GetTransform is clicked
    if (trackerWidget->calibrationButton->isChecked() == true)
    {
      // Initialize Variables
      cv::Mat hsvLeft;
	  cv::Mat hsvRight;
      cv::Mat thresholdLeft;
	  cv::Mat thresholdRight;
	  cv::Mat thresholdFinalLeft;
	  cv::Mat thresholdFinalRight;

	  leftMixer->GetChannel()->GetTrackedFrame(leftMixerFrame);
	  rightMixer->GetChannel()->GetTrackedFrame(rightMixerFrame);

	  vtkImageData *leftImage = leftMixerFrame.GetImageData()->GetImage();
	  vtkImageData *rightImage = rightMixerFrame.GetImageData()->GetImage();
	  int leftDims[3];
	  int rightDims[3];
	  leftImage->GetDimensions(leftDims);
	  rightImage->GetDimensions(rightDims);

	  // Copy vtkImage to cv::Mat
	  matLeft = cv::Mat(leftDims[1], leftDims[0], CV_8UC3, leftImage->GetScalarPointer(0, 0, 0));
	  matRight = cv::Mat(rightDims[1], rightDims[0], CV_8UC3, rightImage->GetScalarPointer(0, 0, 0));

	  undistortedLeft = cv::Mat(leftDims[1], leftDims[0], CV_8UC3);
	  undistortedRight = cv::Mat(rightDims[1], rightDims[0], CV_8UC3);

	  undistort(matLeft, undistortedLeft, intrinsicLeft, distortionLeft);
	  undistort(matRight, undistortedRight, intrinsicRight, distortionRight);

	  cv::flip(undistortedLeft, undistortedLeft, 0);
	  cv::flip(undistortedRight, undistortedRight, 0);

      // Convert RGB image to HSV image
	  cv::cvtColor(undistortedLeft, hsvLeft, COLOR_RGB2HSV);
	  cv::cvtColor(undistortedRight, hsvRight, COLOR_RGB2HSV);

      // Filter everything except red - (0, 70, 50) -> (10, 255, 255) & (160, 70, 50) -> (179, 255, 255)
      cv::inRange(hsvLeft, cv::Scalar(HMinLower->value(), SMinLower->value(), VMinLower->value()), cv::Scalar(HMaxLower->value(), SMaxLower->value(), VMaxLower->value()), thresholdFinalLeft);
      cv::inRange(hsvLeft, cv::Scalar(HMinUpper->value(), SMinUpper->value(), VMinUpper->value()), cv::Scalar(HMaxUpper->value(), SMaxUpper->value(), VMaxUpper->value()), thresholdLeft);
	  cv::inRange(hsvRight, cv::Scalar(HMinLower->value(), SMinLower->value(), VMinLower->value()), cv::Scalar(HMaxLower->value(), SMaxLower->value(), VMaxLower->value()), thresholdFinalRight);
	  cv::inRange(hsvRight, cv::Scalar(HMinUpper->value(), SMinUpper->value(), VMinUpper->value()), cv::Scalar(HMaxUpper->value(), SMaxUpper->value(), VMaxUpper->value()), thresholdRight);

      cv::Mat maskLeft;
	  cv::Mat maskRight;
      cv::addWeighted(thresholdFinalLeft, 1.0, thresholdLeft, 1.0, 0.0, maskLeft);
	  cv::addWeighted(thresholdFinalRight, 1.0, thresholdRight, 1.0, 0.0, maskRight);

      // Create a Gaussian & median Blur Filter
      medianBlur(maskLeft, maskLeft, 5);
      GaussianBlur(maskLeft, maskLeft, Size(9, 9), 2, 2);

      // Select point for left camera
      namedWindow("Select Left Point", CV_WINDOW_AUTOSIZE);

      cv::setMouseCallback("Select Left Point", onMouse, 0);
      imshow("Select Left Point", maskLeft);
      waitKey(0);

	  // Draw selected point 
      circle(maskLeft, pt1, 3, (0, 100, 100), -1, 8, 0);

	  // Show user results
      namedWindow("Manual Selection Left ", CV_WINDOW_AUTOSIZE);
      imshow("Manual Selection Left", maskLeft);
      cv::waitKey(0);

	  // Delete the last circle centroid coordinates and replace with new
	  if (poseCentersLeft.size() != 0)
	  {
		  poseCentersLeft.pop_back();
		  int numRows = dataTable->verticalHeader()->count();
		  dataTable->removeRow(numRows);
		  dataTable->removeRow(numRows - 1);
	  }
	  poseCentersLeft.push_back(pt1);

	  getLeftTransform();

	  // Select point for right camera
	  namedWindow("Select Right Point", CV_WINDOW_AUTOSIZE);

	  cv::setMouseCallback("Select Right Point", onMouse, 0);
	  imshow("Select Right Point", maskRight);
	  waitKey(0);

	  // Draw selected point 
	  circle(maskRight, pt1, 3, (0, 100, 100), -1, 8, 0);

	  // Show user results
	  namedWindow("Manual Selection Right", CV_WINDOW_AUTOSIZE);
	  imshow("Manual Selection Right", maskRight);
	  cv::waitKey(0);

      // Delete the last circle centroid coordinates and replace with new
      if (poseCentersRight.size() != 0)
      {
        poseCentersRight.pop_back();
        int numRows = dataTable->verticalHeader()->count();
        dataTable->removeRow(numRows);
        dataTable->removeRow(numRows - 1);
      }
      poseCentersRight.push_back(pt1);

      getRightTransform();
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
	inputConfigFileName = "./config/config.xml";
	intrinsicsFileName = "./config/left_calibration.xml";

	// Create output directory
	results_root_dir = "./Results/";
	if (CreateDirectory(results_root_dir.c_str(), NULL) ||
		ERROR_ALREADY_EXISTS != GetLastError())
		std::cout << "Output directory Created." << std::endl;

	// Create calibration saves directory
	calibration_root_dir = "./Calibration";
	if (CreateDirectory(calibration_root_dir.c_str(), NULL) ||
		ERROR_ALREADY_EXISTS != GetLastError())
		std::cout << "Calibration directory Created." << std::endl;

	trackerDevice = NULL;

	// Read configuration
	if (PlusXmlUtils::ReadDeviceSetConfigurationFromFile(configRootElement, inputConfigFileName.c_str()) == PLUS_FAIL)
	{
		LOG_ERROR("Unable to read configuration from file" << inputConfigFileName.c_str());
		exit;
	}

	vtkPlusConfig::GetInstance()->SetDeviceSetConfigurationData(configRootElement);

	// Read configuration file
	if (dataCollector->ReadConfiguration(configRootElement) != PLUS_SUCCESS)
	{
		LOG_ERROR("Configuration incorrect for vtkPlusDataCollector.");
		exit;
	}

	camera2TrackerName.SetTransformName("CameraToTracker");
	probe2TrackerName.SetTransformName("ProbeToTracker");
	probe2CameraName.SetTransformName("ProbeToCamera");
	tip2ProbeName.SetTransformName("PointerTipToProbe");
	tip2LImageName.SetTransformName("PointerTipToLeftImagePlane");
	tip2CameraName.SetTransformName("PointerTipToCamera");
	camera2LImageName.SetTransformName("CameraToLeftImagePlane");

	isViewScene = false;
	isTrackerInit = false;
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
	renWindow->AddRenderer(ren);
	renInt->SetRenderWindow(renWindow);
	isTrackerInit = false;
}

void mainWidget::setupARRendering()
{
	// Set intrinsic calibration
	leftIntrinsicParam = Matrix<double>(3, 3);
	leftIntrinsicParam[0][0] = intrinsicLeft.at<double>(0, 0) = 4.4308778509658629e+02;
	leftIntrinsicParam[0][1] = intrinsicLeft.at<double>(0, 1) = 0;
	leftIntrinsicParam[0][2] = intrinsicLeft.at<double>(0, 2) = 4.9137630327079307e+02;
	leftIntrinsicParam[1][0] = intrinsicLeft.at<double>(1, 0) = 0;
	leftIntrinsicParam[1][1] = intrinsicLeft.at<double>(1, 1) = 4.4088255151097923e+02;
	leftIntrinsicParam[1][2] = intrinsicLeft.at<double>(1, 2) = 4.7733731041974312e+02;
	leftIntrinsicParam[2][0] = intrinsicLeft.at<double>(2, 0) = 0;
	leftIntrinsicParam[2][1] = intrinsicLeft.at<double>(2, 1) = 0;
	leftIntrinsicParam[2][2] = intrinsicLeft.at<double>(2, 2) = 1;

	rightIntrinsicParam = Matrix<double>(3, 3);
	rightIntrinsicParam[0][0] = intrinsicRight.at<double>(0, 0) = 4.3642091715320345e+02;
	rightIntrinsicParam[0][1] = intrinsicRight.at<double>(0, 1) = 0;
	rightIntrinsicParam[0][2] = intrinsicRight.at<double>(0, 2) = 5.0206905222553593e+02;
	rightIntrinsicParam[1][0] = intrinsicRight.at<double>(1, 0) = 0;
	rightIntrinsicParam[1][1] = intrinsicRight.at<double>(1, 1) = 4.3449181164428381e+02;
	rightIntrinsicParam[1][2] = intrinsicRight.at<double>(1, 2) = 4.5455132093848829e+02;
	rightIntrinsicParam[2][0] = intrinsicRight.at<double>(2, 0) = 0;
	rightIntrinsicParam[2][1] = intrinsicRight.at<double>(2, 1) = 0;
	rightIntrinsicParam[2][2] = intrinsicRight.at<double>(2, 2) = 1;

	// Distortion Parameters
	distortionLeft.at<double>(0, 0) = -3.4217579502885637e-01;
	distortionLeft.at<double>(0, 1) = 1.5322858206254297e-01;
	distortionLeft.at<double>(0, 2) = 7.0265221404534526e-04;
	distortionLeft.at<double>(0, 3) = -1.0123352757817517e-03;

	distortionRight.at<double>(0, 0) = 2.2901883203387270e-04;
	distortionRight.at<double>(0, 1) = -2.8041302114925171e-02;
	distortionRight.at<double>(0, 2) = -1.3213753434011032e-02;
	distortionRight.at<double>(0, 3) = 1.2788565654151332e-02;
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
	  ren->ResetCameraClippingRange();

	  // Get updated tracking information
	  trackerChannel->GetTrackedFrame(trackedFrame);

	  bool isProbeMatrixValid(false);
	  repository->SetTransforms(trackedFrame);

	  // Check if probe is visible
	  if (repository->GetTransform(probe2TrackerName, tProbe2Tracker, &isProbeMatrixValid) == PLUS_SUCCESS && isProbeMatrixValid)
	  {
		  trackerWidget->lightWidgets[0]->GreenOn();
	  }
	  else
	  {
		  trackerWidget->lightWidgets[0]->RedOn();
	  }

	  // Check if HMD is visible
	  bool isCamMatrixValid(false);
	  if (repository->GetTransform(camera2TrackerName, tCamera2Tracker, &isCamMatrixValid) == PLUS_SUCCESS && isCamMatrixValid)
	  {
		  trackerWidget->lightWidgets[1]->GreenOn();
	  }
	  else
	  {
		  trackerWidget->lightWidgets[1]->RedOn();
	  }

	  bool isLMatrixValid(false);
	  repository->SetTransforms(leftMixerFrame);
	  if (repository->GetTransform(PlusTransformName("PointerTip", "LeftImagePlane"), tTip2ImageL, &isLMatrixValid) == PLUS_SUCCESS && isLMatrixValid);
	  {
		  posMatrixLeft->PostMultiply();
		  posMatrixLeft->Identity();
		  posMatrixLeft->Concatenate(tTip2ImageL);
	  }

	  bool isRMatrixValid(false);
	  repository->SetTransforms(rightMixerFrame);
	  if (repository->GetTransform(PlusTransformName("PointerTip", "RightImagePlane"), tTip2ImageR, &isRMatrixValid) == PLUS_SUCCESS && isRMatrixValid)
	  {
		  posMatrixRight->PostMultiply();
		  posMatrixRight->Identity();
		  posMatrixRight->Concatenate(tTip2ImageR);
	  }

	  if (isViewScene)
	  {
		  // Get position of circle centroid
		  double posePositionLeft[3];
		  double posePositionRight[3];
		  posMatrixLeft->GetPosition(posePositionLeft);
		  posMatrixRight->GetPosition(posePositionRight);

		  // Project point
		  double xPrimeLeft = posePositionLeft[0] / posePositionLeft[2];
		  double yPrimeLeft = posePositionLeft[1] / posePositionLeft[2];
		  double xPrimeRight = posePositionRight[0] / posePositionRight[2];
		  double yPrimeRight = posePositionRight[1] / posePositionRight[2];

		  double uLeft = (leftIntrinsicParam[0][0] * xPrimeLeft) + leftIntrinsicParam[0][2];
		  double vLeft = (leftIntrinsicParam[1][1] * yPrimeLeft) + leftIntrinsicParam[1][2];

		  double uRight = (rightIntrinsicParam[0][0] * xPrimeRight) + rightIntrinsicParam[0][2];
		  double vRight = (rightIntrinsicParam[1][1] * yPrimeRight) + rightIntrinsicParam[1][2];

		  vector<Point2f> centerLeft(1);
		  vector<Point2f> centerRight(1);
		  centerLeft[0].x = uLeft;
		  centerLeft[0].y = vLeft;
		  centerRight[0].x = uRight;
		  centerRight[0].y = vRight;

		  // Get left and right frames
		  leftMixer->GetChannel()->GetTrackedFrame(leftMixerFrame);
		  rightMixer->GetChannel()->GetTrackedFrame(rightMixerFrame);

		  // Get left and right images
		  vtkImageData *leftImage = leftMixerFrame.GetImageData()->GetImage();
		  vtkImageData *rightImage = rightMixerFrame.GetImageData()->GetImage();
		  int leftDims[3];
		  int rightDims[3];
		  leftImage->GetDimensions(leftDims);
		  rightImage->GetDimensions(rightDims);

		  // Copy vtkImage to cv::Mat
		  matLeft = cv::Mat(leftDims[1], leftDims[0], CV_8UC3, leftImage->GetScalarPointer(0, 0, 0));
		  matRight = cv::Mat(rightDims[1], rightDims[0], CV_8UC3, rightImage->GetScalarPointer(0, 0, 0));
		  
		  // Undistort images
		  undistortedLeft = cv::Mat(leftDims[1], leftDims[0], CV_8UC3);
		  undistortedRight = cv::Mat(rightDims[1], rightDims[0], CV_8UC3);

		  undistort(matLeft, undistortedLeft, intrinsicLeft, distortionLeft);
		  undistort(matRight, undistortedRight, intrinsicRight, distortionRight);

		  // Flip images to draw circles
		 cv::flip(undistortedLeft, finalMatLeft, 0);
		 cv::flip(undistortedRight, finalMatRight, 0);

		  // Only draw pointer tip if both pointer and camera are visible
		  if (repository->GetTransform(probe2TrackerName, tProbe2Tracker, &isProbeMatrixValid) == PLUS_SUCCESS && isProbeMatrixValid &&
			  repository->GetTransform(camera2TrackerName, tCamera2Tracker, &isCamMatrixValid) == PLUS_SUCCESS && isCamMatrixValid)
		  {
			  // circle center
			  circle(finalMatLeft, centerLeft[0], 3, (0, 100, 100), -1, 8, 0);
			  circle(finalMatRight, centerRight[0], 3, (0, 100, 100), -1, 8, 0);

			  // circle outline
			  circle(finalMatLeft, centerLeft[0], 14, Scalar(100, 100, 100), 3, 8, 0);
			  circle(finalMatRight, centerRight[0], 14, Scalar(100, 100, 100), 3, 8, 0);
		  }

		  // Flip back for vtk
		  cv::flip(finalMatLeft, finalMatLeft, 0);
		  cv::flip(finalMatRight, finalMatRight, 0);

		  // Update 
		  imageImportLeft->Modified();
		  imageImportLeft->Update();
		  imageImportRight->Modified();
		  imageImportRight->Update();

		  textureLeft->Modified();
		  textureRight->Modified();
		  textureLeft->Update();
		  textureRight->Update();

		  vrWindow->Render();
	  }
	 
	  renderAR = true;
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
			// Get tracker
			if (dataCollector->GetDevice(trackerDevice, "TrackerDevice") != PLUS_SUCCESS)
			{
				LOG_ERROR("Unable to locate the device with ID = \"TrackerDevice\". Check config file.");
				exit;
			}

			// Get Ovrvision Pro device
			if (dataCollector->GetDevice(ovrDevice, "VideoDevice") != PLUS_SUCCESS)
			{
				LOG_ERROR("Unable to locate the device with ID = \"VideoDevice\". Check config file.");
				exit;
			}

			// Get virtual mixer
			if (dataCollector->GetDevice(leftMixerDevice, "LeftTrackedVideoDevice") != PLUS_SUCCESS)
			{
				LOG_ERROR("Unable to locate the device with ID = \"LeftTrackedVideoDevice\". Check config file.");
				exit;
			}

			// Get virtual mixer
			if (dataCollector->GetDevice(rightMixerDevice, "RightTrackedVideoDevice") != PLUS_SUCCESS)
			{
				LOG_ERROR("Unable to locate the device with ID = \"RightTrackedVideoDevice\". Check config file.");
				exit;

			}
			ovrVideo = dynamic_cast<vtkPlusOvrvisionProVideoSource *>(ovrDevice);
			ndiTracker = dynamic_cast<vtkPlusNDITracker *>(trackerDevice);
			leftMixer = dynamic_cast<vtkPlusVirtualMixer *>(leftMixerDevice);
			rightMixer = dynamic_cast<vtkPlusVirtualMixer *>(rightMixerDevice);

			if (ndiTracker == NULL)
			{
				LOG_ERROR("Tracking device is not NDI Polaris/Aurora. Could not connect.");
				exit(EXIT_FAILURE);
			}

			// Connect to devices
			std::cout << "Connecting to NDI Polaris through COM" << ndiTracker->GetSerialPort();
			if (dataCollector->Connect() != PLUS_SUCCESS)
			{
				std::cout << ".................... [FAILED]" << std::endl;
				LOG_ERROR("Failed to connect to devices!");
				exit;
			}

			if (dataCollector->Start() != PLUS_SUCCESS)
			{
				LOG_ERROR("Failed to connect to devices!");
				exit;
			}
			std::cout << ".................... [OK]" << std::endl;

			if (repository->ReadConfiguration(configRootElement) != PLUS_SUCCESS)
			{
				LOG_ERROR("Configuration incorrect for vtkPlusTransformRepository.");
				exit(EXIT_FAILURE);
			}

			if (ndiTracker->GetOutputChannelByName(trackerChannel, "TrackerStream") != PLUS_SUCCESS)
			{
				LOG_ERROR("Unable to locate the channel with Id=\"TrackerStream\". Check config file.");
				exit(EXIT_FAILURE);
			}

			if (ovrDevice->GetOutputChannelByName(leftVideoChannel, "LeftVideoStream") != PLUS_SUCCESS)
			{
				LOG_ERROR("Unable to locate the channel with Id=\"OvrVideoStream\". Check config file.");
				exit(EXIT_FAILURE);
			}

			if (ovrDevice->GetOutputChannelByName(rightVideoChannel, "RightVideoStream") != PLUS_SUCCESS)
			{
				LOG_ERROR("Unable to locate the channel with Id=\"OvrVideoStream\". Check config file.");
				exit(EXIT_FAILURE);
			}
			trackerChannel->GetTrackedFrame(trackedFrame);
			leftMixer->GetChannel()->GetTrackedFrame(leftMixerFrame);
			rightMixer->GetChannel()->GetTrackedFrame(rightMixerFrame);

			// Get left and right images
			vtkImageData *leftImage = leftMixerFrame.GetImageData()->GetImage();
			vtkImageData *rightImage = rightMixerFrame.GetImageData()->GetImage();
			int leftDims[3];
			int rightDims[3];
			leftImage->GetDimensions(leftDims);
			rightImage->GetDimensions(rightDims);

			// Copy vtkImage to cv::Mat
			matLeft = cv::Mat(leftDims[1], leftDims[0], CV_8UC3, leftImage->GetScalarPointer(0, 0, 0));
			matRight = cv::Mat(rightDims[1], rightDims[0], CV_8UC3, rightImage->GetScalarPointer(0, 0, 0));

			// Undistort images
			undistort(matLeft, finalMatLeft, intrinsicLeft, distortionLeft);
			undistort(matRight, finalMatRight, intrinsicRight, distortionRight);

			// Convert image from opencv to vtk
			imageImportLeft->SetDataSpacing(1, 1, 1);
			imageImportLeft->SetDataOrigin(0, 0, 0);
			imageImportLeft->SetWholeExtent(0, finalMatLeft.size().width - 1, 0, finalMatLeft.size().height - 1, 0, 0);
			imageImportLeft->SetDataExtentToWholeExtent();
			imageImportLeft->SetDataScalarTypeToUnsignedChar();
			imageImportLeft->SetNumberOfScalarComponents(finalMatLeft.channels());
			imageImportLeft->SetImportVoidPointer(finalMatLeft.data);
			imageImportLeft->Modified();
			imageImportLeft->Update();

			imageImportRight->SetDataSpacing(1, 1, 1);
			imageImportRight->SetDataOrigin(0, 0, 0);
			imageImportRight->SetWholeExtent(0, finalMatRight.size().width - 1, 0, finalMatRight.size().height - 1, 0, 0);
			imageImportRight->SetDataExtentToWholeExtent();
			imageImportRight->SetDataScalarTypeToUnsignedChar();
			imageImportRight->SetNumberOfScalarComponents(finalMatRight.channels());
			imageImportRight->SetImportVoidPointer(finalMatRight.data);
			imageImportRight->Modified();
			imageImportRight->Update(); 

			textureLeft->SetInputConnection(imageImportLeft->GetOutputPort());
			textureRight->SetInputConnection(imageImportRight->GetOutputPort());

			vrWindow = dynamic_cast<vtkOpenVRRenderWindow*>(renWindow.Get());
			vrWindow->SetTexturedBackground(true);
			vrWindow->AddRenderer(ren);
			vrWindow->SetLeftBackgroundTexture(textureLeft);
			vrWindow->SetRightBackgroundTexture(textureRight);

			vrWindow->Render();

			bool isLMatrixValid(false);
			repository->SetTransforms(leftMixerFrame);
			if (repository->GetTransform(PlusTransformName("PointerTip", "LeftImagePlane"), tTip2ImageL, &isLMatrixValid) == PLUS_SUCCESS && isLMatrixValid)
			{
				posMatrixLeft->PostMultiply();
				posMatrixLeft->Identity();
				posMatrixLeft->Concatenate(tTip2ImageL);
			}

			bool isRMatrixValid(false);
			repository->SetTransforms(rightMixerFrame);
			if (repository->GetTransform(PlusTransformName("PointerTip", "RightImagePlane"), tTip2ImageR, &isRMatrixValid) == PLUS_SUCCESS && isRMatrixValid)
			{
				posMatrixRight->PostMultiply();
				posMatrixRight->Identity();
				posMatrixRight->Concatenate(tTip2ImageR);
			}

			isTrackerInit = true;
			trackerWidget->viewSceneButton->setEnabled(true);
			trackerWidget->calibrationButton->setEnabled(true);
			trackerWidget->collectPoses->setEnabled(true);
		}

		/*!
		* If tracker is initialized, start tracking.
		*/
		if (isTrackerInit)
		{
			statusBar()->showMessage(tr("Tracking started."), 5000);

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
			dataCollector->Stop();

			/*!
			* Turn all the light widgets to blue.
			*/
			for (int i = 0; i < 4; i++)
			{
				lightWidgets[i]->BlueOn();
			}
			statusBar()->showMessage(tr("stopping tracker"), 5000);

			trackerWidget->calibrationButton->setDisabled(true);
			trackerWidget->nextPoseButton->setDisabled(true);
			trackerWidget->manualButton->setDisabled(true);
			trackerWidget->collectPoses->setDisabled(true);
			trackerWidget->viewSceneButton->setDisabled(true);
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
	trackerWidget->calibrationButton->setDisabled(true);

    connect(trackerWidget->nextPoseButton, SIGNAL(toggled(bool)),
            this, SLOT(nextPose(bool)));
	trackerWidget->nextPoseButton->setDisabled(true);

    connect(trackerWidget->manualButton, SIGNAL(toggled(bool)),
            this, SLOT(manualSelection(bool)));
	trackerWidget->manualButton->setDisabled(true);

	connect(trackerWidget->collectPoses, SIGNAL(clicked()), this, 
		SLOT(collectPose()));
	trackerWidget->collectPoses->setDisabled(true);

	connect(trackerWidget->viewSceneButton, SIGNAL(toggled(bool)), 
		this, SLOT(viewScene(bool)));
	trackerWidget->viewSceneButton->setDisabled(true);

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

void mainWidget::getLeftTransform()
{
	if (poseCentersLeft.size() <= 15)
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
			bool isMatrixValid(false);
			repository->GetTransform(PlusTransformName("PointerTip", "Camera"), tTip2Camera, &isMatrixValid);

			Transform->Identity();
			Transform->Concatenate(tTip2Camera);
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
		XLeft[0][poseCentersLeft.size() - 1] = position[0];
		XLeft[1][poseCentersLeft.size() - 1] = position[1];
		XLeft[2][poseCentersLeft.size() - 1] = position[2];

		// Origin matrix - always 0,0,0
		originLeft[0][poseCentersLeft.size() - 1] = 0;
		originLeft[1][poseCentersLeft.size() - 1] = 0;
		originLeft[2][poseCentersLeft.size() - 1] = 0;

		Matrix<double> pixel(3, 1);
		pixel[0][0] = poseCentersLeft[poseCentersLeft.size() - 1].x;
		pixel[1][0] = poseCentersLeft[poseCentersLeft.size() - 1].y;
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
		pointCoords[0].x = poseCentersLeft[poseCentersLeft.size() - 1].x;
		pointCoords[0].y = poseCentersLeft[poseCentersLeft.size() - 1].y;

		// Normalize the D matrix
		double sum1;
		sum1 = (dMatrix[0][0] * dMatrix[0][0]) + (dMatrix[1][0] * dMatrix[1][0]) + (dMatrix[2][0] * dMatrix[2][0]);
		dNormalizedLeft[0][poseCentersLeft.size() - 1] = dMatrix[0][0] / sqrt(sum1);
		dNormalizedLeft[1][poseCentersLeft.size() - 1] = dMatrix[1][0] / sqrt(sum1);
		dNormalizedLeft[2][poseCentersLeft.size() - 1] = dMatrix[2][0] / sqrt(sum1);

		// Send data to table
		int numRows = dataTable->verticalHeader()->count();
		dataTable->insertRow(numRows);
		dataTable->setItem(numRows, 0, new QTableWidgetItem("X-Left"));
		dataTable->setItem(numRows, 1, new QTableWidgetItem(QString::number(XLeft[0][poseCentersLeft.size() - 1])));
		dataTable->setItem(numRows, 2, new QTableWidgetItem(QString::number(XLeft[1][poseCentersLeft.size() - 1])));
		dataTable->setItem(numRows, 3, new QTableWidgetItem(QString::number(XLeft[2][poseCentersLeft.size() - 1])));

		dataTable->insertRow(numRows + 1);
		dataTable->setItem(numRows + 1, 0, new QTableWidgetItem("D"));
		dataTable->setItem(numRows + 1, 1, new QTableWidgetItem(QString::number(dNormalizedLeft[0][poseCentersLeft.size() - 1])));
		dataTable->setItem(numRows + 1, 2, new QTableWidgetItem(QString::number(dNormalizedLeft[1][poseCentersLeft.size() - 1])));
		dataTable->setItem(numRows + 1, 3, new QTableWidgetItem(QString::number(dNormalizedLeft[2][poseCentersLeft.size() - 1])));
	}

	if (poseCentersLeft.size() == 15)
	{
		Matrix<double> rotation;
		Matrix<double> translation;
		double tol = 1e-9;
		double error = 0;

		// Calculate point to line
		p2l(XLeft, originLeft, dNormalizedLeft, tol, rotation, translation, error);

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

		point2LineLeft->SetElement(0, 0, row1col1);
		point2LineLeft->SetElement(0, 1, row1col2);
		point2LineLeft->SetElement(0, 2, row1col3);

		point2LineLeft->SetElement(1, 0, row2col1);
		point2LineLeft->SetElement(1, 1, row2col2);
		point2LineLeft->SetElement(1, 2, row2col3);

		point2LineLeft->SetElement(2, 0, row3col1);
		point2LineLeft->SetElement(2, 1, row3col2);
		point2LineLeft->SetElement(2, 2, row3col3);

		point2LineLeft->SetElement(0, 3, x);
		point2LineLeft->SetElement(1, 3, y);
		point2LineLeft->SetElement(2, 3, z);

		point2LineLeft->SetElement(3, 0, 0);
		point2LineLeft->SetElement(3, 1, 0);
		point2LineLeft->SetElement(3, 2, 0);
		point2LineLeft->SetElement(3, 3, 1);

		// tool transform = referenceCoil->GetTransform()
		// HMD transform inverse = oculusHMD->GetTransform()->GetLinearInverse()
		bool isMatrixValid(false);
		repository->GetTransform(PlusTransformName("PointerTip", "Camera"), tTip2Camera, &isMatrixValid);

		posMatrixLeft->PostMultiply();
		posMatrixLeft->Identity();
		posMatrixLeft->Concatenate(tTip2Camera);
		posMatrixLeft->Concatenate(point2LineLeft);

		double posePosition[3];
		Matrix<double> posePositionM(3, 1);
		Matrix<double> result(3, 1);
		posMatrixLeft->GetPosition(posePosition);

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

		vector<Point2d> projectedPoints;
		double xPrime = objectPoints.at<double>(0, 0) / objectPoints.at<double>(0, 2);
		double yPrime = objectPoints.at<double>(0, 1) / objectPoints.at<double>(0, 2);

		double u = (leftIntrinsicParam[0][0] * xPrime) + leftIntrinsicParam[0][2];
		double v = (leftIntrinsicParam[1][1] * yPrime) + leftIntrinsicParam[1][2];

		vector<Point2f> center(1);
		center[0].x = u;
		center[0].y = v;
		
		// Get left frame
		leftMixer->GetChannel()->GetTrackedFrame(leftMixerFrame);

		// Get left image
		vtkImageData *leftImage = leftMixerFrame.GetImageData()->GetImage();
		int leftDims[3];
		leftImage->GetDimensions(leftDims);

		// Copy vtkImage to cv::Mat
		matLeft = cv::Mat(leftDims[1], leftDims[0], CV_8UC3, leftImage->GetScalarPointer(0, 0, 0));

		// Undistort image
		undistortedLeft = cv::Mat(leftDims[1], leftDims[0], CV_8UC3);
		undistort(matLeft, undistortedLeft, intrinsicLeft, distortionRight);

		// Flip image to draw circle
		cv::flip(undistortedLeft, finalMatLeft, 0);

		// circle center
		circle(finalMatLeft, center[0], 3, (0, 100, 100), -1, 8, 0);

		// circle outline
		circle(finalMatLeft, center[0], 14, Scalar(100, 100, 100), 3, 8, 0);

		// Flip back for vtk
		cv::flip(finalMatLeft, finalMatLeft, 0);
	}
}

void mainWidget::getRightTransform()
{
	if (poseCentersRight.size() <= 15)
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
			bool isMatrixValid(false);
			repository->GetTransform(PlusTransformName("PointerTip", "Camera"), tTip2Camera, &isMatrixValid);

			Transform->Identity();
			Transform->Concatenate(tTip2Camera);
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
		XRight[0][poseCentersRight.size() - 1] = position[0];
		XRight[1][poseCentersRight.size() - 1] = position[1];
		XRight[2][poseCentersRight.size() - 1] = position[2];

		// Origin matrix - always 0,0,0
		originRight[0][poseCentersRight.size() - 1] = 0;
		originRight[1][poseCentersRight.size() - 1] = 0;
		originRight[2][poseCentersRight.size() - 1] = 0;

		Matrix<double> pixel(3, 1);
		pixel[0][0] = poseCentersRight[poseCentersRight.size() - 1].x;
		pixel[1][0] = poseCentersRight[poseCentersRight.size() - 1].y;
		pixel[2][0] = 1;

		// Find the inverse of the camera intrinsic param matrix
		Matrix<double> rightIntrinsicInv(3, 3);
		invm3x3(rightIntrinsicParam, rightIntrinsicInv);

		// Calculate D matrix by multiplying the inverse of the
		// intrinsic param matrix by the pixel matrix
		Matrix<double> dMatrix(3, 1);
		dMatrix = rightIntrinsicInv * pixel;

		// Multiply by inverse of distortion coefficients
		vector<Point2d> pointCoords(1);
		vector<Point2d> undistortedPoints;
		pointCoords[0].x = poseCentersRight[poseCentersRight.size() - 1].x;
		pointCoords[0].y = poseCentersRight[poseCentersRight.size() - 1].y;

		// Normalize the D matrix
		double sum1;
		sum1 = (dMatrix[0][0] * dMatrix[0][0]) + (dMatrix[1][0] * dMatrix[1][0]) + (dMatrix[2][0] * dMatrix[2][0]);
		dNormalizedRight[0][poseCentersRight.size() - 1] = dMatrix[0][0] / sqrt(sum1);
		dNormalizedRight[1][poseCentersRight.size() - 1] = dMatrix[1][0] / sqrt(sum1);
		dNormalizedRight[2][poseCentersRight.size() - 1] = dMatrix[2][0] / sqrt(sum1);

		// Send data to table
		int numRows = dataTable->verticalHeader()->count();
		dataTable->insertRow(numRows);
		dataTable->setItem(numRows, 0, new QTableWidgetItem("X-Right"));
		dataTable->setItem(numRows, 1, new QTableWidgetItem(QString::number(XRight[0][poseCentersRight.size() - 1])));
		dataTable->setItem(numRows, 2, new QTableWidgetItem(QString::number(XRight[1][poseCentersRight.size() - 1])));
		dataTable->setItem(numRows, 3, new QTableWidgetItem(QString::number(XRight[2][poseCentersRight.size() - 1])));

		dataTable->insertRow(numRows + 1);
		dataTable->setItem(numRows + 1, 0, new QTableWidgetItem("D-Right"));
		dataTable->setItem(numRows + 1, 1, new QTableWidgetItem(QString::number(dNormalizedRight[0][poseCentersRight.size() - 1])));
		dataTable->setItem(numRows + 1, 2, new QTableWidgetItem(QString::number(dNormalizedRight[1][poseCentersRight.size() - 1])));
		dataTable->setItem(numRows + 1, 3, new QTableWidgetItem(QString::number(dNormalizedRight[2][poseCentersRight.size() - 1])));
	}

	if (poseCentersRight.size() == 15)
	{
		Matrix<double> rotation;
		Matrix<double> translation;
		double tol = 1e-9;
		double error = 0;

		// Calculate point to line
		p2l(XRight, originRight, dNormalizedRight, tol, rotation, translation, error);

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

		point2LineRight->SetElement(0, 0, row1col1);
		point2LineRight->SetElement(0, 1, row1col2);
		point2LineRight->SetElement(0, 2, row1col3);

		point2LineRight->SetElement(1, 0, row2col1);
		point2LineRight->SetElement(1, 1, row2col2);
		point2LineRight->SetElement(1, 2, row2col3);

		point2LineRight->SetElement(2, 0, row3col1);
		point2LineRight->SetElement(2, 1, row3col2);
		point2LineRight->SetElement(2, 2, row3col3);

		point2LineRight->SetElement(0, 3, x);
		point2LineRight->SetElement(1, 3, y);
		point2LineRight->SetElement(2, 3, z);

		point2LineRight->SetElement(3, 0, 0);
		point2LineRight->SetElement(3, 1, 0);
		point2LineRight->SetElement(3, 2, 0);
		point2LineRight->SetElement(3, 3, 1);

		// tool transform = referenceCoil->GetTransform()
		// HMD transform inverse = oculusHMD->GetTransform()->GetLinearInverse()
		bool isMatrixValid(false);
		repository->GetTransform(PlusTransformName("PointerTip", "Camera"), tTip2Camera, &isMatrixValid);

		posMatrixRight->PostMultiply();
		posMatrixRight->Identity();
		posMatrixRight->Concatenate(tTip2Camera);
		posMatrixRight->Concatenate(point2LineRight);
		double posePosition[3];
		Matrix<double> posePositionM(3, 1);
		Matrix<double> result(3, 1);
		posMatrixRight->GetPosition(posePosition);

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

		vector<Point2d> projectedPoints;
		double xPrime = objectPoints.at<double>(0, 0) / objectPoints.at<double>(0, 2);
		double yPrime = objectPoints.at<double>(0, 1) / objectPoints.at<double>(0, 2);

		double u = (rightIntrinsicParam[0][0] * xPrime) + rightIntrinsicParam[0][2];
		double v = (rightIntrinsicParam[1][1] * yPrime) + rightIntrinsicParam[1][2];

		vector<Point2f> center(1);
		center[0].x = u;
		center[0].y = v;

		// Get right frame
		rightMixer->GetChannel()->GetTrackedFrame(rightMixerFrame);

		// Get right image
		vtkImageData *rightImage = rightMixerFrame.GetImageData()->GetImage();
		int rightDims[3];
		rightImage->GetDimensions(rightDims);

		// Copy vtkImage to cv::Mat
		matRight = cv::Mat(rightDims[1], rightDims[0], CV_8UC3, rightImage->GetScalarPointer(0, 0, 0));

		// Undistort images
		undistortedRight = cv::Mat(rightDims[1], rightDims[0], CV_8UC3);
		undistort(matRight, undistortedRight, intrinsicRight, distortionRight);

		// Flip image to draw circles
		cv::flip(undistortedRight, finalMatRight, 0);

		// circle center
		circle(finalMatRight, center[0], 3, (0, 100, 100), -1, 8, 0);

		// circle outline
		circle(finalMatRight, center[0], 14, Scalar(100, 100, 100), 3, 8, 0);

		// Flip back for vtk
		cv::flip(finalMatRight, finalMatRight, 0);
	}
}

void mainWidget::startCalibration(bool checked)
{
  if (checked)
  {
    if (poseCentersLeft.size() != 0)
    {
      poseCentersLeft.resize(0);
    }
	if (poseCentersRight.size() != 0)
	{
		poseCentersRight.resize(0);
	}

    // Refresh table
    for (int i = dataTable->verticalHeader()->count(); i >= 0; i--)
    {
      dataTable->removeRow(i);
    }

    // Initialize matrices
    XLeft.newsize(3, 15);
	XRight.newsize(3, 15);
    originLeft.newsize(3, 15);
	originRight.newsize(3, 15);
    dNormalizedLeft.newsize(3, 15);
	dNormalizedRight.newsize(3, 15);

	trackerWidget->nextPoseButton->setEnabled(true);
	trackerWidget->manualButton->setEnabled(true);
  }
  else
  {
	  trackerWidget->nextPoseButton->setDisabled(true);
	  trackerWidget->manualButton->setDisabled(true);
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

}
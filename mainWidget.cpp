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
#include <I:/d/VTK/ThirdParty/glew/vtkglew/include/GL/glew.h>

// local includes
#include "mainWidget.h"
#include "eccTrackerWidget.h"

// C++ includes
#include <vector>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

// QT includes
#include <QtGui>
#include <QLabel>
#include <QLineEdit>
#include "qstatusbar.h"
#include "qaction.h"
#include "qmenubar.h"
#include "qmenu.h"
#include "qdockwidget.h"
#include "qmessagebox.h"
#include "QVBoxLayout"
#include "qframe.h"
#include "qboxlayout.h"
#include <QHeaderView>
#include <QTableWidget>

// VTK includes
#include <QVTKWidget.h>
#include <vtkNDITracker.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkTrackerTool.h>
#include <vtkTransform.h>

// VTK
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTexture.h>
#include <vtkImageImport.h>
#include <vtkImageMapper.h>
#include <vtkImageData.h>
#include <vtkTextureMapToPlane.h>

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

using namespace cv;
using namespace std;
OVR::OvrvisionPro OvrvisionProHandle;

mainWidget::mainWidget(QWidget *parent)
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

	/*!
	* create the rest of the QT/GUI.
	*/
	controlDock = 0;
	createActions();
	createMenus();
	createStatusBar();
	createToolInformation();
	ovrvision();
}

mainWidget::~mainWidget()
{
	myTracker->StopTracking();   /*!< Make sure the tracker is stopped when the program exits. */
	this->destroyVTKObjects();   /*!< VTK cleanup. */
	
	if (OvrvisionProHandle.isOpen()) /*!< Close ovrvision device if still open upon exit. */
	{
		OvrvisionProHandle.Close();
	}
}

void mainWidget::ovrvision()
{
	// Set exposure, gain and BLC to enhance images
	OvrvisionProHandle.SetCameraExposure(16808); 
	OvrvisionProHandle.SetCameraGain(40);
	OvrvisionProHandle.SetCameraBLC(32);

	OVR::Camprop RequestedFormat(OVR::OV_CAMVR_FULL); /*!< Requested capture format - 960x950. */
	bool CameraSync(true);

	// 10DE is NVIDIA vendor ID
	auto vendor = "NVIDIA Corporation";
	if (!OvrvisionProHandle.Open(0, RequestedFormat, vendor)) // We don't need to share it with OpenGL/D3D, but in the future we could access the images in GPU memory
	{
		printf("Unable to connect to OvrvisionPro device.");
		exit(1);
	}

	OvrvisionProHandle.SetCameraSyncMode(CameraSync);

	// Grab Left and Right Images
	cv::Mat matLeft(OvrvisionProHandle.GetCamHeight(), OvrvisionProHandle.GetCamWidth(), CV_8UC4, OvrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_LEFT));
	cv::Mat matRight(OvrvisionProHandle.GetCamHeight(), OvrvisionProHandle.GetCamWidth(), CV_8UC4, OvrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_RIGHT));

	// Check Focal and Principal Points
	OVR::OvrvisionSetting ovrset(&OvrvisionProHandle);
	ovrset.ReadEEPROM();
	cv::Mat leftIntrinsic;
	cv::Mat leftDistortion;
	leftIntrinsic = ovrset.m_leftCameraInstric;
	leftDistortionParam = ovrset.m_leftCameraDistortion;
	leftIntrinsicParam = Matrix<double>(3, 3);
	leftIntrinsicParam[0][0] = leftIntrinsic.at<double>(0, 0);
	leftIntrinsicParam[0][1] = 0;
	leftIntrinsicParam[0][2] = leftIntrinsic.at<double>(0, 2);
	leftIntrinsicParam[1][0] = 0;
	leftIntrinsicParam[1][1] = leftIntrinsic.at<double>(1, 1);
	leftIntrinsicParam[1][2] = leftIntrinsic.at<double>(1, 2);
	leftIntrinsicParam[2][0] = 0;
	leftIntrinsicParam[2][1] = 0;
	leftIntrinsicParam[2][2] = 1;

	double focalLeftX = leftIntrinsic.at<double>(0, 0);
	double focalLeftY = leftIntrinsic.at<double>(1, 1);
	double principalLeftX = leftIntrinsic.at<double>(0, 2);
	double principalLeftY = leftIntrinsic.at<double>(1, 2);

	cv::Mat rightIntrinsic;
	rightIntrinsic = ovrset.m_rightCameraInstric;
	double focalRightX = rightIntrinsic.at<double>(0, 0);
	double focalRightY = rightIntrinsic.at<double>(1, 1);
	double principalRightX = rightIntrinsic.at<double>(0, 2);
	double principalRightY = rightIntrinsic.at<double>(1, 2);

	// Query the SDK for the latest frames
	OvrvisionProHandle.PreStoreCamData(OVR::OV_CAMQT_DMSRMP);
}

void mainWidget::ovrvisionUpdate()
{
	// Initialize Variables
	cv::Mat hsv;
	cv::Mat thresholdFinal;

	// Query the SDK for the latest frames
	OvrvisionProHandle.PreStoreCamData(OVR::OV_CAMQT_DMSRMP);

	// Grab Left and Right Images
	cv::Mat matLeft(OvrvisionProHandle.GetCamHeight(), OvrvisionProHandle.GetCamWidth(), CV_8UC4, OvrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_LEFT));
	cv::Mat matRight(OvrvisionProHandle.GetCamHeight(), OvrvisionProHandle.GetCamWidth(), CV_8UC4, OvrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_RIGHT));
}

Point pt1;
void mainWidget::onMouse(int event, int x, int y, int flags, void *param)
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

// on accept the next pose is calculated
void mainWidget::nextPose(bool checked)
{
	if (checked)
	{
		ovrvisionUpdate();

		// if GetTransform is clicked
		if (trackerWidget->calibrationButton->isChecked() == true)
		{
			// Initialize Variables
			cv::Mat hsv;
			cv::Mat threshold;

			// Query the SDK for the latest frames
			OvrvisionProHandle.PreStoreCamData(OVR::OV_CAMQT_DMSRMP);

			// Grab Left and Right Images
			cv::Mat matLeft(OvrvisionProHandle.GetCamHeight(), OvrvisionProHandle.GetCamWidth(), CV_8UC4, OvrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_LEFT));
			cv::Mat matRight(OvrvisionProHandle.GetCamHeight(), OvrvisionProHandle.GetCamWidth(), CV_8UC4, OvrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_RIGHT));

			namedWindow("BGRA", CV_WINDOW_AUTOSIZE);
			imshow("BGRA", matLeft);
			cv::waitKey(0);

			// Convert BGRA image to HSV image
			cv::cvtColor(matLeft, hsv, COLOR_BGR2HSV);

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
			vector<Vec3f> circles;

			// Apply the Hough Transform to find the circles
			HoughCircles(mask, circles, CV_HOUGH_GRADIENT, 2, mask.rows / 16, 255, 30);

			// Show Blur result
			namedWindow("Blur", CV_WINDOW_AUTOSIZE);
			imshow("Blur", mask);
			cv::waitKey(0);

			// Draw the circles detected
			Mat canny_output;
			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;

			// Outline circle and centroid
			if (circles.size() > 0)
			{
				Point center(cvRound(circles[0][0]), cvRound(circles[0][1]));
				int radius = cvRound(circles[0][2]);
				poseCenters.push_back(center);

				// circle center
				circle(mask, center, 3, (0, 100, 100), -1, 8, 0);

				// circle outline
				circle(mask, center, radius, Scalar(100, 100, 100), 3, 8, 0);
			}
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

				poseCenters.push_back(center[0]);
			}

			getTransform();

			/// Show your results
			namedWindow("Circle Found", CV_WINDOW_AUTOSIZE);
			imshow("Circle Found", mask);
			cv::waitKey(0);
		}
	}

	// Uncheck next Pose button
	trackerWidget->nextPoseButton->setChecked(false);
}

// On manual select
void mainWidget::manualSelection(bool checked)
{
	if (checked)
	{
		ovrvisionUpdate();

		// if GetTransform is clicked
		if (trackerWidget->calibrationButton->isChecked() == true)
		{
			// Initialize Variables
			cv::Mat hsv;
			cv::Mat threshold;

			// Query the SDK for the latest frames
			OvrvisionProHandle.PreStoreCamData(OVR::OV_CAMQT_DMSRMP);

			// Grab Left and Right Images
			cv::Mat matLeft(OvrvisionProHandle.GetCamHeight(), OvrvisionProHandle.GetCamWidth(), CV_8UC4, OvrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_LEFT));
			cv::Mat matRight(OvrvisionProHandle.GetCamHeight(), OvrvisionProHandle.GetCamWidth(), CV_8UC4, OvrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_RIGHT));

			namedWindow("BGRA", CV_WINDOW_AUTOSIZE);
			imshow("BGRA", matLeft);
			cv::waitKey(0);

			// Convert BGRA image to HSV image
			cv::cvtColor(matLeft, hsv, COLOR_BGR2HSV);

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

			circle(mask, pt1, 3, (0, 100, 100), -1, 8, 0);

			/// Show your results
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
void mainWidget::createVTKObjects() {

	myTracker = vtkSmartPointer< vtkNDITracker >::New();
	ren = vtkSmartPointer< vtkRenderer >::New();
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
void mainWidget::setupVTKPipeline() {

	ren->SetBackground(.1, .2, .4);
	qvtk->GetRenderWindow()->AddRenderer(ren);

	myTracker->LoadVirtualSROM(4, "I:/Stylus_PRevision3.rom"); // reference rom in port 4
	referenceCoil = myTracker->GetTool(4);
	myTracker->LoadVirtualSROM(5, "I:/HMD.rom"); // oculusHMD rom in port 5
	oculusHMD = myTracker->GetTool(5);

	// reset the camera according to visible actors
	ren->ResetCamera();
	ren->ResetCameraClippingRange();
	qvtk->GetRenderWindow()->Render();
	isTrackerInit = false;

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
		{  /*!< update the US probe button */
			trackerWidget->lightWidgets[0]->GreenOn();
		}
		else
		{
			trackerWidget->lightWidgets[0]->RedOn();
		}

		if (!oculusHMD->IsMissing() &&
			!oculusHMD->IsOutOfView() &&
			!oculusHMD->IsOutOfVolume())
		{  /*!< update the US probe button */
			trackerWidget->lightWidgets[1]->GreenOn();
		}
		else
		{
			trackerWidget->lightWidgets[1]->RedOn();
		}
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
	{  /*! Bottom is un-toggled. */
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
	if (checked) {
		std::cerr << "checked" << std::endl;

		double m[16] = { 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1 };

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
	else {
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
		//isStylusCalibrated = true;
	}
}

void mainWidget::viewScene(bool checked)
{
	if (checked)
	{
		if (OvrvisionProHandle.isOpen())
		{
			OvrvisionProHandle.Close();
		}

		// Requested capture format
		OVR::OvrvisionPro OvrvisionProHandle;
		OVR::Camprop RequestedFormat(OVR::OV_CAMVR_FULL); // 960x950
		bool CameraSync(true);

		//Create a renderer, render window, and interactor
		vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
		vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
		renderWindow->AddRenderer(renderer);

		vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
		renderWindowInteractor->SetRenderWindow(renderWindow);

		vtkOpenVRRenderWindow* vrWindow = dynamic_cast<vtkOpenVRRenderWindow*>(renderWindow.Get());

		vrWindow->SetTexturedBackground(true);

		// 10DE is NVIDIA vendor ID
		auto vendor = "NVIDIA Corporation";
		if (!OvrvisionProHandle.Open(0, RequestedFormat, vendor)) // We don't need to share it with OpenGL/D3D, but in the future we could access the images in GPU memory
		{
			printf("Unable to connect to OvrvisionPro device.");
			exit(1);
		}

		OvrvisionProHandle.SetCameraSyncMode(CameraSync);

		//Render Window
		vtkSmartPointer<vtkTexture> textureLeft =
			vtkSmartPointer<vtkTexture>::New();
		vtkSmartPointer<vtkTexture> textureRight =
			vtkSmartPointer<vtkTexture>::New();

		vtkSmartPointer<vtkImageImport> imageImportLeft =
			vtkSmartPointer<vtkImageImport>::New();

		vtkSmartPointer<vtkImageImport> imageImportRight =
			vtkSmartPointer<vtkImageImport>::New();

		vrWindow->AddRenderer(renderer);

		// Grab Left and Right Images
		cv::Mat matLeft(OvrvisionProHandle.GetCamHeight(), OvrvisionProHandle.GetCamWidth(), CV_8UC4, OvrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_LEFT));
		cv::Mat matRight(OvrvisionProHandle.GetCamHeight(), OvrvisionProHandle.GetCamWidth(), CV_8UC4, OvrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_RIGHT));

		cv::Mat rgbMatLeft;
		cv::Mat finalMatLeft;
		cv::Mat rgbMatRight;
		cv::Mat finalMatRight;

		// Check Focal and Principal Points
		OVR::OvrvisionSetting ovrset(&OvrvisionProHandle);
		ovrset.ReadEEPROM();
		cv::Mat leftIntrinsic;
		leftIntrinsic = ovrset.m_leftCameraInstric;
		double focalLeftX = leftIntrinsic.at<double>(0, 0);
		double focalLeftY = leftIntrinsic.at<double>(1, 1);
		double principalLeftX = leftIntrinsic.at<double>(0, 2);
		double principalLeftY = leftIntrinsic.at<double>(1, 2);

		cv::Mat rightIntrinsic;
		rightIntrinsic = ovrset.m_rightCameraInstric;
		double focalRightX = rightIntrinsic.at<double>(0, 0);
		double focalRightY = rightIntrinsic.at<double>(1, 1);
		double principalRightX = rightIntrinsic.at<double>(0, 2);
		double principalRightY = rightIntrinsic.at<double>(1, 2);

		// Convert From BGRA to RGB
		cv::cvtColor(matLeft, rgbMatLeft, CV_BGRA2RGB);
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

		vrWindow->SetLeftBackgroundTexture(textureLeft);
		vrWindow->SetRightBackgroundTexture(textureRight);

		while (trackerWidget->viewSceneButton->isChecked() == true)
		{
			// Query the SDK for the latest frames
			OvrvisionProHandle.PreStoreCamData(OVR::OV_CAMQT_DMSRMP);

			// Grab Left and Right Images
			cv::Mat matLeft(OvrvisionProHandle.GetCamHeight(), OvrvisionProHandle.GetCamWidth(), CV_8UC4, OvrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_LEFT));
			cv::Mat matRight(OvrvisionProHandle.GetCamHeight(), OvrvisionProHandle.GetCamWidth(), CV_8UC4, OvrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_RIGHT));

			// Convert From BGRA to RGB
			cv::cvtColor(matLeft, rgbMatLeft, CV_BGRA2RGB);
			cv::cvtColor(matRight, rgbMatRight, CV_BGRA2RGB);

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
			vrWindow->Render();
			SDL_Event event;
			SDL_WaitEvent(&event);
		}
	}

	if (!checked)
	{
		if (OvrvisionProHandle.isOpen())
		{
			OvrvisionProHandle.Close();
		}
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

		QFrame *mainFrame = new QFrame;
		mainFrame->setFrameStyle(QFrame::WinPanel | QFrame::Sunken);
		mainFrame->setLineWidth(2);

		//QVBoxLayout *vlayout = new QVBoxLayout;
		QGridLayout *controlsLayout = new QGridLayout;
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

		QLabel *HMinLowerLabel = new QLabel(tr("Lower Hue Minimum: "));
		QLabel *HMaxLowerLabel = new QLabel(tr("Lower Hue Maximum: "));
		QLabel *SMinLowerLabel = new QLabel(tr("Lower Saturation Minimum: "));
		QLabel *SMaxLowerLabel = new QLabel(tr("Lower Saturation Maximum: "));
		QLabel *VMinLowerLabel = new QLabel(tr("Lower Value Minimum: "));
		QLabel *VMaxLowerLabel = new QLabel(tr("Lower Value Maximum: "));

		QLabel *HMinUpperLabel = new QLabel(tr("Upper Hue Minimum: "));
		QLabel *HMaxUpperLabel = new QLabel(tr("Upper Hue Maximum: "));
		QLabel *SMinUpperLabel = new QLabel(tr("Upper Saturation Minimum: "));
		QLabel *SMaxUpperLabel = new QLabel(tr("Upper Saturation Maximum: "));
		QLabel *VMinUpperLabel = new QLabel(tr("Upper Value Minimum: "));
		QLabel *VMaxUpperLabel = new QLabel(tr("Upper Value Maximum: "));

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
		controlsLayout->addWidget(HMinUpper , 6, 1);
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

		connect(trackerWidget->viewSceneButton, SIGNAL(toggled(bool)),
			this, SLOT(viewScene(bool)));

		// Calibration widget
		QDockWidget *stylusDock = new QDockWidget(tr("Stylus tip calibration"), this);
		stylusDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
		stylusDock->setFeatures(QDockWidget::AllDockWidgetFeatures);
		stylusDock->setMinimumWidth(180);
		addDockWidget(Qt::RightDockWidgetArea, stylusDock);

		QFrame *frame = new QFrame;
		frame->setFrameStyle(QFrame::WinPanel | QFrame::Sunken);
		frame->setLineWidth(2);

		QVBoxLayout *vl = new QVBoxLayout;
		vl->setMargin(0);
		vl->setSpacing(10);
		vl->setAlignment(Qt::AlignTop);
		frame->setLayout(vl);
		stylusDock->setWidget(frame);

		QPushButton *pivotButton = new QPushButton(tr("Pivot"), this);
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

		QLabel *rms = new QLabel(tr("RMS"), this);
		stylusTipRMS = new QLineEdit;
		stylusTipRMS->setText(tempString);
		vl->addWidget(rms);
		vl->addWidget(stylusTipRMS);
		vl->setAlignment(Qt::AlignTop);

	}
}

void mainWidget::getTransform()
{
	if(poseCenters.size() <= 15 )
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

		vtkSmartPointer<vtkMatrix4x4> point2Line = vtkSmartPointer<vtkMatrix4x4>::New();
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
		vtkSmartPointer< vtkTransform > posMatrix =
			vtkSmartPointer< vtkTransform >::New();
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
		distortion.at<double>(0, 0) = leftDistortionParam[0];
		distortion.at<double>(0, 1) = leftDistortionParam[1];
		distortion.at<double>(0, 2) = leftDistortionParam[2];
		distortion.at<double>(0, 3) = leftDistortionParam[3];
		distortion.at<double>(0, 4) = leftDistortionParam[4];
		distortion.at<double>(0, 5) = leftDistortionParam[5];
		distortion.at<double>(0, 6) = leftDistortionParam[6];
		distortion.at<double>(0, 7) = leftDistortionParam[7];

		vector<Point2d> projectedPoints;
		projectPoints(objectPoints, rvec, tvec, intrinsic, distortion, projectedPoints);

		vector<Point2f> center(1);
		center[0].x = projectedPoints[0].x;
		center[0].y = projectedPoints[0].y;

		// Query the SDK for the latest frames
		OvrvisionProHandle.PreStoreCamData(OVR::OV_CAMQT_DMSRMP);

		// Grab Left and Right Images
		cv::Mat matLeft(OvrvisionProHandle.GetCamHeight(), OvrvisionProHandle.GetCamWidth(), CV_8UC4, OvrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_LEFT));
		cv::Mat matRight(OvrvisionProHandle.GetCamHeight(), OvrvisionProHandle.GetCamWidth(), CV_8UC4, OvrvisionProHandle.GetCamImageBGRA(OVR::OV_CAMEYE_RIGHT));

		// circle center
		circle(matLeft, center[0], 3, (0, 100, 100), -1, 8, 0);
		// circle outline
		circle(matLeft, center[0], 14, Scalar(100, 100, 100), 3, 8, 0);

		/// Show your results
		cv::namedWindow("Circle Found", CV_WINDOW_AUTOSIZE);
		cv::imshow("Circle Found", matLeft);
		cv::waitKey(0);

		cv::Mat hsv;
		cv::Mat threshold;
		// Convert BGRA image to HSV image
		cv::cvtColor(matLeft, hsv, COLOR_BGR2HSV);

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
		namedWindow("Blur", CV_WINDOW_AUTOSIZE);
		imshow("Blur", mask);
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
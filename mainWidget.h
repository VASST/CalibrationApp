#pragma once
/*=========================================================================

Program:   Phantom-Less Calibration GUI
Module:    $RCSfile: mainWidget.h,v $
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
//! a QT object class that handles both the visualization and data acquisition.
/*!
This is the class where everything is happening.  It handles both the
visualization (QT/VTK) and data acquisition (AIGS for tracking, and
vtkSonixVideoSource for US image).  A QT timer is used to acquire
data in realtime, and updates the screen accordingly.
*/
#ifndef __MAINWIDGET_H__
#define __MAINWIDGET_H__

// C++ includes
#include <vector>
#include <fstream>

// QT include
#include <QtGui>
#include <QTableWidget>
#include <QSpinBox>

// VTK includes
#include <vtkSmartPointer.h>

// local includes
#include "qmainwindow.h"
#include "qpushbutton.h"
#include "qlightwidget.h"
#include "matrix.h"

// OpenCV
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

// VTK forward declaration
class QVTKWidget;
class vtkRenderer;
class vtkNDITracker;
class vtkTrackerTool;

// local forward declaration
class eccTrackerWidget;
using namespace std;
using namespace cv;

class mainWidget : public QMainWindow {

	Q_OBJECT

public:
	//! A constructor
	mainWidget(QWidget *parent = 0);

	//! A destructor
	~mainWidget();
	void ovrvision();

private: /*!< Private functions, QT related */
	void createActions();
	void createMenus();
	void createStatusBar();

private:
	/*!
	* Centralized place to create all vtk objects.
	*/
	void createVTKObjects();

	/*!
	* A centralized place to delete all vtk objects.
	*/
	void destroyVTKObjects();

	/*!
	* Centralized place to setup all vtk pipelines.
	*/
	void setupVTKPipeline();

	/*!
	* Check the vtkTrackerTool flags to determine
	* what tools are connected.
	*/
	void checkToolPorts();

	private slots: /*! VTK-related slots */
	void updateTrackerInfo();
	void startTrackerSlot(bool);

	private slots:
	/*!
	* A QT slot for display information about this application
	*/
	void about();

	/*!
	* A QT slot for display information about Robarts
	*/
	void aboutRobarts();

	/*!
	* Create a dock window for controlling NDI tracker
	*/
	void createControlDock();

	/*!
	* Update the information from the ovrvision camera
	*/
	void ovrvisionUpdate();
	
	/*!
	* Create a dock window for the tracked tools information
	*/
	void createToolInformation();

	/*!
	* Get transform information from tracking device
	*/
	void getTransform();

	
	/*!
	* Pivot calibration routine
	*/
	void pivotCalibration(bool checked);

	/*!
	* Grab x,y cordinates in image when called
	*/
	static void onMouse(int event, int x, int y, int, void* userdata);

	void nextPose(bool checked);

	void manualSelection(bool checked);
	
	void startCalibration(bool checked);

	void viewScene(bool checked);

private: /*!< Private QT members. */
	QAction                                         *aboutAct;
	QAction                                         *quitAct;
	QAction                                         *aboutRobartsAct;
	QAction                                         *controlAct;
	QPushButton                                     *trackerButton;
	QTimer                                          *trackerTimer;
	QTimer                                          *ovrvisionTimer;
	QMenu                                           *fileMenu;
	QMenu                                           *helpMenu;
	QMenu                                           *calibMenu;
	QMenu                                           *controlMenu;
	QDockWidget                                     *controlDock;
	QTimer											*mTimer;
	QDockWidget										*toolInfo;
	QTableWidget									*dataTable;
	QSpinBox										*HMinLower;
	QSpinBox										*HMaxLower;
	QSpinBox										*SMinLower;
	QSpinBox										*VMinLower;
	QSpinBox										*VMaxLower;
	QSpinBox										*SMaxLower;
	QSpinBox										*HMinUpper;
	QSpinBox										*HMaxUpper;
	QSpinBox										*SMinUpper;
	QSpinBox										*VMinUpper;
	QSpinBox										*VMaxUpper;
	QSpinBox										*SMaxUpper;
	QLineEdit										*stylusTipRMS;

private: /*!< Private VTK members. */
	QVTKWidget                                      *qvtk;
	std::vector< QLightWidget *>                    lightWidgets;
	vtkSmartPointer< vtkRenderer >                  ren;

	/*!
	* Tracker related objects.
	*/
	vtkSmartPointer< vtkNDITracker >                myTracker;
	vtkSmartPointer< vtkTrackerTool >               oculusHMD;
	vtkSmartPointer< vtkTrackerTool >               referenceCoil;
	bool											isProbeVisible, isOculusVisible;

	Matrix<double>									leftIntrinsicParam;
	vector<double>									leftDistortionParam;
	cv::Mat											thresholdFinal;
	vector<Point2f>									poseCenters;
	Matrix<double>									X;
	Matrix<double>									origin;
	Matrix<double>									dNormalized;

private:
	eccTrackerWidget                                *trackerWidget;

private: /*!< Misc variables */
	bool                                            isTrackerInit;

	int qLightOrder[4];
};

#endif // of __MAINWIDGET_H__
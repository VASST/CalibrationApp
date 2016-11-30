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
#include <vtk_glew.h>

// C++ includes
#include <vector>
#include <fstream>

// QT include
#include <QtGui>
#include <QTableWidget>
#include <QSpinBox>

// VTK includes
#include <vtkSmartPointer.h>
#include <vtkMatrix4x4.h>
#include <vtkActor.h>

// local includes
#include "qmainwindow.h"
#include "qpushbutton.h"
#include "qlightwidget.h"
#include "matrix.h"

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// stl includes
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

// VTK includes
#include <vtkOpenVRRenderWindowInteractor.h>
#include <vtkOpenVRRenderWindow.h>
#include <vtkOpenVRRenderer.h>
#include <vtkOpenVRCamera.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataReader.h>

// Plus includes
#include <vtkPlusDataCollector.h>
#include "PlusConfigure.h"
#include "vtkPlusTransformRepository.h"
#include "PlusTrackedFrame.h"
#include "vtkPlusChannel.h"
#include <vtkPlusNDITracker.h>
#include <vtkPlusVolumeReconstructor.h>
#include <vtkPlusOvrvisionProVideoSource.h>

// Ovrvision includes
#include <ovrvision_pro.h>

// VTK forward declaration
class QVTKWidget;
class vtkRenderer;
class vtkTexture;
class vtkImageImport;

// local forward declaration
class eccTrackerWidget;
using namespace std;
using namespace cv;

class mainWidget : public QMainWindow
{
	Q_OBJECT

public:
	//! A constructor
	mainWidget(QWidget* parent = 0);

	//! A destructor
	~mainWidget();

private: /*!< Private functions, QT related */
	/*!
	* Set up GUI
	*/
	void createActions();

	/*!
	* Set up GUI
	*/
	void createMenus();
  
	/*!
	* Set up GUI
	*/
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
  
	/*!
	* Get intrinsics and distortion params
	*/
	void setupARRendering();

private slots:
	/*!
	* Update tracking and image data on timeout
	*/
	void updateTrackerInfo();

	/*!
	* Start tracking
	*/
	void startTrackerSlot(bool);

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
	* Create a dock window for the tracked tools information
	*/
	void createToolInformation();

	/*!
	* Get transform information from tracking device for left camera
	*/
	void getLeftTransform();

	/*!
	* Get transform information from tracking device for right camera
	*/
	void getRightTransform();
	
	/*!
	* Grab x,y coordinates in image when called
	*/
	static void onMouse(int event, int x, int y, int, void* userdata);

	/*!
	* Grabs the next pose for calibration
	*/
	void nextPose(bool checked);

	/*!
	* Allows user to manually select the point if previous circle detection was wrong
	*/
	void manualSelection(bool checked);

	/*!
	* Start hand-eye calibration
	*/
	void startCalibration(bool checked);

	/*!
	* View scene from camera
	*/
	void viewScene(bool checked);

	/*! 
	* Save the image and 4x4 matrix of tracked camera
	*/
	void collectPose();

private: /*!< Private QT members. */
	QAction*										aboutAct;
	QAction*										quitAct;
	QAction*										aboutRobartsAct;
	QAction*										controlAct;
	QPushButton*									trackerButton;
	QTimer*											trackerTimer;
	QTimer*											ovrvisionTimer;
	QMenu*											fileMenu;
	QMenu*											helpMenu;
	QMenu*											calibMenu;
	QMenu*											controlMenu;
	QDockWidget*									controlDock;
	QTimer*											mTimer;
	QDockWidget*									toolInfo;
	QTableWidget*									dataTable;
	QSpinBox*										HMinLower;
	QSpinBox*										HMaxLower;
	QSpinBox*										SMinLower;
	QSpinBox*										VMinLower;
	QSpinBox*										VMaxLower;
	QSpinBox*										SMaxLower;
	QSpinBox*										HMinUpper;
	QSpinBox*										HMaxUpper;
	QSpinBox*										SMinUpper;
	QSpinBox*										VMinUpper;
	QSpinBox*										VMaxUpper;
	QSpinBox*										SMaxUpper;
	QLineEdit*										stylusTipRMS;
	QTimer*											uiUpdateTimer;

private: /*!< Private VTK members. */
	QVTKWidget*                                      qvtk;
	std::vector< QLightWidget*>                      lightWidgets;
	vtkSmartPointer< vtkOpenVRRenderer >             ren							= vtkSmartPointer<vtkOpenVRRenderer>::New();
	vtkSmartPointer<vtkOpenVRCamera>				 vrCamera						= vtkSmartPointer<vtkOpenVRCamera>::New();
	vtkSmartPointer<vtkOpenVRRenderWindow>			 renWindow						= vtkSmartPointer<vtkOpenVRRenderWindow>::New();
	vtkSmartPointer<vtkOpenVRRenderWindowInteractor> renInt							= vtkSmartPointer<vtkOpenVRRenderWindowInteractor>::New();

	bool											isProbeVisible;
	bool											isOculusVisible;
	bool											isViewScene;
	int imageCount = 0;
	echen::Matrix<double>							leftIntrinsicParam;
	echen::Matrix<double>							rightIntrinsicParam;
	vector<double>									leftDistortionParam;
	vector<double>									rightDistortionParam;
	vector<Point2f>									poseCentersLeft;
	vector<Point2f>									poseCentersRight;
	echen::Matrix<double>							XLeft;
	echen::Matrix<double>							XRight;
	echen::Matrix<double>							originLeft;
	echen::Matrix<double>							originRight;
	echen::Matrix<double>							dNormalizedLeft;
	echen::Matrix<double>							dNormalizedRight;

	vtkSmartPointer<vtkTexture>						textureLeft						= vtkSmartPointer<vtkTexture>::New();
	vtkSmartPointer<vtkTexture>						textureRight					= vtkSmartPointer<vtkTexture>::New();
	vtkSmartPointer<vtkImageImport>					imageImportLeft					= vtkSmartPointer<vtkImageImport>::New();
	vtkSmartPointer<vtkImageImport>					imageImportRight				= vtkSmartPointer<vtkImageImport>::New();
	vtkSmartPointer<vtkMatrix4x4>					point2LineLeft					= vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					point2LineRight					= vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tempP2L							= vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkActor>						phantomActor					= vtkSmartPointer<vtkActor>::New();
	vtkSmartPointer<vtkTransform>					phantomToPhysicalTransform		= vtkSmartPointer<vtkTransform>::New();
	vtkSmartPointer<vtkOpenVRRenderWindow>			vrWindow						= vtkSmartPointer<vtkOpenVRRenderWindow>::New();
	vtkSmartPointer<vtkTransform>					transformP2L					= vtkSmartPointer<vtkTransform>::New();
	vtkSmartPointer<vtkTransform>					transformP2LInv					= vtkSmartPointer<vtkTransform>::New();
	vtkSmartPointer<vtkTransform>					cameraTransform					= vtkSmartPointer<vtkTransform>::New();
	vtkSmartPointer<vtkTransform>					cameraInvTransform				= vtkSmartPointer<vtkTransform>::New();
	vtkSmartPointer<vtkActor>						sphereActor						= vtkSmartPointer<vtkActor>::New();

	// Open cv matrices for images
	cv::Mat											matLeft;
	cv::Mat											matRight;
	cv::Mat											finalMatLeft;
	cv::Mat											finalMatRight;
	cv::Mat											undistortedLeft;
	cv::Mat											undistortedRight;

	// Plus members
	vtkSmartPointer< vtkXMLDataElement>				configRootElement				= vtkSmartPointer<vtkXMLDataElement>::New();
	vtkSmartPointer<vtkPlusDataCollector>			dataCollector					= vtkSmartPointer<vtkPlusDataCollector>::New();;
	vtkSmartPointer<vtkPlusTransformRepository>		repository						= vtkSmartPointer<vtkPlusTransformRepository>::New();

	vtkPlusDevice									*trackerDevice;
	vtkPlusDevice									*ovrDevice;
	vtkPlusDevice									*leftMixerDevice;
	vtkPlusDevice									*rightMixerDevice;
	vtkPlusChannel									*trackerChannel;
	vtkPlusChannel									*leftVideoChannel;
	vtkPlusChannel									*rightVideoChannel;
	vtkPlusChannel									*leftMixerChannel;
	vtkPlusChannel									*rightMixerChannel;

	vtkPlusNDITracker								*ndiTracker;
	vtkPlusOvrvisionProVideoSource					*ovrVideo;

	vtkSmartPointer<vtkPlusVirtualMixer>			leftMixer						= vtkSmartPointer<vtkPlusVirtualMixer>::New();
	vtkSmartPointer<vtkPlusVirtualMixer>			rightMixer						= vtkSmartPointer<vtkPlusVirtualMixer>::New();

	PlusTrackedFrame								trackedFrame;
	PlusTrackedFrame								leftVideoFrame;
	PlusTrackedFrame								rightVideoFrame;
	PlusTrackedFrame								leftMixerFrame;
	PlusTrackedFrame								rightMixerFrame;

	// Plus Transform Names
	PlusTransformName								camera2TrackerName;
	PlusTransformName								probe2TrackerName;
	PlusTransformName								probe2CameraName;
	PlusTransformName								tip2ProbeName;
	PlusTransformName								tip2LImageName;
	PlusTransformName								tip2CameraName;
	PlusTransformName								camera2LImageName;

	// Plus Transforms
	vtkSmartPointer<vtkMatrix4x4>					tProbe2Tracker					= vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tCamera2Image					= vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tTracker2Camera					= vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tCamera2Tracker					= vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tProbe2Image					= vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tTip2ImageL						= vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tTip2ImageR						= vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tTip2Camera						= vtkSmartPointer<vtkMatrix4x4>::New();

	// Auxiliary memers
	vtkMatrix4x4									*pointerPose;

	// input configuration file name
	std::string										inputConfigFileName;

	// Camera intrinsics file name
	std::string										intrinsicsFileName;

	//File stream for experimental output
	std::ofstream									expOutFile;

	// subject ID
	std::string										subjectID;

	// path to experimental results
	std::string										results_root_dir;

	// Path to calibration saved
	std::string										calibration_root_dir;

	// Calibration pose File
	std::ofstream									cam_pose_file;

	// Cam calibration file prefix and post-fix
	std::string										cam_calib_file_prefix;
	std::string										cam_calib_file_postfix;

	// Image counter
	int												img_index;

	bool											renderAR = false;
	vtkSmartPointer<vtkOpenVRRenderer>				arRenderer						= vtkSmartPointer<vtkOpenVRRenderer>::New();
	OVR::OvrvisionPro								ovrvisionProHandle;
	vtkSmartPointer< vtkTransform >					posMatrixLeft					= vtkSmartPointer< vtkTransform >::New();
	vtkSmartPointer< vtkTransform >					posMatrixRight					= vtkSmartPointer< vtkTransform >::New();
	cv::Mat											intrinsicLeft					= Mat(3, 3, CV_64FC1);
	cv::Mat											intrinsicRight					= Mat(3, 3, CV_64FC1);
	cv::Mat											distortionLeft					= Mat(1, 4, CV_64FC1);
	cv::Mat											distortionRight					= Mat(1, 4, CV_64FC1);

private:
  eccTrackerWidget*									trackerWidget;

private: /*!< Misc variables */
  bool												isTrackerInit;

  int												qLightOrder[4];
};

#endif // of __MAINWIDGET_H__
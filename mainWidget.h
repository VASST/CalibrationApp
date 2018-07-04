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
visualization (QT/VTK) and data acquisition (Plus library).  A QT timer is used to acquire
data in realtime, and updates the screen accordingly.
*/
#ifndef __MAINWIDGET_H__
#define __MAINWIDGET_H__
#include <vtk_glew.h>

// QT includes
#include <QMainWindow>

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
#include <opencv2/core.hpp>

// STD includes
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

// Plus includes
#include <PlusConfigure.h>
#include <PlusTrackedFrame.h>

class QAction;
class QDockWidget;
class QLineEdit;
class QMenu;
class QSpinBox;
class QTableWidget;
class QTimer;
class QVTKWidget;
class eccTrackerWidget;
class vtkImageImport;
class vtkOpenVRCamera;
class vtkOpenVRRenderWindow;
class vtkOpenVRRenderWindowInteractor;
class vtkOpenVRRenderer;
class vtkPlusChannel;
class vtkPlusDataCollector;
class vtkPlusDevice;
class vtkPlusNDITracker;
class vtkPlusTransformRepository;
class vtkPlusVirtualMixer;
class vtkPlusVolumeReconstructor;
class vtkRenderer;
class vtkTexture;

class mainWidget : public QMainWindow
{
  Q_OBJECT

public:
  //! A constructor
  mainWidget(QWidget* parent = 0);

  //! A destructor
  ~mainWidget();

protected:
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

  /*!
  * Get transform information from tracking device for camera
  */
  bool getTransform(cv::Mat& image,
                    cv::Mat& undistorted,
                    cv::Mat& intrinParamCv,
                    cv::Mat& distortion,
                    cv::Mat& finalImage,
                    std::vector<cv::Point2f>& poseCenters,
                    echen::Matrix<double>& x,
                    vtkTransform& posMatrix,
                    echen::Matrix<double>& origin,
                    echen::Matrix<double>& intrinParam,
                    echen::Matrix<double>& dNormalized,
                    vtkMatrix4x4& pointToLine,
                    vtkPlusVirtualMixer* mixer,
                    PlusTrackedFrame& frame);

protected slots:
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

protected:
  QAction*                    AboutAction;
  QAction*                    QuitAction;
  QAction*                    AboutRobartsAction;
  QAction*                    ControlAction;
  QTimer*                     TrackerTimer;
  QMenu*                      FileMenu;
  QMenu*                      HelpMenu;
  QMenu*                      CalibMenu;
  QMenu*                      ControlMenu;
  QDockWidget*                ControlDock;
  QDockWidget*                ToolInfo;
  QTableWidget*               DataTable;
  QSpinBox*                   HMinLower;
  QSpinBox*                   HMaxLower;
  QSpinBox*                   SMinLower;
  QSpinBox*                   VMinLower;
  QSpinBox*                   VMaxLower;
  QSpinBox*                   SMaxLower;
  QSpinBox*                   HMinUpper;
  QSpinBox*                   HMaxUpper;
  QSpinBox*                   SMinUpper;
  QSpinBox*                   VMinUpper;
  QSpinBox*                   VMaxUpper;
  QSpinBox*                   SMaxUpper;
  QLineEdit*                  StylusTipRMS;

protected:
  QVTKWidget*                                       QVTK;
  std::vector<QLightWidget*>                        LightWidgets;
  vtkSmartPointer<vtkOpenVRRenderer>                Renderer = vtkSmartPointer<vtkOpenVRRenderer>::New();
  vtkSmartPointer<vtkOpenVRCamera>                  VRCamera = vtkSmartPointer<vtkOpenVRCamera>::New();
  vtkSmartPointer<vtkOpenVRRenderWindow>            RenWindow = vtkSmartPointer<vtkOpenVRRenderWindow>::New();
  vtkSmartPointer<vtkOpenVRRenderWindowInteractor>  RenInt = vtkSmartPointer<vtkOpenVRRenderWindowInteractor>::New();

  bool                                              isProbeVisible;
  bool                                              isOculusVisible;
  bool                                              isViewScene;
  int                                               ImageCount = 0;
  echen::Matrix<double>                             LeftIntrinsicParam;
  echen::Matrix<double>                             RightIntrinsicParam;
  std::vector<cv::Point2f>                          PoseCentersLeft;
  std::vector<cv::Point2f>                          PoseCentersRight;
  echen::Matrix<double>                             XLeft;
  echen::Matrix<double>                             XRight;
  echen::Matrix<double>                             OriginLeft;
  echen::Matrix<double>                             OriginRight;
  echen::Matrix<double>                             DNormalizedLeft;
  echen::Matrix<double>                             DNormalizedRight;

  vtkSmartPointer<vtkTexture>                       TextureLeft = vtkSmartPointer<vtkTexture>::New();
  vtkSmartPointer<vtkTexture>                       TextureRight = vtkSmartPointer<vtkTexture>::New();
  vtkSmartPointer<vtkImageImport>                   ImageImportLeft = vtkSmartPointer<vtkImageImport>::New();
  vtkSmartPointer<vtkImageImport>                   ImageImportRight = vtkSmartPointer<vtkImageImport>::New();
  vtkSmartPointer<vtkMatrix4x4>                     Point2LineLeft = vtkSmartPointer<vtkMatrix4x4>::New();
  vtkSmartPointer<vtkMatrix4x4>                     Point2LineRight = vtkSmartPointer<vtkMatrix4x4>::New();
  vtkSmartPointer<vtkOpenVRRenderWindow>            VRWindow = vtkSmartPointer<vtkOpenVRRenderWindow>::New();

  // Open cv matrices for images
  cv::Mat                                           ImageLeft;
  cv::Mat                                           ImageRight;
  cv::Mat                                           FinalImageLeft;
  cv::Mat                                           FinalImageRight;
  cv::Mat                                           UndistortedLeft;
  cv::Mat                                           UndistortedRight;

  // Plus members
  vtkSmartPointer<vtkXMLDataElement>                ConfigRootElement = vtkSmartPointer<vtkXMLDataElement>::New();
  vtkSmartPointer<vtkPlusDataCollector>             DataCollector = vtkSmartPointer<vtkPlusDataCollector>::New();;
  vtkSmartPointer<vtkPlusTransformRepository>       Repository = vtkSmartPointer<vtkPlusTransformRepository>::New();

  vtkPlusDevice*                                    TrackerDevice;
  vtkPlusDevice*                                    OVRDevice;
  vtkPlusDevice*                                    LeftMixerDevice;
  vtkPlusDevice*                                    RightMixerDevice;
  vtkPlusChannel*                                   TrackerChannel;
  vtkPlusChannel*                                   LeftVideoChannel;
  vtkPlusChannel*                                   RightVideoChannel;
  vtkPlusChannel*                                   LeftMixerChannel;
  vtkPlusChannel*                                   RightMixerChannel;

  vtkSmartPointer<vtkPlusVirtualMixer>              LeftMixer = vtkSmartPointer<vtkPlusVirtualMixer>::New();
  vtkSmartPointer<vtkPlusVirtualMixer>              RightMixer = vtkSmartPointer<vtkPlusVirtualMixer>::New();

  PlusTrackedFrame                                  TrackedFrame;
  PlusTrackedFrame                                  LeftMixerFrame;
  PlusTrackedFrame                                  RightMixerFrame;

  // Plus Transform Names
  PlusTransformName                                 Camera2TrackerName;
  PlusTransformName                                 Probe2TrackerName;

  // Plus Transforms
  vtkSmartPointer<vtkMatrix4x4>                     Probe2Tracker = vtkSmartPointer<vtkMatrix4x4>::New();
  vtkSmartPointer<vtkMatrix4x4>                     Camera2Tracker = vtkSmartPointer<vtkMatrix4x4>::New();
  vtkSmartPointer<vtkMatrix4x4>                     Tip2ImageL = vtkSmartPointer<vtkMatrix4x4>::New();
  vtkSmartPointer<vtkMatrix4x4>                     Tip2ImageR = vtkSmartPointer<vtkMatrix4x4>::New();
  vtkSmartPointer<vtkMatrix4x4>                     Tip2Camera = vtkSmartPointer<vtkMatrix4x4>::New();

  // input configuration file name
  std::string                                       InputConfigFileName;

  // path to experimental results
  std::string                                       ResultsRootDir;

  // Path to calibration saved
  std::string                                       CalibrationRootDir;

  vtkSmartPointer<vtkTransform>                     PosMatrixLeft = vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkTransform>                     PosMatrixRight = vtkSmartPointer<vtkTransform>::New();
  cv::Mat                                           IntrinsicLeft = cv::Mat(3, 3, CV_64FC1);
  cv::Mat                                           IntrinsicRight = cv::Mat(3, 3, CV_64FC1);
  cv::Mat                                           DistortionLeft = cv::Mat(1, 4, CV_64FC1);
  cv::Mat                                           DistortionRight = cv::Mat(1, 4, CV_64FC1);

  eccTrackerWidget*                                 TrackerWidget;

  cv::Point                                         MouseCapturePoint;

  bool                                              IsTrackerInit;
};

#endif // of __MAINWIDGET_H__
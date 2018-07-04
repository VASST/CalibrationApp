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
#include <QSpinBox>
#include <QStatusBar>
#include <QTableWidget>
#include <QVBoxLayout>

// VTK includes
#include <QVTKWidget.h>
#include <vtkImageData.h>
#include <vtkImageImport.h>
#include <vtkImageMapper.h>
#include <vtkOpenVRCamera.h>
#include <vtkOpenVRRenderWindow.h>
#include <vtkOpenVRRenderWindowInteractor.h>
#include <vtkOpenVRRenderer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataReader.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTexture.h>
#include <vtkTransform.h>
#include <vtksys/SystemTools.hxx>

// Plus
#include <PlusTrackedFrame.h>
#include <vtkPlusChannel.h>
#include <vtkPlusDataCollector.h>
#include <vtkPlusDevice.h>
#include <vtkPlusTransformRepository.h>
#include <vtkPlusVirtualMixer.h>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Elvis Math
#include "matrix.h"
#include "mathUtils.h"

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
  createActions();
  createMenus();
  createStatusBar();
  createToolInformation();
}

mainWidget::~mainWidget()
{
  this->destroyVTKObjects();
  this->DataCollector->Stop();
}

void mainWidget::onMouse(int event, int x, int y, int flags, void* param)
{
  mainWidget* wid = (mainWidget*)param;
  switch (event)
  {
    case cv::EVENT_LBUTTONDOWN:
    {
      wid->MouseCapturePoint.x = x;
      wid->MouseCapturePoint.y = y;
      break;
    }
    case cv::EVENT_LBUTTONUP:
    {
      break;
    }
  }
}

void mainWidget::viewScene(bool checked)
{
  if (checked)
  {
    if (!this->IsTrackerInit)
    {
      this->TrackerChannel->GetTrackedFrame(this->TrackedFrame);

      // Get left and right frames
      this->LeftMixer->GetChannel()->GetTrackedFrame(this->LeftMixerFrame);
      this->RightMixer->GetChannel()->GetTrackedFrame(this->RightMixerFrame);

      // Get left and right images
      vtkImageData* leftImage = this->LeftMixerFrame.GetImageData()->GetImage();
      vtkImageData* rightImage = this->RightMixerFrame.GetImageData()->GetImage();
      int leftDims[3];
      int rightDims[3];
      leftImage->GetDimensions(leftDims);
      rightImage->GetDimensions(rightDims);

      // Copy vtkImage to cv::Mat
      this->FinalImageLeft = cv::Mat(leftDims[1], leftDims[0], CV_8UC3, leftImage->GetScalarPointer(0, 0, 0));
      this->FinalImageRight = cv::Mat(rightDims[1], rightDims[0], CV_8UC3, rightImage->GetScalarPointer(0, 0, 0));

      // Undistort images
      //undistort(this->ImageLeft, this->FinalImageLeft, this->IntrinsicLeft, this->DistortionLeft);
      //undistort(this->ImageRight, this->FinalImageRight, this->IntrinsicRight, this->DistortionRight);

      // Convert image from opencv to vtk
      this->ImageImportLeft->SetDataSpacing(1, 1, 1);
      this->ImageImportLeft->SetDataOrigin(0, 0, 0);
      this->ImageImportLeft->SetWholeExtent(0, this->FinalImageLeft.size().width - 1, 0, this->FinalImageLeft.size().height - 1, 0, 0);
      this->ImageImportLeft->SetDataExtentToWholeExtent();
      this->ImageImportLeft->SetDataScalarTypeToUnsignedChar();
      this->ImageImportLeft->SetNumberOfScalarComponents(this->FinalImageLeft.channels());
      this->ImageImportLeft->SetImportVoidPointer(this->FinalImageLeft.data);
      this->ImageImportLeft->Update();

      this->ImageImportRight->SetDataSpacing(1, 1, 1);
      this->ImageImportRight->SetDataOrigin(0, 0, 0);
      this->ImageImportRight->SetWholeExtent(0, this->FinalImageRight.size().width - 1, 0, this->FinalImageRight.size().height - 1, 0, 0);
      this->ImageImportRight->SetDataExtentToWholeExtent();
      this->ImageImportRight->SetDataScalarTypeToUnsignedChar();
      this->ImageImportRight->SetNumberOfScalarComponents(this->FinalImageRight.channels());
      this->ImageImportRight->SetImportVoidPointer(this->FinalImageRight.data);
      this->ImageImportRight->Update();

      this->TextureLeft->SetInputConnection(this->ImageImportLeft->GetOutputPort());
      this->TextureRight->SetInputConnection(this->ImageImportRight->GetOutputPort());

      // left eye point to line
      double p2l[16] = { 0.0677294484076749, -0.992989179048552, -0.0968773044158076, -61.6285338697333,
                         0.737296753348973, 0.11523277439025, -0.665668765383645, -14.6388968911687,
                         0.672165321419851, -0.0263419437173227, 0.739932350071099, -4.60575695614759, 0, 0, 0, 1
                       };

      VRWindow = dynamic_cast<vtkOpenVRRenderWindow*>(this->RenWindow.Get());
      //VRWindow->SetTexturedBackground(true);
      //VRWindow->SetLeftBackgroundTexture(this->TextureLeft);
      //VRWindow->SetRightBackgroundTexture(this->TextureRight);
      VRWindow->AddRenderer(this->Renderer);

      //  this->Renderer->ResetCameraClippingRange();
      VRWindow->Render();

      bool isLMatrixValid(false);
      this->Repository->SetTransforms(this->LeftMixerFrame);
      if (this->Repository->GetTransform(PlusTransformName("PointerTip", "LeftImagePlane"), this->Tip2ImageL, &isLMatrixValid) == PLUS_SUCCESS && isLMatrixValid)
      {
        this->PosMatrixLeft->PostMultiply();
        this->PosMatrixLeft->Identity();
        this->PosMatrixLeft->Concatenate(this->Tip2ImageL);
      }

      bool isRMatrixValid(false);
      this->Repository->SetTransforms(this->RightMixerFrame);
      if (this->Repository->GetTransform(PlusTransformName("PointerTip", "RightImagePlane"), this->Tip2ImageR, &isRMatrixValid) == PLUS_SUCCESS && isRMatrixValid)
      {
        this->PosMatrixRight->PostMultiply();
        this->PosMatrixRight->Identity();
        this->PosMatrixRight->Concatenate(this->Tip2ImageR);
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
  this->TrackerChannel->GetTrackedFrame(this->TrackedFrame);

  // Get left and right frames
  this->LeftMixer->GetChannel()->GetTrackedFrame(this->LeftMixerFrame);
  this->RightMixer->GetChannel()->GetTrackedFrame(this->RightMixerFrame);

  // Get left and right images
  vtkImageData* leftImage = this->LeftMixerFrame.GetImageData()->GetImage();
  vtkImageData* rightImage = this->RightMixerFrame.GetImageData()->GetImage();
  int leftDims[3];
  int rightDims[3];
  leftImage->GetDimensions(leftDims);
  rightImage->GetDimensions(rightDims);

  // Copy vtkImage to cv::Mat
  this->ImageLeft = cv::Mat(leftDims[1], leftDims[0], CV_8UC3, leftImage->GetScalarPointer(0, 0, 0));
  this->ImageRight = cv::Mat(rightDims[1], rightDims[0], CV_8UC3, rightImage->GetScalarPointer(0, 0, 0));

  // Undistort images
  cv::undistort(this->ImageLeft, this->FinalImageLeft, this->IntrinsicLeft, this->DistortionLeft);
  cv::undistort(this->ImageRight, this->FinalImageRight, this->IntrinsicRight, this->DistortionRight);

  bool isLMatrixValid(false);
  if (this->Repository->GetTransform(PlusTransformName("PointerTip", "LeftImagePlane"), this->Tip2ImageL, &isLMatrixValid) == PLUS_SUCCESS && isLMatrixValid)
  {
    // Save data
    ofstream myfile("./Results/poseL.csv");
    myfile << this->Tip2ImageL->GetElement(0, 0) << "," << this->Tip2ImageL->GetElement(0, 1) << "," << this->Tip2ImageL->GetElement(0, 2) << "," << this->Tip2ImageL->GetElement(0, 3)
           << "," << this->Tip2ImageL->GetElement(1, 0) << "," << this->Tip2ImageL->GetElement(1, 1) << "," << this->Tip2ImageL->GetElement(1, 2) << "," << this->Tip2ImageL->GetElement(1, 3)
           << "," << this->Tip2ImageL->GetElement(2, 0) << "," << this->Tip2ImageL->GetElement(2, 1) << "," << this->Tip2ImageL->GetElement(2, 2) << "," << this->Tip2ImageL->GetElement(2, 3)
           << "," << this->Tip2ImageL->GetElement(3, 0) << "," << this->Tip2ImageL->GetElement(3, 1) << "," << this->Tip2ImageL->GetElement(3, 2) << "," << this->Tip2ImageL->GetElement(3, 3);
  }

  bool isRMatrixValid(false);
  if (this->Repository->GetTransform(PlusTransformName("PointerTip", "RightImagePlane"), this->Tip2ImageR, &isRMatrixValid) == PLUS_SUCCESS && isRMatrixValid)
  {
    // Save data
    ofstream myfile("./Results/poseR.csv");
    myfile << this->Tip2ImageR->GetElement(0, 0) << "," << this->Tip2ImageR->GetElement(0, 1) << "," << this->Tip2ImageR->GetElement(0, 2) << "," << this->Tip2ImageR->GetElement(0, 3)
           << "," << this->Tip2ImageR->GetElement(1, 0) << "," << this->Tip2ImageR->GetElement(1, 1) << "," << this->Tip2ImageR->GetElement(1, 2) << "," << this->Tip2ImageR->GetElement(1, 3)
           << "," << this->Tip2ImageR->GetElement(2, 0) << "," << this->Tip2ImageR->GetElement(2, 1) << "," << this->Tip2ImageR->GetElement(2, 2) << "," << this->Tip2ImageR->GetElement(2, 3)
           << "," << this->Tip2ImageR->GetElement(3, 0) << "," << this->Tip2ImageR->GetElement(3, 1) << "," << this->Tip2ImageR->GetElement(3, 2) << "," << this->Tip2ImageR->GetElement(3, 3);
  }
  std::string number = std::to_string(ImageCount);
  std::string name = "./Results/poseR" + number;
  std::string fullName = name + ".png";

  // Save left and right images
  cv::imwrite("./Results/poseL.png", this->FinalImageLeft);
  cv::imwrite(fullName, this->ImageRight);
  this->ImageCount++;
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

    this->LeftMixer->GetChannel()->GetTrackedFrame(this->LeftMixerFrame);
    this->RightMixer->GetChannel()->GetTrackedFrame(this->RightMixerFrame);

    vtkImageData* leftImage = this->LeftMixerFrame.GetImageData()->GetImage();
    vtkImageData* rightImage = this->RightMixerFrame.GetImageData()->GetImage();
    int leftDims[3];
    int rightDims[3];
    leftImage->GetDimensions(leftDims);
    rightImage->GetDimensions(rightDims);

    // Copy vtkImage to cv::Mat
    this->ImageLeft = cv::Mat(leftDims[1], leftDims[0], CV_8UC3, leftImage->GetScalarPointer(0, 0, 0));
    this->ImageRight = cv::Mat(rightDims[1], rightDims[0], CV_8UC3, rightImage->GetScalarPointer(0, 0, 0));

    this->UndistortedLeft = cv::Mat(leftDims[1], leftDims[0], CV_8UC3);
    this->UndistortedRight = cv::Mat(rightDims[1], rightDims[0], CV_8UC3);

    cv::undistort(this->ImageLeft, this->UndistortedLeft, this->IntrinsicLeft, this->DistortionLeft);
    cv::undistort(this->ImageRight, this->UndistortedRight, this->IntrinsicRight, this->DistortionRight);

    cv::flip(this->UndistortedLeft, this->UndistortedLeft, 0);
    cv::flip(this->UndistortedRight, this->UndistortedRight, 0);

    // Convert RGB image to HSV image
    cv::cvtColor(this->UndistortedLeft, hsvLeft, cv::COLOR_RGB2HSV);
    cv::cvtColor(this->UndistortedRight, hsvRight, cv::COLOR_RGB2HSV);

    cv::Mat drawingLeft;
    cv::Mat drawingRight;

    cv::cvtColor(this->UndistortedLeft, drawingLeft, cv::COLOR_RGB2BGR);
    cv::cvtColor(this->UndistortedRight, drawingRight, cv::COLOR_RGB2BGR);
    drawingLeft.setTo(cv::Scalar(0, 0, 0));
    drawingRight.setTo(cv::Scalar(0, 0, 0));

    cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE);
    cv::imshow("RGB", this->UndistortedRight);
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
    cv::medianBlur(maskLeft, maskLeft, 5);
    cv::GaussianBlur(maskLeft, maskLeft, cv::Size(9, 9), 2, 2);
    cv::medianBlur(maskRight, maskRight, 5);
    cv::GaussianBlur(maskRight, maskRight, cv::Size(9, 9), 2, 2);

    cv::namedWindow("Blur", cv::WINDOW_AUTOSIZE);
    cv::imshow("Blur", maskRight);
    cv::waitKey(0);

    std::vector<cv::Vec3f> circlesLeft;
    std::vector<cv::Vec3f> circlesRight;

    // Draw the circles detected
    cv::Mat cannyOutputLeft;
    cv::Mat cannyOutputRight;
    std::vector<std::vector<cv::Point>> contoursLeft;
    std::vector<std::vector<cv::Point>> contoursRight;
    std::vector<cv::Vec4i> hierarchyLeft;
    std::vector<cv::Vec4i> hierarchyRight;

    // Apply the Hough Transform to find the circles
    cv::HoughCircles(maskLeft, circlesLeft, cv::HOUGH_GRADIENT, 2, maskLeft.rows / 16, 255, 30);
    cv::HoughCircles(maskRight, circlesRight, cv::HOUGH_GRADIENT, 2, maskRight.rows / 16, 255, 30);

    // Outline circle and centroid in left image
    if (circlesLeft.size() > 0)
    {
      cv::Point2f center(circlesLeft[0][0], circlesLeft[0][1]);
      int radius = circlesLeft[0][2];

      // Draw detected circle
      cv::circle(drawingLeft, center, radius, cv::Scalar(100, 100, 100), -1, 8, 0);

      // Set thresholds for contour detection
      int thresh = 100;
      int max_thresh = 255;
      cv::RNG rng(12345);

      // Detect edges using canny
      cv::Canny(drawingLeft, cannyOutputLeft, thresh, thresh * 2, 3);

      // Find contours
      cv::findContours(cannyOutputLeft, contoursLeft, hierarchyLeft, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

      /// Approximate contours to polygons + get bounding rects and circles
      std::vector<std::vector<cv::Point>> contours_poly(contoursLeft.size());
      std::vector<cv::Point2f>centerTwo(contoursLeft.size());
      std::vector<float>radiusTwo(contoursLeft.size());

      for (std::vector<std::vector<cv::Point>>::size_type i = 0; i < contoursLeft.size(); i++)
      {
        cv::approxPolyDP(cv::Mat(contoursLeft[i]), contours_poly[i], 3, true); // Finds polygon
        cv::minEnclosingCircle((cv::Mat)contours_poly[i], centerTwo[i], radiusTwo[i]); // Finds circle

        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

        // Draw circle
        cv::circle(drawingLeft, centerTwo[i], (int)radiusTwo[0], color, 2, 8, 0);

        // Draw circle center
        cv::circle(drawingLeft, centerTwo[i], 3, color, 2, 8, 0);
      }

      // Save circle position data
      this->PoseCentersLeft.push_back(centerTwo[0]);
      int numRows = this->DataTable->verticalHeader()->count();

      // Write to table in GUI
      this->DataTable->insertRow(numRows);
      this->DataTable->setItem(numRows, 0, new QTableWidgetItem("Circle"));
      this->DataTable->setItem(numRows, 1, new QTableWidgetItem(QString::number(centerTwo[0].x)));
      this->DataTable->setItem(numRows, 2, new QTableWidgetItem(QString::number(centerTwo[0].y)));
      this->DataTable->setItem(numRows, 3, new QTableWidgetItem(QString::number(radiusTwo[0])));
    }

    // If Hough Circles fails to find circle
    else if (circlesLeft.size() == 0)
    {
      int thresh = 100;
      int max_thresh = 255;
      cv::RNG rng(12345);

      cv::medianBlur(maskLeft, maskLeft, 3);

      // Detect edges using canny
      cv::Canny(maskLeft, cannyOutputLeft, thresh, thresh * 2, 3);

      // Find contours
      cv::findContours(cannyOutputLeft, contoursLeft, hierarchyLeft, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

      /// Approximate contours to polygons + get bounding rects and circles
      std::vector<std::vector<cv::Point>> contours_poly(contoursLeft.size());
      std::vector<cv::Rect> boundRect(contoursLeft.size());
      std::vector<cv::Point2f> center(contoursLeft.size());
      std::vector<float> radius(contoursLeft.size());

      for (std::vector<std::vector<cv::Point>>::size_type i = 0; i < contoursLeft.size(); i++)
      {
        cv::approxPolyDP(cv::Mat(contoursLeft[i]), contours_poly[i], 3, true); // Finds polygon
        boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));      // Finds rectangle
        cv::minEnclosingCircle((cv::Mat)contours_poly[i], center[i], radius[i]); // Finds circle
      }

      /// Draw circle
      cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

      // Draw circle
      cv::circle(maskLeft, center[0], (int)radius[0], color, 2, 8, 0);

      // Draw circle center
      cv::circle(maskLeft, center[0], 3, color, 2, 8, 0);

      // Save circle data for point-line calibration
      this->PoseCentersLeft.push_back(center[0]);

      // Write data to table in GUI
      int numRows = this->DataTable->verticalHeader()->count();
      this->DataTable->insertRow(numRows);
      this->DataTable->setItem(numRows, 0, new QTableWidgetItem("Circle"));
      this->DataTable->setItem(numRows, 1, new QTableWidgetItem(QString::number(center[0].x)));
      this->DataTable->setItem(numRows, 2, new QTableWidgetItem(QString::number(center[0].y)));
      this->DataTable->setItem(numRows, 3, new QTableWidgetItem(QString::number(radius[0])));
    }

    // Get left transform
    getTransform(this->ImageLeft, this->UndistortedLeft, this->IntrinsicLeft, this->UndistortedLeft, this->FinalImageLeft, this->PoseCentersLeft, this->XLeft, *this->PosMatrixLeft, this->OriginLeft, this->LeftIntrinsicParam, this->DNormalizedLeft, *this->Point2LineLeft, this->LeftMixer, this->TrackedFrame);

    // Outline circle and centroid in right image
    if (circlesRight.size() > 0)
    {
      cv::Point2f center(circlesRight[0][0], circlesRight[0][1]);
      int radius = circlesRight[0][2];

      // Draw detected circle
      cv::circle(drawingRight, center, radius, cv::Scalar(100, 100, 100), -1, 8, 0);

      // Show detected circle
      cv::namedWindow("Circle", cv::WINDOW_AUTOSIZE);
      cv::imshow("Circle", drawingRight);
      cv::waitKey(0);

      // Set thresholds for contour detection
      int thresh = 100;
      int max_thresh = 255;
      cv::RNG rng(12345);

      // Detect edges using canny
      cv::Canny(drawingRight, cannyOutputRight, thresh, thresh * 2, 3);

      // Find contours
      cv::findContours(cannyOutputRight, contoursRight, hierarchyRight, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

      /// Approximate contours to polygons + get bounding rects and circles
      std::vector<std::vector<cv::Point>> contours_poly(contoursRight.size());
      std::vector<cv::Point2f> centerTwo(contoursRight.size());
      std::vector<float> radiusTwo(contoursRight.size());

      for (std::vector<std::vector<cv::Point>>::size_type i = 0; i < contoursRight.size(); i++)
      {
        cv::approxPolyDP(cv::Mat(contoursRight[i]), contours_poly[i], 3, true); // Finds polygon
        cv::minEnclosingCircle((cv::Mat)contours_poly[i], centerTwo[i], radiusTwo[i]); // Finds circle

        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

        // Draw circle
        cv::circle(drawingRight, centerTwo[i], (int)radiusTwo[0], color, 2, 8, 0);

        // Draw circle center
        cv::circle(drawingRight, centerTwo[i], 3, color, 2, 8, 0);
      }

      // Show final result
      cv::namedWindow("Final Detection", cv::WINDOW_AUTOSIZE);
      cv::imshow("Final Detection", drawingRight);
      cv::waitKey(0);

      // Save circle position data
      this->PoseCentersRight.push_back(centerTwo[0]);
      int numRows = this->DataTable->verticalHeader()->count();

      // Write to table in GUI
      this->DataTable->insertRow(numRows);
      this->DataTable->setItem(numRows, 0, new QTableWidgetItem("Circle"));
      this->DataTable->setItem(numRows, 1, new QTableWidgetItem(QString::number(centerTwo[0].x)));
      this->DataTable->setItem(numRows, 2, new QTableWidgetItem(QString::number(centerTwo[0].y)));
      this->DataTable->setItem(numRows, 3, new QTableWidgetItem(QString::number(radiusTwo[0])));
    }

    // If Hough Circles fails to find circle
    else if (circlesRight.size() == 0)
    {
      int thresh = 100;
      int max_thresh = 255;
      cv::RNG rng(12345);

      cv::medianBlur(maskRight, maskRight, 3);

      // Detect edges using canny
      cv::Canny(maskRight, cannyOutputRight, thresh, thresh * 2, 3);

      // Find contours
      cv::findContours(cannyOutputRight, contoursRight, hierarchyRight, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

      /// Approximate contours to polygons + get bounding rects and circles
      std::vector<std::vector<cv::Point>> contours_poly(contoursRight.size());
      std::vector<cv::Rect> boundRect(contoursRight.size());
      std::vector<cv::Point2f> center(contoursRight.size());
      std::vector<float> radius(contoursRight.size());

      for (std::vector<std::vector<cv::Point>>::size_type i = 0; i < contoursRight.size(); i++)
      {
        cv::approxPolyDP(cv::Mat(contoursRight[i]), contours_poly[i], 3, true);   // Finds polygon
        boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));               // Finds rectangle
        cv::minEnclosingCircle((cv::Mat)contours_poly[i], center[i], radius[i]);  // Finds circle
      }

      /// Draw circle
      cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

      // Draw circle
      cv::circle(maskRight, center[0], (int)radius[0], color, 2, 8, 0);

      // Draw circle center
      cv::circle(maskRight, center[0], 3, color, 2, 8, 0);

      // Save circle data for point-line calibration
      this->PoseCentersRight.push_back(center[0]);

      // Write data to table in GUI
      int numRows = this->DataTable->verticalHeader()->count();
      this->DataTable->insertRow(numRows);
      this->DataTable->setItem(numRows, 0, new QTableWidgetItem("Circle"));
      this->DataTable->setItem(numRows, 1, new QTableWidgetItem(QString::number(center[0].x)));
      this->DataTable->setItem(numRows, 2, new QTableWidgetItem(QString::number(center[0].y)));
      this->DataTable->setItem(numRows, 3, new QTableWidgetItem(QString::number(radius[0])));
    }

    // Get right transform
    getTransform(this->ImageRight, this->UndistortedRight, this->IntrinsicRight, this->DistortionRight, this->FinalImageRight, this->PoseCentersRight, this->XRight, *this->PosMatrixRight, this->OriginRight, this->RightIntrinsicParam, this->DNormalizedRight, *this->Point2LineRight, this->RightMixer, this->TrackedFrame);
  }

  // Uncheck next Pose button
  this->TrackerWidget->nextPoseButton->setChecked(false);
}

// On manual select
void mainWidget::manualSelection(bool checked)
{
  if (checked)
  {
    // if GetTransform is clicked
    if (this->TrackerWidget->calibrationButton->isChecked() == true)
    {
      // Initialize Variables
      cv::Mat hsvLeft;
      cv::Mat hsvRight;
      cv::Mat thresholdLeft;
      cv::Mat thresholdRight;
      cv::Mat thresholdFinalLeft;
      cv::Mat thresholdFinalRight;

      this->LeftMixer->GetChannel()->GetTrackedFrame(this->LeftMixerFrame);
      this->RightMixer->GetChannel()->GetTrackedFrame(this->RightMixerFrame);

      vtkImageData* leftImage = this->LeftMixerFrame.GetImageData()->GetImage();
      vtkImageData* rightImage = this->RightMixerFrame.GetImageData()->GetImage();
      int leftDims[3];
      int rightDims[3];
      leftImage->GetDimensions(leftDims);
      rightImage->GetDimensions(rightDims);

      // Copy vtkImage to cv::Mat
      this->ImageLeft = cv::Mat(leftDims[1], leftDims[0], CV_8UC3, leftImage->GetScalarPointer(0, 0, 0));
      this->ImageRight = cv::Mat(rightDims[1], rightDims[0], CV_8UC3, rightImage->GetScalarPointer(0, 0, 0));

      this->UndistortedLeft = cv::Mat(leftDims[1], leftDims[0], CV_8UC3);
      this->UndistortedRight = cv::Mat(rightDims[1], rightDims[0], CV_8UC3);

      undistort(this->ImageLeft, this->UndistortedLeft, this->IntrinsicLeft, this->DistortionLeft);
      undistort(this->ImageRight, this->UndistortedRight, this->IntrinsicRight, this->DistortionRight);

      cv::flip(this->UndistortedLeft, this->UndistortedLeft, 0);
      cv::flip(this->UndistortedRight, this->UndistortedRight, 0);

      // Convert RGB image to HSV image
      cv::cvtColor(this->UndistortedLeft, hsvLeft, cv::COLOR_RGB2HSV);
      cv::cvtColor(this->UndistortedRight, hsvRight, cv::COLOR_RGB2HSV);

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
      cv::medianBlur(maskLeft, maskLeft, 5);
      cv::GaussianBlur(maskLeft, maskLeft, cv::Size(9, 9), 2, 2);

      // Select point for left camera
      cv::namedWindow("Select Left Point", cv::WINDOW_AUTOSIZE);

      cv::setMouseCallback("Select Left Point", onMouse, this);
      cv::imshow("Select Left Point", maskLeft);
      cv::waitKey(0);

      // Draw selected point
      cv::circle(maskLeft, this->MouseCapturePoint, 3, (0, 100, 100), -1, 8, 0);

      // Show user results
      cv::namedWindow("Manual Selection Left ", cv::WINDOW_AUTOSIZE);
      cv::imshow("Manual Selection Left", maskLeft);
      cv::waitKey(0);

      // Delete the last circle centroid coordinates and replace with new
      if (this->PoseCentersLeft.size() != 0)
      {
        this->PoseCentersLeft.pop_back();
        int numRows = this->DataTable->verticalHeader()->count();
        this->DataTable->removeRow(numRows);
        this->DataTable->removeRow(numRows - 1);
      }
      this->PoseCentersLeft.push_back(this->MouseCapturePoint);

      getTransform(this->ImageLeft, this->UndistortedLeft, this->IntrinsicLeft, this->UndistortedLeft, this->FinalImageLeft, this->PoseCentersLeft, this->XLeft, *this->PosMatrixLeft, this->OriginLeft, this->LeftIntrinsicParam, this->DNormalizedLeft, *this->Point2LineLeft, this->LeftMixer, this->TrackedFrame);

      // Select point for right camera
      cv::namedWindow("Select Right Point", cv::WINDOW_AUTOSIZE);

      cv::setMouseCallback("Select Right Point", onMouse, this);
      cv::imshow("Select Right Point", maskRight);
      cv::waitKey(0);

      // Draw selected point
      cv::circle(maskRight, this->MouseCapturePoint, 3, (0, 100, 100), -1, 8, 0);

      // Show user results
      cv::namedWindow("Manual Selection Right", cv::WINDOW_AUTOSIZE);
      cv::imshow("Manual Selection Right", maskRight);
      cv::waitKey(0);

      // Delete the last circle centroid coordinates and replace with new
      if (this->PoseCentersRight.size() != 0)
      {
        this->PoseCentersRight.pop_back();
        int numRows = this->DataTable->verticalHeader()->count();
        this->DataTable->removeRow(numRows);
        this->DataTable->removeRow(numRows - 1);
      }
      this->PoseCentersRight.push_back(this->MouseCapturePoint);

      getTransform(this->ImageRight, this->UndistortedRight, this->IntrinsicRight, this->DistortionRight, this->FinalImageRight, this->PoseCentersRight, this->XRight, *this->PosMatrixRight, this->OriginRight, this->RightIntrinsicParam, this->DNormalizedRight, *this->Point2LineRight, this->RightMixer, this->TrackedFrame);
    }
  }

  // Uncheck manual button
  this->TrackerWidget->manualButton->setChecked(false);
}

/*!
* Centralized place to create all vtk objects.
*/
void mainWidget::createVTKObjects()
{
  this->InputConfigFileName = "./config/config.xml";

  // Create output directory
  this->ResultsRootDir = "./Results/";
  if (vtksys::SystemTools::MakeDirectory(this->ResultsRootDir))
  {
    std::cout << "Output directory Created." << std::endl;
  }

  // Create calibration saves directory
  this->CalibrationRootDir = "./Calibration";
  if (vtksys::SystemTools::MakeDirectory(this->CalibrationRootDir))
  {
    std::cout << "Calibration directory Created." << std::endl;
  }

  this->TrackerDevice = NULL;

  // Read configuration
  if (PlusXmlUtils::ReadDeviceSetConfigurationFromFile(this->ConfigRootElement, this->InputConfigFileName.c_str()) == PLUS_FAIL)
  {
    LOG_ERROR("Unable to read configuration from file" << this->InputConfigFileName.c_str());
    exit(EXIT_FAILURE);
  }

  vtkPlusConfig::GetInstance()->SetDeviceSetConfigurationData(this->ConfigRootElement);

  // Read configuration file
  if (this->DataCollector->ReadConfiguration(this->ConfigRootElement) != PLUS_SUCCESS)
  {
    LOG_ERROR("Configuration incorrect for vtkPlusDataCollector.");
    exit(EXIT_FAILURE);
  }

  this->Camera2TrackerName.SetTransformName("CameraToTracker");
  this->Probe2TrackerName.SetTransformName("ProbeToTracker");

  isViewScene = false;
  this->IsTrackerInit = false;
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
  this->Renderer->SetBackground(.1, .2, .4);
  this->RenWindow->AddRenderer(this->Renderer);
  this->RenInt->SetRenderWindow(this->RenWindow);
  this->IsTrackerInit = false;
}

void mainWidget::setupARRendering()
{
  // Set intrinsic calibration
  this->LeftIntrinsicParam = echen::Matrix<double>(3, 3);
  this->LeftIntrinsicParam[0][0] = this->IntrinsicLeft.at<double>(0, 0) = 4.4308778509658629e+02;
  this->LeftIntrinsicParam[0][1] = this->IntrinsicLeft.at<double>(0, 1) = 0;
  this->LeftIntrinsicParam[0][2] = this->IntrinsicLeft.at<double>(0, 2) = 4.9137630327079307e+02;
  this->LeftIntrinsicParam[1][0] = this->IntrinsicLeft.at<double>(1, 0) = 0;
  this->LeftIntrinsicParam[1][1] = this->IntrinsicLeft.at<double>(1, 1) = 4.4088255151097923e+02;
  this->LeftIntrinsicParam[1][2] = this->IntrinsicLeft.at<double>(1, 2) = 4.7733731041974312e+02;
  this->LeftIntrinsicParam[2][0] = this->IntrinsicLeft.at<double>(2, 0) = 0;
  this->LeftIntrinsicParam[2][1] = this->IntrinsicLeft.at<double>(2, 1) = 0;
  this->LeftIntrinsicParam[2][2] = this->IntrinsicLeft.at<double>(2, 2) = 1;

  this->RightIntrinsicParam = echen::Matrix<double>(3, 3);
  this->RightIntrinsicParam[0][0] = this->IntrinsicRight.at<double>(0, 0) = 4.3642091715320345e+02;
  this->RightIntrinsicParam[0][1] = this->IntrinsicRight.at<double>(0, 1) = 0;
  this->RightIntrinsicParam[0][2] = this->IntrinsicRight.at<double>(0, 2) = 5.0206905222553593e+02;
  this->RightIntrinsicParam[1][0] = this->IntrinsicRight.at<double>(1, 0) = 0;
  this->RightIntrinsicParam[1][1] = this->IntrinsicRight.at<double>(1, 1) = 4.3449181164428381e+02;
  this->RightIntrinsicParam[1][2] = this->IntrinsicRight.at<double>(1, 2) = 4.5455132093848829e+02;
  this->RightIntrinsicParam[2][0] = this->IntrinsicRight.at<double>(2, 0) = 0;
  this->RightIntrinsicParam[2][1] = this->IntrinsicRight.at<double>(2, 1) = 0;
  this->RightIntrinsicParam[2][2] = this->IntrinsicRight.at<double>(2, 2) = 1;

  // Distortion Parameters
  this->DistortionLeft.at<double>(0, 0) = -3.4217579502885637e-01;
  this->DistortionLeft.at<double>(0, 1) = 1.5322858206254297e-01;
  this->DistortionLeft.at<double>(0, 2) = 7.0265221404534526e-04;
  this->DistortionLeft.at<double>(0, 3) = -1.0123352757817517e-03;

  this->DistortionRight.at<double>(0, 0) = 2.2901883203387270e-04;
  this->DistortionRight.at<double>(0, 1) = -2.8041302114925171e-02;
  this->DistortionRight.at<double>(0, 2) = -1.3213753434011032e-02;
  this->DistortionRight.at<double>(0, 3) = 1.2788565654151332e-02;
}

//
// a slot for updating tracker information and this->Rendererdering
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
  if (this->IsTrackerInit)     /*!< Make sure the tracker is initialized. */
  {
    this->Renderer->ResetCameraClippingRange();

    // Get updated tracking information
    this->TrackerChannel->GetTrackedFrame(this->TrackedFrame);

    bool isProbeMatrixValid(false);
    this->Repository->SetTransforms(this->TrackedFrame);

    // Check if probe is visible
    if (this->Repository->GetTransform(this->Probe2TrackerName, this->Probe2Tracker, &isProbeMatrixValid) == PLUS_SUCCESS && isProbeMatrixValid)
    {
      this->TrackerWidget->lightWidgets[0]->GreenOn();
    }
    else
    {
      this->TrackerWidget->lightWidgets[0]->RedOn();
    }

    // Check if HMD is visible
    bool isCamMatrixValid(false);
    if (this->Repository->GetTransform(this->Camera2TrackerName, this->Camera2Tracker, &isCamMatrixValid) == PLUS_SUCCESS && isCamMatrixValid)
    {
      this->TrackerWidget->lightWidgets[1]->GreenOn();
    }
    else
    {
      this->TrackerWidget->lightWidgets[1]->RedOn();
    }

    bool isLMatrixValid(false);
    this->Repository->SetTransforms(this->LeftMixerFrame);
    if (this->Repository->GetTransform(PlusTransformName("PointerTip", "LeftImagePlane"), this->Tip2ImageL, &isLMatrixValid) == PLUS_SUCCESS && isLMatrixValid)
    {
      this->PosMatrixLeft->PostMultiply();
      this->PosMatrixLeft->Identity();
      this->PosMatrixLeft->Concatenate(this->Tip2ImageL);
    }

    bool isRMatrixValid(false);
    this->Repository->SetTransforms(this->RightMixerFrame);
    if (this->Repository->GetTransform(PlusTransformName("PointerTip", "RightImagePlane"), this->Tip2ImageR, &isRMatrixValid) == PLUS_SUCCESS && isRMatrixValid)
    {
      this->PosMatrixRight->PostMultiply();
      this->PosMatrixRight->Identity();
      this->PosMatrixRight->Concatenate(this->Tip2ImageR);
    }

    if (isViewScene)
    {
      // Get position of circle centroid
      double posePositionLeft[3];
      double posePositionRight[3];
      this->PosMatrixLeft->GetPosition(posePositionLeft);
      this->PosMatrixRight->GetPosition(posePositionRight);

      // Project point
      double xPrimeLeft = posePositionLeft[0] / posePositionLeft[2];
      double yPrimeLeft = posePositionLeft[1] / posePositionLeft[2];
      double xPrimeRight = posePositionRight[0] / posePositionRight[2];
      double yPrimeRight = posePositionRight[1] / posePositionRight[2];

      double uLeft = (this->LeftIntrinsicParam[0][0] * xPrimeLeft) + this->LeftIntrinsicParam[0][2];
      double vLeft = (this->LeftIntrinsicParam[1][1] * yPrimeLeft) + this->LeftIntrinsicParam[1][2];

      double uRight = (this->RightIntrinsicParam[0][0] * xPrimeRight) + this->RightIntrinsicParam[0][2];
      double vRight = (this->RightIntrinsicParam[1][1] * yPrimeRight) + this->RightIntrinsicParam[1][2];

      std::vector<cv::Point2f> centerLeft(1);
      std::vector<cv::Point2f> centerRight(1);
      centerLeft[0].x = uLeft;
      centerLeft[0].y = vLeft;
      centerRight[0].x = uRight;
      centerRight[0].y = vRight;

      // Get left and right frames
      this->LeftMixer->GetChannel()->GetTrackedFrame(this->LeftMixerFrame);
      this->RightMixer->GetChannel()->GetTrackedFrame(this->RightMixerFrame);

      // Get left and right images
      vtkImageData* leftImage = this->LeftMixerFrame.GetImageData()->GetImage();
      vtkImageData* rightImage = this->RightMixerFrame.GetImageData()->GetImage();
      int leftDims[3];
      int rightDims[3];
      leftImage->GetDimensions(leftDims);
      rightImage->GetDimensions(rightDims);

      // Copy vtkImage to cv::Mat
      this->ImageLeft = cv::Mat(leftDims[1], leftDims[0], CV_8UC3, leftImage->GetScalarPointer(0, 0, 0));
      this->ImageRight = cv::Mat(rightDims[1], rightDims[0], CV_8UC3, rightImage->GetScalarPointer(0, 0, 0));

      // Undistort images
      this->UndistortedLeft = cv::Mat(leftDims[1], leftDims[0], CV_8UC3);
      this->UndistortedRight = cv::Mat(rightDims[1], rightDims[0], CV_8UC3);

      cv::undistort(this->ImageLeft, this->UndistortedLeft, this->IntrinsicLeft, this->DistortionLeft);
      cv::undistort(this->ImageRight, this->UndistortedRight, this->IntrinsicRight, this->DistortionRight);

      // Flip images to draw circles
      cv::flip(this->UndistortedLeft, this->FinalImageLeft, 0);
      cv::flip(this->UndistortedRight, this->FinalImageRight, 0);

      // Only draw pointer tip if both pointer and camera are visible
      if (this->Repository->GetTransform(this->Probe2TrackerName, this->Probe2Tracker, &isProbeMatrixValid) == PLUS_SUCCESS && isProbeMatrixValid &&
          this->Repository->GetTransform(this->Camera2TrackerName, this->Camera2Tracker, &isCamMatrixValid) == PLUS_SUCCESS && isCamMatrixValid)
      {
        // circle center
        cv::circle(this->FinalImageLeft, centerLeft[0], 3, (0, 100, 100), -1, 8, 0);
        cv::circle(this->FinalImageRight, centerRight[0], 3, (0, 100, 100), -1, 8, 0);

        // circle outline
        cv::circle(this->FinalImageLeft, centerLeft[0], 14, cv::Scalar(100, 100, 100), 3, 8, 0);
        cv::circle(this->FinalImageRight, centerRight[0], 14, cv::Scalar(100, 100, 100), 3, 8, 0);
      }

      // Flip back for vtk
      cv::flip(this->FinalImageLeft, this->FinalImageLeft, 0);
      cv::flip(this->FinalImageRight, this->FinalImageRight, 0);

      // Update
      this->ImageImportLeft->Modified();
      this->ImageImportLeft->Update();
      this->ImageImportRight->Modified();
      this->ImageImportRight->Update();

      this->TextureLeft->Modified();
      this->TextureRight->Modified();
      this->TextureLeft->Update();
      this->TextureRight->Update();

      VRWindow->Render();
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
    if (!this->IsTrackerInit)
    {
      // Get tracker
      if (this->DataCollector->GetDevice(this->TrackerDevice, "TrackerDevice") != PLUS_SUCCESS)
      {
        LOG_ERROR("Unable to locate the device with ID = \"TrackerDevice\". Check config file.");
        return;
      }

      // Get virtual mixer
      if (this->DataCollector->GetDevice(this->LeftMixerDevice, "LeftTrackedVideoDevice") != PLUS_SUCCESS)
      {
        LOG_ERROR("Unable to locate the device with ID = \"LeftTrackedVideoDevice\". Check config file.");
        return;
      }

      // Get virtual mixer
      if (this->DataCollector->GetDevice(this->RightMixerDevice, "RightTrackedVideoDevice") != PLUS_SUCCESS)
      {
        LOG_ERROR("Unable to locate the device with ID = \"RightTrackedVideoDevice\". Check config file.");
        return;
      }

      this->LeftMixer = dynamic_cast<vtkPlusVirtualMixer*>(this->LeftMixerDevice);
      this->RightMixer = dynamic_cast<vtkPlusVirtualMixer*>(this->RightMixerDevice);

      // Connect to devices
      if (this->DataCollector->Connect() != PLUS_SUCCESS)
      {
        std::cout << ".................... [FAILED]" << std::endl;
        LOG_ERROR("Failed to connect to devices!");
        return;
      }

      if (this->DataCollector->Start() != PLUS_SUCCESS)
      {
        LOG_ERROR("Failed to connect to devices!");
        return;
      }
      std::cout << ".................... [OK]" << std::endl;

      if (this->Repository->ReadConfiguration(this->ConfigRootElement) != PLUS_SUCCESS)
      {
        LOG_ERROR("Configuration incorrect for vtkPlusTransformRepository.");
        return;
      }

      if (this->DataCollector->GetChannel(this->TrackerChannel, "TrackerStream") != PLUS_SUCCESS)
      {
        LOG_ERROR("Unable to locate the channel with Id=\"TrackerStream\". Check config file.");
        return;
      }

      if (this->DataCollector->GetChannel(this->LeftVideoChannel, "LeftVideoStream") != PLUS_SUCCESS)
      {
        LOG_ERROR("Unable to locate the channel with Id=\"OvrVideoStream\". Check config file.");
        return;
      }

      if (this->DataCollector->GetChannel(this->RightVideoChannel, "RightVideoStream") != PLUS_SUCCESS)
      {
        LOG_ERROR("Unable to locate the channel with Id=\"OvrVideoStream\". Check config file.");
        return;
      }
      this->TrackerChannel->GetTrackedFrame(this->TrackedFrame);
      this->LeftMixer->GetChannel()->GetTrackedFrame(this->LeftMixerFrame);
      this->RightMixer->GetChannel()->GetTrackedFrame(this->RightMixerFrame);

      // Get left and right images
      vtkImageData* leftImage = this->LeftMixerFrame.GetImageData()->GetImage();
      vtkImageData* rightImage = this->RightMixerFrame.GetImageData()->GetImage();
      int leftDims[3];
      int rightDims[3];
      leftImage->GetDimensions(leftDims);
      rightImage->GetDimensions(rightDims);

      // Copy vtkImage to cv::Mat
      this->ImageLeft = cv::Mat(leftDims[1], leftDims[0], CV_8UC3, leftImage->GetScalarPointer(0, 0, 0));
      this->ImageRight = cv::Mat(rightDims[1], rightDims[0], CV_8UC3, rightImage->GetScalarPointer(0, 0, 0));

      // Undistort images
      cv::undistort(this->ImageLeft, this->FinalImageLeft, this->IntrinsicLeft, this->DistortionLeft);
      cv::undistort(this->ImageRight, this->FinalImageRight, this->IntrinsicRight, this->DistortionRight);

      // Convert image from opencv to vtk
      this->ImageImportLeft->SetDataSpacing(1, 1, 1);
      this->ImageImportLeft->SetDataOrigin(0, 0, 0);
      this->ImageImportLeft->SetWholeExtent(0, this->FinalImageLeft.size().width - 1, 0, this->FinalImageLeft.size().height - 1, 0, 0);
      this->ImageImportLeft->SetDataExtentToWholeExtent();
      this->ImageImportLeft->SetDataScalarTypeToUnsignedChar();
      this->ImageImportLeft->SetNumberOfScalarComponents(this->FinalImageLeft.channels());
      this->ImageImportLeft->SetImportVoidPointer(this->FinalImageLeft.data);
      this->ImageImportLeft->Modified();
      this->ImageImportLeft->Update();

      this->ImageImportRight->SetDataSpacing(1, 1, 1);
      this->ImageImportRight->SetDataOrigin(0, 0, 0);
      this->ImageImportRight->SetWholeExtent(0, this->FinalImageRight.size().width - 1, 0, this->FinalImageRight.size().height - 1, 0, 0);
      this->ImageImportRight->SetDataExtentToWholeExtent();
      this->ImageImportRight->SetDataScalarTypeToUnsignedChar();
      this->ImageImportRight->SetNumberOfScalarComponents(this->FinalImageRight.channels());
      this->ImageImportRight->SetImportVoidPointer(this->FinalImageRight.data);
      this->ImageImportRight->Modified();
      this->ImageImportRight->Update();

      this->TextureLeft->SetInputConnection(this->ImageImportLeft->GetOutputPort());
      this->TextureRight->SetInputConnection(this->ImageImportRight->GetOutputPort());

      VRWindow = dynamic_cast<vtkOpenVRRenderWindow*>(this->RenWindow.Get());
      //VRWindow->SetTexturedBackground(true);
      VRWindow->AddRenderer(this->Renderer);
      //VRWindow->SetLeftBackgroundTexture(this->TextureLeft);
      //VRWindow->SetRightBackgroundTexture(this->TextureRight);

      VRWindow->Render();

      bool isLMatrixValid(false);
      this->Repository->SetTransforms(this->LeftMixerFrame);
      if (this->Repository->GetTransform(PlusTransformName("PointerTip", "LeftImagePlane"), this->Tip2ImageL, &isLMatrixValid) == PLUS_SUCCESS && isLMatrixValid)
      {
        this->PosMatrixLeft->PostMultiply();
        this->PosMatrixLeft->Identity();
        this->PosMatrixLeft->Concatenate(this->Tip2ImageL);
      }

      bool isRMatrixValid(false);
      this->Repository->SetTransforms(this->RightMixerFrame);
      if (this->Repository->GetTransform(PlusTransformName("PointerTip", "RightImagePlane"), this->Tip2ImageR, &isRMatrixValid) == PLUS_SUCCESS && isRMatrixValid)
      {
        this->PosMatrixRight->PostMultiply();
        this->PosMatrixRight->Identity();
        this->PosMatrixRight->Concatenate(this->Tip2ImageR);
      }

      this->IsTrackerInit = true;
      this->TrackerWidget->viewSceneButton->setEnabled(true);
      this->TrackerWidget->calibrationButton->setEnabled(true);
      this->TrackerWidget->collectPoses->setEnabled(true);
    }

    /*!
    * If tracker is initialized, start tracking.
    */
    if (this->IsTrackerInit)
    {
      statusBar()->showMessage(tr("Tracking started."), 5000);

      checkToolPorts();

      //this->TrackerTimer->start( 0 ); /*!< Update the tracker as quickly as we can. */
      this->TrackerTimer->start(35); /*!< The vtk pipeline takes about 15msec, so this is roughly 20 FPS. */
    }
  }
  else
  {
    /*! Bottom is un-toggled. */
    if (this->IsTrackerInit)
    {
      this->TrackerTimer->stop();
      this->DataCollector->Stop();

      /*!
      * Turn all the light widgets to blue.
      */
      for (int i = 0; i < 4; i++)
      {
        this->LightWidgets[i]->BlueOn();
      }
      statusBar()->showMessage(tr("stopping tracker"), 5000);

      this->TrackerWidget->calibrationButton->setDisabled(true);
      this->TrackerWidget->nextPoseButton->setDisabled(true);
      this->TrackerWidget->manualButton->setDisabled(true);
      this->TrackerWidget->collectPoses->setDisabled(true);
      this->TrackerWidget->viewSceneButton->setDisabled(true);
    }
  }
}

void mainWidget::createActions()
{
  this->QuitAction = new QAction(tr("&Quit"), this);
  this->QuitAction->setShortcuts(QKeySequence::Quit);
  this->QuitAction->setStatusTip(tr("Quit the application"));
  connect(this->QuitAction, SIGNAL(triggered()), this, SLOT(close()));

  this->AboutAction = new QAction(tr("&About"), this);
  this->AboutAction->setStatusTip(tr("About this application"));
  connect(this->AboutAction, SIGNAL(triggered()), this, SLOT(about()));

  this->ControlAction = new QAction(tr("&Tracker controls"), this);
  this->ControlAction->setStatusTip(tr(""));
  connect(this->ControlAction, SIGNAL(triggered()), this, SLOT(createControlDock()));

  this->AboutRobartsAction = new QAction(tr("About &Robarts"), this);
  this->AboutRobartsAction->setStatusTip(tr("About Robarts Research Institute"));
  connect(this->AboutRobartsAction, SIGNAL(triggered()), this, SLOT(aboutRobarts()));

}

void mainWidget::createMenus()
{
  this->FileMenu = menuBar()->addMenu(tr("&File"));
  this->FileMenu->addSeparator();
  this->FileMenu->addAction(this->QuitAction);

  this->CalibMenu = menuBar()->addMenu(tr("&Calibration"));

  this->ControlMenu = menuBar()->addMenu(tr("&Control"));
  this->ControlMenu->addAction(this->ControlAction);

  this->HelpMenu = menuBar()->addMenu(tr("&Help"));
  this->HelpMenu->addSeparator();
  this->HelpMenu->addAction(this->AboutAction);
  this->HelpMenu->addAction(this->AboutRobartsAction);
}

void mainWidget::createToolInformation()
{
  /*!
  * Create a dock widget for the tool information
  */
  this->ToolInfo = new QDockWidget(tr("Tool Information"), this);
  this->ToolInfo->setAllowedAreas(Qt::RightDockWidgetArea);
  this->ToolInfo->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable);
  addDockWidget(Qt::RightDockWidgetArea, this->ToolInfo);
  this->ToolInfo->setMinimumWidth(406);

  /*!
  * Setup layout and frame
  */
  QFrame* mainFrame = new QFrame;
  mainFrame->setFrameStyle(QFrame::WinPanel | QFrame::Sunken);
  mainFrame->setLineWidth(2);

  QVBoxLayout* vlayout = new QVBoxLayout;
  vlayout->setMargin(0);
  vlayout->setSpacing(10);
  vlayout->setAlignment(Qt::AlignTop);
  mainFrame->setLayout(vlayout);

  /*!
  * Create table to hold tool information
  */
  this->DataTable = new QTableWidget();
  this->DataTable->setRowCount(3);
  this->DataTable->setColumnCount(4);
  this->DataTable->setItem(0, 0, new QTableWidgetItem("Tracked Object"));

  this->DataTable->setItem(0, 1, new QTableWidgetItem("x"));
  this->DataTable->setItem(0, 2, new QTableWidgetItem("y"));
  this->DataTable->setItem(0, 3, new QTableWidgetItem("z"));

  this->DataTable->setShowGrid(true);
  this->DataTable->horizontalHeader()->hide();
  this->DataTable->verticalHeader()->hide();

  this->ToolInfo->setWidget(mainFrame);
  vlayout->addWidget(this->DataTable);
}

/*!
* Create a dock window for controlling NDI tracker
*/
void mainWidget::createControlDock()
{
  if (this->ControlDock)
  {
    this->ControlDock->show();
  }
  else
  {
    /*!
    * create a timer here for the tracker
    */
    this->TrackerTimer = new QTimer(this);
    connect(this->TrackerTimer, SIGNAL(timeout()), this, SLOT(updateTrackerInfo()));

    this->ControlDock = new QDockWidget(tr("Tracker Control"), this);
    this->ControlDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    this->ControlDock->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable);
    addDockWidget(Qt::LeftDockWidgetArea, this->ControlDock);
    this->ControlDock->setMinimumWidth(180);

    QFrame* mainFrame = new QFrame;
    mainFrame->setFrameStyle(QFrame::WinPanel | QFrame::Sunken);
    mainFrame->setLineWidth(2);

    QGridLayout* controlsLayout = new QGridLayout;
    controlsLayout->setMargin(0);
    controlsLayout->setSpacing(10);
    controlsLayout->setAlignment(Qt::AlignTop);
    mainFrame->setLayout(controlsLayout);

    this->ControlDock->setWidget(mainFrame);

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

    // Add HSV Controls Widget
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
    this->TrackerWidget = new eccTrackerWidget();
    controlsLayout->addWidget(this->TrackerWidget);

    this->TrackerWidget->setLabel(0, tr("(Port 0)"));
    this->TrackerWidget->setLabel(1, tr("(Port 1)"));
    this->TrackerWidget->setLabel(2, tr("(Port 2)"));
    this->TrackerWidget->setLabel(3, tr("(Port 3)"));

    connect(this->TrackerWidget->trackerButton, SIGNAL(toggled(bool)), this, SLOT(startTrackerSlot(bool)));

    connect(this->TrackerWidget->calibrationButton, SIGNAL(toggled(bool)), this, SLOT(startCalibration(bool)));
    this->TrackerWidget->calibrationButton->setDisabled(true);

    connect(this->TrackerWidget->nextPoseButton, SIGNAL(toggled(bool)), this, SLOT(nextPose(bool)));
    this->TrackerWidget->nextPoseButton->setDisabled(true);

    connect(this->TrackerWidget->manualButton, SIGNAL(toggled(bool)), this, SLOT(manualSelection(bool)));
    this->TrackerWidget->manualButton->setDisabled(true);

    connect(this->TrackerWidget->collectPoses, SIGNAL(clicked()), this,            SLOT(collectPose()));
    this->TrackerWidget->collectPoses->setDisabled(true);

    connect(this->TrackerWidget->viewSceneButton, SIGNAL(toggled(bool)), this, SLOT(viewScene(bool)));
    this->TrackerWidget->viewSceneButton->setDisabled(true);

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
    this->StylusTipRMS = new QLineEdit;
    this->StylusTipRMS->setText(tempString);
    vl->addWidget(rms);
    vl->addWidget(this->StylusTipRMS);
    vl->setAlignment(Qt::AlignTop);
  }
}

bool mainWidget::getTransform(cv::Mat& image,
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
                              PlusTrackedFrame& frame
                             )
{
  if (poseCenters.size() <= 15)
  {
    double pos[3];
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->PostMultiply();

    bool isMatrixValid(false);
    this->Repository->GetTransform(PlusTransformName("PointerTip", "Camera"), this->Tip2Camera, &isMatrixValid);

    if (!isMatrixValid)
    {
      return false;
    }

    transform->Identity();
    transform->Concatenate(this->Tip2Camera);
    transform->Modified();

    transform->GetPosition(pos);

    transform->Modified();

    double position[3];
    transform->GetPosition(position);
    x[0][poseCenters.size() - 1] = position[0];
    x[1][poseCenters.size() - 1] = position[1];
    x[2][poseCenters.size() - 1] = position[2];

    // Origin matrix - always 0,0,0
    origin[0][poseCenters.size() - 1] = 0;
    origin[1][poseCenters.size() - 1] = 0;
    origin[2][poseCenters.size() - 1] = 0;

    echen::Matrix<double> pixel(3, 1);
    pixel[0][0] = poseCenters[poseCenters.size() - 1].x;
    pixel[1][0] = poseCenters[poseCenters.size() - 1].y;
    pixel[2][0] = 1;

    // Find the inverse of the camera intrinsic param matrix
    echen::Matrix<double> intrinsicInv(3, 3);
    echen::invm3x3(intrinParam, intrinsicInv);

    // Calculate D matrix by multiplying the inverse of the
    // intrinsic param matrix by the pixel matrix
    echen::Matrix<double> dMatrix(3, 1);
    dMatrix = intrinsicInv * pixel;

    // Multiply by inverse of distortion coefficients
    std::vector<cv::Point2d> pointCoords(1);
    std::vector<cv::Point2d> undistortedPoints;
    pointCoords[0].x = poseCenters[poseCenters.size() - 1].x;
    pointCoords[0].y = poseCenters[poseCenters.size() - 1].y;

    // Normalize the D matrix
    double sum1;
    sum1 = (dMatrix[0][0] * dMatrix[0][0]) + (dMatrix[1][0] * dMatrix[1][0]) + (dMatrix[2][0] * dMatrix[2][0]);
    dNormalized[0][poseCenters.size() - 1] = dMatrix[0][0] / sqrt(sum1);
    dNormalized[1][poseCenters.size() - 1] = dMatrix[1][0] / sqrt(sum1);
    dNormalized[2][poseCenters.size() - 1] = dMatrix[2][0] / sqrt(sum1);

    // Send data to table
    int numRows = this->DataTable->verticalHeader()->count();
    this->DataTable->insertRow(numRows);
    this->DataTable->setItem(numRows, 0, new QTableWidgetItem("X"));
    this->DataTable->setItem(numRows, 1, new QTableWidgetItem(QString::number(x[0][poseCenters.size() - 1])));
    this->DataTable->setItem(numRows, 2, new QTableWidgetItem(QString::number(x[1][poseCenters.size() - 1])));
    this->DataTable->setItem(numRows, 3, new QTableWidgetItem(QString::number(x[2][poseCenters.size() - 1])));

    this->DataTable->insertRow(numRows + 1);
    this->DataTable->setItem(numRows + 1, 0, new QTableWidgetItem("D"));
    this->DataTable->setItem(numRows + 1, 1, new QTableWidgetItem(QString::number(dNormalized[0][poseCenters.size() - 1])));
    this->DataTable->setItem(numRows + 1, 2, new QTableWidgetItem(QString::number(dNormalized[1][poseCenters.size() - 1])));
    this->DataTable->setItem(numRows + 1, 3, new QTableWidgetItem(QString::number(dNormalized[2][poseCenters.size() - 1])));
  }

  if (poseCenters.size() == 15)
  {
    echen::Matrix<double> rotation;
    echen::Matrix<double> translation;
    double tol = 1e-9;
    double error = 0;

    // Calculate point to line
    echen::p2l(x, origin, dNormalized, tol, rotation, translation, error);

    pointToLine.Identity();
    pointToLine.SetElement(0, 0, rotation[0][0]);
    pointToLine.SetElement(0, 1, rotation[0][1]);
    pointToLine.SetElement(0, 2, rotation[0][2]);
    pointToLine.SetElement(1, 0, rotation[1][0]);
    pointToLine.SetElement(1, 1, rotation[1][1]);
    pointToLine.SetElement(1, 2, rotation[1][2]);
    pointToLine.SetElement(2, 0, rotation[2][0]);
    pointToLine.SetElement(2, 1, rotation[2][1]);
    pointToLine.SetElement(2, 2, rotation[2][2]);
    pointToLine.SetElement(0, 3, translation[0][0]);
    pointToLine.SetElement(1, 3, translation[1][0]);
    pointToLine.SetElement(2, 3, translation[2][0]);

    bool isMatrixValid(false);
    this->Repository->GetTransform(PlusTransformName("PointerTip", "Camera"), this->Tip2Camera, &isMatrixValid);

    posMatrix.PostMultiply();
    posMatrix.Identity();
    posMatrix.Concatenate(this->Tip2Camera);
    posMatrix.Concatenate(&pointToLine);

    double posePosition[3];
    echen::Matrix<double> posePositionM(3, 1);
    echen::Matrix<double> result(3, 1);
    posMatrix.GetPosition(posePosition);

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

    std::vector<cv::Point2d> projectedPoints;
    double xPrime = objectPoints.at<double>(0, 0) / objectPoints.at<double>(0, 2);
    double yPrime = objectPoints.at<double>(0, 1) / objectPoints.at<double>(0, 2);

    double u = (intrinParam[0][0] * xPrime) + intrinParam[0][2];
    double v = (intrinParam[1][1] * yPrime) + intrinParam[1][2];

    std::vector<cv::Point2f> center(1);
    center[0].x = u;
    center[0].y = v;

    // Get left frame
    mixer->GetChannel()->GetTrackedFrame(frame);

    // Get left image
    vtkImageData* vtkImage = frame.GetImageData()->GetImage();
    int dims[3];
    vtkImage->GetDimensions(dims);

    // Copy vtkImage to cv::Mat
    image = cv::Mat(dims[1], dims[0], CV_8UC3, vtkImage->GetScalarPointer(0, 0, 0));

    // Undistort image
    undistorted = cv::Mat(dims[1], dims[0], CV_8UC3);
    undistort(image, undistorted, intrinParamCv, distortion);

    // Flip image to draw circle
    cv::flip(undistorted, finalImage, 0);

    // circle center
    cv::circle(finalImage, center[0], 3, (0, 100, 100), -1, 8, 0);

    // circle outline
    cv::circle(finalImage, center[0], 14, cv::Scalar(100, 100, 100), 3, 8, 0);

    // Flip back for vtk
    cv::flip(finalImage, finalImage, 0);
  }

  return true;
}

void mainWidget::startCalibration(bool checked)
{
  if (checked)
  {
    if (this->PoseCentersLeft.size() != 0)
    {
      this->PoseCentersLeft.resize(0);
    }
    if (this->PoseCentersRight.size() != 0)
    {
      this->PoseCentersRight.resize(0);
    }

    // Refresh table
    for (int i = this->DataTable->verticalHeader()->count(); i >= 0; i--)
    {
      this->DataTable->removeRow(i);
    }

    // Initialize matrices
    this->XLeft.newsize(3, 15);
    this->XRight.newsize(3, 15);
    this->OriginLeft.newsize(3, 15);
    this->OriginRight.newsize(3, 15);
    this->DNormalizedLeft.newsize(3, 15);
    this->DNormalizedRight.newsize(3, 15);

    this->TrackerWidget->nextPoseButton->setEnabled(true);
    this->TrackerWidget->manualButton->setEnabled(true);
  }
  else
  {
    this->TrackerWidget->nextPoseButton->setDisabled(true);
    this->TrackerWidget->manualButton->setDisabled(true);
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
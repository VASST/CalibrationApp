# Calibration-App
Calibration App for camera to tracking device. 

Runtime in debug is slow, substantially improves in release.

This application requires the following:
- OpenVR: https://github.com/ValveSoftware/openvr.git
- SDL2
- Plus - Build with NDI, Ovrvision, OpenIGTLink
- OpenCV - 3.0 and up
- VTK - Use my fork of vtk with branch "im/vtk" as master branch of vtk 7.1 fails to send video pass through to the Oculus, build with RenderingOpenVR module and RenderingParallel module github.com/imorgan1618/VTK.git
- Qt5
- Option: Build Robarts VTK from https://github.com/imorgan1618/RobartsVTKBuild.git to build Plus, OpenCV and VTK

How to use:
- Select: Control->Tracker Controls
- Select: "Start Tracker" to start tracking (Note: must be connected to NDI Spectra/Vicra)
- Select: "View Scene" to see a live stream from the Logitech.
- Select: "View Centroid" to view the Centroid Position in the view video stream.
- Select: "View Model" To view model overlay, NOTE: This is not accurate
- Select: "Pivot" to complete pivot calibration on the tool, select again once pivot is complete.
- Select: "Start Calibration" to start the point-line calibration procedure
- Select: "Next Pose" to capture the next position during the point-line calibration (until 15 poses have been acquired).
- Select: "Manual Selection" if the previous automatic centroid detection was unnaceptable, and select the point manually.
- Select: "Collect Poses" to save the current frame and transform to a file.
- Alter lower and upper HSV values as needed for automatic segmentation


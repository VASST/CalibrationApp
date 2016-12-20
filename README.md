# Calibration-App
Calibration App for camera to tracking device. 

Runtime in debug is slow, substantially improves in release.

This application requires the following:
- OpenVR
- SDL2
- Plus - Build with NDI and Ovrvision
- Vtk - Use my fork of vtk with branch "im/vtk" as master branch of vtk 7.1 fails to send video pass through to the Oculus, build with RenderingOpenVR module and RenderingParallel module github.com/imorgan1618/VTK.git
- Qt5

How to use:
- Select: Control->Tracker Controls
- Select: "Start Tracker" to start tracking (Note: must be connected to NDI Spectra/Vicra)
- Select: "View Scene" to see a live stream from the Logitech.
- Select: "View Centroid" to view the Centroid Position in the view video stream.-
 Select: "Use Calibration" to use the current point-line calibration once calibration is complete, else the pre-calculated point-line calibration will be used.
 - Select: "Pivot" to complete pivot calibration on the tool, select again once pivot is complete.
 - Select: "Start Calibration" to start the point-line calibration procedure
 - Select: "Next Pose" to capture the next position during the point-line calibration (until 15 poses have been acquired).
 - Select: "Manual Selection" if the previous automatic centroid detection was unnaceptable, and select the point manually.
 - Select: "Collect Poses" to save the current frame and transform to a file.
 - Alter lower and upper HSV values as needed for automatic segmentation

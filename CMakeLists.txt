PROJECT( VRApp )

CMAKE_MINIMUM_REQUIRED( VERSION 3.0 )

IF ( COMMAND CMAKE_POLICY )
  CMAKE_POLICY( SET CMP0003 NEW )
ENDIF( COMMAND CMAKE_POLICY )

set(CMAKE_INCLUDE_CURRENT_DIR ON)

FIND_PACKAGE( Qt5 REQUIRED COMPONENTS Widgets OpenGL Core Gui )
SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)

FIND_PACKAGE( vtk REQUIRED )
INCLUDE( ${VTK_USE_FILE} )

FIND_PACKAGE( PlusLib REQUIRED NO_MODULE)

FIND_PACKAGE( OpenCV REQUIRED NO_MODULE)

FIND_PACKAGE( OvrvisionPro REQUIRED)
FIND_PACKAGE( OpenVR REQUIRED )
FIND_PACKAGE( SDL REQUIRED)


ADD_EXECUTABLE( VRApp
  main.cpp
  mainWidget.cpp
  mainWidget.h
  qlightwidget.cpp
  qlightwidget.h
  eccTrackerWidget.cpp
  eccTrackerWidget.h
  jacobi.h
  lu.h
  mathUtils.cpp
  mathUtils.h
  matrix.h
  svd.h
)

target_include_directories( VRApp PRIVATE ${SDL_INCLUDE_DIR} ${OPENVR_INCLUDE_DIRS})

TARGET_LINK_LIBRARIES( VRApp
  Qt5::Widgets #Qt
  Qt5::OpenGL #Qt
  Qt5::Core
  Qt5::Gui
  OvrvisionPro
  vtkndicapi
  vtkRenderingOpenVR
  vtkRenderingFreeType
  vtkIOImage
  vtkFiltersTexture
  vtkFiltersSources
  vtkGUISupportQt
  vtkRenderingContextOpenGL2
  vtkParallelCore
  vtkCommonCore
  vtkCommonDataModel
  vtkCommonMath
  vtkCommonMisc
  vtkCommonSystem
  vtkCommonTransforms
  vtkCommonColor
  vtkImagingCore
  vtkImagingHybrid
  vtkInteractionStyle
  vtkIOLegacy
  vtkRenderingAnnotation
  vtkRenderingContext2D
  vtkRenderingCore
  vtkRenderingParallel
  vtkRenderingGL2PSOpenGL2
  vtkRenderingLabel
  vtkRenderingOpenGL2
  vtkRenderingVolume
  vtkRenderingQt
  vtkRenderingImage
  vtkRenderingVolumeOpenGL2
  vtksys
  vtkglew
  vtkRenderingGL2PS${VTK_RENDERING_BACKEND}
  ${OpenCV_LIBS}
  vtkPlusCalibration
  vtkPlusCommon
  vtkPlusDataCollection
  vtkPlusHaptics
  vtkPlusImageProcessing
  vtkPlusOpenIGTLink
  vtkPlusRendering
  vtkPlusServer
  vtkPlusUsSimulator
  vtkPlusVolumeReconstruction
  OvrvisionPro
)
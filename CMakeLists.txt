PROJECT( CameraCalibration )

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


ADD_EXECUTABLE( ${PROJECT_NAME}
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

target_include_directories( ${PROJECT_NAME} PRIVATE ${SDL_INCLUDE_DIR})

TARGET_LINK_LIBRARIES( ${PROJECT_NAME}
  Qt5::Widgets #Qt
  Qt5::OpenGL #Qt
  Qt5::Core
  Qt5::Gui
  vtkRenderingOpenVR
  vtkRenderingGL2PS${VTK_RENDERING_BACKEND}
  ${OpenCV_LIBS}
  vtkPlusDataCollection
)
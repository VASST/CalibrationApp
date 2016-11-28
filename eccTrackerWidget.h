#ifndef __ECCTRACKERWIDGET_H__
#define __ECCTRACKERWIDGET_H__

#include <QFrame>

// C++ includes
#include <vector>

// local includes
#include "qlightwidget.h"

// forward declaration
class QLabel;
class QPushButton;
class QString;

class eccTrackerWidget : public QFrame {
  
  Q_OBJECT

public:

  eccTrackerWidget( QWidget *parent = 0 );
  ~eccTrackerWidget() {};

public: // public functions
  /*!
   * set the label
   */
  void setLabel( int idx, QString s );

public:
  std::vector< QLightWidget *>                    lightWidgets;
  std::vector< QLabel *>                          labelWidgets;
  QPushButton                                     *trackerButton;
  QPushButton                                     *calibrationButton;
  QPushButton                                     *nextPoseButton;
  QPushButton                                     *manualButton;
  QPushButton                                     *viewSceneButton;
  QPushButton									  *collectPoses;
  };
#endif //  __TRACKERWIDGET_H__
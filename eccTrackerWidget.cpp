// local includes
#include "eccTrackerWidget.h"


// qt includes
#include <QLabel>
#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>

eccTrackerWidget::eccTrackerWidget( QWidget *parent )
: QFrame( parent )
  {
  QVBoxLayout *vbox = new QVBoxLayout;

  trackerButton = new QPushButton;
  trackerButton->setCheckable( true );
  trackerButton->setText( "Start Tracker" );
  vbox->addWidget(trackerButton);

  calibrationButton = new QPushButton;
  calibrationButton->setCheckable(true);
  calibrationButton->setText("Start Calibration");
  vbox->addWidget(calibrationButton);

  nextPoseButton = new QPushButton;
  nextPoseButton->setCheckable(true);
  nextPoseButton->setText("Next Pose");
  vbox->addWidget(nextPoseButton);

  manualButton = new QPushButton;
  manualButton->setCheckable(true);
  manualButton->setText("Manual Selection");
  vbox->addWidget(manualButton);

  viewSceneButton = new QPushButton;
  viewSceneButton->setCheckable(true);
  viewSceneButton->setText("View Scene");
  vbox->addWidget(viewSceneButton);

  QHBoxLayout *hbox = new QHBoxLayout;

  QLabel *label = new QLabel;
  label->setText( tr( "Stylus (port 0)" ) );
  label->setAlignment( Qt::AlignRight | Qt::AlignVCenter );
  hbox->addWidget( label );
  QLightWidget *lw = new QLightWidget;
  hbox->addWidget( lw );
  lw->BlueOn();
  lightWidgets.push_back( lw );
  labelWidgets.push_back( label );
  vbox->addLayout( hbox );

  hbox = new QHBoxLayout;
  label = new QLabel;
  label->setText( tr( "Stylus (port 1)" ) );
  label->setAlignment( Qt::AlignRight | Qt::AlignVCenter );
  hbox->addWidget( label );
  lw = new QLightWidget;
  hbox->addWidget( lw );
  lw->BlueOn();
  lightWidgets.push_back( lw );
  labelWidgets.push_back( label );
  vbox->addLayout( hbox );
  
  hbox = new QHBoxLayout;
  label = new QLabel;
  label->setText( tr( "Stylus (port 2)" ) );
  label->setAlignment( Qt::AlignRight | Qt::AlignVCenter );
  hbox->addWidget( label );
  lw = new QLightWidget;
  hbox->addWidget( lw );
  lw->BlueOn();
  lightWidgets.push_back( lw );
  labelWidgets.push_back( label );
  vbox->addLayout( hbox );

  
  hbox = new QHBoxLayout;
  label = new QLabel;
  label->setText( tr( "Stylus (port 3)" ) );
  label->setAlignment( Qt::AlignRight | Qt::AlignVCenter );
  hbox->addWidget( label );
  lw = new QLightWidget;
  hbox->addWidget( lw );
  lw->BlueOn();
  lightWidgets.push_back( lw );
  labelWidgets.push_back( label );
  vbox->addLayout( hbox );


  this->setLayout( vbox );
  }


void eccTrackerWidget::setLabel( int idx, QString s )
  {
  if ( idx < labelWidgets.size() )
    {
    labelWidgets[idx]->setText( s );
    }
  }
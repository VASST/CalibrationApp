/*=========================================================================

Program:   Phantom-Less Calibration GUI
Module:    $RCSfile: qlightwidget.cpp,v $
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

/*
 * shamelessly copied from Usaf
 */

#include "qlightwidget.h"
#include <QColor>
#include <QPainter>


//////////////////////////////////////////////////////////////////////////
QLightWidget::QLightWidget(QWidget *parent)
: QWidget(parent)
  {
  mColor=Qt::red;
  setFixedSize(20,20);
  }

//////////////////////////////////////////////////////////////////////////
void QLightWidget::paintEvent(QPaintEvent *e)
  {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  painter.setPen(Qt::NoPen);
  painter.setBrush(Qt::darkGray);
  painter.drawEllipse(0, 0, width(), height());

  mColor.setAlpha(255);

  painter.setPen(Qt::NoPen);
  painter.setBrush(QBrush(mColor, Qt::Dense4Pattern));
  painter.drawEllipse(1, 1, width()-1, height()-1);
  /*
  QRadialGradient g(-3, -3, 10);
  g.setColorAt(0, Qt::yellow);
  g.setColorAt(1, Qt::darkYellow);

  painter.setBrush(g);
  painter.setPen(QPen(Qt::black, 0));
  painter.drawEllipse(0, 0, width(), height());
  */
  }

//////////////////////////////////////////////////////////////////////////
void QLightWidget::GreenOn()
  {
  mColor=Qt::green;
  update();
  }

//////////////////////////////////////////////////////////////////////////
void QLightWidget::YellowOn()
  {
  mColor=Qt::yellow;
  update();
  }

//////////////////////////////////////////////////////////////////////////
void QLightWidget::RedOn()
  {
  mColor=Qt::red;
  update();
  }

//////////////////////////////////////////////////////////////////////////
void QLightWidget::BlueOn()
  {
  mColor=Qt::blue;
  update();
  }

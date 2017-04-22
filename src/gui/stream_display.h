#ifndef _GUI_STREAM_DISPLAY_H_
#define _GUI_STREAM_DISPLAY_H_

#include <QtWidgets/QWidget>

namespace gui {

class StreamDisplay : public QWidget {
  Q_OBJECT
 public:
  StreamDisplay(QWidget* parent = nullptr);

 protected:
  void paintEvent(QPaintEvent* event);

 private:
};

}  // namespace gui

#endif  // _GUI_STREAM_DISPLAY_H_
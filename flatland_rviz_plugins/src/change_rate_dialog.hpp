#ifndef FLATLAND_RVIZ_PLUGINS__CHANGE_RATE_DIALOG_HPP_
#define FLATLAND_RVIZ_PLUGINS__CHANGE_RATE_DIALOG_HPP_

#include <QCursor>
#include <QDialog>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>
#include <QWidget>
#include <rviz_common/display_context.hpp>

#include "flatland_rviz_plugins/change_rate_tool.hpp"

namespace flatland_rviz_plugins {

class ChangeRateDialog : public QDialog {
 public:
  ChangeRateDialog(QWidget *parent, rviz_common::DisplayContext *context,
                   ChangeRateTool *tool);

 private:
  rviz_common::DisplayContext *context_;

  QLineEdit *r_edit;  // name lineEdit widget

  ChangeRateTool *tool_;

 public Q_SLOTS:
  /**
   * @name                CancelButtonClicked
   * @brief               Cancel button was clicked, dismiss dialog
   */
  void CancelButtonClicked();
  /**
   * @name                OkButtonClicked
   * @brief               Ok button was clicked, begin placement
   */
  void OkButtonClicked();
};

}  // namespace flatland_rviz_plugins

#endif  // FLATLAND_RVIZ_PLUGINS__CHANGE_RATE_DIALOG_HPP_

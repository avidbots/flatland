#include "change_rate_dialog.hpp"

#include <rviz_common/tool_manager.hpp>

namespace flatland_rviz_plugins {

ChangeRateDialog::ChangeRateDialog(QWidget *parent,
                                   rviz_common::DisplayContext *context,
                                   ChangeRateTool *tool)
    : QDialog(parent), context_(context), tool_(tool) {
  r_edit = new QLineEdit;
  auto *okButton = new QPushButton("ok");
  auto *cancelButton = new QPushButton("cancel");

  r_edit->setFocusPolicy(Qt::ClickFocus);  // only name gets focus
  okButton->setFocusPolicy(Qt::NoFocus);
  cancelButton->setFocusPolicy(Qt::NoFocus);

  connect(okButton, &QAbstractButton::clicked, this,
          &ChangeRateDialog::OkButtonClicked);
  connect(cancelButton, &QAbstractButton::clicked, this,
          &ChangeRateDialog::CancelButtonClicked);

  auto *h0_layout = new QHBoxLayout;
  h0_layout->addWidget(new QLabel("rate:"));
  h0_layout->addWidget(r_edit);

  auto *h1_layout = new QHBoxLayout;
  h1_layout->addWidget(okButton);
  h1_layout->addWidget(cancelButton);

  auto *v_layout = new QVBoxLayout;
  v_layout->addLayout(h0_layout);
  v_layout->addLayout(h1_layout);

  setLayout(v_layout);

  // delete the Dialog if the user clicks on the x to close window
  this->setAttribute(Qt::WA_DeleteOnClose, true);
}

void ChangeRateDialog::CancelButtonClicked() {
  RCLCPP_INFO_STREAM(rclcpp::get_logger("ModelDialog"),
                     "LoadModelDialog::CancelButtonClicked");

  auto tool_man = context_->getToolManager();
  auto dflt_tool = tool_man->getDefaultTool();
  tool_man->setCurrentTool(dflt_tool);

  this->close();
}

void ChangeRateDialog::OkButtonClicked() {
  RCLCPP_INFO_STREAM(rclcpp::get_logger("ModelDialog"),
                     "LoadModelDialog::OkButtonClicked");
  bool ok(false);
  double rate = r_edit->text().toDouble(&ok);
  if (ok) {
    tool_->setRate(rate);

    auto tool_man = context_->getToolManager();
    auto dflt_tool = tool_man->getDefaultTool();
    tool_man->setCurrentTool(dflt_tool);

    this->close();
  } else {
    QMessageBox msgBox;
    msgBox.setText(
        "Error: The rate needs to be a valid double (float 64-bit) value.");
    msgBox.exec();
  }
}

}  // namespace flatland_rviz_plugins

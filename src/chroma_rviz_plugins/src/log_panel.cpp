#include "chroma_rviz_plugins/log_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <QScrollBar>
#include <QString>

namespace chroma_rviz_plugins
{

  LogPanel::LogPanel(QWidget *parent) : rviz_common::Panel(parent)
  {
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setContentsMargins(4, 4, 4, 4);

    QHBoxLayout *controls_layout = new QHBoxLayout();

    cb_info_ = new QCheckBox("INFO");
    cb_info_->setChecked(true);
    cb_warn_ = new QCheckBox("WARN");
    cb_warn_->setChecked(true);
    cb_error_ = new QCheckBox("ERROR");
    cb_error_->setChecked(true);

    controls_layout->addWidget(cb_info_);
    controls_layout->addWidget(cb_warn_);
    controls_layout->addWidget(cb_error_);
    controls_layout->addStretch();

    clear_btn_ = new QPushButton("Clear");
    controls_layout->addWidget(clear_btn_);

    layout->addLayout(controls_layout);

    text_edit_ = new QTextEdit();
    text_edit_->setReadOnly(true);

    text_edit_->setStyleSheet(
        "QTextEdit {"
        "  background-color: #0F172A;"
        "  color: #E2E8F0;"
        "  font-family: 'Courier New', monospace;"
        "  font-size: 11px;"
        "  border: 1px solid #334155;"
        "  border-radius: 4px;"
        "}");
    layout->addWidget(text_edit_);

    connect(clear_btn_, SIGNAL(clicked()), this, SLOT(onClearClicked()));
  }

  LogPanel::~LogPanel() = default;

  void LogPanel::onInitialize()
  {
    node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    log_sub_ = node_->create_subscription<rcl_interfaces::msg::Log>(
        "/rosout", 100,
        std::bind(&LogPanel::logCallback, this, std::placeholders::_1));
  }

  void LogPanel::logCallback(const rcl_interfaces::msg::Log::SharedPtr msg)
  {
    std::string node_name = msg->name;
    if (node_name.find("rviz") != std::string::npos ||
        node_name.find("gazebo") != std::string::npos ||
        node_name.find("transform") != std::string::npos)
    {
      return;
    }

    QString color;
    QString level;
    int msg_level = msg->level;

    switch (msg_level)
    {
    case rcl_interfaces::msg::Log::DEBUG:
      return;
    case rcl_interfaces::msg::Log::INFO:
      color = "#38BDF8"; // Blue
      level = "[INFO]";
      break;
    case rcl_interfaces::msg::Log::WARN:
      color = "#FBBF24"; // Yellow
      level = "[WARN]";
      break;
    case rcl_interfaces::msg::Log::ERROR:
    case rcl_interfaces::msg::Log::FATAL:
      color = "#EF4444"; // Red
      level = "[ERROR]";
      break;
    default:
      color = "#E2E8F0"; // White
      level = "[LOG]";
    }

    QString formatted_msg = QString("<span style='color:%1;'>%2 [%3]: %4</span>")
                                .arg(color, level, QString::fromStdString(node_name), QString::fromStdString(msg->msg));

    QMetaObject::invokeMethod(this, [this, msg_level, formatted_msg](){
      bool show_log = false;
      if (msg_level == rcl_interfaces::msg::Log::INFO && cb_info_->isChecked()) show_log = true;
      else if (msg_level == rcl_interfaces::msg::Log::WARN && cb_warn_->isChecked()) show_log = true;
      else if ((msg_level == rcl_interfaces::msg::Log::ERROR || msg_level == rcl_interfaces::msg::Log::FATAL) && cb_error_->isChecked()) show_log = true;

      if (show_log) {
        text_edit_->append(formatted_msg);

        QScrollBar *sb = text_edit_->verticalScrollBar();
        sb->setValue(sb->maximum());
      } 
    });
  }

  void LogPanel::onClearClicked()
  {
    text_edit_->clear();
  }

}

PLUGINLIB_EXPORT_CLASS(chroma_rviz_plugins::LogPanel, rviz_common::Panel)
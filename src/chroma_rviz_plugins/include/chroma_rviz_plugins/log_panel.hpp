#ifndef CHROMA_RVIZ_PLUGINS__LOG_PANEL_HPP_
#define CHROMA_RVIZ_PLUGINS__LOG_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextEdit>
#include <QPushButton>
#include <QCheckBox>
#include <mutex>
#include <vector>
#include <string>

namespace chroma_rviz_plugins
{
  class LogPanel : public rviz_common::Panel
  {
    Q_OBJECT
  public:
    explicit LogPanel(QWidget *parent = nullptr);
    ~LogPanel() override;
    void onInitialize() override;

  protected Q_SLOTS:
    void onClearClicked();

  private:
    void logCallback(const rcl_interfaces::msg::Log::SharedPtr msg);

    QTextEdit *text_edit_;
    QPushButton *clear_btn_;
    QCheckBox *cb_info_;
    QCheckBox *cb_warn_;
    QCheckBox *cb_error_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr log_sub_;
  };

}

#endif
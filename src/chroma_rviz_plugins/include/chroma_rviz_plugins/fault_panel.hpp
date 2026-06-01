#ifndef CHROMA_RVIZ_PLUGINS__FAULT_PANEL_HPP_
#define CHROMA_RVIZ_PLUGINS__FAULT_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <QLineEdit>
#include <QComboBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QLabel>
#include <map>
#include <string>

namespace chroma_rviz_plugins
{

  class FaultPanel : public rviz_common::Panel
  {
    Q_OBJECT
  public:
    explicit FaultPanel(QWidget *parent = nullptr);
    ~FaultPanel() override;
    void onInitialize() override;

  protected Q_SLOTS:
    void onInjectClicked();
    void onClearClicked();

  private:
    QLineEdit *robot_input_;
    QComboBox *fault_dropdown_;
    QPushButton *inject_btn_;
    QPushButton *clear_btn_;
    QLabel *status_label_;

    rclcpp::Node::SharedPtr node_;

    std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> fault_pubs_;

    void ensurePublisherExists(const std::string &robot_id);
    bool robotExists(const std::string &robot_id);
  };

}

#endif
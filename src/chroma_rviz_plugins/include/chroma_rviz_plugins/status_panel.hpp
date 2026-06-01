#ifndef CHROMA_RVIZ_PLUGINS__STATUS_PANEL_HPP_
#define CHROMA_RVIZ_PLUGINS__STATUS_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chroma_interfaces/msg/swarm_status.hpp>

#include <QTimer>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QFrame>
#include <QLabel>
#include <QProgressBar>
#include <map>
#include <mutex>
#include <string>

namespace chroma_rviz_plugins
{
  class RobotCard : public QFrame
  {
    Q_OBJECT
  public:
    explicit RobotCard(QWidget *parent = nullptr);
    void updateData(const chroma_interfaces::msg::SwarmStatus &msg);

  private:
    QLabel *id_label_;
    QLabel *type_label_;
    QVBoxLayout *caps_layout_;
    std::map<std::string, QProgressBar *> cap_bars_;
    std::map<std::string, QLabel *> cap_vals_;
  };

  class StatusPanel : public rviz_common::Panel
  {
    Q_OBJECT
  public:
    explicit StatusPanel(QWidget *parent = nullptr);
    ~StatusPanel() override;
    void onInitialize() override;

  protected Q_SLOTS:
    void updateUI();

  private:
    void statusCallback(const chroma_interfaces::msg::SwarmStatus::SharedPtr msg);

    QVBoxLayout *scroll_layout_;
    std::map<std::string, RobotCard *> robot_cards_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<chroma_interfaces::msg::SwarmStatus>::SharedPtr status_sub_;

    std::map<std::string, chroma_interfaces::msg::SwarmStatus> latest_statuses_;
    std::mutex status_mutex_;
    QTimer *update_timer_;
  };

}

#endif
#ifndef CHROMA_RVIZ_PLUGINS__TASK_PANEL_HPP_
#define CHROMA_RVIZ_PLUGINS__TASK_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chroma_interfaces/msg/task.hpp>
#include <chroma_interfaces/msg/task_allocation.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <QTimer>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <map>
#include <mutex>
#include <string>

namespace chroma_rviz_plugins
{

struct TaskData {
  std::string id;
  std::string type = "UNKNOWN";
  std::string robot = "UNASSIGNED";
  std::string status = "PENDING";
  std::string requirements = "";
};

class TaskCard : public QFrame
{
  Q_OBJECT
public:
  explicit TaskCard(QWidget * parent = nullptr);
  void updateData(const TaskData & data);
  void setAborting();

Q_SIGNALS:
  void abortRequested(std::string task_id);

private Q_SLOTS:
  void onAbortClicked();

private:
  QLabel * id_label_;
  QLabel * type_label_;
  QLabel * robot_label_;
  QLabel * status_label_;
  QLabel * req_label_;
  QPushButton * abort_btn_;
  
  std::string current_task_id_;
};

class TaskPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit TaskPanel(QWidget * parent = nullptr);
  ~TaskPanel() override;
  void onInitialize() override;

protected Q_SLOTS:
  void updateUI();
  void handleAbort(std::string task_id);

private:
  void taskCallback(const chroma_interfaces::msg::Task::SharedPtr msg);
  void allocationCallback(const chroma_interfaces::msg::TaskAllocation::SharedPtr msg);

  QVBoxLayout * scroll_layout_;
  std::map<std::string, TaskCard *> task_cards_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<chroma_interfaces::msg::Task>::SharedPtr task_sub_;
  rclcpp::Subscription<chroma_interfaces::msg::TaskAllocation>::SharedPtr alloc_sub_;

  rclcpp::Publisher<chroma_interfaces::msg::TaskAllocation>::SharedPtr alloc_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::map<std::string, TaskData> active_tasks_;
  std::mutex task_mutex_;
  QTimer * update_timer_;
};

}

#endif
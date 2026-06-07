#include "chroma_rviz_plugins/task_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace chroma_rviz_plugins
{
  TaskCard::TaskCard(QWidget *parent) : QFrame(parent)
  {
    this->setStyleSheet("TaskCard { background-color: #F1F5F9; border-radius: 6px; border: 1px solid #CBD5E1; margin: 2px; }");

    QVBoxLayout *main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(6, 6, 6, 6);
    main_layout->setSpacing(2);

    QHBoxLayout *top_row = new QHBoxLayout();
    id_label_ = new QLabel("TASK_XXXX");
    id_label_->setStyleSheet("font-weight: bold; font-size: 12px; color: #334155;");

    status_label_ = new QLabel("PENDING");
    status_label_->setStyleSheet("font-weight: bold; font-size: 11px; color: #64748B;");
    status_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    abort_btn_ = new QPushButton("X");
    abort_btn_->setFixedSize(20, 20);
    abort_btn_->setStyleSheet("background-color: #EF4444; color: white; font-weight: bold; border-radius: 4px; border: none;");
    abort_btn_->setToolTip("Abort Task");

    top_row->addWidget(id_label_);
    top_row->addWidget(status_label_);
    top_row->addWidget(abort_btn_);
    main_layout->addLayout(top_row);

    QHBoxLayout *bot_row = new QHBoxLayout();
    type_label_ = new QLabel("Type: UNKNOWN");
    type_label_->setStyleSheet("font-size: 10px; color: #475569;");

    robot_label_ = new QLabel("Robot: UNASSIGNED");
    robot_label_->setStyleSheet("font-size: 10px; font-weight: bold; color: #475569;");
    robot_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    bot_row->addWidget(type_label_);
    bot_row->addWidget(robot_label_);
    main_layout->addLayout(bot_row);

    req_label_ = new QLabel("Req: None");
    req_label_->setStyleSheet("font-size: 9px; color: #94A3B8; font-style: italic;");
    main_layout->addWidget(req_label_);

    connect(abort_btn_, SIGNAL(clicked()), this, SLOT(onAbortClicked()));
  }

  void TaskCard::onAbortClicked()
  {
    Q_EMIT abortRequested(current_task_id_);
  }

  void TaskCard::setAborting()
  {
    abort_btn_->setEnabled(false);
    abort_btn_->setStyleSheet("background-color: #94A3B8; color: white; font-weight: bold; border-radius: 4px; border: none;");
    status_label_->setText("ABORTING...");
    status_label_->setStyleSheet("font-weight: bold; font-size: 11px; color: #EF4444;");
  }

  void TaskCard::updateData(const TaskData &data)
  {
    if (!abort_btn_->isEnabled())
      return;

    current_task_id_ = data.id;
    id_label_->setText(QString::fromStdString(data.id));
    type_label_->setText(QString::fromStdString(data.type));
    robot_label_->setText(QString::fromStdString(data.robot));
    status_label_->setText(QString::fromStdString(data.status));
    req_label_->setText(QString::fromStdString("Req: " + data.requirements));

    if (data.status == "COMPLETED")
    {
      status_label_->setStyleSheet("font-weight: bold; font-size: 11px; color: #10B981;");
    }
    else if (data.status == "ABORTED" || data.status == "FAILED")
    {
      status_label_->setStyleSheet("font-weight: bold; font-size: 11px; color: #EF4444;");
    }
    else if (data.status == "ASSIGNED")
    {
      status_label_->setStyleSheet("font-weight: bold; font-size: 11px; color: #3B82F6;");
    }
    else
    {
      status_label_->setStyleSheet("font-weight: bold; font-size: 11px; color: #F59E0B;");
    }
  }

  TaskPanel::TaskPanel(QWidget *parent) : rviz_common::Panel(parent)
  {
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    QScrollArea *scroll_area = new QScrollArea();
    scroll_area->setWidgetResizable(true);
    scroll_area->setFrameShape(QFrame::NoFrame);

    QWidget *scroll_content = new QWidget();
    scroll_layout_ = new QVBoxLayout(scroll_content);
    scroll_layout_->setAlignment(Qt::AlignTop);
    scroll_layout_->setContentsMargins(4, 4, 4, 4);

    scroll_area->setWidget(scroll_content);
    layout->addWidget(scroll_area);

    update_timer_ = new QTimer(this);
    connect(update_timer_, SIGNAL(timeout()), this, SLOT(updateUI()));
    update_timer_->start(500);
  }

  TaskPanel::~TaskPanel() = default;

  void TaskPanel::onInitialize()
  {
    node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    task_sub_ = node_->create_subscription<chroma_interfaces::msg::Task>(
        "/mission/tasks", 10, std::bind(&TaskPanel::taskCallback, this, std::placeholders::_1));

    alloc_sub_ = node_->create_subscription<chroma_interfaces::msg::TaskAllocation>(
        "/swarm/allocation", 10, std::bind(&TaskPanel::allocationCallback, this, std::placeholders::_1));

    alloc_pub_ = node_->create_publisher<chroma_interfaces::msg::TaskAllocation>("/swarm/allocation", 10);
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/mission/markers", 10);
  }

  void TaskPanel::taskCallback(const chroma_interfaces::msg::Task::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(task_mutex_);

    std::string reqs = "[";
    for (size_t i = 0; i < msg->required_capabilities.size(); i++)
    {
      reqs += msg->required_capabilities[i];
      if (i < msg->required_capabilities.size() - 1)
        reqs += ", ";
    }
    reqs += "]";

    if (active_tasks_.find(msg->task_id) == active_tasks_.end())
    {
      TaskData td;
      td.id = msg->task_id;
      td.type = msg->type;
      td.requirements = reqs;
      active_tasks_[msg->task_id] = td;
    }
    else
    {
      active_tasks_[msg->task_id].type = msg->type;
      active_tasks_[msg->task_id].requirements = reqs;
    }
  }

  void TaskPanel::allocationCallback(const chroma_interfaces::msg::TaskAllocation::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(task_mutex_);

    if (msg->status == "CANCELLED")
      return;

    if (active_tasks_.find(msg->task_id) == active_tasks_.end())
    {
      TaskData td;
      td.id = msg->task_id;
      active_tasks_[msg->task_id] = td;
    }

    active_tasks_[msg->task_id].robot = msg->robot_id;
    active_tasks_[msg->task_id].status = msg->status;
  }

  void TaskPanel::handleAbort(std::string task_id)
  {
    if (task_cards_.find(task_id) != task_cards_.end())
    {
      task_cards_[task_id]->setAborting();
    }

    auto cancel_msg = chroma_interfaces::msg::TaskAllocation();
    cancel_msg.task_id = task_id;
    cancel_msg.robot_id = "OPERATOR";
    cancel_msg.status = "CANCELLED";
    alloc_pub_->publish(cancel_msg);

    QTimer::singleShot(1500, [this, task_id]()
                       {
    std::lock_guard<std::mutex> lock(task_mutex_);

    // Erase Markers
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = 3;
    delete_marker.ns = "task_" + task_id; 
    marker_array.markers.push_back(delete_marker);
    marker_pub_->publish(marker_array);

    // Erase UI Card
    if (task_cards_.find(task_id) != task_cards_.end()) {
      TaskCard * card = task_cards_[task_id];
      scroll_layout_->removeWidget(card);
      delete card;
      task_cards_.erase(task_id);
    }
    active_tasks_.erase(task_id); });
  }

  void TaskPanel::updateUI()
  {
    std::lock_guard<std::mutex> lock(task_mutex_);

    for (const auto &pair : active_tasks_)
    {
      const std::string &tid = pair.first;
      const TaskData &data = pair.second;

      if (task_cards_.find(tid) == task_cards_.end())
      {
        TaskCard *new_card = new TaskCard();
        connect(new_card, SIGNAL(abortRequested(std::string)), this, SLOT(handleAbort(std::string)));
        scroll_layout_->addWidget(new_card);
        task_cards_[tid] = new_card;
      }

      task_cards_[tid]->updateData(data);
    }
  }

}

PLUGINLIB_EXPORT_CLASS(chroma_rviz_plugins::TaskPanel, rviz_common::Panel)
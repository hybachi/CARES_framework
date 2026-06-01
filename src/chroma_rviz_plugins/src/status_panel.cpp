#include "chroma_rviz_plugins/status_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace chroma_rviz_plugins
{
  // ROBOT CARD WIDGET
  RobotCard::RobotCard(QWidget *parent) : QFrame(parent)
  {
    this->setAttribute(Qt::WA_StyledBackground, true);
    this->setStyleSheet("RobotCard { background-color: #1E293B; border-radius: 6px; border: 1px solid #334155; margin: 2px; }");

    QVBoxLayout *main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(8, 8, 8, 8);
    main_layout->setSpacing(2);

    QHBoxLayout *header_layout = new QHBoxLayout();
    id_label_ = new QLabel("UNKNOWN");
    id_label_->setStyleSheet("font-weight: bold; font-size: 13px; color: #10B981;");
    type_label_ = new QLabel("Type: UNKNOWN");
    type_label_->setStyleSheet("font-size: 9px; color: #94A3B8;");
    type_label_->setAlignment(Qt::AlignRight | Qt::AlignBottom);

    header_layout->addWidget(id_label_);
    header_layout->addWidget(type_label_);
    main_layout->addLayout(header_layout);

    QFrame *line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setStyleSheet("background-color: #334155;");
    main_layout->addWidget(line);

    caps_layout_ = new QVBoxLayout();
    caps_layout_->setSpacing(4);
    main_layout->addLayout(caps_layout_);
  }

  void RobotCard::updateData(const chroma_interfaces::msg::SwarmStatus &msg)
  {
    id_label_->setText(QString::fromStdString(msg.robot_id));
    type_label_->setText(QString::fromStdString(msg.robot_type));

    for (const auto &cap : msg.capabilities)
    {
      std::string cap_name = cap.type;
      int percentage = static_cast<int>(cap.value * 100.0);

      if (cap_bars_.find(cap_name) == cap_bars_.end())
      {
        QHBoxLayout *row = new QHBoxLayout();
        row->setContentsMargins(0, 0, 0, 0);

        QLabel *lbl = new QLabel(QString::fromStdString(cap_name));
        lbl->setStyleSheet("font-size: 9px; font-weight: bold; color: #334155;");
        lbl->setFixedWidth(55);

        QProgressBar *bar = new QProgressBar();
        bar->setRange(0, 100);
        bar->setFixedHeight(8);
        bar->setTextVisible(false);

        QLabel *val_lbl = new QLabel("100%");
        val_lbl->setStyleSheet("font-size: 10px; color: #334155;");
        val_lbl->setFixedWidth(30);
        val_lbl->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

        row->addWidget(lbl);
        row->addWidget(bar);
        row->addWidget(val_lbl);

        caps_layout_->addLayout(row);
        cap_bars_[cap_name] = bar;
        cap_vals_[cap_name] = val_lbl;
      }

      cap_bars_[cap_name]->setValue(percentage);
      cap_vals_[cap_name]->setText(QString::number(percentage) + "%");

      QString bar_style = "QProgressBar { background-color: #334155; border: none; border-radius: 4px; } "
                          "QProgressBar::chunk { border-radius: 4px; ";

      if (cap.is_degraded)
      {
        bar_style += "background-color: #EF4444; }"; // Red
      }
      else if (percentage < 60)
      {
        bar_style += "background-color: #F59E0B; }"; // Yellow
      }
      else
      {
        bar_style += "background-color: #10B981; }"; // Green
      }
      cap_bars_[cap_name]->setStyleSheet(bar_style);
    }
  }

  // STATUS PANEL WIDGET
  StatusPanel::StatusPanel(QWidget *parent) : rviz_common::Panel(parent)
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

  StatusPanel::~StatusPanel() = default;

  void StatusPanel::onInitialize()
  {
    node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    status_sub_ = node_->create_subscription<chroma_interfaces::msg::SwarmStatus>(
        "/swarm/status", rclcpp::QoS(10).transient_local(),
        std::bind(&StatusPanel::statusCallback, this, std::placeholders::_1));
  }

  void StatusPanel::statusCallback(const chroma_interfaces::msg::SwarmStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    latest_statuses_[msg->robot_id] = *msg;
  }

  void StatusPanel::updateUI()
  {
    std::lock_guard<std::mutex> lock(status_mutex_);

    for (const auto &pair : latest_statuses_)
    {
      const std::string &rid = pair.first;
      const auto &status_msg = pair.second;

      if (robot_cards_.find(rid) == robot_cards_.end())
      {
        RobotCard *new_card = new RobotCard();
        scroll_layout_->addWidget(new_card);
        robot_cards_[rid] = new_card;
      }

      robot_cards_[rid]->updateData(status_msg);
    }
  }

}

PLUGINLIB_EXPORT_CLASS(chroma_rviz_plugins::StatusPanel, rviz_common::Panel)
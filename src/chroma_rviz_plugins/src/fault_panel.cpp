#include "chroma_rviz_plugins/fault_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <QString>
#include <vector>

namespace chroma_rviz_plugins
{

  FaultPanel::FaultPanel(QWidget *parent) : rviz_common::Panel(parent)
  {
    QVBoxLayout *layout = new QVBoxLayout(this);

    layout->addWidget(new QLabel("Target Robot ID:"));
    robot_input_ = new QLineEdit();
    robot_input_->setPlaceholderText("e.g. tb3_0");
    layout->addWidget(robot_input_);

    layout->addWidget(new QLabel("Fault Type:"));
    fault_dropdown_ = new QComboBox();
    fault_dropdown_->addItem("mobility_penalty");
    fault_dropdown_->addItem("mobility_failure");
    fault_dropdown_->addItem("vision_penalty");
    fault_dropdown_->addItem("vision_failure");
    fault_dropdown_->setEditable(true);
    layout->addWidget(fault_dropdown_);

    inject_btn_ = new QPushButton("Inject Fault");
    layout->addWidget(inject_btn_);

    clear_btn_ = new QPushButton("Clear All Faults");
    layout->addWidget(clear_btn_);

    status_label_ = new QLabel("Ready.");
    status_label_->setStyleSheet("font-size: 10px; color: #64748B; margin-top: 5px;");
    layout->addWidget(status_label_);

    connect(inject_btn_, SIGNAL(clicked()), this, SLOT(onInjectClicked()));
    connect(clear_btn_, SIGNAL(clicked()), this, SLOT(onClearClicked()));
  }

  FaultPanel::~FaultPanel() = default;

  void FaultPanel::onInitialize()
  {
    node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  }

  bool FaultPanel::robotExists(const std::string &robot_id)
  {
    if (robot_id.empty())
      return false;

    std::vector<std::string> node_names = node_->get_node_names();

    std::string exact_target = "/" + robot_id + "/capability_manager";

    for (const auto &name : node_names)
    {
      if (name == exact_target)
      {
        return true;
      }
    }
    return false;
  }

  void FaultPanel::ensurePublisherExists(const std::string &robot_id)
  {
    if (fault_pubs_.find(robot_id) == fault_pubs_.end())
    {
      std::string topic_name = "/" + robot_id + "/inject_fault";
      fault_pubs_[robot_id] = node_->create_publisher<std_msgs::msg::String>(topic_name, 10);
    }
  }

  void FaultPanel::onInjectClicked()
  {
    std::string robot_id = robot_input_->text().trimmed().toStdString();
    std::string fault_type = fault_dropdown_->currentText().trimmed().toStdString();

    if (robot_id.empty())
    {
      status_label_->setText("Error: Please enter a Robot ID.");
      status_label_->setStyleSheet("font-size: 10px; color: #F59E0B; font-weight: bold;");
      return;
    }

    if (!robotExists(robot_id))
    {
      status_label_->setText(QString("Error: Robot '%1' not found.").arg(QString::fromStdString(robot_id)));
      status_label_->setStyleSheet("font-size: 10px; color: #F59E0B; font-weight: bold;");
      return;
    }

    ensurePublisherExists(robot_id);

    auto msg = std_msgs::msg::String();
    msg.data = fault_type;
    fault_pubs_[robot_id]->publish(msg);

    status_label_->setText(QString("Injected '%1' into %2")
                               .arg(QString::fromStdString(fault_type), QString::fromStdString(robot_id)));
    status_label_->setStyleSheet("font-size: 10px; color: #EF4444; font-weight: bold;");
  }

  void FaultPanel::onClearClicked()
  {
    std::string robot_id = robot_input_->text().trimmed().toStdString();

    if (robot_id.empty() || !robotExists(robot_id))
    {
      status_label_->setText("Error: Valid Robot ID required to clear faults.");
      status_label_->setStyleSheet("font-size: 10px; color: #F59E0B; font-weight: bold;");
      return;
    }

    ensurePublisherExists(robot_id);

    auto msg = std_msgs::msg::String();
    msg.data = "reset";
    fault_pubs_[robot_id]->publish(msg);

    status_label_->setText(QString("Cleared faults for %1").arg(QString::fromStdString(robot_id)));
    status_label_->setStyleSheet("font-size: 10px; color: #10B981; font-weight: bold;");
  }

}

PLUGINLIB_EXPORT_CLASS(chroma_rviz_plugins::FaultPanel, rviz_common::Panel)
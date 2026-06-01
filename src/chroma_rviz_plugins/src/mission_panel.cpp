#include "chroma_rviz_plugins/mission_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <QString>
#include <cstdlib>
#include <ctime>

namespace chroma_rviz_plugins
{

    MissionPanel::MissionPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        std::srand(std::time(nullptr));

        QVBoxLayout *layout = new QVBoxLayout;
        task_type_dropdown_ = new QComboBox();
        task_type_dropdown_->addItem("SEARCH");
        task_type_dropdown_->addItem("DELIVERY");
        layout->addWidget(new QLabel("Select Task Type:"));
        layout->addWidget(task_type_dropdown_);

        req_label_ = new QLabel("Requires: [VISION, MOBILITY]");
        req_label_->setStyleSheet("font-size: 10px; color: #64748B; font-style: italic; margin-bottom: 8px;");
        layout->addWidget(req_label_);

        status_label_ = new QLabel("Points captured: 0/2\nUse 'Publish Point' tool on the map.");
        layout->addWidget(status_label_);

        publish_btn_ = new QPushButton("Publish Mission");
        publish_btn_->setEnabled(false);
        layout->addWidget(publish_btn_);

        clear_btn_ = new QPushButton("Clear Points");
        layout->addWidget(clear_btn_);

        setLayout(layout);

        connect(publish_btn_, SIGNAL(clicked()), this, SLOT(onPublishClicked()));
        connect(clear_btn_, SIGNAL(clicked()), this, SLOT(onClearClicked()));
        connect(task_type_dropdown_, SIGNAL(currentIndexChanged(int)), this, SLOT(updateUI()));
    }

    MissionPanel::~MissionPanel() = default;

    void MissionPanel::onInitialize()
    {
        node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        point_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10,
            std::bind(&MissionPanel::pointCallback, this, std::placeholders::_1));

        task_pub_ = node_->create_publisher<chroma_interfaces::msg::Task>("/mission/tasks", 10);
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/mission/markers", 10);
    }

    void MissionPanel::pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (clicked_points_.size() < 2)
        {
            clicked_points_.push_back(msg->point);
            updateUI();
            publishMarkers(false);
        }
    }

    void MissionPanel::onClearClicked()
    {
        clearPendingMarkers();
        clicked_points_.clear();
        updateUI();
        status_label_->setText("Points captured: 0/2\nUse 'Publish Point' tool on the map.");
    }

    void MissionPanel::onPublishClicked()
    {
        if (clicked_points_.size() < 2)
            return;

        auto task_msg = chroma_interfaces::msg::Task();
        int random_id = std::rand() % 10000;
        task_msg.task_id = "TASK_" + std::to_string(random_id);
        task_msg.type = task_type_dropdown_->currentText().toStdString();
        task_msg.priority = 1.0;

        task_msg.location = clicked_points_[0];
        task_msg.target_area.push_back(clicked_points_[1]);

        if (task_msg.type == "SEARCH")
        {
            task_msg.required_capabilities = {"VISION", "MOBILITY"};
            task_msg.min_capability_score = 0.4;
        }
        else
        {
            task_msg.required_capabilities = {"PAYLOAD", "MOBILITY"};
            task_msg.min_capability_score = 0.4;
        }

        task_pub_->publish(task_msg);

        publishMarkers(true, task_msg.task_id);

        clearPendingMarkers();
        clicked_points_.clear();

        status_label_->setText(QString("Mission %1 Published!\nWaiting for new points...").arg(QString::fromStdString(task_msg.task_id)));
        publish_btn_->setEnabled(false);
    }

    void MissionPanel::updateUI()
    {
        status_label_->setText(QString("Points captured: %1/2").arg(clicked_points_.size()));
        publish_btn_->setEnabled(clicked_points_.size() == 2);

        if (task_type_dropdown_->currentText() == "SEARCH") {
            req_label_->setText("Requires: [VISION, MOBILITY]");
        } else {
            req_label_->setText("Requires: [PAYLOAD, MOBILITY]");
        }

        if (clicked_points_.size() > 0)
        {
            publishMarkers(false);
        }
    }

    void MissionPanel::clearPendingMarkers()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        for (int i = 0; i < 15; i++)
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.ns = "pending_task";
            m.id = i;
            m.action = visualization_msgs::msg::Marker::DELETE;
            marker_array.markers.push_back(m);
        }
        marker_pub_->publish(marker_array);
    }

    void MissionPanel::publishMarkers(bool is_published, std::string task_id)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        std::string ns = is_published ? ("task_" + task_id) : "pending_task";

        float r = is_published ? 0.1 : 0.9;
        float g = is_published ? 0.9 : 0.1;
        float b = 0.1;

        for (size_t i = 0; i < clicked_points_.size(); i++)
        {
            // The Sphere
            visualization_msgs::msg::Marker sphere;
            sphere.header.frame_id = "map";
            sphere.header.stamp = node_->now();
            sphere.ns = ns;
            sphere.id = i;
            sphere.type = visualization_msgs::msg::Marker::SPHERE;
            sphere.action = visualization_msgs::msg::Marker::ADD;
            sphere.pose.position = clicked_points_[i];
            sphere.pose.position.z = 0.05;
            sphere.scale.x = 0.1;
            sphere.scale.y = 0.1;
            sphere.scale.z = 0.1;
            sphere.color.r = r;
            sphere.color.g = g;
            sphere.color.b = b;
            sphere.color.a = 0.8;
            marker_array.markers.push_back(sphere);

            // The Coordinate Label
            visualization_msgs::msg::Marker coord_text;
            coord_text.header.frame_id = "map";
            coord_text.header.stamp = node_->now();
            coord_text.ns = ns;
            coord_text.id = 10 + i;
            coord_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            coord_text.action = visualization_msgs::msg::Marker::ADD;

            coord_text.pose.position = clicked_points_[i];
            if (i == 0)
            {
                coord_text.pose.position.x += 0.30; // Shift right
            }
            else
            {
                coord_text.pose.position.x -= 0.30; // Shift left
            }
            coord_text.pose.position.z = 0.05;

            coord_text.scale.x = 0.0;
            coord_text.scale.y = 0.0;
            coord_text.scale.z = 0.1;

            coord_text.color.r = 0.3;
            coord_text.color.g = 0.3;
            coord_text.color.b = 0.3;
            coord_text.color.a = 1.0;

            QString coord_str = QString("(%1,%2)")
                                    .arg(clicked_points_[i].x, 0, 'f', 2)
                                    .arg(clicked_points_[i].y, 0, 'f', 2);
            coord_text.text = coord_str.toStdString();

            marker_array.markers.push_back(coord_text);
        }

        if (clicked_points_.size() == 2)
        {
            std::string task_type = task_type_dropdown_->currentText().toStdString();

            visualization_msgs::msg::Marker text;
            text.header.frame_id = "map";
            text.header.stamp = node_->now();
            text.ns = ns;
            text.id = 2;
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;
            text.pose.position.x = (clicked_points_[0].x + clicked_points_[1].x) / 2.0;
            text.pose.position.y = (clicked_points_[0].y + clicked_points_[1].y) / 2.0;
            text.pose.position.z = 0.3;
            text.scale.z = 0.18;

            text.color.r = 0.2;
            text.color.g = 0.2;
            text.color.b = 0.2;
            text.color.a = 0.8;

            if (task_type == "SEARCH")
            {
                text.text = is_published ? ("SEARCH\n" + task_id) : "Publish\nSearch";

                visualization_msgs::msg::Marker box;
                box.header.frame_id = "map";
                box.header.stamp = node_->now();
                box.ns = ns;
                box.id = 3;
                box.type = visualization_msgs::msg::Marker::CUBE;
                box.action = visualization_msgs::msg::Marker::ADD;
                box.pose.position.x = text.pose.position.x;
                box.pose.position.y = text.pose.position.y;
                box.pose.position.z = 0.01;
                box.scale.x = std::abs(clicked_points_[0].x - clicked_points_[1].x);
                box.scale.y = std::abs(clicked_points_[0].y - clicked_points_[1].y);
                box.scale.z = 0.01;
                box.color.r = r;
                box.color.g = g;
                box.color.b = b;
                box.color.a = 0.2;
                marker_array.markers.push_back(box);
            }
            else if (task_type == "DELIVERY")
            {
                text.text = is_published ? ("DELIVERY\n" + task_id) : "Publish\nDelivery";

                visualization_msgs::msg::Marker line;
                line.header.frame_id = "map";
                line.header.stamp = node_->now();
                line.ns = ns;
                line.id = 3;
                line.type = visualization_msgs::msg::Marker::LINE_STRIP;
                line.action = visualization_msgs::msg::Marker::ADD;
                line.scale.x = 0.04;
                line.color.r = r;
                line.color.g = g;
                line.color.b = b;
                line.color.a = 0.3;
                line.points.push_back(clicked_points_[0]);
                line.points.push_back(clicked_points_[1]);
                marker_array.markers.push_back(line);

                // Label Point A
                visualization_msgs::msg::Marker label_a;
                label_a.header.frame_id = "map";
                label_a.header.stamp = node_->now();
                label_a.ns = ns;
                label_a.id = 4;
                label_a.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                label_a.action = visualization_msgs::msg::Marker::ADD;
                label_a.pose.position = clicked_points_[0];
                label_a.pose.position.z = 0.15;
                label_a.scale.z = 0.15;
                label_a.color.r = 0.2;
                label_a.color.g = 0.2;
                label_a.color.b = 0.2;
                label_a.color.a = 0.9;
                label_a.text = "A";
                marker_array.markers.push_back(label_a);

                // Label Point B
                visualization_msgs::msg::Marker label_b;
                label_b.header.frame_id = "map";
                label_b.header.stamp = node_->now();
                label_b.ns = ns;
                label_b.id = 5;
                label_b.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                label_b.action = visualization_msgs::msg::Marker::ADD;
                label_b.pose.position = clicked_points_[1];
                label_b.pose.position.z = 0.15;
                label_b.scale.z = 0.15;
                label_b.color.r = 0.2;
                label_b.color.g = 0.2;
                label_b.color.b = 0.2;
                label_b.color.a = 0.9;
                label_b.text = "B";
                marker_array.markers.push_back(label_b);
            }
            marker_array.markers.push_back(text);
        }

        marker_pub_->publish(marker_array);
    }

}

PLUGINLIB_EXPORT_CLASS(chroma_rviz_plugins::MissionPanel, rviz_common::Panel)
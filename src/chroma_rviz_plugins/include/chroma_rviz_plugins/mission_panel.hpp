#ifndef CHROMA_RVIZ_PLUGINS__MISSION_PANEL_HPP_
#define CHROMA_RVIZ_PLUGINS__MISSION_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <chroma_interfaces/msg/task.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QVBoxLayout>
#include <vector>

namespace chroma_rviz_plugins
{
    class MissionPanel : public rviz_common::Panel
    {
        Q_OBJECT

    public:
        explicit MissionPanel(QWidget *parent = nullptr);
        ~MissionPanel() override;

        void onInitialize() override;

    protected Q_SLOTS:
        void onPublishClicked();
        void onClearClicked();

    private:
        void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
        void updateUI();
        void publishMarkers(bool is_published, std::string task_id = "");
        void clearPendingMarkers();

        QComboBox *task_type_dropdown_;
        QLabel *req_label_;
        QLabel *status_label_;
        QPushButton *publish_btn_;
        QPushButton *clear_btn_;

        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
        rclcpp::Publisher<chroma_interfaces::msg::Task>::SharedPtr task_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

        std::vector<geometry_msgs::msg::Point> clicked_points_;
    };

}

#endif
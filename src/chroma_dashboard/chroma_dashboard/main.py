import rclpy
import threading
from nicegui import ui
from chroma_dashboard.ros_bridge import RosBridge
from chroma_dashboard.pages import overview, status, missions

def main(args=None):
    rclpy.init(args=args)
    
    # Start ROS Node
    ros_node = RosBridge()
    
    # Inject ROS into pages
    missions.ros_driver = ros_node
    
    # pin in background
    t = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
    t.start()
    
    # Start UI
    ui.run(title="CHROMA Dashboard", port=8080, dark=True, reload=False)

    if rclpy.ok():
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#include <rclcpp/rclcpp.hpp>

class DummyPlanner : public rclcpp::Node
{
public:
    DummyPlanner() : Node("rm65_plan_path_server")
    {
        RCLCPP_INFO(this->get_logger(), "Planner stub running. No custom service.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyPlanner>());
    rclcpp::shutdown();
    return 0;
}

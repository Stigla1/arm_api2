#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif


static const rclcpp::Logger LOGGER = rclcpp::get_logger("add_collision_object");

class ObjectNode
{
public:
  ObjectNode(const rclcpp::NodeOptions& options);
  void setupPlanningScene();

private:
  rclcpp::Node::SharedPtr node_;
};

ObjectNode::ObjectNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("add_collision_object_node", options) }
{
}

void ObjectNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "floor";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = {2, 2, 0.01};

  geometry_msgs::msg::Pose pose;
  pose.position.z = -0.0055;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto mtc_task_node = std::make_shared<ObjectNode>(options);
  mtc_task_node->setupPlanningScene();
  return 0;
}

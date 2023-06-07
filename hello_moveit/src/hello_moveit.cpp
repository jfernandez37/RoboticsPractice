#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
# include <moveit/planning_scene_interface/planning_scene_interface.h>
int main(int argc,char *argv[]){
    rclcpp::init(argc,argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto const logger = rclcpp::get_logger("hello_moveit");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "panda_arm");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success_mp = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);


    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        node, "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
        move_group_interface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    // Create closures for visualization
    auto const draw_title = [&moveit_visual_tools](auto text) {
        auto const text_pose = [] {
            auto msg = Eigen::Isometry3d::Identity();
            msg.translation().z() = 1.0;  // Place text 1m above the base link
            return msg;
        }();
        moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
    };
    auto const prompt = [&moveit_visual_tools](auto text) {
        moveit_visual_tools.prompt(text);
    };
    auto const draw_trajectory_tool_path =
        [&moveit_visual_tools,
        jmg = move_group_interface.getRobotModel()->getJointModelGroup(
            "panda_arm")](auto const trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
    };



    auto const target_pose=[]{
	    geometry_msgs::msg::Pose msg;
	    msg.orientation.w = 1.0;
	    msg.position.x = 0.3;
	    msg.position.y = 0.4;
	    msg.position.z = 0.75;
	    return msg;
    }();
    
    move_group_interface.setPoseTarget(target_pose);
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()]{
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "box1";
        shape_msgs::msg::SolidPrimitive primitive;

        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.5;
        primitive.dimensions[primitive.BOX_Y] = 0.1;
        primitive.dimensions[primitive.BOX_Z] = 0.55;

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1;
        /*box_pose.orientation.x=1; // rotation of the object in the x direction
        box_pose.orientation.y=1; // rotation of the object in the y direction
        box_pose.orientation.z=1; // rotation of the object in the z direction*/
        box_pose.position.x = 0.2; // position of the object in the x direction
        box_pose.position.y = 0.2; // position of the object in the y direction
        box_pose.position.z = 0.275; // position of the object in the z direction

        collision_object.primitives.push_back(primitive); // adds the size of the collision box to the object
        collision_object.primitive_poses.push_back(box_pose); // adds the orientation and the position of the collision box to the object
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();
    auto const collision_object_2 = [frame_id = move_group_interface.getPlanningFrame()]{
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "box2";
        shape_msgs::msg::SolidPrimitive primitive;

        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.5;
        primitive.dimensions[primitive.BOX_Y] = 0.1;
        primitive.dimensions[primitive.BOX_Z] = 0.3;

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1;
        /*box_pose.orientation.x=5; // rotation of the object in the x direction
        box_pose.orientation.y=0.6; // rotation of the object in the y direction
        box_pose.orientation.z=-3; // rotation of the object in the z direction*/
        box_pose.position.x = 0.2; // position of the object in the x direction
        box_pose.position.y = -0.2; // position of the object in the y direction
        box_pose.position.z = 0.15; // position of the object in the z direction

        collision_object.primitives.push_back(primitive); // adds the size of the collision box to the object
        collision_object.primitive_poses.push_back(box_pose); // adds the orientation and the position of the collision box to the object
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();
    collision_objects.push_back(collision_object);
    collision_objects.push_back(collision_object_2);

    shape_msgs::msg::SolidPrimitive primitive;
    moveit_msgs::msg::CollisionObject object_to_attach; // This object will attach directly to the robot
    object_to_attach.id = "cylinder";

    shape_msgs::msg::SolidPrimitive cylinder_primitive; //The object which attaches will be a cylinder
    cylinder_primitive.type = primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2); //how many dimensions for the object (cylinder has height and radius, so 2)
    cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.30;//dimensions of the cylinder
    cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.1;


    object_to_attach.header.frame_id = move_group_interface.getEndEffectorLink();
    geometry_msgs::msg::Pose grab_pose;
    grab_pose.orientation.w=1.0;
    grab_pose.position.z=0.25;

    object_to_attach.primitives.push_back(cylinder_primitive);
    object_to_attach.primitive_poses.push_back(grab_pose);
    object_to_attach.operation = object_to_attach.ADD;
    collision_objects.push_back(object_to_attach); //adds the cylinder to the collision objects

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObjects(collision_objects);//adds multiple collision objects
    
    RCLCPP_INFO(logger, "Attach the object to the robot");
    std::vector<std::string> touch_links;
    touch_links.push_back("panda_rightfinger");
    touch_links.push_back("panda_leftfinger");
    move_group_interface.attachObject(object_to_attach.id,"panda_hand",touch_links);
    
    prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
    draw_title("Planning");
    move_group_interface.setStartStateToCurrentState();
    RCLCPP_INFO(logger, "Visualizing plan 7 (move around cuboid with cylinder) %s", success_mp ? "" : "FAILED");
    moveit_visual_tools.trigger();
    moveit_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");
    moveit_visual_tools.trigger();
    auto const [success, plan] = [&move_group_interface]{
	    moveit::planning_interface::MoveGroupInterface::Plan msg;
	    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
	    return std::make_pair(ok,msg);
    }();
    if(success){
        draw_trajectory_tool_path(plan.trajectory_);
        moveit_visual_tools.trigger();
        prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
        draw_title("Executing");
        moveit_visual_tools.trigger();
	    move_group_interface.execute(plan);
    }else{
        draw_title("Planning Failed!");
        moveit_visual_tools.trigger();
	    RCLCPP_ERROR(logger,"Planning failed !");
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}

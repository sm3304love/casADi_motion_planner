

#include <nmpc_motion_planner/nmpc_prob.hpp>

class MotionPlanner
{
  public:
    MotionPlanner()
    {
        joint_state_sub = nh_.subscribe("/ur20/joint_states", 100, &MotionPlanner::joint_states_callback, this);
        target_state_sub = nh_.subscribe("/gazebo/model_states", 100, &MotionPlanner::target_states_callback, this);
        joint_vel_command_pub = nh_.advertise<std_msgs::Float64MultiArray>("/ur20/ur20_joint_controller/command", 100);
        input_pub = nh_.advertise<std_msgs::Float64MultiArray>("/input", 100);

        q << 0.0, -1.0, 1.0, 0.0, 0.0, 0.0; // Initial joint position
        x << q, Eigen::VectorXd::Zero(6);
        joint_pose_lower_limit << -6.283185307179586, -6.283185307179586, -3.141592653589793, -6.283185307179586,
            -6.283185307179586, -6.283185307179586;
        joint_pose_upper_limit << 6.283185307179586, 6.283185307179586, 3.141592653589793, 6.283185307179586,
            6.283185307179586, 6.283185307179586;
        joint_vel_lower_limit << -2.0943951023931953, -2.0943951023931953, -2.6179938779914944, -3.6651914291880923,
            -3.6651914291880923, -3.6651914291880923;
        joint_vel_upper_limit << 2.0943951023931953, 2.0943951023931953, 2.6179938779914944, 3.6651914291880923,
            3.6651914291880923, 3.6651914291880923;
    }

    void joint_states_callback(const sensor_msgs::JointState::ConstPtr &JointState) // FIXED
    {

        q[0] = JointState->position[2];
        q[1] = JointState->position[1];
        q[2] = JointState->position[0];
        q[3] = JointState->position[3];
        q[4] = JointState->position[4];
        q[5] = JointState->position[5];
        q_dot[0] = JointState->velocity[2];
        q_dot[1] = JointState->velocity[1];
        q_dot[2] = JointState->velocity[0];
        q_dot[3] = JointState->velocity[3];
        q_dot[4] = JointState->velocity[4];
        q_dot[5] = JointState->velocity[5];

        x << q, q_dot;
    }

    void target_states_callback(const gazebo_msgs::ModelStates::ConstPtr &ModelState)
    {
        for (int i = 0; i < ModelState->name.size(); i++)
        {

            if (ModelState->name[i] == "target")
            {

                target_pose.position.x = ModelState->pose[i].position.x;
                target_pose.position.y = ModelState->pose[i].position.y;
                target_pose.position.z = ModelState->pose[i].position.z;
                target_pose.orientation = ModelState->pose[i].orientation;
            }
        }
        position_ref << target_pose.position.x, target_pose.position.y, target_pose.position.z;
        orientation_ref.w() = target_pose.orientation.w;
        orientation_ref.x() = target_pose.orientation.x;
        orientation_ref.y() = target_pose.orientation.y;
        orientation_ref.z() = target_pose.orientation.z;
    }

    void run()
    {
        using namespace casadi_mpc_template;

        auto prob = std::make_shared<MotionPlanningProb>(Problem::DynamicsType::ContinuesRK4, 12, 6, 10, dt);

        Eigen::VectorXd u_lb = (Eigen::VectorXd(6) << -5.0, -5.0, -5.0, -5.0, -5.0, -5.0).finished();
        Eigen::VectorXd u_Ub = (Eigen::VectorXd(6) << 5.0, 5.0, 5.0, 5.0, 5.0, 5.0).finished();
        Eigen::VectorXd x_lb = (Eigen::VectorXd(12) << joint_pose_lower_limit, joint_vel_lower_limit).finished();
        Eigen::VectorXd x_Ub = (Eigen::VectorXd(12) << joint_pose_upper_limit, joint_vel_upper_limit).finished();

        prob->set_input_bound(u_lb, u_Ub);
        prob->set_state_bound(x_lb, x_Ub);

        auto t_all_start = std::chrono::system_clock::now();

        while (ros::ok())
        {

            prob->x_quat_ref = {orientation_ref.w(), orientation_ref.x(), orientation_ref.y(), orientation_ref.z()};
            prob->x_pose_ref = {position_ref(0), position_ref(1), position_ref(2)};

            MPC mpc(prob);

            auto t_start = std::chrono::system_clock::now();

            // Solve for optimal input using MPC
            Eigen::VectorXd u = mpc.solve(x);

            Eigen::VectorXd x_sim = prob->discretized_dynamics(dt, x, u);

            q_dot_desired += u * dt;

            auto t_end = std::chrono::system_clock::now();

            double solve_time = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count() * 1e-6;
            std::cout << "Solve time: " << solve_time << std::endl;

            std::cout << "state: " << std::endl << x.transpose() << std::endl;
            std::cout << "input: " << std::endl << u.transpose() << std::endl;
            std::cout << "velocity: " << std::endl << q_dot_desired.transpose() << std::endl;
            // std::cout << "x_sim: " << std::endl << x_sim.transpose() << std::endl;

            std_msgs::Float64MultiArray joint_vel_command;
            for (int i = 0; i < q_dot_desired.size(); ++i)
            {
                joint_vel_command.data.push_back(q_dot_desired(i));
            }

            joint_vel_command_pub.publish(joint_vel_command);

            std_msgs::Float64MultiArray input;
            for (int i = 0; i < u.size(); ++i)
            {
                input.data.push_back(u(i));
            }

            input_pub.publish(input);

            ros::spinOnce();
            // loop_rate.sleep();
        }
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_sub;
    ros::Subscriber target_state_sub;
    ros::Publisher joint_vel_command_pub;
    ros::Publisher input_pub;

    geometry_msgs::Pose target_pose;

    const double dt = 0.01;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd q_dot_desired = Eigen::VectorXd::Zero(6);

    Eigen::VectorXd joint_pose_lower_limit = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd joint_pose_upper_limit = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd joint_vel_lower_limit = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd joint_vel_upper_limit = Eigen::VectorXd::Zero(6);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(12);

    Eigen::VectorXd position_ref = Eigen::VectorXd::Zero(3);
    Eigen::Quaterniond orientation_ref = Eigen::Quaterniond::Identity();
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_NMPC_motion_planner");
    MotionPlanner MotionPlanner;
    MotionPlanner.run();
    return 0;
}

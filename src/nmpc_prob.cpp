#include <nmpc_motion_planner/nmpc_prob.hpp>

using namespace casadi_mpc_template;

MotionPlanningProb::MotionPlanningProb(DynamicsType dynamics_type, int state_dim, int control_dim, int horizon_length,
                                       double dt)
    : Problem(dynamics_type, state_dim, control_dim, horizon_length, dt)
{
    using namespace casadi;
    Q_trans = DM::diag({800.0, 800.0, 800.0});
    Q_ori = DM::diag({500.0, 500.0, 500.0});
    Q_vel = DM::diag({10, 10, 10, 10, 10, 10});
    R = DM::diag({0.01, 0.01, 0.01, 0.01, 0.01, 0.01});
}

casadi::MX MotionPlanningProb::dynamics(casadi::MX x, casadi::MX u)
{
    using namespace casadi;

    MX A_c = MX::zeros(12, 12);
    A_c(Slice(0, 6), Slice(6, 12)) = MX::eye(6);

    MX B_c = MX::zeros(12, 6);
    B_c(Slice(6, 12), Slice(0, 6)) = MX::eye(6);

    MX x_dot = mtimes(A_c, x) + mtimes(B_c, u);

    return x_dot;
}

Eigen::VectorXd MotionPlanningProb::discretized_dynamics(double dt, Eigen::VectorXd x, Eigen::VectorXd u)
{
    auto dynamics = [&](Eigen::VectorXd x, Eigen::VectorXd u) -> Eigen::VectorXd {
        using namespace casadi;

        Eigen::MatrixXd A_c = Eigen::MatrixXd::Zero(12, 12);
        A_c.block<6, 6>(0, 6) = Eigen::MatrixXd::Identity(6, 6);

        Eigen::MatrixXd B_c = Eigen::MatrixXd::Zero(12, 6);
        B_c.block<6, 6>(6, 0) = Eigen::MatrixXd::Identity(6, 6);

        std::vector<double> x_std(x.data(), x.data() + x.size());

        // Calculate the derivative of the state
        Eigen::VectorXd x_dot = A_c * x + B_c * u;

        return x_dot;
    };

    return casadi_mpc_template::integrate_dynamics_rk4<Eigen::VectorXd>(dt, x, u, dynamics);
}

casadi::MX MotionPlanningProb::forward_kinematics(casadi::MX q)
{
    using namespace casadi;
    MX T = MX::eye(4);

    // DH parameters
    std::vector<double> d = {0.2363, 0, 0, 0.2010, 0.1593, 0.1543};
    std::vector<double> r = {0, -0.8620, -0.7287, 0, 0, 0};
    std::vector<double> alpha = {M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0};

    for (int i = 0; i < 6; i++)
    {
        MX theta = q(i); // Update theta with the value from q

        MX A = MX::zeros(4, 4);
        A(0, 0) = cos(theta);
        A(0, 1) = -sin(theta) * cos(alpha[i]);
        A(0, 2) = sin(theta) * sin(alpha[i]);
        A(0, 3) = r[i] * cos(theta);
        A(1, 0) = sin(theta);
        A(1, 1) = cos(theta) * cos(alpha[i]);
        A(1, 2) = -cos(theta) * sin(alpha[i]);
        A(1, 3) = r[i] * sin(theta);
        A(2, 1) = sin(alpha[i]);
        A(2, 2) = cos(alpha[i]);
        A(2, 3) = d[i];
        A(3, 3) = 1;

        T = mtimes(T, A);
    }

    return T;
}

casadi::MX MotionPlanningProb::compute_trans_error(casadi::MX x_pose)
{
    return x_pose - x_pose_ref;
}

casadi::MX MotionPlanningProb::compute_ori_error(casadi::MX x_quat)
{
    using namespace casadi;
    MX x_quat_ref_inv = MX::vertcat({x_quat_ref(0), -x_quat_ref(1), -x_quat_ref(2), -x_quat_ref(3)});

    MX e_ori_temp = MX::vertcat({x_quat(0) * x_quat_ref_inv(0) - x_quat(1) * x_quat_ref_inv(1) -
                                     x_quat(2) * x_quat_ref_inv(2) - x_quat(3) * x_quat_ref_inv(3),
                                 x_quat(0) * x_quat_ref_inv(1) + x_quat(1) * x_quat_ref_inv(0) +
                                     x_quat(2) * x_quat_ref_inv(3) - x_quat(3) * x_quat_ref_inv(2),
                                 x_quat(0) * x_quat_ref_inv(2) - x_quat(1) * x_quat_ref_inv(3) +
                                     x_quat(2) * x_quat_ref_inv(0) + x_quat(3) * x_quat_ref_inv(1),
                                 x_quat(0) * x_quat_ref_inv(3) + x_quat(1) * x_quat_ref_inv(2) -
                                     x_quat(2) * x_quat_ref_inv(1) + x_quat(3) * x_quat_ref_inv(0)});
    return e_ori_temp(Slice(1, 4));
}

casadi::MX MotionPlanningProb::stage_cost(casadi::MX x, casadi::MX u)
{
    using namespace casadi;
    MX L = 0;

    auto q = x(Slice(0, 6));
    auto T = forward_kinematics(q);

    x_pose = T(Slice(0, 3), 3);

    MX Rot = T(Slice(0, 3), Slice(0, 3));
    MX trace = Rot(0, 0) + Rot(1, 1) + Rot(2, 2);
    MX q0 = MX::sqrt(trace + 1) / 2;
    MX q1 = (Rot(2, 1) - Rot(1, 2)) / (4 * q0);
    MX q2 = (Rot(0, 2) - Rot(2, 0)) / (4 * q0);
    MX q3 = (Rot(1, 0) - Rot(0, 1)) / (4 * q0);

    MX x_quat = MX::vertcat({q0, q1, q2, q3});

    auto e_ori = compute_ori_error(x_quat);

    auto e_trans = compute_trans_error(x_pose);

    auto q_dot = x(Slice(6, 12));

    L += 0.5 * mtimes(e_trans.T(), mtimes(Q_trans, e_trans));
    L += 0.5 * mtimes(e_ori.T(), mtimes(Q_ori, e_ori));
    L += 0.5 * mtimes(q_dot.T(), mtimes(Q_vel, q_dot));
    L += 0.5 * mtimes(u.T(), mtimes(R, u));

    return dt() * L;
}

// casadi::MX MotionPlanningProb::terminal_cost(casadi::MX x)
// {
//     using namespace casadi;
//     auto e = x - x_ref;
//     return 0.5 * mtimes(e.T(), mtimes(Qf, e));
// }
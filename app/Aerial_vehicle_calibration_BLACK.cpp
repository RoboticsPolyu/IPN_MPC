#include "calibration/Calibration_factor.h"
#include "quadrotor_simulator/Dynamics_params.h"

#include <yaml-cpp/yaml.h>

using namespace gtsam;
using namespace std;
using namespace UAVFactor;


using symbol_shorthand::K; // kf
using symbol_shorthand::J; // inertial of moments
using symbol_shorthand::M; // km
using symbol_shorthand::P; // position of rotor
using symbol_shorthand::R; // rotation of gravity
using symbol_shorthand::S; // angular speed
using symbol_shorthand::T; // thrust and moments
using symbol_shorthand::U; // actuator_input
using symbol_shorthand::V; // velocity
using symbol_shorthand::X; // pose
using symbol_shorthand::D; // drag
using symbol_shorthand::H; // Pose_B_M
using symbol_shorthand::A; 
using symbol_shorthand::B;


typedef struct State
{
    int            id;
    double         timestamp;
    gtsam::Pose3   pose;
    gtsam::Vector3 vel;
    gtsam::Vector3 body_rate;
    gtsam::Vector4 actuator_output;

} State;

typedef struct Quad_pwm
{
    int            id;
    double         timestamp;
    gtsam::Vector4 actuator_output;

} Quad_pwm;

// Pose slerp interpolation
Pose3 interpolateRt(const::Pose3& T_l, const Pose3& T, double t) 
{
    return Pose3(gtsam::interpolate<Rot3>(T_l.rotation(), T.rotation(), t), 
        gtsam::interpolate<Point3>(T_l.translation(), T.translation(), t));
}

int main(void)
{        
    DynamicsParams quad_params;
    double      dt         = 0.01f;
    quad_params.mass       = 0.915f;
    quad_params.Ixx        = 4.9* 1e-2;
    quad_params.Iyy        = 4.9* 1e-2;
    quad_params.Izz        = 6.9* 1e-2;
    quad_params.k_f        = 2.27* 10e-8 + 1.5* 10e-8; 
    quad_params.k_m        = 0;
    quad_params.arm_length = 0.13f;

    // Configuration file 
    YAML::Node CAL_config = YAML::LoadFile("../config/dynamics_BLACK.yaml");  
    double     rotor_px   = CAL_config["ROTOR_P_X"].as<double>();
    double     rotor_py   = CAL_config["ROTOR_P_Y"].as<double>();
    uint16_t DATASET_S    = CAL_config["DATASET_S"].as<uint16_t>();
    uint16_t DATASET_LENS = CAL_config["DATASET_L"].as<uint16_t>();
    double p_thrust_sigma = CAL_config["P_T_SIGMA"].as<double>();
    double unmodel_thrust_sigma = CAL_config["U_T_SIGMA"].as<double>();
    std::string file_path = CAL_config["ROOT_PATH"].as<std::string>();
    bool enable_drag      = CAL_config["ENABLE_DRAG"].as<bool>();
    bool enable_inertia   = CAL_config["ENABLE_I"].as<bool>();
    bool ENABLE_COG       = CAL_config["ENABLE_COG"].as<bool>();
    bool ENABLE_VISCOUS   = CAL_config["ENABLE_VISCOUS"].as<bool>();
    quad_params.Ixx       = CAL_config["INERTIA_IX"].as<double>();
    quad_params.Iyy       = CAL_config["INERTIA_IY"].as<double>();
    quad_params.Izz       = CAL_config["INERTIA_IZ"].as<double>();

    gtsam::Vector3 thrust_sigma(unmodel_thrust_sigma, unmodel_thrust_sigma, 2* p_thrust_sigma);
    // sigma = (rotor_p_x* 2* P_thrust_single_rotor, rotor_p_x* 2* P_thrust_single_rotor, k_m * 2 * P_thrust_single_rotor) 
    gtsam::Vector3 moments_sigma(p_thrust_sigma * 2 * rotor_py, p_thrust_sigma * 2 * rotor_px, 0.01f * 2 * p_thrust_sigma);

    auto dyn_noise   = noiseModel::Diagonal::Sigmas((Vector(12) << thrust_sigma * 0.5f * dt * dt, Vector3::Constant(0.001 * dt), 
                                                                   thrust_sigma * dt, moments_sigma * dt).finished());
    auto vicon_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.0005), Vector3::Constant(0.001)).finished());

    auto im_noise    = noiseModel::Diagonal::Sigmas(Vector3(0.000001, 0.000001, 0.000001));
    auto rp_noise    = noiseModel::Diagonal::Sigmas(Vector3(0.00001, 0.00001, 0.00001));
    auto gr_noise    = noiseModel::Diagonal::Sigmas(Vector3(0.0001, 0.0001, 0.0001));
    auto km_noise    = noiseModel::Diagonal::Sigmas(Vector1(0.00001));
    auto bm_nosie    = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.00001), Vector3::Constant(0.00001)).finished());

    std::ofstream calib_log;
    std::string file_name = "../data/calib_";
    file_name.append("debug");
    file_name.append("_log.txt");
    calib_log.open(file_name);
    

    std::vector<State>   Interp_states;
    std::vector<State>   Uav_states;
    std::vector<Quad_pwm> Uav_pwms;


    std::ifstream state_file;
    std::string state_file_path = file_path + std::string("/state_black.txt");
    state_file.open(state_file_path);
    std::ifstream actuator_black_file;
    std::string actuator_file_path = file_path + std::string("/interp_actuator_black.txt");
    actuator_black_file.open(actuator_file_path);
    std::ifstream rpm_black_file;
    std::string rpm_file_path = file_path + std::string("/rpm_black.txt");
    rpm_black_file.open(rpm_file_path);

    double gt_t, gt_qw, gt_qx, gt_qy, gt_qz, gt_x, gt_y, gt_z, pwm_t, pwm1, pwm2, pwm3, pwm4;

    while (state_file >> gt_t >> gt_x >> gt_y >> gt_z >> gt_qw >> gt_qx >> gt_qy >> gt_qz)
    {
        State _state;
        _state.timestamp   = gt_t;
        _state.pose        = gtsam::Pose3(gtsam::Quaternion(gt_qw, gt_qx, gt_qy, gt_qz), gtsam::Vector3(gt_x, gt_y, gt_z));
        gtsam::Rot3 rx_pi  = gtsam::Rot3::Rx(M_PI);
        gtsam::Rot3 rz_pi4 = gtsam::Rot3::Rz(-0/4.0);

        _state.pose        = gtsam::Pose3(rx_pi, gtsam::Vector3(0,0,0)) * _state.pose * gtsam::Pose3(rx_pi, gtsam::Vector3(0,0,0)) * gtsam::Pose3(rz_pi4, gtsam::Vector3(0,0,0));
        Uav_states.push_back(_state);
    }

    // while (rpm_black_file >> pwm_t >> pwm1 >> pwm2 >> pwm3 >> pwm4)
    while (rpm_black_file >> pwm_t >> pwm1 >> pwm2 >> pwm3 >> pwm4)
    {
        Quad_pwm _uav_pwm;
        _uav_pwm.timestamp       = pwm_t;
        _uav_pwm.actuator_output = gtsam::Vector4(pwm1, pwm2, pwm3, pwm4);
        Uav_pwms.push_back(_uav_pwm);
    }

    // while (actuator_black_file >> pwm_t >> pwm1 >> pwm2 >> pwm3 >> pwm4)
    // {
    //     Quad_pwm _uav_pwm;
    //     _uav_pwm.timestamp      = pwm_t;
    //     _uav_pwm.actuator_output = gtsam::Vector4(pwm1, pwm2, pwm3, pwm4);
    //     Uav_pwms.push_back(_uav_pwm);
    // }

    State _interp_state;
    for(int i = 0; i < Uav_pwms.size(); i++)
    {
        for(int j = 0; j < Uav_states.size() -1; j++)
        {
            if(Uav_pwms[i].timestamp >= Uav_states[j].timestamp && Uav_pwms[i].timestamp < Uav_states[j+1].timestamp)
            {
                double t = (Uav_pwms[i].timestamp - Uav_states[j].timestamp) / (Uav_states[j+1].timestamp - Uav_states[j].timestamp);

                gtsam::Pose3 interp_pose = interpolateRt(Uav_states[j].pose, Uav_states[j+1].pose, t);
                
                _interp_state.actuator_output = Uav_pwms[i].actuator_output;
                _interp_state.timestamp       = Uav_pwms[i].timestamp;
                _interp_state.pose            = interp_pose;

                gtsam::Vector3 vel_l   = (Uav_states.at(j+1).pose.translation() - Uav_states.at(j-1).pose.translation())/dt/2.0;
                gtsam::Vector3 vel_r   = (Uav_states.at(j+2).pose.translation() - Uav_states.at(j).pose.translation())/dt/2.0;

                gtsam::Vector3 omega_l = gtsam::Rot3::Logmap(Uav_states.at(j-1).pose.rotation().between(Uav_states.at(j+1).pose.rotation()))/2.0 /dt;
                gtsam::Vector3 omega_r = gtsam::Rot3::Logmap(Uav_states.at(j).pose.rotation().between(Uav_states.at(j+2).pose.rotation()))/2.0 /dt;

                _interp_state.vel      = (1-t)* vel_l + t* vel_r;
                _interp_state.body_rate    = (1-t)* omega_l + t* omega_r;

                Interp_states.push_back(_interp_state);
            }
        }
    }


    NonlinearFactorGraph dyn_factor_graph;
    Values initial_value_dyn;
    dyn_factor_graph.empty();

    for(uint32_t idx = DATASET_S; idx < DATASET_LENS; idx++)
    {   
        gtsam::Vector3 vel;
        gtsam::Vector3 omega;

        dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx), Interp_states.at(idx).pose, vicon_noise));

        initial_value_dyn.insert(X(idx), Interp_states.at(idx).pose);
        initial_value_dyn.insert(V(idx), Interp_states.at(idx).vel);
        initial_value_dyn.insert(S(idx), Interp_states.at(idx).body_rate);

        if( idx == DATASET_LENS-1)
        {
            dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx+1), Interp_states.at(idx+1).pose,  vicon_noise));

            initial_value_dyn.insert(X(idx+1), Interp_states.at(idx+1).pose);
            initial_value_dyn.insert(V(idx+1), Interp_states.at(idx+1).vel);
            initial_value_dyn.insert(S(idx+1), Interp_states.at(idx+1).body_rate);
        }
        

        // pose, velocity, angular speed, pose, velocity, angular speed, inertial of moments, rot of g, position of rotot, kf, km        
        DynamcisCaliFactor_RS_AB dynamicsCalibFactor(X(idx), V(idx), S(idx), X(idx + 1), V(idx + 1), S(idx + 1), J(0), R(0), P(0), K(0), M(0), H(0), D(0), A(0), B(0), Interp_states.at(idx).actuator_output, dt, quad_params.mass, dyn_noise);
        dyn_factor_graph.add(dynamicsCalibFactor);
    }

    initial_value_dyn.insert(J(0), gtsam::Vector3(quad_params.Ixx, quad_params.Iyy, quad_params.Izz) );
    initial_value_dyn.insert(R(0), gtsam::Rot3::Ry(-3.0/180.0*M_PI)* gtsam::Rot3::Rx(5.0/180.0*M_PI));
    initial_value_dyn.insert(K(0), quad_params.k_f);
    initial_value_dyn.insert(M(0), quad_params.k_m);
    initial_value_dyn.insert(H(0), gtsam::Pose3::identity());
    gtsam::Vector3 rotor_p(rotor_px, rotor_py, 0);
    initial_value_dyn.insert(P(0), rotor_p);
    gtsam::Vector3 drag_k(-0.1f, -0.1f, -0.1f);
    gtsam::Vector3 A_k(0.000f, 0.000f, 0.000f);
    gtsam::Vector3 B_k(0.000f, 0.000f, 0.000f);
    initial_value_dyn.insert(D(0), drag_k);
    initial_value_dyn.insert(A(0), A_k);
    initial_value_dyn.insert(B(0), B_k);

    if(!enable_inertia)
    {
        dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(J(0), gtsam::Vector3(quad_params.Ixx, quad_params.Iyy, quad_params.Izz), im_noise));
    }

    if(!ENABLE_COG)
    {
        dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(A(0), gtsam::Vector3::Zero(), im_noise)); 
    }

    if(!ENABLE_VISCOUS)   
    {
        dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(B(0), gtsam::Vector3::Zero(), im_noise)); 
    }

    dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(P(0), rotor_p, rp_noise)); 

    if(!enable_drag)
    {
        dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(D(0), gtsam::Vector3::Zero(), im_noise)); 
    }
    

    gtsam::LevenbergMarquardtParams parameters;
    parameters.absoluteErrorTol = 1e-8;
    parameters.relativeErrorTol = 1e-8;
    parameters.maxIterations    = 500;
    parameters.verbosity        = gtsam::NonlinearOptimizerParams::ERROR;
    parameters.verbosityLM      = gtsam::LevenbergMarquardtParams::SUMMARY;
    std::cout << "###################### init contoller optimizer ######################" << std::endl;
    LevenbergMarquardtOptimizer optimizer(dyn_factor_graph, initial_value_dyn, parameters);
    std::cout << "###################### begin optimize ######################" << std::endl;
    Values result = optimizer.optimize();
    

    gtsam::Vector3  IM = result.at<gtsam::Vector3>(J(0));
    gtsam::Rot3    rot = result.at<gtsam::Rot3>(R(0));
    gtsam::Vector3   p = result.at<gtsam::Vector3>(P(0));
    double          kf = result.at<double>(K(0));
    double          km = result.at<double>(M(0)); 
    gtsam::Pose3   bTm = result.at<gtsam::Pose3>(H(0)); 
    gtsam::Vector3  dk = result.at<gtsam::Vector3>(D(0));
    gtsam::Vector3  ak = result.at<gtsam::Vector3>(A(0));
    gtsam::Vector3  bk = result.at<gtsam::Vector3>(B(0));


    for(uint32_t idx = DATASET_S; idx < DATASET_LENS; idx++)
    {
        gtsam::Pose3    pi = result.at<gtsam::Pose3>(X(idx));
        gtsam::Vector3  vi = result.at<gtsam::Vector3>(V(idx));
        gtsam::Vector3  vj = result.at<gtsam::Vector3>(V(idx+1));
        gtsam::Vector3  oi = result.at<gtsam::Vector3>(S(idx));
        gtsam::Vector3  oj = result.at<gtsam::Vector3>(S(idx+1));

        DynamcisCaliFactor_RS_AB dyn_err(X(idx), V(idx), S(idx), X(idx + 1), V(idx + 1), S(idx + 1), J(0), R(0), P(0), K(0), M(0), H(0), D(0), A(0), B(0), Interp_states.at(idx).actuator_output, dt, quad_params.mass, dyn_noise);
        
        gtsam::Vector12 dyn_e = dyn_err.evaluateError(Interp_states.at(idx).pose, vi, oi, Interp_states.at(idx+1).pose, vj, oj, IM, rot, p, kf, km, bTm, dk, ak, bk);
        gtsam::Vector4 rpm_square = Interp_states.at(idx).actuator_output.cwiseAbs2();
        gtsam::Vector6 thrust_torque = dyn_err.Thrust_Torque(rpm_square, kf, km, p);

        gtsam::Matrix3 drag_matrix;
        drag_matrix.setZero();
        drag_matrix.diagonal() << dk;  

        gtsam::Vector3 drag_force = quad_params.mass * drag_matrix * pi.rotation().unrotate(vi);

        gtsam::Pose3 pose = result.at<gtsam::Pose3>(X(idx));
        gtsam::Vector3 dyn_pos_err = pose.rotation() * gtsam::Vector3(dyn_e(0), dyn_e(1), dyn_e(2)) / quad_params.mass;

        gtsam::Vector3 vel_body = pose.rotation().unrotate(vi);

        float hor_thrust = ak.z() * (vi.x()* vi.x() + vi.y()* vi.y());

        // calib_log << std::setprecision(19) << Interp_states.at(idx).timestamp << std::setprecision(5) 
        //     << " " << dyn_pos_err(0) << " " << dyn_pos_err(1) << " " << dyn_pos_err(2) << " " << dyn_e(3) << " " << dyn_e(4) << " " << dyn_e(5) << " " << dyn_e(6) << " " << dyn_e(7) << " " << dyn_e(8) << " " << dyn_e(9) << " " << dyn_e(10) << " " << dyn_e(11) 
        //     << " " << thrust_torque(0)<< " " << thrust_torque(1) << " " << thrust_torque(2) << " " << thrust_torque(3) << " " << thrust_torque(4) << " " << thrust_torque(5) 
        //     << " " << vel_body(0) << " " << vel_body(1) << " " << vel_body(2) << " " << oi(0) << " " << oi(1) << " " << oi(2) << " " << hor_thrust << std::endl; // << " " << Interp_states.at(idx).actuator_output(0) << " " << Interp_states.at(idx).actuator_output(1) << " " << Interp_states.at(idx).actuator_output(2) << " " << Interp_states.at(idx).actuator_output(3) << " " << pi.translation().x() << " " << pi.translation().y() << " " << pi.translation().z() << " " << pi.rotation().xyz().x() << " " << pi.rotation().xyz().y() << " " << pi.rotation().xyz().z() << "\n";

        calib_log << std::setprecision(19) << Interp_states.at(idx).timestamp << std::setprecision(5) 
        << " " << dyn_pos_err(0) << " " << dyn_pos_err(1) << " " << dyn_pos_err(2) << " " << dyn_e(3) << " " << dyn_e(4) << " " << dyn_e(5) << " " << dyn_e(6) 
        << " " << dyn_e(7) << " " << dyn_e(8) << " " << dyn_e(9) << " " << dyn_e(10) << " " << dyn_e(11) 
        << " " << thrust_torque(0)<< " " << thrust_torque(1) << " " << thrust_torque(2) << " " << thrust_torque(3) << " " << thrust_torque(4) << " " << thrust_torque(5) 
        << " " << vel_body(0) << " " << vel_body(1) << " " << vel_body(2) 
        << " " << Interp_states.at(idx).body_rate.x() << " " << Interp_states.at(idx).body_rate.y() << " " << Interp_states.at(idx).body_rate.z() 
        << " " << drag_force(0) << " " << drag_force(1) << " " << drag_force(2) << " " << hor_thrust << std::endl;
    }


    std::cout << "Inertial of moment:" << IM.transpose() << "\n";
    std::cout << "Gravity Rot:     " << rot.rpy().transpose() << "\n";
    std::cout << "Rotor p:         " << p.transpose() << "\n";
    std::cout << "Kf:              " << kf << "\n";
    std::cout << "Km:              " << km << std::endl;
    std::cout << "bTm t:         " << bTm.translation().transpose() << "\n";
    std::cout << "bTm r:          " << bTm.rotation().rpy().transpose() << "\n";
    std::cout << "drag k:         " << dk.transpose() << "\n";
    std::cout << "A k:         " << ak.transpose() << "\n";
    std::cout << "B k:         " << bk.transpose() << "\n";

    return 0;
}

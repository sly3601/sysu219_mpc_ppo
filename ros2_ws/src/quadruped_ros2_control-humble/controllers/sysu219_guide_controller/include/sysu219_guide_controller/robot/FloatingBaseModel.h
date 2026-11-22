#pragma once

#include <Eigen/Dense>
#include <limits>
#include <array>
#include <string>
#include <vector>

namespace sysu219::wbc {

using Vec3 = Eigen::Vector3d;                       // 单个三维量，如位置或角速度。
using Vec6 = Eigen::Matrix<double, 6, 1>;          // 浮动基座的六个分量。
using Vec12 = Eigen::Matrix<double, 12, 1>;        // 四腿共12个关节分量。
using Vec18 = Eigen::Matrix<double, 18, 1>;        // 基座6维和关节12维的广义量。
using Mat18 = Eigen::Matrix<double, 18, 18>;       // 完整浮动基质量矩阵的尺寸。
using FootJacobian = Eigen::Matrix<double, 3, 18>; // 单足雅可比：输出xyz，输入18维广义量。
using Feet = Eigen::Matrix<double, 3, 4>;          // 每列是一只脚，三行依次是xyz。
using Contact = std::array<int, 4>;               // 四足当前接触状态。
constexpr double kUnset = std::numeric_limits<double>::quiet_NaN();  // kUnset是double类型的常量NaN，表示未设置或无效值。

struct WbcInput {
  Eigen::Matrix3d R_OB = Eigen::Matrix3d::Constant(kUnset);  // 当前机身到世界系的旋转。
  Vec3 Theta = Vec3::Constant(kUnset);  // 当前横滚、俯仰、偏航，单位rad，与R_GB一致。
  Vec3 Theta_d = Vec3::Constant(kUnset);  // 期望机身欧拉角，采用相同ZYX旋转约定。
  // 当前接入以URDF的base_name为基座原点，与估计器、M和J一致。
  // p_com保留原命名，但此处表示基座原点的位置，不是随腿运动变化的整机质心。
  Vec3 p_com_O = Vec3::Constant(kUnset);  // 当前世界系位置，单位m。
  Vec3 p_com_d_O = Vec3::Constant(kUnset);  // 期望世界系位置。
  Vec3 pdot_com_d_O = Vec3::Zero();  // 期望世界系线速度，单位m/s。
  Vec3 omega_d_O = Vec3::Zero();  // 期望世界系角速度，单位rad/s。
  // 保留论文中的加速度前馈入口；论文未规划加速度，因此默认零。
  Vec3 pddot_com_d_O = Vec3::Zero();  // 期望世界系线加速度，单位m/s²。
  Vec3 alpha_d_O = Vec3::Zero();  // 期望世界系角加速度，单位rad/s²。
  Vec12 q_j = Vec12::Constant(kUnset);  // 当前12个关节角，单位rad。
  // 当前实测速度：先机身B系角速度，再机身B系线速度，最后12个关节速度。
  Vec18 qdot = Vec18::Constant(kUnset);
  Contact contact{{1, 1, 1, 1}};  // 1支撑、0摆动，与松弛模块使用同一时刻的接触状态。
  Eigen::Vector4d swing_phase = Eigen::Vector4d::Constant(0.5); // 保留摆动进度接口；髋回零不再依赖相位。
  Feet x_f_O = Feet::Constant(kUnset);  // 当前各足世界系位置，单位m。
  Feet x_f_d_O = Feet::Constant(kUnset);  // 摆动足期望位置，支撑足不使用。
  Feet xdot_f_d_O = Feet::Zero();  // 摆动足期望世界系线速度。
  Feet xddot_f_d_O = Feet::Zero();  // 摆动足加速度前馈，未规划时保持零。
  
  // 四足各自的完整雅可比，映射广义速度到该足世界系线速度。
  // J_F_O，J是雅可比，F是足端，O是世界系，J_F_O是足端在世界系的雅可比矩阵
  std::array<FootJacobian, 4> J_f_O{{
      FootJacobian::Constant(kUnset), FootJacobian::Constant(kUnset),
      FootJacobian::Constant(kUnset), FootJacobian::Constant(kUnset)}};
  // 足端雅可比未覆盖的加速度项，包含运动导致的速度变化。
  // 用空间加速度递推计算时，需补上角速度与线速度的叉乘，再转到世界系。
  Feet Jdot_f_qdot_O = Feet::Constant(kUnset);
  // 保留中文论文的零基座偏置默认，接模型时仍需核对后端约定。
  // 若基座线加速度表示局部速度的导数，线性偏置需填机身角/线速度叉乘的世界系值。
  Vec3 Jdot_orientation_qdot_O = Vec3::Zero();  // 转动任务的加速度偏置。
  Vec3 Jdot_position_qdot_O = Vec3::Zero();   // 平动任务的加速度偏置。
};

// 接触估计和质心辨识共用的 URDF 浮动基模型。
// [接入约定] 基座原点使用base_name，与估计器一致，模型与任务均取此原点。
// 速度顺序为B系角速度、B系线速度、12关节速度；不把估计器位置当作整机质心。
// 包含URDF刚体惯性、关节B/Fc摩擦，以及identified_dynamics中的输出轴额外惯量Ja。
class FloatingBaseModel {
 public:
  FloatingBaseModel(const std::string& urdf, const std::string& base_name,
                    const std::vector<std::string>& feet_names);

  // 读取input的姿态、基座位置、关节位置和实测速度。
  // 输出M含关节Ja，C含重力、科氏/离心和克服当前滑动摩擦所需的力矩。
  // 同时填写四足位置、完整雅可比和加速度偏置；不修改反馈力矩或硬件方向。
  // URDF解析和缓存分配只在构造时进行，update正常路径不分配堆内存。
  void update(WbcInput& input, Mat18& M, Vec18& C);

 private:
  struct JointDynamics {
    double B = 0.0;   // 黏性摩擦，N*m*s/rad，来自joint/dynamics/@damping。
    double Fc = 0.0;  // 库仑摩擦幅值，N*m，来自joint/dynamics/@friction。
    double Ja = 0.0;  // 输出轴额外惯量，kg*m^2，来自joint/identified_dynamics/@Ja。
  };

  struct Link {
    int parent = -1;
    int joint = -1; // -1为固定连接，0～11与FR、FL、RR、RL关节顺序一致。
    Vec3 origin = Vec3::Zero();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Vec3 axis = Vec3::Zero();
    double mass = 0.0;
    Vec3 com = Vec3::Zero();
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero(); // 连杆系下、绕该连杆质心的惯量。
  };

  struct State {
    Vec3 position = Vec3::Zero();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    FootJacobian J_v = FootJacobian::Zero();
    FootJacobian J_w = FootJacobian::Zero();
    Vec3 omega = Vec3::Zero();
    Vec3 acceleration_bias = Vec3::Zero();
    Vec3 angular_acceleration_bias = Vec3::Zero();
  };

  std::vector<Link> links_;       // 父节点始终排在子节点之前，运行时只顺序遍历。
  std::vector<State> states_;     // 长度在初始化时确定，控制周期内不resize。
  std::array<int, 4> feet_{};      // 足端对应的连杆编号。
  std::array<JointDynamics, 12> joint_dynamics_{}; // 初始化按完整关节名映射，不按XML顺序。
};

}  // namespace sysu219::wbc

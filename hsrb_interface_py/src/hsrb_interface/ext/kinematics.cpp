/// @brief 運動学ライブラリのpython wrapper
/// @brief Copyright (C) 2017 TOYOTA MOTOR CORPORATION
#include <string>
#include <vector>
#include <boost/python.hpp>
#include <hsr_kinematics/head_kinematics.hpp>

namespace bp = boost::python;

namespace {
// 頭部Yaw関節の関節名
const char* const kHeadYawJoint = "head_pan_joint";
// 頭部pitch関節の関節名
const char* const kHeadPitchJoint = "head_tilt_joint";
}  // namespace

namespace hsrb_interface_plugin {

/// 運動学インターフェース，任意の座標に視線を向ける頭部関節角度計算のみ実装
class KinematicsInterface {
 public:
  /// コンストラクタ
  /// @param[in] robot_description  ロボットモデル
  explicit KinematicsInterface(const std::string& robot_description) {
    std::vector<std::string> head_joint_names;
    head_joint_names.push_back(kHeadYawJoint);
    head_joint_names.push_back(kHeadPitchJoint);
    head_kinematics_.reset(
        new hsr_kinematics::HsrHeadKinematics(robot_description,
                                              head_joint_names));
  }

  /// 頭部関節角度と台車の旋回量を計算する
  /// @param[in] baselink_to_point ロボット基準の注視点座標, [x, y, z]
  /// @param[in] camera_frame 対象とするカメラのフレーム名
  /// @return boost::python::dict 頭部パン・チルトの関節角度
  ///                             計算に失敗した場合は空の辞書を返す
  bp::dict CalculateAngles(const bp::list& baselink_to_point,
                           const std::string& camera_frame) {
    if (bp::len(baselink_to_point) != 3) {
      throw std::invalid_argument("Invalid input point.");
    }
    Eigen::Translation3d target(
        static_cast<double>(bp::extract<double>(baselink_to_point[0])),
        static_cast<double>(bp::extract<double>(baselink_to_point[1])),
        static_cast<double>(bp::extract<double>(baselink_to_point[2])));
    // 第1引数robot_base_frameは，実は使われていない
    // 第4引数current_joint_stateは，HSR-Bの頭部構成では空で十分
    tmc_manipulation_types::JointState head_joint_state;
    if (!head_kinematics_->CalculateAngleToGazePoint(
            "hoge", target, camera_frame,
            tmc_manipulation_types::JointState(), head_joint_state)) {
      return bp::dict();
    }
    bp::dict result;
    for (uint32_t i = 0; i < head_joint_state.name.size(); ++i) {
      result[head_joint_state.name[i]] = head_joint_state.position[i];
    }
    return result;
  }

 private:
  hsr_kinematics::HsrHeadKinematics::Ptr head_kinematics_;
};

BOOST_PYTHON_MODULE(_extension) {
  bp::class_<hsrb_interface_plugin::KinematicsInterface>("KinematicsInterface", bp::init<std::string>())
      .def("calculate_gazing_angles", &hsrb_interface_plugin::KinematicsInterface::CalculateAngles);
}
}  // namespace hsrb_interface_plugin

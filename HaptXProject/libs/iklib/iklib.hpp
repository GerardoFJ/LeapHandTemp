#pragma once
#include "options/options.hpp"
#include <string>
#include "haptx/haptx.hpp"

struct IKConfig {
  std::string ik_method = "alternate"; 
  bool legacy_blend = true;
  bool ikmode = true;

  void register_options(Options& o) {
    o.add("ik.method", ik_method, "IK mode ['default', 'alternate']");
    o.add("ik.legacy_blend", legacy_blend, "AlternateIK option for accurate position over natural");
  }

  int process_options(){
    //process IK variables 
    if(ik_method == "alternate"){
      ikmode = true;
    }
    else if(ik_method == "default"){
      ikmode = false;
    }
    else{
      printf("IK method not recognized please select a valid method \n");
      return 1;
    }
    return 0;
  }
};

// Finger names
const char* const FINGER_NAMES[F_LAST] = {"THUMB", "INDEX", "MIDDLE", "RING", "PINKY"};

// Structure to organize finger data
struct FingerData {
  bool has_angles = false;
  float joint_rad[FJ_LAST] = {0.0f, 0.0f, 0.0f};
  float adduction_rad = 0.0f;
  bool has_orientations = false;
  Quaternion orientations[FJ_LAST] = {Quaternion::identity(), Quaternion::identity(),
                                      Quaternion::identity()};
};

// Pulls one finger out of an AnimFrame.
inline FingerData fingerFrom(const AnimFrame& anim_frame, RelativeDirection handedness, Finger finger) {
  FingerData out;

  const auto angle_it = anim_frame.angles.find(finger);
  if (angle_it != anim_frame.angles.end() && angle_it->second.flexion.isValid()) {
    const FingerAngleFrame& frame = angle_it->second;
    out.has_angles = true;
    out.joint_rad[FJ_JOINT1] = frame.flexion.theta_mcp_rad;
    out.joint_rad[FJ_JOINT2] = frame.flexion.theta_pip_rad;
    out.joint_rad[FJ_JOINT3] = frame.flexion.theta_dip_rad;
    out.adduction_rad = frame.adduction.isValid() ? frame.adduction.angle_rad : 0.0f;
  }

  out.has_orientations = true;
  for (unsigned int joint = 0; joint < FJ_LAST; joint++) {
    const BodyPartJoint key =
        getBodyPartJoint(handedness, finger, static_cast<FingerJoint>(joint));
    const auto it = anim_frame.l_orientations.find(key);
    if (it == anim_frame.l_orientations.end()) {
      out.has_orientations = false;
      break;
    }
    out.orientations[joint] = it->second;
  }
  return out;
}


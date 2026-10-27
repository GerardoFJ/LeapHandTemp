#pragma once
#include "options/options.hpp"
#include <string>

struct IKConfig {
  std::string ik_method = "alternate"; 
  bool legacy_blend = true;
  bool ikmode = true;

  void register_options(Options& o) {
    o.add("ik.method", ik_method, "IK mode ['default', 'alternate']");
    o.add("ik.legacy_blend", ik_method, "AlternateIK option for accurate position over natural");
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

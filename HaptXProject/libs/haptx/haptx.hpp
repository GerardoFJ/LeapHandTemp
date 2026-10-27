#pragma once
#include "options/options.hpp"
#include <string>
#include <unordered_set>

#include <HaptxApi/haptx_system.h>
#include <HaptxApi/airpack.h>

using namespace HaptxApi;

struct HaptXConfig {
  std::string hand = "both";  
  std::string host = "127.0.0.1";
  int port = 9871;
  double rate_hz = 90.0;
  std::vector<std::pair<std::shared_ptr<MocapSystem>,std::shared_ptr<Glove>>> gloves; 
  std::unordered_set<RelativeDirection> hand_selection;
  

  void register_options(Options& o) {
    o.add("haptx.hand", hand, "Hand selection ['right', 'left', 'both']");
    o.add("haptx.host", host, "Receiver Host");
    o.add("haptx.port", port, "Receiver Port");
    o.add("haptx.rate", rate_hz, "Data Rate hz");
    
  }

  int process_options(){
    //process hand states 
    if (hand == "both"){
      hand_selection.insert(RD_LEFT);
      hand_selection.insert(RD_RIGHT);
    }
    else if (hand == "left"){
      hand_selection.insert(RD_LEFT);
    }
    else if (hand == "right"){
      hand_selection.insert(RD_RIGHT);
    }
    else{
      printf("Hand selection '%s' is not valid.\n", hand.c_str());
      return 1;
    }
    return 0;
  }
};


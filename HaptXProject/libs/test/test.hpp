#pragma once
#include "options/options.hpp"

struct TestConfig {
  double rate = 50.0;
  int id = 1;
  std::vector<double> gains{1.0, 0.1, 0.0};

  void register_options(Options& o) {
    o.add("test.gains", gains, "PID gains");
    o.add("test.id", id, "ID value");
    o.add("test.rate", rate, "Rate value");
  }
  void validate_options(){
     if(rate < 100){
      printf("Valid rate \n");

    }
    else{
      printf("Incorrect rate \n");
      rate = 60;
    }
  }
};

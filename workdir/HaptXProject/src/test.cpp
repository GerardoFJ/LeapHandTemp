#include "test/test.hpp"
#include <iostream>

int main(int argc, char** argv) {
  Options opts("teleop");
  TestConfig test; 
  test.register_options(opts);
  opts.parse(argc, argv);
  printf("Params value rate %f  \n", test.rate);
  printf("Params value id %d \n ", test.id);
  printf("Param value gain:");
  for(auto gain : test.gains ){
      printf(" %f", gain);
    }
  printf("\n");

}

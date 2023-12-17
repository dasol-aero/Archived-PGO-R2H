#include <iostream>

#include "pgo_r2h/lib/conversion.hpp"
#include "pgo_r2h/lib/ros2.hpp"
#include "pgo_r2h/lib/time.hpp"


int main(void){

  std::cout << lib::conversion::get_transformation_matrix(1, 2, 3, -1, -2, -3) << std::endl;

  auto stamp = lib::ros2::get_stamp();
  std::cout << stamp.sec << stamp.nanosec << std::endl;

  std::cout << lib::time::get_time_since_epoch_ns_int64() << std::endl;

  return 0;

}

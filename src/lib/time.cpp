#include "pgo_r2h/lib/time.hpp"


namespace lib{
namespace time{


int64_t get_time_since_epoch_ns_int64(void){
  // NOTE: time point ---> duration ---> duration in ns ---> count in ns
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()
  ).count();
}


int64_t get_time_since_epoch_us_int64(void){
  // NOTE: time point ---> duration ---> duration in us ---> count in us
  return std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::system_clock::now().time_since_epoch()
  ).count();
}


int64_t get_time_since_epoch_ms_int64(void){
  // NOTE: time point ---> duration ---> duration in ms ---> count in ms
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()
  ).count();
}


}
}

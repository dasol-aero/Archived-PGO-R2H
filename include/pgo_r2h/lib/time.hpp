#ifndef _PGO_R2H_LIB_TIME_HPP_
#define _PGO_R2H_LIB_TIME_HPP_


#include <chrono>
#include <cstdint>


namespace lib{
namespace time{


// NOTE: Reference: https://en.cppreference.com/w/cpp/chrono/duration
int64_t get_time_since_epoch_ns_int64(void); // NOTE:  nano-seconds in int64_t
int64_t get_time_since_epoch_us_int64(void); // NOTE: micro-seconds in int64_t
int64_t get_time_since_epoch_ms_int64(void); // NOTE: milli-seconds in int64_t


}
}


#endif

#include "pgo_r2h/pgo.hpp"


PGO::PGO(const rclcpp::Node::SharedPtr& node) : node_(node) {

  /* print */
  std::printf("[INFO] class PGO has been created\n");

}


void PGO::init(void){

}


void PGO::run(void){

  /* spin */
  rclcpp::spin(node_);

}


/* --------------------------------------------------------------------------------------------- */

#include "pgo_r2h/pgo.hpp"


int main(int argc, char** argv)
{

  /* buffering mode */
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  /* node */
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("node_pgo");

  /* class */
  PGO pgo(node);

  /* initialization */
  pgo.init();

  /* run */
  pgo.run();

  /* shutdown and return */
  rclcpp::shutdown();
  return 0;

}

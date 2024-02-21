// 类加载器相关的头文件
#include <core_pack/polygon_base.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "example_node");
  // 类加载器 -- 参数1:基类功能包名称 参数2:基类全限定名称
  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader(
      "learning_pluginlib", "polygon_base::RegularPolygon");

  try {
    // 创建插件类实例 -- 参数:插件类全限定名称
    boost::shared_ptr<polygon_base::RegularPolygon> triangle =
        poly_loader.createInstance("polygon_plugins::Triangle");
    triangle->initialize(10.0);

    boost::shared_ptr<polygon_base::RegularPolygon> square =
        poly_loader.createInstance("polygon_plugins::Square");
    square->initialize(10.0);

    ROS_INFO("Triangle area: %.2f", triangle->area());
    ROS_INFO("Square area: %.2f", square->area());
  } catch (pluginlib::PluginlibException& ex) {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s",
              ex.what());
  }
  ROS_INFO("waiting for exit...");
  ros::spin();
  return 0;
}

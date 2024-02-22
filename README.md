# pluginlib用法

通俗一点说，plugin的用法就像是面向对象编程里面的多态。插件就是子类，插件实现父类的方法，那么系统调用父类方法时就是子类的方法，也就实现了替换。对应ROS里面只需要在launch文件中选择想要的插件，无需修改预案系统即可完成替换。

这里实现一个常用的plugin场景。

core_pack功能包相当于系统，里面定义了基类RegularPolygon，并且在main函数中调用area函数求多边形面积。

imp_plugin功能包实现了两个插件，继承自RegularPolygon，一个是三角形，一个是正方形，对基类的方法进行了重写。

这样在不修改core_pack的情况下，实现了对core_pack中功能的更改和扩展。

## core_pack

需要注意的点是。CMakeLists.txt中

```cmake
catkin_package(
 INCLUDE_DIRS include #必须有，插件的功能包需要依赖该功能包include中文件。相当于什么该功能包导出的include文件。
)
add_executable(polygon_loader src/polygon_loader.cpp)
# 上面两句话不能颠倒，如果颠倒，rosrun命令无法找到可执行文件
```

## imp_plugin(重点)

这个功能包继承了core_pack中的基类，实现了两个子类三角形和正方形，这两个class在ros中就叫插件。

实现完基类后还需要配置4个文件。

1、在实现插件类的cpp文件中将插件类导出

```c++
//pluginlib 宏，可以注册插件类
#include <pluginlib/class_list_macros.h>
#include <core_pack/polygon_base.h>
#include <imp_plugin/polygon_plugins.h>
//参数:衍生类 参数:基类
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)
```

2、配置package.xml

```xml
<!-- 插件的实现依赖于父类的功能包 -->
<build_depend>core_pack</build_depend>
<build_export_depend>core_pack</build_export_depend>
<exec_depend>core_pack</exec_depend>

<!-- 这里相当于导出插件，core_pack是父类的功能包 -->
<export>
  <core_pack plugin="${prefix}/polygon_plugins.xml" />
</export>
```

3、新增xml文件，配置插件导出关系。polygon_plugins.xml，放在插件功能包目录下

```xml
<!-- 插件库的相对路径 -->
<library path="lib/libpolygon_plugins">
    <!-- type="插件类" base_class_type="基类" -->
    <class type="polygon_plugins::Triangle" base_class_type="polygon_base::RegularPolygon">
        <!-- 描述信息 -->
        <description>This is a triangle plugin.</description>
    </class>
    <class type="polygon_plugins::Square" base_class_type="polygon_base::RegularPolygon">
        <description>This is a square plugin.</description>
    </class>
</library>
```

最后正常配置cmakelists.txt文件，把插件编译成库文件，添加库文件的依赖，依赖core_pack。

## 编译方法

```
mkdir -p ~/catkin_ws/src
```


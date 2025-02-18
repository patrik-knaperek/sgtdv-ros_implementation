# **SGT Driverless Codestyle guidelines**

___

&copy; **SGT Driverless**

**Authors:** Patrik Knaperek

**Objective:** Guidelines and recommendations for code style, structure and organization of ROS C++ code in SGT-Driverless organization.
___

## General guidelines

* **Read and follow [ROS C++ Style Guide](https://wiki.ros.org/CppStyleGuide)**, especially chapters:
  * 4 Naming
  * 6 Formatting
  * 8 Console-output

### Header files

#### Includes
* Use `#pragma once` statement at the beginning of each header file - prevents from multiple includes of the same file
* Sort included libraries by their origin (C++, ROS, SGT-DV, AMZ FSSIM...) - better readability
* Only include those libraries that are used in the header file. The libraries that are used in the source file only should be included there - may reduce compile time.

#### Constructor, destructor
* If the class constructor has only 1 parameter, declare the constructor with `explicit` statement - prevents from ambiguous copies of the objects from that class.
* If there is no implementation for class constructor/destructor, make it `= default`.

#### Functions and variables
* make `public` only those functions, that are used outside class implementation
* If you pass objects bigger than 64 bites (size of a pointer) as function arguments, use reference `&`.
* Use `const T&` for input and `*T` for output arguments (does the same thing, just a good practice).
* If the function is read-only, use `const` statement after argument declaration.

* **do not** use `public` variables
* To **define a constant**, use:
  - `#define` only for string constants - the compiler takes the value always as string, for numeric values default type is used, which makes such definition ambiguous
  - `static constexpr` for numeric constants, arrays, etc.

### Source files

#### Initializer list
* If not specified in the class constructor, the member variables are initialized with their default constructors in the order they are declared in the class declaration (header file).
* If you need to initialize the member variables to a certain value, use **initializer list** instead of initializing them in the constructor's body.
* The order of constructors on the initializer list has to follow the order they are declared in the class declaration.
* Initialize ROS publishers and subscribers in the initializer list for better readability.

#### Local variables initialization
* If the variable type is obvious from the right side of the statement, you can use `auto` statement instead of the type name on the left side.
* Initialize local variables in the lowest possible scope / as close as possible to the place of use.
* Use `const` for variables, that are assigned only during initialization.
* Use `static` for variables, which value needs to be preserved after leaving the scope of a function and is not needed in any other function.

#### Memory allocation
* after `new` statement there must be `delete` statement somewhere. It is however strongly recommended to use `unique_ptr` and `shared_ptr` instead.

#### Other recommendations
* For variable retyping, use `static_cast<T>` instead of C syntax.
* For console output, use [`ROS_LOG` macros](https://wiki.ros.org/roscpp/Overview/Logging) instead of `std::cout` stream.
* For **iteration through an array or vector**, in cases where you don't explicitly need the numerical position of the element use
  ```cpp
  for(auto& i : vector)
  {
    // access the vector element using i
  }
  ```
  instead of
  ```cpp
  for(int i = 0 ; i < vector.size() ; i++)
  {
    // access the vector element using vector[i]
  }
  ```

## Package structure
```
my_module
├── include
│   my_module.h
├── src
│   my_module_node.src
│   my_module_ros.src
│   my_module.src
├── params
│   my_module.yaml
│   ...
├── launch
│   my_module.launch
│   ...
├── msg
│   MyMessage.msg
│   ...
├── srv
│   MyService.srv
│   ...
├── data
│   ...
├── doc
│   ...
├── CMakeLists.txt
├── package.xml
└── README.md
```

* **include** : one header file for each class defined in the package
* **src** : 
  - `*_node.cpp` file containing (preferably) only `main()` function with either a `ros::spin()` call (for modules activated by callbacks only) or a while loop with defined `ros::Rate()` and `ros::Rate::spinOnce()` call (for modules running at desired frequency), 
  - `*_ros.cpp`file that handles ROS interface of the module (topics, services, parameters)
  - `*.cpp` implementation file(s) for each other class
* params : one or more configuration files allowing
  * changing program parameters without a need to recompile
  * launching under different configuration setups
* launch : one or more launch files allowing for different launch conditions
* *msg* : custom message definition - AVOID message definition in other packages than `sgtdv_msgs`
* *srv* : custom service definition - AVOID service definition in other packages than `sgtdv_msgs`
* data : custom logs, CSV files, etc.
* doc : support documentation files for README (e. g. schemes, chart flows...)
* bag : rosbags for offline launch
* rviz : RViz config files
* [**CMakeLists.txt**](https://wiki.ros.org/catkin/CMakeLists.txt) : build input file
* [**package.xml**](https://wiki.ros.org/catkin/package.xml) : package manifest file
* **README.md** : describe the package functionality in a way, that no one really needs to read the code to understand it, especially:
  - package name
  - authors and short main objective (1 sentence)
  - package overview
    - further brief explanation of the package purpose and functionality
    - ROS interface description
    - reference to related packages
  - installation guide (in case of external libraries requirements)
  - how to compile
  - how to launch
  - configuration options description
  - workflow diagram
  - any references related to the implementation methods

## Launchfile tips

* load parameters from YAML file
  ```xml
  <rosparam command="load" file="$(find package_name)/params/param_file.yaml"/>
  ```
* launch another launchfile
  ```xml
  <include file="$(find package_name)/launch/launch_file.launch"/>
  ```
* launch a node
  ```xml
  <node pkg="package_name" type="node_execution_name" name="node_display_name" output="screen"/>
  ```
* launch a rosbag
  ```xml
  <node
      pkg="rosbag"
      type="play"
      name="player" 
      output="screen"
      args="$(find package_name)/bags/$(arg bag_name)"
    />
  ```
* launch RViz
  ```xml
    <arg name="rviz_config" default="$(find package_name)/rviz/rviz_config.rviz"/>
    <node pkg="rviz" type="rviz" name="node_name_rviz"  respawn="false"
        args="-d $(arg rviz_config)" output="screen">
    </node>
  ```
* static transformation
  ```xml
  <node pkg="tf2_ros" type="static_transform_publisher" name="child_frame_to_parent_frame" 
    args="tx ty tz rx ry rz parent_frame child_frame" />
  ```
* set log level
  ```xml
  <node pkg="rosservice" 
    type="rosservice" 
    name="set_node_name_log_level" 
    args="call /node_name/set_logger_level 'ros.node_name' 'log_level'" />
  ```

## Schemes and flowcharts

We use **draw.io** to create and edit schemes and flowcharts related to AS. We store the diagrams in SVG format from 2 reasons:
  1. it's lossless (vector graphics)
  2. its changes are trackable by Git

Edit links:
* [Driverless architecture](https://drive.google.com/file/d/1iMFfZ8oNLDB1jh61XK2_ugi4TykpmphU/view?usp=sharing)
* [Software flowcharts](https://drive.google.com/file/d/1dqrngFT6DD64BGUCgTsuppBCYCkYrPgU/view?usp=sharing)
* [`sgtdv_msgs` decomposition](https://drive.google.com/file/d/1O1jDG0HykTuPU9wxy1FZ5LMXHLNu2jPT/view?usp=sharing)

After editing, export diagram as SVG:
1. Go to `File` -> `Export as` -> `SVG...`
2. Edit export setup. Recommended:
   * Zoom: 100%
   * Border Width: 10
   * Size: Diagram (or Selection Only)
   * Transparent Background: No
   * Appearance: Light
   * Shadow: No
   * Include a copy of my diagram: No
   * Embeded Images: Yes
   * Embeded Fonts: Yes
   * Links: Automatic 
3. click `Export`
4. Set export filename and destination (recommended: Filename: default, Where: Device) and click `Save`
5. Replace the original diagram and submit changes to Git.
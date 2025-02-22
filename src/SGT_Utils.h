#pragma once

/* C++ */
#include <XmlRpcException.h>
#include <Eigen/Eigen>

/* ROS */
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

class Utils
{
  Utils() = default;
  ~Utils() = default;

  public:
  /**
   * @brief Load parameter of any type from parameter server into given storage variable
   * 
   * @tparam T parameter type
   * @param handle ROS node handle
   * @param name parameter name on parameter server
   * @param storage place where the parameter value is stored to
   * @return `true` if parameter is set on the parameter server,
   * @return `false` otherwise
   */
  template<typename T> static bool loadParam(const ros::NodeHandle &handle, const std::string &name, T* storage)
  {
    if (!handle.getParam(name, *storage))
  {
    ROS_ERROR("Failed to get parameter \"%s\" from server\n", name.data());
    return false;
    }
    return true;
  };

  /**
   * @brief Load parameter of any type from parameter server into given storage variable with default value
   * 
   * @tparam T parameter type
   * @param handle ROS node handle
   * @param name parameter name on parameter server
   * @param defaultValue default parameter value (if not set on the parameter server)
   * @param storage place where the parameter value is stored to
   * @return `true` if parameter is set on the parameter server,
   * @return `false` otherwise
   */
  template<typename T> static bool loadParam(const ros::NodeHandle &handle, const std::string &name,
                                  const T &defaultValue, T* storage)
  {
    if (!handle.param<T>(name, *storage, defaultValue))
    {
      ROS_WARN_STREAM("Failed to get parameter " << name.data() << " from server, setting default: " << defaultValue);
      return false;
    }
    return true;
  };

  /**
   * @brief Read multidimensional array from parameter server
   * 
   * @param handle ROS node handle
   * @param paramName parameter name on parameter server
   * @param rows number of array rows
   * @param cols number of array collums
   * @return Array converted to `Eigen::Array`
   */
  static Eigen::ArrayXXd loadArray(const ros::NodeHandle &handle, const std::string &paramName, const int rows, const int cols)
  {
    XmlRpc::XmlRpcValue paramValue;
    Eigen::ArrayXXd arrayValue = Eigen::ArrayXXd::Zero(rows, cols);
    if (loadParam(handle, paramName, &paramValue))
    {
      try
      {
        ROS_ASSERT(paramValue.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int i = 0; i < rows; i++)
        {
          for (int j = 0; j < cols; j++)
          {
            try
            {
              std::ostringstream ostr;
              ostr << paramValue[cols * i  + j];
              std::istringstream istr(ostr.str());
              istr >> arrayValue(i, j);
            }
            catch(XmlRpc::XmlRpcException &e)
            {
              throw e;
            }
            catch(...)
            {
              throw;
            }
          }
        }
      }
      catch(XmlRpc::XmlRpcException &e)
      {
        ROS_ERROR_STREAM("ERROR reading from server: " << e.getMessage() <<
                        " for " << paramName << "(type: " << paramValue.getType() << ")");
      }
    }
    return arrayValue;
  };

  template<typename T> struct Range
  {
    Range<T>() = default;
    Range<T>(const T& Min, const T& Max) :
      min(Min), max(Max)
    {
    };
    T min;
    T max;
  };

  static geometry_msgs::PointStamped 
  transformCoords(const tf::TransformListener &listener, const std::string &target_frame,
                  const geometry_msgs::PointStamped &coords_source_frame)
  {
    geometry_msgs::PointStamped coords_target_frame;
    try
    {
      listener.transformPoint(target_frame, coords_source_frame, coords_target_frame);
    }
    catch (tf::TransformException &e)
    {
      ROS_WARN(e.what());
    }
    return coords_target_frame;
  }
};
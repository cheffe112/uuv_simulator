// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __UUV_UNDERWATER_CAMERA_ROS_PLUGIN_HH__
#define __UUV_UNDERWATER_CAMERA_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/plugins/DepthCameraPlugin.hh>
#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <uuv_sensor_ros_plugins/Common.hh>
#include <opencv2/opencv.hpp>
#include <uuv_sensor_ros_plugins/UnderwaterCameraParametersConfig.h>

namespace gazebo
{
  class UnderwaterCameraROSPlugin :
    public DepthCameraPlugin, public GazeboRosCameraUtils
  {
    /// \brief Class constructor
    public: UnderwaterCameraROSPlugin();

    /// \brief Class destructor
    public: virtual ~UnderwaterCameraROSPlugin();

    /// \brief Load plugin and its configuration from sdf.
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    public: virtual void OnNewDepthFrame(const float *_image,
      unsigned int _width, unsigned int _height, unsigned int _depth,
      const std::string& _format);

    /// \brief Update the controller
    public: virtual void OnNewRGBPointCloud(const float *_pcd,
      unsigned int _width, unsigned int _height, unsigned int _depth,
      const std::string& _format);

    public: virtual void OnNewImageFrame(const unsigned char* _image,
      unsigned int _width, unsigned int _height,  unsigned int _depth,
      const std::string& _format);
    
    /// \brief Callback for camera parameter dynamic reconfigure
    private: virtual void SetParameters(uuv_sensor_ros_plugins::UnderwaterCameraParametersConfig &config, uint32_t level);

    /// \brief Add underwater light damping to image
    protected: virtual void SimulateUnderwater(
     const cv::Mat& _inputImage, const cv::Mat& _inputDepth,
     cv::Mat& _outputImage);
    
    /// \brief Convert HSV to RGB values
    private: virtual void HSVtoRGB(float h, float s, float v, float& r, float& g, float& b);

    /// \brief Temporarily store pointer to previous depth image.
    protected: const float * lastDepth;

    /// \brief Latest simulated image.
    protected: unsigned char * lastImage;

    /// \brief Depth to range lookup table (LUT)
    protected: float* depth2rangeLUT;

    /// \brief Attenuation constants per channel (RGB)
    protected: float attenuation[3];

    /// \brief Background constants per channel (RGB)
    protected: unsigned char background[3];
    
    /// \brief Dynamic Reconfigure server for the camera parameters
    private: boost::shared_ptr<dynamic_reconfigure::Server<uuv_sensor_ros_plugins::UnderwaterCameraParametersConfig> > dynamic_reconfigure_server;

  };
}

#endif // __UUV_UNDERWATER_CAMERA_ROS_PLUGIN_HH__

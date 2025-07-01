/*
 * camera_loader.h
 *
 *  Created on: Feb 11, 2014
 *      Author: cforster
 */

#ifndef VIKIT_CAMERA_LOADER_H_
#define VIKIT_CAMERA_LOADER_H_

#include <string>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/omni_camera.h>
#include <vikit/equidistant_camera.h>
#include <vikit/polynomial_camera.h>
#include <vikit/params_helper.h>

namespace vk {
namespace camera_loader {

/// Load from ROS Namespace
bool loadFromRosNs(rclcpp::Node::SharedPtr nh, const std::string& ns, vk::AbstractCamera*& cam)
{
  bool res = true;
  std::string cam_model;
  if (!nh->get_parameter(ns + ".cam_model", cam_model)) {
    RCLCPP_ERROR(nh->get_logger(), "Camera model parameter not found: %s.cam_model", ns.c_str());
    return false;
  }

  if (cam_model == "Ocam") {
    std::string calib_file;
    nh->get_parameter(ns + ".cam_calib_file", calib_file);
    cam = new vk::OmniCamera(calib_file);
  }
  else if (cam_model == "Pinhole") 
  {
    int cam_width = 640;
    int cam_height = 480;
    double scale = 1.0;
    double cam_fx = 0.0;
    double cam_fy = 0.0;
    double cam_cx = 0.0;
    double cam_cy = 0.0;
    double cam_d0 = 0.0;
    double cam_d1 = 0.0;
    double cam_d2 = 0.0;
    double cam_d3 = 0.0;

    nh->get_parameter(ns + ".cam_width", cam_width);
    nh->get_parameter(ns + ".cam_height", cam_height);
    nh->get_parameter(ns + ".scale", scale);
    nh->get_parameter(ns + ".cam_fx", cam_fx);
    nh->get_parameter(ns + ".cam_fy", cam_fy);
    nh->get_parameter(ns + ".cam_cx", cam_cx);
    nh->get_parameter(ns + ".cam_cy", cam_cy);
    nh->get_parameter(ns + ".cam_d0", cam_d0);
    nh->get_parameter(ns + ".cam_d1", cam_d1);
    nh->get_parameter(ns + ".cam_d2", cam_d2);
    nh->get_parameter(ns + ".cam_d3", cam_d3);

    cam = new vk::PinholeCamera(
        cam_width, cam_height, scale,
        cam_fx, cam_fy, cam_cx, cam_cy,
        cam_d0, cam_d1, cam_d2, cam_d3
      );
  }
  else if(cam_model == "EquidistantCamera")
  {
    int cam_width = 640;
    int cam_height = 480;
    double scale = 1.0;
    double cam_fx = 0.0;
    double cam_fy = 0.0;
    double cam_cx = 0.0;
    double cam_cy = 0.0;
    double k1 = 0.0;
    double k2 = 0.0;
    double k3 = 0.0;
    double k4 = 0.0;

    nh->get_parameter(ns + ".cam_width", cam_width);
    nh->get_parameter(ns + ".cam_height", cam_height);
    nh->get_parameter(ns + ".scale", scale);
    nh->get_parameter(ns + ".cam_fx", cam_fx);
    nh->get_parameter(ns + ".cam_fy", cam_fy);
    nh->get_parameter(ns + ".cam_cx", cam_cx);
    nh->get_parameter(ns + ".cam_cy", cam_cy);
    nh->get_parameter(ns + ".k1", k1);
    nh->get_parameter(ns + ".k2", k2);
    nh->get_parameter(ns + ".k3", k3);
    nh->get_parameter(ns + ".k4", k4);

    cam = new vk::EquidistantCamera(
        cam_width, cam_height, scale,
        cam_fx, cam_fy, cam_cx, cam_cy,
        k1, k2, k3, k4
      );
  }
  else if(cam_model == "PolynomialCamera")
  {
    double cam_width = 640;
    double cam_height = 480;
    double scale = 1.0;
    double cam_fx = 0.0;
    double cam_fy = 0.0;
    double cam_cx = 0.0;
    double cam_cy = 0.0;
    double cam_skew;
    double k2 = 0.0;
    double k3 = 0.0;
    double k4 = 0.0;
    double k5 = 0.0;
    double k6 = 0.0;
    double k7 = 0.0;
    nh->get_parameter(ns + ".cam_width", cam_width);
    nh->get_parameter(ns + ".cam_height", cam_height);
    // nh->get_parameter(ns + ".scale", scale);
    nh->get_parameter(ns + ".cam_fx", cam_fx);
    nh->get_parameter(ns + ".cam_fy", cam_fy);
    nh->get_parameter(ns + ".cam_cx", cam_cx);
    nh->get_parameter(ns + ".cam_cy", cam_cy);
    nh->get_parameter(ns + ".cam_skew", cam_skew);
    nh->get_parameter(ns + ".k2", k2);
    nh->get_parameter(ns + ".k3", k3);
    nh->get_parameter(ns + ".k4", k4);
    nh->get_parameter(ns + ".k5", k5);
    nh->get_parameter(ns + ".k6", k6);
    nh->get_parameter(ns + ".k7", k7);
    
    cam = new vk::PolynomialCamera(
      cam_width, cam_height,
      cam_fx, cam_fy, cam_cx, cam_cy,
      cam_skew, k2, k3, k4, k5, k6, k7
    );
  }
  else if(cam_model == "ATAN")
  {
    int cam_width = 640;
    int cam_height = 480;
    double cam_fx = 0.0;
    double cam_fy = 0.0;
    double cam_cx = 0.0;
    double cam_cy = 0.0;
    double cam_d0 = 0.0;

    nh->get_parameter(ns + ".cam_width", cam_width);
    nh->get_parameter(ns + ".cam_height", cam_height);
    nh->get_parameter(ns + ".cam_fx", cam_fx);
    nh->get_parameter(ns + ".cam_fy", cam_fy);
    nh->get_parameter(ns + ".cam_cx", cam_cx);
    nh->get_parameter(ns + ".cam_cy", cam_cy);
    nh->get_parameter(ns + ".cam_d0", cam_d0);

    cam = new vk::ATANCamera(
      cam_width, cam_height,
      cam_fx, cam_fy, cam_cx, cam_cy,
      cam_d0
    );
  }
  else
  {
    cam = NULL;
    res = false;
  }
  return res;
}

bool loadFromRosNs(rclcpp::Node::SharedPtr nh, const std::string& ns, std::vector<vk::AbstractCamera*>& cam_list)
{
  bool res = true;
  int cam_num;
  if (!nh->get_parameter(ns + ".cam_num", cam_num)) {
    RCLCPP_ERROR(nh->get_logger(), "Camera number parameter not found: %s.cam_num", ns.c_str());
    return false;
  }

  for (int i = 0; i < cam_num; i++) {
    std::string cam_ns = ns + ".cam_" + std::to_string(i);
    vk::AbstractCamera *cam;
    
    if (!loadFromRosNs(nh, cam_ns, cam)) {
      RCLCPP_WARN(nh->get_logger(), "Failed to load camera from namespace: %s", cam_ns.c_str());
      res = false;
    } else {
      cam_list.push_back(cam);
    }
  }
  
  return res;
}

} // namespace camera_loader
} // namespace vk

#endif // VIKIT_CAMERA_LOADER_H_

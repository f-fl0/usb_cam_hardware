#ifndef USB_CAM_CONTROLLERS_FORMAT_CONTROLLER
#define USB_CAM_CONTROLLERS_FORMAT_CONTROLLER

#include <string>

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <usb_cam_controllers/simple_packet_controller.hpp>
#include <usb_cam_hardware_interface/packet_interface.hpp>


namespace usb_cam_controllers {

enum RotateCode { ROTATE_NONE, ROTATE_90_CW, ROTATE_90_CCW, ROTATE_180 };

class FormatController : public SimplePacketController {
public:
  FormatController() {}

  virtual ~FormatController() {}


protected:

  virtual bool initImpl(usb_cam_hardware_interface::PacketInterface *hw, ros::NodeHandle &root_nh,
                        ros::NodeHandle &controller_nh) {

    controller_nh.param<std::string>("camera_frame_id", frame_id_, "head_camera");
    controller_nh.param<int>("image_width", width_, 640);
    controller_nh.param<int>("image_height", height_, 480);
    controller_nh.param<int>("padding_left", padding_left_, 0);
    controller_nh.param<int>("padding_top", padding_top_, 0);
    controller_nh.param<int>("padding_right", padding_right_, 0);
    controller_nh.param<int>("padding_bottom", padding_bottom_, 0);

    std::string rotate_code_str;
    controller_nh.param<std::string>("rotate_code", rotate_code_str, "");
    if(rotate_code_str == "90CW_ROT") {
      rotate_code_ = ROTATE_90_CW;
    }
    else if(rotate_code_str == "90CCW_ROT") {
      rotate_code_ = ROTATE_90_CCW;
    }
    else if(rotate_code_str == "180_ROT") {
      rotate_code_ = ROTATE_180;
    }
    else {
      if (rotate_code_str != "") {
        ROS_WARN("Invalild rotate code: [%s]. No rotation will be applied.", rotate_code_str.c_str());
      }
      rotate_code_ = ROTATE_NONE;
    }
    return true;
  }

  virtual void startingImpl(const ros::Time &time) {
    // nothing to do
  }

  virtual void updateImpl(const ros::Time &time, const ros::Duration &period) = 0;


  virtual void stoppingImpl(const ros::Time &time) {
    // nothing to do
  }
  std::string frame_id_;
  int height_, width_;
  int padding_top_, padding_right_, padding_bottom_, padding_left_;
  RotateCode rotate_code_;
};

} // namespace usb_cam_controllers

#endif

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>

#include "usb_cam_controllers/camera_info_controller.hpp"
#include "usb_cam_controllers/image_controllers.hpp"

PLUGINLIB_EXPORT_CLASS(usb_cam_controllers::CameraInfoController,
                       controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(usb_cam_controllers::UYVYController, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(usb_cam_controllers::YUYVController, controller_interface::ControllerBase);

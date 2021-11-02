#ifndef USB_CAM_CONTROLLERS_CAMERA_INFO_CONTROLLER
#define USB_CAM_CONTROLLERS_CAMERA_INFO_CONTROLLER

#include <string>

#include <camera_info_manager/camera_info_manager.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>
#include <usb_cam_controllers/format_controller.hpp>
#include <usb_cam_hardware_interface/packet_interface.hpp>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

namespace usb_cam_controllers {

class CameraInfoController : public FormatController {
public:
  CameraInfoController() {}

  virtual ~CameraInfoController() {}

protected:
  virtual bool initImpl(usb_cam_hardware_interface::PacketInterface *hw, ros::NodeHandle &root_nh,
                        ros::NodeHandle &controller_nh) {
    if (!FormatController::initImpl(hw, root_nh, controller_nh)) {
      return false;
    }
    std::string camera_name, camera_info_url;
    controller_nh.param< std::string >("camera_name", camera_name, "head_camera");
    controller_nh.param< std::string >("camera_info_url", camera_info_url, "");
    info_manager_ = boost::make_shared< camera_info_manager::CameraInfoManager >(
        controller_nh, camera_name, camera_info_url);
    if (!info_manager_->isCalibrated()) {
      info_manager_->setCameraName(camera_name);
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.frame_id = frame_id_;
      camera_info.width = width_;
      camera_info.height = height_;
      info_manager_->setCameraInfo(camera_info);
    }
    publisher_ = controller_nh.advertise< sensor_msgs::CameraInfo >("camera_info", 1);
    return true;
  }

  virtual void updateImpl(const ros::Time &time, const ros::Duration &period) {
    // publish the camera info
    const sensor_msgs::CameraInfoPtr msg(new sensor_msgs::CameraInfo());
    *msg = info_manager_->getCameraInfo();
    msg->header.stamp = packet_iface_.getStamp();
    msg->header.frame_id = frame_id_;

    if (padding_left_ != 0 || padding_top_ != 0 ||
        padding_right_ != 0 || padding_bottom_ != 0)
    {
      msg->height += padding_top_ + padding_bottom_;
      msg->width += padding_left_ + padding_right_;
      msg->K[2] += padding_left_;
      msg->K[3 + 2] += padding_top_;
      msg->P[2] += padding_left_;
      msg->P[4 + 2] += padding_top_;

      if (msg->roi.width != 0 && msg->roi.height != 0)
      {
        msg->roi.x_offset += padding_left_;
        msg->roi.y_offset += padding_top_;
      }
    }

    updateCameraInfo(msg, rotate_code_);
    publisher_.publish(msg);
  }

private:
  ros::Publisher publisher_;
  boost::shared_ptr< camera_info_manager::CameraInfoManager > info_manager_;

  void updateCameraInfo(sensor_msgs::CameraInfoPtr ci, const RotateCode rotate_code) {
    if (rotate_code == ROTATE_NONE) {
      return;
    }

    if (rotate_code == ROTATE_180) {
      // update cx and cy
      ci->K[2] = ci->width - ci->K[2];
      ci->K[3 + 2] = ci->height - ci->K[3 + 2];
      ci->P[2] = ci->width - ci->P[2];
      ci->P[4 + 2] = ci->height - ci->P[4 + 2];

      // update ROI
      if (ci->roi.width != 0 && ci->roi.height != 0) {
        ci->roi.x_offset = ci->width - ci->roi.x_offset;
        ci->roi.y_offset = ci->height - ci->roi.y_offset;
      }

      return;
    }

    // swap fx and fy
    std::swap(ci->K[0], ci->K[4]);
    std::swap(ci->P[0], ci->P[5]);

    if (rotate_code == ROTATE_90_CW) {
      // update cx and cy
      const double tmp_kcx = ci->K[2];
      ci->K[2] = ci->height - ci->K[3 + 2];
      ci->K[3 + 2] = tmp_kcx;
      const double tmp_pcx = ci->P[2];
      ci->P[2] = ci->height - ci->P[4 + 2];
      ci->P[4 + 2] = tmp_pcx;

      // update ROI offset
      if (ci->roi.width != 0 && ci->roi.height != 0) {
        const double tmp_x_offset = ci->roi.x_offset;
        ci->roi.x_offset = ci->height - ci->roi.y_offset;
        ci->roi.y_offset = tmp_x_offset;
      }
    }

    if (rotate_code == ROTATE_90_CCW) {
      // update cx and cy
      const double tmp_kcx = ci->K[2];
      ci->K[2] = ci->K[3 + 2];
      ci->K[3 + 2] = ci->width - tmp_kcx;
      const double tmp_pcx = ci->P[2];
      ci->P[2] = ci->P[4 + 2];
      ci->P[4 + 2] = ci->width - tmp_pcx;

      // update ROI offset
      if (ci->roi.width != 0 && ci->roi.height != 0) {
        const double tmp_x_offset = ci->roi.x_offset;
        ci->roi.x_offset = ci->roi.y_offset;
        ci->roi.y_offset = ci->width - tmp_x_offset;
      }
    }

    // update ROI width, height
    if (ci->roi.width != 0 && ci->roi.height != 0) {
      const double tmp_roi_width = ci->roi.width;
      ci->roi.width = ci->roi.height;
      ci->roi.height = tmp_roi_width;
    }

    // update the canmera info width and height values
    // as width and height have been swapped.
    const double tmp_ciw = ci->width;
    ci->width = ci->height;
    ci->height = tmp_ciw;
  }
};

} // namespace usb_cam_controllers

#endif

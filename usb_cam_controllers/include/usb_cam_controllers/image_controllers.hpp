#ifndef USB_CAM_CONTROLLERS_IMAGE_CONTROLLERS
#define USB_CAM_CONTROLLERS_IMAGE_CONTROLLERS

#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/image_encodings.h>
#include <usb_cam_controllers/format_controller.hpp>
#include <usb_cam_hardware_interface/packet_interface.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace usb_cam_controllers {

template < cv::ColorConversionCodes ConversionCode, const std::string *DstEncoding >
class ImageController : public FormatController {
public:
  ImageController() {}

  virtual ~ImageController() {}

protected:
  virtual bool initImpl(usb_cam_hardware_interface::PacketInterface *hw, ros::NodeHandle &root_nh,
                        ros::NodeHandle &controller_nh) {
    if (!FormatController::initImpl(hw, root_nh, controller_nh)) {
      return false;
    }

    if(rotate_code_ == ROTATE_90_CW)
    {
      cv_rotate_code_ = cv::ROTATE_90_CLOCKWISE;
    }
    else if(rotate_code_ == ROTATE_90_CCW)
    {
      cv_rotate_code_ = cv::ROTATE_90_COUNTERCLOCKWISE;
    }
    else if(rotate_code_ == ROTATE_180)
    {
      cv_rotate_code_ = cv::ROTATE_180;
    }
    else
    {
      cv_rotate_code_ = -1;
    }

    publisher_ = image_transport::ImageTransport(controller_nh).advertise("image", 1);

    return true;
  }

  virtual void updateImpl(const ros::Time &time, const ros::Duration &period) {
    cv_bridge::CvImage out;
    out.header.stamp = packet_iface_.getStamp();
    out.encoding = *DstEncoding;

    try {
      cv::cvtColor(cv::Mat(height_, width_, CV_8UC(packet_iface_.getLength() / (height_ * width_)),
                           const_cast< uint8_t * >(packet_iface_.getStartAs< uint8_t >())),
                   out.image, ConversionCode);
    } catch (const cv::Exception &ex) {
      ROS_ERROR_STREAM(ex.what());
      return;
    }

    // pad output image
    if (padding_left_ != 0 || padding_top_ != 0 || padding_right_ != 0 || padding_bottom_ != 0)
    {
      cv::copyMakeBorder(out.image, out.image,
        padding_top_, padding_bottom_, padding_left_, padding_right_,
        cv::BORDER_CONSTANT, (0, 0, 0));
    }

    if (cv_rotate_code_ >= 0)
    {
      cv::rotate(out.image, out.image, cv_rotate_code_);
    }
    publisher_.publish(out.toImageMsg());
  }

private:
  int cv_rotate_code_;

  image_transport::Publisher publisher_;
};

typedef ImageController< cv::COLOR_RGB2BGR, &sensor_msgs::image_encodings::BGR8 >
    RGB24Controller;
typedef ImageController< cv::COLOR_YUV2BGR_UYVY, &sensor_msgs::image_encodings::BGR8 >
    UYVYController;
typedef ImageController< cv::COLOR_YUV2BGR_YUYV, &sensor_msgs::image_encodings::BGR8 >
    YUYVController;

} // namespace usb_cam_controllers

#endif

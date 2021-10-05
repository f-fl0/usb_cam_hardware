#ifndef USB_CAM_HARDWARE_USB_CAM_HARDWARE
#define USB_CAM_HARDWARE_USB_CAM_HARDWARE

#include <cstring> // for std::memset()
#include <string>
#include <vector>

#include <hardware_interface/robot_hw.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <usb_cam_hardware_interface/packet_interface.hpp>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <linux/videodev2.h>

namespace usb_cam_hardware {

class USBCamHardware : public hardware_interface::RobotHW {
public:
  USBCamHardware() : fd_(-1) {}

  virtual ~USBCamHardware() { uninit(); }

  // init the camera.
  // returns the time-per-frame which the camera is operated at on success, or negative value on
  // failure.
  ros::Duration init(ros::NodeHandle param_nh) {
    // register the packet buffer to the interface so that controllers can see the packet
    {
      packet_.stamp = ros::Time(0);
      packet_.start = NULL;
      packet_.length = 0;
      packet_.buffer_index = -1;
      packet_interface_.registerHandle(usb_cam_hardware_interface::PacketHandle(
          "packet", &packet_.stamp, &packet_.start, &packet_.length));
      registerInterface(&packet_interface_);
    }

    // open the device
    {
      video_device_ = param_nh.param< std::string >("video_device", "/dev/video0");
      fd_ = open(video_device_.c_str(), O_RDWR | O_NONBLOCK);
      if (fd_ < 0) {
        ROS_ERROR_STREAM("Cannot open \"" << video_device_ << "\"");
        return ros::Duration(-1.);
      }
    }

    // TODO: check device capability

    // cancel device-side cropping and scaling
    {
      v4l2_cropcap cropcap;
      std::memset(&cropcap, 0, sizeof(cropcap));
      // get the cropping capability
      if (xioctl(fd_, VIDIOC_CROPCAP, &cropcap) == 0) {
        // set the default (no) cropping
        v4l2_crop crop;
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect;
        xioctl(fd_, VIDIOC_S_CROP, &crop);
      }
    }

    // set image format
    {
      v4l2_format format;
      std::memset(&format, 0, sizeof(format));
      format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      format.fmt.pix.width = param_nh.param("image_width", 640);
      format.fmt.pix.height = param_nh.param("image_height", 480);
      const std::string pixel_format(param_nh.param< std::string >("pixel_format", "mjpeg"));
      if (pixel_format == "grey") {
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
      } else if (pixel_format == "h264") {
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
      } else if (pixel_format == "mjpeg") {
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
      } else if (pixel_format == "rgb24") {
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
      } else if (pixel_format == "uyvy") {
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
      } else if (pixel_format == "yuyv") {
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      } else {
        ROS_ERROR_STREAM("Pixel format \"" << pixel_format << "\" is not supported");
        return ros::Duration(-1.);
      }
      format.fmt.pix.field = V4L2_FIELD_INTERLACED;
      if (xioctl(fd_, VIDIOC_S_FMT, &format) < 0) {
        ROS_ERROR("Cannot set format");
        return ros::Duration(-1.);
      }
    }

    // set framerate
    ros::Duration time_per_frame;
    {
      v4l2_streamparm streamparm;
      std::memset(&streamparm, 0, sizeof(streamparm));
      streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (xioctl(fd_, VIDIOC_G_PARM, &streamparm) < 0) {
        ROS_ERROR("Cannot get streaming parameters");
        return ros::Duration(-1.);
      }
      const int fps = param_nh.param<int>("framerate", 30);
      v4l2_fract &timeperframe(streamparm.parm.capture.timeperframe);
      timeperframe.numerator = 1;
      timeperframe.denominator = fps;
      if (xioctl(fd_, VIDIOC_S_PARM, &streamparm) < 0) {
        ROS_ERROR("Cannot set capture framerate");
        return ros::Duration(-1.);
      }

      if (timeperframe.numerator) {
        const double fps_new = timeperframe.denominator / timeperframe.numerator;
        if ((double)fps != fps_new) {
          ROS_WARN("Unsupported capture framerate: %d. Framerate will be set to %f", fps, fps_new);
        } else {
          ROS_INFO("Framerate set to: %d, %u/%u", fps, timeperframe.denominator, timeperframe.numerator);
        }
      }
      time_per_frame =
          ros::Duration(static_cast< double >(timeperframe.numerator) / timeperframe.denominator);
    }

    // allocate buffers
    // TODO: support not only mmap, but also userp and read
    {
      v4l2_requestbuffers reqbufs;
      std::memset(&reqbufs, 0, sizeof(reqbufs));
      reqbufs.count = 4;
      reqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      reqbufs.memory = V4L2_MEMORY_MMAP;
      if (xioctl(fd_, VIDIOC_REQBUFS, &reqbufs) < 0) {
        ROS_ERROR("Cannot request buffers");
        return ros::Duration(-1.);
      }
      if (reqbufs.count < 2) {
        ROS_ERROR("Insufficient buffer memory on the device");
        return ros::Duration(-1.);
      }

      for (std::size_t i = 0; i < reqbufs.count; ++i) {
        v4l2_buffer buffer;
        std::memset(&buffer, 0, sizeof(buffer));
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;
        if (xioctl(fd_, VIDIOC_QUERYBUF, &buffer) < 0) {
          ROS_ERROR("Cannot query buffer");
          return ros::Duration(-1.);
        }

        Buffer mapped_buffer;
        mapped_buffer.start = mmap(NULL /* start anywhere */, buffer.length, PROT_READ | PROT_WRITE,
                                   MAP_SHARED, fd_, buffer.m.offset);
        mapped_buffer.length = buffer.length;
        if (mapped_buffer.start == MAP_FAILED) {
          ROS_ERROR("Cannot map memory");
          return ros::Duration(-1.);
        }

        buffers_.push_back(mapped_buffer);
      }
    }

    int exposure, brightness, contrast, saturation, sharpness, focus, white_balance, gain;
    bool auto_focus, auto_exposure, auto_white_balance;
    param_nh.param("brightness", brightness, -1); //0-255, -1 "leave alone"
    param_nh.param("contrast", contrast, -1); //0-255, -1 "leave alone"
    param_nh.param("saturation", saturation, -1); //0-255, -1 "leave alone"
    param_nh.param("sharpness", sharpness, -1); //0-255, -1 "leave alone"
    // enable/disable auto_focus
    param_nh.param("auto_focus", auto_focus, false);
    param_nh.param("focus", focus, -1); //0-255, -1 "leave alone"
    // enable/disable auto_exposure
    param_nh.param("auto_exposure", auto_exposure, true);
    param_nh.param("exposure", exposure, 100);
    param_nh.param("gain", gain, -1); //0-100?, -1 "leave alone"
    // enable/disable auto white balance temperature
    param_nh.param("auto_white_balance", auto_white_balance, true);
    param_nh.param("white_balance", white_balance, 4000);

    // set camera parameters
    if (brightness >= 0)
    {
      set_v4l_parameter("brightness", brightness);
    }

    if (contrast >= 0)
    {
      set_v4l_parameter("contrast", contrast);
    }

    if (saturation >= 0)
    {
      set_v4l_parameter("saturation", saturation);
    }

    if (sharpness >= 0)
    {
      set_v4l_parameter("sharpness", sharpness);
    }

    if (gain >= 0)
    {
      set_v4l_parameter("gain", gain);
    }

    // check auto white balance
    if (auto_white_balance)
    {
      set_v4l_parameter("white_balance_temperature_auto", 1);
    }
    else
    {
      set_v4l_parameter("white_balance_temperature_auto", 0);
      set_v4l_parameter("white_balance_temperature", white_balance);
    }

    // check auto exposure
    if (!auto_exposure)
    {
      // turn down exposure control (from max of 3)
      set_v4l_parameter("exposure_auto", 1);
      // change the exposure level
      set_v4l_parameter("exposure_absolute", exposure);
    }

    // check auto focus
    if (auto_focus)
    {
      set_auto_focus(1);
      set_v4l_parameter("focus_auto", 1);
    }
    else
    {
      set_v4l_parameter("focus_auto", 0);
      if (focus >= 0)
      {
        set_v4l_parameter("focus_absolute", focus);
      }
    }

    // start streaming
    {
      for (std::size_t i = 0; i < buffers_.size(); ++i) {
        v4l2_buffer buffer;
        std::memset(&buffer, 0, sizeof(buffer));
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;
        if (xioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
          ROS_ERROR("Cannot enqueue buffer");
          return ros::Duration(-1.);
        }
      }

      v4l2_buf_type buf_type;
      buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (xioctl(fd_, VIDIOC_STREAMON, &buf_type) < 0) {
        ROS_ERROR("Cannot start streaming");
        return ros::Duration(-1.);
      }
    }

    return time_per_frame;
  }

  void set_v4l_parameter(const std::string& param, int value)
  {
    set_v4l_parameter(param, boost::lexical_cast<std::string>(value));
  }

  void set_v4l_parameter(const std::string& param, const std::string& value)
  {
    // build the command
    std::stringstream ss;
    ss << "v4l2-ctl --device=" << video_device_ << " -c " << param << "=" << value << " 2>&1";
    std::string cmd = ss.str();

    // capture the output
    std::string output;
    int buffer_size = 256;
    char buffer[buffer_size];
    FILE *stream = popen(cmd.c_str(), "r");
    if (stream)
    {
      while (!feof(stream)) {
        if (fgets(buffer, buffer_size, stream) != NULL) {
          output.append(buffer);
        }
      }
      pclose(stream);
      // any output should be an error
      if (output.length() > 0) {
        ROS_WARN("%s", output.c_str());
      }
    }
    else {
      ROS_WARN("usb_cam_node could not run '%s'", cmd.c_str());
    }
  }

  // enables/disables auto focus
  void set_auto_focus(int value)
  {
    struct v4l2_queryctrl queryctrl;
    struct v4l2_ext_control control;

    memset(&queryctrl, 0, sizeof(queryctrl));
    queryctrl.id = V4L2_CID_FOCUS_AUTO;

    if (-1 == xioctl(fd_, VIDIOC_QUERYCTRL, &queryctrl))
    {
      if (errno != EINVAL)
      {
        perror("VIDIOC_QUERYCTRL");
        return;
      }
      else
      {
        ROS_INFO("V4L2_CID_FOCUS_AUTO is not supported");
        return;
      }
    }
    else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
    {
      ROS_INFO("V4L2_CID_FOCUS_AUTO is not supported");
      return;
    }
    else
    {
      memset(&control, 0, sizeof(control));
      control.id = V4L2_CID_FOCUS_AUTO;
      control.value = value;

      if (-1 == xioctl(fd_, VIDIOC_S_CTRL, &control))
      {
        perror("VIDIOC_S_CTRL");
        return;
      }
    }
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    //
    if (packet_.buffer_index >= 0) {
      ROS_ERROR("last packet is not cleared. call write() first.");
      return;
    }

    // pop buffer
    v4l2_buffer buffer;
    std::memset(&buffer, 0, sizeof(buffer));
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_DQBUF, &buffer) < 0) {
      switch (errno) {
      case EAGAIN:
        /* */
        return;
      case EIO:
        // TODO: code here is just copied from the original usb_cam.
        //       understand why falling through is ok.
        break;
      default:
        ROS_ERROR("Cannot dequeue buffer");
        return;
      }
    }

    // fill packet info with the poped buffer
    packet_.stamp = ros::Time::now();
    packet_.start = buffers_[buffer.index].start;
    packet_.length = buffer.bytesused;
    packet_.buffer_index = buffer.index;
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // push buffer
    if (packet_.buffer_index >= 0) {
      v4l2_buffer buffer;
      std::memset(&buffer, 0, sizeof(buffer));
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_MMAP;
      buffer.index = packet_.buffer_index;
      if (xioctl(fd_, VIDIOC_QBUF, &buffer) == 0) {
        packet_.stamp = ros::Time(0);
        packet_.start = NULL;
        packet_.length = 0;
        packet_.buffer_index = -1;
      } else {
        ROS_ERROR("Cannot enqueue buffer");
      }
    }
  }

private:
  bool uninit() {
    // stop streaming
    {
      v4l2_buf_type buf_type;
      buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (xioctl(fd_, VIDIOC_STREAMOFF, &buf_type) < 0) {
        ROS_ERROR("Cannot stop streaming");
        return false;
      }
    }

    // unmap memory
    for (Buffer &buffer : buffers_) {
      if (munmap(buffer.start, buffer.length) < 0) {
        ROS_ERROR("Cannot unmap memory");
        return false;
      }
    }
    buffers_.clear();

    // close device
    if (close(fd_) < 0) {
      ROS_ERROR("Cannot close the device");
      return false;
    }

    return true;
  }

  static int xioctl(int fd, int request, void *arg) {
    int result;
    do {
      result = ioctl(fd, request, arg);
      // retry if failed because of temporary signal interruption
    } while (result < 0 && errno == EINTR);
    return result;
  }

private:
  struct Packet {
    ros::Time stamp;
    const void *start;
    std::size_t length;
    int buffer_index;
  };
  struct Buffer {
    void *start;
    std::size_t length;
  };

  int fd_;
  std::string video_device_;

  usb_cam_hardware_interface::PacketInterface packet_interface_;
  Packet packet_;

  std::vector< Buffer > buffers_;
};

} // namespace usb_cam_hardware

#endif

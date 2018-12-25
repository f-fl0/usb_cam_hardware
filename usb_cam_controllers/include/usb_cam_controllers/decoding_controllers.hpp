#ifndef USB_CAM_CONTROLLERS_DECODING_CONTROLLERS
#define USB_CAM_CONTROLLERS_DECODING_CONTROLLERS

#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <usb_cam_controllers/simple_packet_controller.hpp>
#include <usb_cam_hardware_interface/packet_interface.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

namespace usb_cam_controllers {

template < AVCodecID CodecId > class DecodingController : public SimplePacketController {
public:
  DecodingController() {}

  virtual ~DecodingController() {}

protected:
  virtual bool initImpl(usb_cam_hardware_interface::PacketInterface *hw, ros::NodeHandle &root_nh,
                        ros::NodeHandle &controller_nh) {
    // init libavcodec
    avcodec_register_all();
    av_log_set_level(AV_LOG_FATAL);

    // find h264 decoder
    AVCodec *const decoder(avcodec_find_decoder(CodecId));
    if (!decoder) {
      ROS_ERROR_STREAM("Cannot find the decoder (codec id: " << CodecId << ")");
      return false;
    }

    // allocate h264 decoder context
    decoder_ctx_.reset(avcodec_alloc_context3(decoder), deleteAVCodecContext);
    if (!decoder_ctx_) {
      ROS_ERROR_STREAM("Cannot allocate a decoder context (codec id: " << CodecId << ")");
      return false;
    }

    // open decoder
    if (avcodec_open2(decoder_ctx_.get(), decoder, NULL) < 0) {
      ROS_ERROR_STREAM("Failed to open the codec (codec id: " << CodecId << ")");
      return false;
    }

    encoding_ = controller_nh.param< std::string >("encoding", sensor_msgs::image_encodings::BGR8);

    // init publisher for decoded images
    publisher_ = image_transport::ImageTransport(controller_nh).advertise("image", 1);

    return true;
  }

  virtual void startingImpl(const ros::Time &time) {
    // nothing to do
  }

  virtual void updateImpl(const ros::Time &time, const ros::Duration &period) {
    // set packet data from the input message
    AVPacket packet;
    av_init_packet(&packet);
    packet.size = packet_iface_.getLength();
    packet.data = const_cast< uint8_t * >(packet_iface_.getStartAs< uint8_t >());

    // repeat decoding until all data in the packet are consumed
    while (packet.size > 0) {
      // decode one frame
      boost::shared_ptr< AVFrame > frame(av_frame_alloc(), deleteAVFrame);
      int got_frame;
      const int len(avcodec_decode_video2(decoder_ctx_.get(), frame.get(), &got_frame, &packet));
      if (len < 0) {
        ROS_ERROR("Cannot decode a frame");
        return;
      }

      // publish the decoded frame
      if (got_frame > 0) {
        // allocate output message
        const sensor_msgs::ImagePtr out(new sensor_msgs::Image());
        out->header.stamp = packet_iface_.getStamp();
        out->height = frame->height;
        out->width = frame->width;
        out->encoding = encoding_;
        out->step = 3 * frame->width;
        out->data.resize(3 * frame->width * frame->height);

        // layout data by converting color spaces (YUV -> RGB)
        boost::shared_ptr< SwsContext > convert_ctx(
            sws_getContext(
                // src formats
                frame->width, frame->height,
                toUndeprecated(static_cast< AVPixelFormat >(frame->format)),
                // dst formats
                frame->width, frame->height, AV_PIX_FMT_BGR24,
                // flags & filters
                SWS_FAST_BILINEAR, NULL, NULL, NULL),
            sws_freeContext);
        int stride = 3 * frame->width;
        uint8_t *dst = &out->data[0];
        sws_scale(convert_ctx.get(),
                  // src data
                  frame->data, frame->linesize, 0, frame->height,
                  // dst data
                  &dst, &stride);

        publisher_.publish(out);
      }

      // consume data in the packet
      packet.size -= len;
      packet.data += len;
    }
  }

  virtual void stoppingImpl(const ros::Time &time) {
    // nothing to do
  }

private:
  // Deleter for auto free/close of libav objects
  static void deleteAVFrame(AVFrame *frame) {
    if (frame) {
      av_frame_free(&frame);
    }
  }

  static void deleteAVCodecContext(AVCodecContext *ctx) {
    if (ctx) {
      avcodec_free_context(&ctx);
    }
  }

  // Workaround to avoid deprecated pixel format warning
  static AVPixelFormat toUndeprecated(const AVPixelFormat format) {
    switch (format) {
    case AV_PIX_FMT_YUVJ420P:
      return AV_PIX_FMT_YUV420P;
    case AV_PIX_FMT_YUVJ411P:
      return AV_PIX_FMT_YUV411P;
    case AV_PIX_FMT_YUVJ422P:
      return AV_PIX_FMT_YUV422P;
    case AV_PIX_FMT_YUVJ440P:
      return AV_PIX_FMT_YUV440P;
    case AV_PIX_FMT_YUVJ444P:
      return AV_PIX_FMT_YUV444P;
    default:
      return format;
    }
  }

private:
  std::string encoding_;

  boost::shared_ptr< AVCodecContext > decoder_ctx_;
  image_transport::Publisher publisher_;
};

typedef DecodingController< AV_CODEC_ID_H264 > H264Controller;
typedef DecodingController< AV_CODEC_ID_MJPEG > MjpegController;

} // namespace usb_cam_controllers

#endif
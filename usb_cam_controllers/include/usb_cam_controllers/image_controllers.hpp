#ifndef USB_CAM_CONTROLLERS_IMAGE_CONTROLLERS
#define USB_CAM_CONTROLLERS_IMAGE_CONTROLLERS

#include <string>

#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/image_encodings.h>
#include <usb_cam_controllers/format_controller.hpp>
#include <usb_cam_hardware_interface/packet_interface.hpp>

namespace usb_cam_controllers {

const uint8_t uchar_clipping_table[] = {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, // -128 - -121
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, // -120 - -113
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, // -112 - -105
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, // -104 -  -97
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -96 -  -89
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -88 -  -81
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -80 -  -73
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -72 -  -65
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -64 -  -57
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -56 -  -49
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -48 -  -41
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -40 -  -33
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -32 -  -25
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -24 -  -17
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //  -16 -   -9
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, //   -8 -   -1
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
    60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88,
    89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113,
    114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136,
    137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
    160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182,
    183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205,
    206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228,
    229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251,
    252, 253, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, // 256-263
    255, 255, 255, 255, 255, 255, 255, 255, // 264-271
    255, 255, 255, 255, 255, 255, 255, 255, // 272-279
    255, 255, 255, 255, 255, 255, 255, 255, // 280-287
    255, 255, 255, 255, 255, 255, 255, 255, // 288-295
    255, 255, 255, 255, 255, 255, 255, 255, // 296-303
    255, 255, 255, 255, 255, 255, 255, 255, // 304-311
    255, 255, 255, 255, 255, 255, 255, 255, // 312-319
    255, 255, 255, 255, 255, 255, 255, 255, // 320-327
    255, 255, 255, 255, 255, 255, 255, 255, // 328-335
    255, 255, 255, 255, 255, 255, 255, 255, // 336-343
    255, 255, 255, 255, 255, 255, 255, 255, // 344-351
    255, 255, 255, 255, 255, 255, 255, 255, // 352-359
    255, 255, 255, 255, 255, 255, 255, 255, // 360-367
    255, 255, 255, 255, 255, 255, 255, 255, // 368-375
    255, 255, 255, 255, 255, 255, 255, 255, // 376-383
    };
const int clipping_table_offset = 128;

/** Clip a value to the range 0<val<255. For speed this is done using an
 * array, so can only cope with numbers in the range -128<val<383.
 */
static uint8_t CLIPVALUE(int val)
{
  // Old method (if)
  /*   val = val < 0 ? 0 : val; */
  /*   return val > 255 ? 255 : val; */

  // New method (array)
  return uchar_clipping_table[val + clipping_table_offset];
}

/**
 * Conversion from YUV to RGB.
 * The normal conversion matrix is due to Julien (surname unknown):
 *
 * [ R ]   [  1.0   0.0     1.403 ] [ Y ]
 * [ G ] = [  1.0  -0.344  -0.714 ] [ U ]
 * [ B ]   [  1.0   1.770   0.0   ] [ V ]
 *
 * and the firewire one is similar:
 *
 * [ R ]   [  1.0   0.0     0.700 ] [ Y ]
 * [ G ] = [  1.0  -0.198  -0.291 ] [ U ]
 * [ B ]   [  1.0   1.015   0.0   ] [ V ]
 *
 * Corrected by BJT (coriander's transforms RGB->YUV and YUV->RGB
 *                   do not get you back to the same RGB!)
 * [ R ]   [  1.0   0.0     1.136 ] [ Y ]
 * [ G ] = [  1.0  -0.396  -0.578 ] [ U ]
 * [ B ]   [  1.0   2.041   0.002 ] [ V ]
 *
 */
static void YUV2RGB(const uint8_t y, const uint8_t u, const uint8_t v, uint8_t* r,
                    uint8_t* g, uint8_t* b)
{
  const int y2 = (int)y;
  const int u2 = (int)u - 128;
  const int v2 = (int)v - 128;
  //std::cerr << "YUV=("<<y2<<","<<u2<<","<<v2<<")"<<std::endl;

  // This is the normal YUV conversion, but
  // appears to be incorrect for the firewire cameras
  //   int r2 = y2 + ( (v2*91947) >> 16);
  //   int g2 = y2 - ( ((u2*22544) + (v2*46793)) >> 16 );
  //   int b2 = y2 + ( (u2*115999) >> 16);
  // This is an adjusted version (UV spread out a bit)
  int r2 = y2 + ((v2 * 37221) >> 15);
  int g2 = y2 - (((u2 * 12975) + (v2 * 18949)) >> 15);
  int b2 = y2 + ((u2 * 66883) >> 15);
  //std::cerr << "   RGB=("<<r2<<","<<g2<<","<<b2<<")"<<std::endl;

  // Cap the values.
  *r = CLIPVALUE(r2);
  *g = CLIPVALUE(g2);
  *b = CLIPVALUE(b2);
}

static void uyvy2rgb(uint8_t *YUV, int width, int height, uint8_t *RGB,
  int pad_left, int pad_top, int pad_right, int pad_bottom)
{
  int i, j;
  uint8_t y0, y1, u, v;
  uint8_t r, g, b;

  int NumPixels = width * height;
  // skip top padding pixels
  int k = (width + pad_left + pad_right) * pad_top * 3;
  // skip top padding pixels
  k += pad_left * 3;
  int p = 0;
  for (i = 0; i < (NumPixels << 1); i += 4)
  {
    u = (uint8_t)YUV[i + 0];
    y0 = (uint8_t)YUV[i + 1];
    v = (uint8_t)YUV[i + 2];
    y1 = (uint8_t)YUV[i + 3];

    YUV2RGB(y0, u, v, &r, &g, &b);
    RGB[k + 0] = r;
    RGB[k + 1] = g;
    RGB[k + 2] = b;
    p++;
    k += 3;
    if (p % width == 0) {
      k += (pad_left + pad_right) * 3;
      p = 0;
    }

    YUV2RGB(y1, u, v, &r, &g, &b);
    RGB[k + 0] = r;
    RGB[k + 1] = g;
    RGB[k + 2] = b;
    k += 3;
    p++;
    if (p % width == 0) {
      k += (pad_left + pad_right) * 3;
      p = 0;
    }
  }
}

static void yuyv2rgb(uint8_t *YUV, int width, int height, uint8_t *RGB,
  int pad_left, int pad_top, int pad_right, int pad_bottom)
{
  int i, j;
  uint8_t y0, y1, u, v;
  uint8_t r, g, b;

  int NumPixels = width * height;
  // skip top padding pixels
  int k = (width + pad_left + pad_right) * pad_top * 3;
  // skip top padding pixels
  k += pad_left * 3;
  int p = 0;
  for (i = 0; i < (NumPixels << 1); i += 4)
  {
    y0 = (uint8_t)YUV[i + 0];
    u = (uint8_t)YUV[i + 1];
    y1 = (uint8_t)YUV[i + 2];
    v = (uint8_t)YUV[i + 3];

    YUV2RGB(y0, u, v, &r, &g, &b);
    RGB[k + 0] = r;
    RGB[k + 1] = g;
    RGB[k + 2] = b;
    p++;
    k += 3;
    if (p % width == 0) {
      k += (pad_left + pad_right) * 3;
      p = 0;
    }

    YUV2RGB(y1, u, v, &r, &g, &b);
    RGB[k + 0] = r;
    RGB[k + 1] = g;
    RGB[k + 2] = b;
    k += 3;
    p++;
    if (p % width == 0) {
      k += (pad_left + pad_right) * 3;
      p = 0;
    }
  }
}

const std::string UYVY2RGB = "uyvy2rgb";
const std::string YUYV2RGB = "yuyv2rgb";

template < const std::string *ConversionCode, const std::string *DstEncoding >
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

    publisher_ = image_transport::ImageTransport(controller_nh).advertise("image", 1);

    return true;
  }

  virtual void updateImpl(const ros::Time &time, const ros::Duration &period) {
    sensor_msgs::Image img;
    img.header.stamp = packet_iface_.getStamp();
    img.encoding = *DstEncoding;
    const int ch = sensor_msgs::image_encodings::numChannels(img.encoding);
    img.height = height_ + padding_top_ + padding_bottom_;
    img.width = width_ + padding_left_ + padding_right_;
    img.step = img.width * ch;
    img.data.resize(img.width * img.height * ch);

    if (*ConversionCode == UYVY2RGB) {
      uyvy2rgb(const_cast< uint8_t * >(packet_iface_.getStartAs< uint8_t >()),
        width_, height_,
        img.data.data(),
        padding_left_, padding_top_, padding_right_, padding_bottom_);
    } else if (*ConversionCode == YUYV2RGB) {
      yuyv2rgb(const_cast< uint8_t * >(packet_iface_.getStartAs< uint8_t >()),
        width_, height_,
        img.data.data(),
        padding_left_, padding_top_, padding_right_, padding_bottom_);
    } else {
      return;
    }

    // TODO
    if (rotate_code_ != ROTATE_NONE) {

    }

    publisher_.publish(img);
  }

private:

  void rotate(const uint8_t *src, uint8_t *dst, const int row, const int col, const int ch, const RotateCode rotate_code)
  {
    if (rotate_code == ROTATE_90_CW)
    {
      for (int i = 0; i < row; i++)
      {
        for (int j = 0; j < col; j++)
        {
          for (int c = 0; c < ch; c++)
          {
            dst[(row * (j + 1) - 1 - i) * ch + c] = src[(col * i + j) * ch + c];
          }
        }
      }
    }
    else if (rotate_code == ROTATE_90_CCW)
    {
      for (int i = 0; i < row; i++)
      {
        for (int j = 0; j < col; j++)
        {
          for (int c = 0; c < ch; c++)
          {
            dst[(row * (col - 1 - j) + i) * ch + c] = src[(col * i + j) * ch + c];
          }
        }
      }
    }
    else if (rotate_code == ROTATE_180)
    {
      for (int i = 0; i < row; i++)
      {
        for (int j = 0; j < col; j++)
        {
          for (int c = 0; c < ch; c++)
          {
            dst[(col * (row - i) - 1 - j) * ch + c] = src[(col * i + j) * ch + c];
          }
        }
      }
    }
  }

  image_transport::Publisher publisher_;
};

typedef ImageController<&UYVY2RGB, &sensor_msgs::image_encodings::RGB8 >
    UYVYController;
typedef ImageController<&YUYV2RGB, &sensor_msgs::image_encodings::RGB8 >
    YUYVController;

} // namespace usb_cam_controllers

#endif

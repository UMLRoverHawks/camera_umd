#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include "uvc_cam/uvc_cam.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"
#include "image_transport/image_transport.h"

#include "uvc_camera/camera.h"

using namespace sensor_msgs;

namespace uvc_camera
{

  Camera::Camera(ros::NodeHandle _comm_nh, ros::NodeHandle _param_nh) :
      node(_comm_nh), pnode(_param_nh), it(_comm_nh),
          info_mgr(_comm_nh, "camera"), cam(0)
  {

    /* default config values, can be changed, most are self explanatory */
    width = 640;
    height = 480;
    fps = 10;
    skip_frames = 0;
    frames_to_skip = 0;
    device = "/dev/video0";
    frame = "camera";
    rotate = false;
    brightness = 128;
    contrast = 32;
    wbt = 5984; //will be ignored unless next value is 0 (white balance temp)
    wbtauto = 1;
    plf = 2;
    gain = 200;
    sharpness = 224;
    backlight = 1;
    focusauto = 1;
    focus = 16; //will be ignored unless prev value is 0
    saturation = 32;
    pan = 0;
    tilt = 0;
    expabs = 250;
    expauto = 1;
    expautop = 1;

    /* set up information manager */
    std::string url;

    pnode.getParam("camera_info_url", url);

    info_mgr.loadCameraInfo(url);

    /* pull other configuration */
    pnode.getParam("device", device);

    pnode.getParam("fps", fps);
    pnode.getParam("skip_frames", skip_frames);

    pnode.getParam("width", width);
    pnode.getParam("height", height);

    pnode.getParam("frame_id", frame);

    pnode.getParam("brightness", brightness);
    pnode.getParam("contrast", contrast);
    pnode.getParam("wbt", wbt);
    pnode.getParam("wbtauto", wbtauto);
    pnode.getParam("plf", plf);
    pnode.getParam("gain", gain);
    pnode.getParam("sharpness", sharpness);
    pnode.getParam("backlight", backlight);
    pnode.getParam("focusauto", focusauto);
    pnode.getParam("focus", focus);
    pnode.getParam("saturation", saturation);
    pnode.getParam("pan", pan);
    pnode.getParam("tilt", tilt);
    pnode.getParam("expabs", expabs);
    pnode.getParam("expauto", expauto);
    pnode.getParam("expautop", expautop);

    std::string modestr;
    uvc_cam::Cam::mode_t mode;
    pnode.param("mode", modestr, std::string("MJPG"));
    if (modestr.find("MJPG"))
    {
      mode = uvc_cam::Cam::MODE_MJPG;
    }
    else if (modestr.find("RGB"))
    {
      mode = uvc_cam::Cam::MODE_RGB;
    }
    else if (modestr.find("YUYV"))
    {
      mode = uvc_cam::Cam::MODE_YUYV;
    }
    else
    {
      ROS_ERROR("Unsupported mode specified");
      node.shutdown();
      pnode.shutdown();
      return;
    }

    std::string output_topic;
    std::string info_topic;

    pnode.getParam("info_topic", info_topic);
    pnode.getParam("output_topic", output_topic);
 //   pnode.param("output_topic", output_topic, std::string("image_raw"));
 //   pnode.param("info_topic", info_topic, std::string("image_raw"));
    /* advertise image streams and info streams */
    pub = it.advertise(output_topic.c_str(), 1);

    info_pub = node.advertise<CameraInfo>(info_topic.c_str(), 1);

    /* initialize the cameras */
    try
    {
      cam = new uvc_cam::Cam(device.c_str(), mode, width, height, fps, brightness, contrast, wbt, wbtauto, plf, gain, sharpness, backlight, focusauto, focus, saturation, pan, tilt, expabs, expauto, expautop);
      cam->set_motion_thresholds(100, -1);
    }
    catch (std::exception &e)
    {
      ROS_ERROR("Unsupported mode specified - %s", e.what());
      node.shutdown();
      pnode.shutdown();
      return;
    }

    /* and turn on the streamer */
    ok = true;
    image_thread = boost::thread(boost::bind(&Camera::feedImages, this));
  }

  void
  Camera::sendInfo(ImagePtr &image, ros::Time time)
  {
    CameraInfoPtr info(new CameraInfo(info_mgr.getCameraInfo()));

    /* Throw out any CamInfo that's not calibrated to this camera mode */
    if (info->K[0] != 0.0
        && (image->width != info->width || image->height != info->height))
    {
      info.reset(new CameraInfo());
    }

    /* If we don't have a calibration, set the image dimensions */
    if (info->K[0] == 0.0)
    {
      info->width = image->width;
      info->height = image->height;
    }

    info->header.stamp = time;
    info->header.frame_id = frame;

    info_pub.publish(info);
  }

  void
  Camera::feedImages()
  {
    unsigned int pair_id = 0;
    while (ok)
    {
      unsigned char *img_frame = NULL;
      uint32_t bytes_used;

      ros::Time capture_time = ros::Time::now();

      int idx = cam->grab(&img_frame, bytes_used);

      /* Read in every frame the camera generates, but only send each
       * (skip_frames + 1)th frame. It's set up this way just because
       * this is based on Stereo...
       */
      if (skip_frames == 0 || frames_to_skip == 0)
      {
        if (img_frame)
        {
          ImagePtr image(new Image);

          image->height = height;
          image->width = width;
          image->step = 3 * width;
          image->encoding = image_encodings::RGB8;

          image->header.stamp = capture_time;
          image->header.seq = pair_id;

          image->header.frame_id = frame;

          image->data.resize(image->step * image->height);

          memcpy(&image->data[0], img_frame, width * height * 3);

          pub.publish(image);

          sendInfo(image, capture_time);

          ++pair_id;
        }

        frames_to_skip = skip_frames;
      }
      else
      {
        frames_to_skip--;
      }

      if (img_frame)
        cam->release(idx);
    }
  }

  Camera::~Camera()
  {
    ok = false;
    image_thread.join();
    if (cam)
      delete cam;
  }

}
;


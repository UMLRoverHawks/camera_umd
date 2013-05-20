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
    width = 1184;
    height = 656;
    fps = 30;
    skip_frames = 0;
    frames_to_skip = 0;
    device = "/dev/video0";
    frame = "camera";
    rotate = false;
    brightness = 128;
    contrast = 33;
    wbt = 5315; //will be ignored unless next value is 0 (white balance temp)
    wbtauto = 1;
    plf = 2; //power line frequency
    gain = 64;
    sharpness = 72;
    backlight = 0;
    focusauto = 1;
    focus = 16; //will be ignored unless prev value is 0
    saturation = 32;
    pan = 0;
    tilt = 0;
    expabs = 166;
    expauto = 3;
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
      mode = uvc_cam::Cam::MODE_MJPG;
    else if (modestr.find("RGB"))
      mode = uvc_cam::Cam::MODE_RGB;
    else if (modestr.find("YUYV"))
      mode = uvc_cam::Cam::MODE_YUYV;
    else
    {
      ROS_ERROR("%s is an unsupported mode! Using MJPG", modestr.c_str());
      mode = uvc_cam::Cam::MODE_MJPG;
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

    // parameters settings subscribers
    brightness_sub = node.subscribe("brightness", 1000, &Camera::brightnessCallback, this);
    contrast_sub = node.subscribe("contrast", 1000, &Camera::contrastCallback, this);
    exposure_sub = node.subscribe("exposure", 1000, &Camera::exposureCallback, this);
    exposureauto_sub = node.subscribe("exposureauto", 1000, &Camera::exposureautoCallback, this);
    exposureautoprio_sub = node.subscribe("exposureautoprio", 1000, &Camera::exposureautoprioCallback, this);
    wbtauto_sub = node.subscribe("wbtauto", 1000, &Camera::wbtautoCallback, this);
    wbt_sub = node.subscribe("wbt", 1000, &Camera::wbtCallback, this);
    gain_sub = node.subscribe("gain", 1000, &Camera::gainCallback, this);
    sharpness_sub = node.subscribe("sharpness", 1000, &Camera::sharpnessCallback, this);
    saturation_sub = node.subscribe("saturation", 1000, &Camera::saturationCallback, this);
    focusauto_sub = node.subscribe("focusauto", 1000, &Camera::focusautoCallback, this);
    focus_sub = node.subscribe("focus", 1000, &Camera::focusCallback, this);
    backlight_sub = node.subscribe("backlight", 1000, &Camera::backlightCallback, this);
    pan_sub = node.subscribe("pan", 1000, &Camera::panCallback, this);
    tilt_sub = node.subscribe("tilt", 1000, &Camera::tiltCallback, this);
   
    // publish camera parameter info 
    brightness_pub = node.advertise<std_msgs::Int32>("brightness_info", 1); // for UI
    contrast_pub = node.advertise<std_msgs::Int32>("contrast_info", 1); // for UI
    exposure_pub = node.advertise<std_msgs::Int32>("exposure_info", 1); // for UI
    wbt_pub = node.advertise<std_msgs::Int32>("wbt_info", 1); // for UI

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
  Camera::sendParameterInfo()
  {
    // the following parameters use std_msgs::Int32
    std_msgs::Int32 msg; 
    msg.data = brightness; 
    brightness_pub.publish( msg );
    msg.data = contrast;
    contrast_pub.publish( msg );
    msg.data = expabs;  
    exposure_pub.publish( msg );
    msg.data = wbt;
    wbt_pub.publish( msg );
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

	  // publish image
          pub.publish(image);
	  // send camera info
          sendInfo(image, capture_time);
	  // publish camera parameter info
	  sendParameterInfo();

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
  
  void Camera::brightnessCallback(const std_msgs::Int32::ConstPtr& msg)
  {
      if(msg->data >= 0 && msg->data <=255)
      {
        ROS_INFO("Setting Brightness to: [%d]", msg->data);
        cam->set_control(9963776,msg->data);
	brightness = msg->data;
      }
  }

  void Camera::contrastCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    if(msg->data >= 0 && msg->data <=255)
      {
        ROS_INFO("Setting Contrast to: [%d]", msg->data);
        cam->set_control(9963777,msg->data);
	contrast = msg->data;
      }
  }
  
    void Camera::exposureCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    if(msg->data >= 3 && msg->data <=2047)
      {
        ROS_INFO("Setting Exposure(Absolute) to: [%d]", msg->data);
	cam->set_control(10094850,msg->data);
	expabs = msg->data;
      }
  }
  
      void Camera::exposureautoCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    if(msg->data == 1 || msg->data ==3)
      {
        ROS_INFO("Setting Exposure(Auto) to: [%d]", msg->data);
	cam->set_control(10094849,msg->data);
      }
  }
  
      void Camera::exposureautoprioCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    if(msg->data >= 0 && msg->data <=1)
      {
        ROS_INFO("Setting Exposure Auto Priority to: [%d]", msg->data);
	cam->set_control(10094851,msg->data);
      }
  }
  
   void Camera::backlightCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    if(msg->data >= 0 && msg->data <=1)
      {
         ROS_INFO("Setting Backlight compensation to: [%d]", msg->data);
	cam->set_control(9963804,msg->data);
      }
  }
  
    void Camera::wbtautoCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    if(msg->data >= 0 && msg->data <=1)
      {
        ROS_INFO("Setting WBT (Auto) to: [%d]", msg->data);
        cam->set_control(9963788,msg->data);
      }
  }
  
    void Camera::wbtCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    if(msg->data >= 2800 && msg->data <=6500)
      {
        ROS_INFO("Setting WBT(Absolute) to: [%d]", msg->data);
	cam->set_control(9963802,msg->data);
      }
      wbt = msg->data;
  }
    void Camera::gainCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    if(msg->data >= 0 && msg->data <=255)
      {
        ROS_INFO("Setting gain to: [%d]", msg->data);
	cam->set_control(9963795,msg->data);
      }
  }
    void Camera::sharpnessCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    if(msg->data >= 0 && msg->data <=255)
      {
        ROS_INFO("Setting sharpness to: [%d]", msg->data);
	cam->set_control(9963803,msg->data);
      }
  }
    void Camera::saturationCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    if(msg->data >= 0 && msg->data <=255)
      {
        ROS_INFO("Setting saturation to: [%d]", msg->data);
        cam->set_control(9963778,msg->data);
      }
  }
      void Camera::focusautoCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    if(msg->data >= 0 && msg->data <=1)
      {
        ROS_INFO("Setting focusauto to: [%d]", msg->data);
	cam->set_control(10094860,msg->data);
      }
  }
  void Camera::focusCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    if(msg->data >= 0 && msg->data <=255)
      {
         cam->set_control(10094858,msg->data);
      }
   } 
  void Camera::tiltCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    if(msg->data >= -36000 && msg->data <=36000)
      {
         cam->set_control(10094857,msg->data);
      }
  }  
  void Camera::panCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    if(msg->data >=-36000 && msg->data <=36000)
      {
         cam->set_control(10094856,msg->data);
      }
   }
}
;


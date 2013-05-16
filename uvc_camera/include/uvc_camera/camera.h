#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "uvc_cam/uvc_cam.h"
#include "std_msgs/Int32.h"
#include <boost/thread.hpp>
#include <camera_info_manager/camera_info_manager.h>

namespace uvc_camera {

class Camera {
  public:
    Camera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh);
    void onInit();
    void sendInfo(sensor_msgs::ImagePtr &image, ros::Time time);
    void feedImages();
    void brightnessCallback(const std_msgs::Int32::ConstPtr& msg);
    void contrastCallback(const std_msgs::Int32::ConstPtr& msg);
    void exposureCallback(const std_msgs::Int32::ConstPtr& msg);
    void exposureautoCallback(const std_msgs::Int32::ConstPtr& msg);
    void exposureautoprioCallback(const std_msgs::Int32::ConstPtr& msg);
    void backlightCallback(const std_msgs::Int32::ConstPtr& msg);
    void wbtautoCallback(const std_msgs::Int32::ConstPtr& msg);
    void wbtCallback(const std_msgs::Int32::ConstPtr& msg);
    void gainCallback(const std_msgs::Int32::ConstPtr& msg);
    void sharpnessCallback(const std_msgs::Int32::ConstPtr& msg);
    void saturationCallback(const std_msgs::Int32::ConstPtr& msg);  
    void focusautoCallback(const std_msgs::Int32::ConstPtr& msg);  
    void focusCallback(const std_msgs::Int32::ConstPtr& msg);  
    void panCallback(const std_msgs::Int32::ConstPtr& msg);  
    void tiltCallback(const std_msgs::Int32::ConstPtr& msg);  
  ~Camera();
  
  private:
    ros::NodeHandle node, pnode;
    image_transport::ImageTransport it;
    bool ok;

    int width, height, fps, skip_frames, frames_to_skip;
    std::string device, frame;
    int brightness, contrast, wbt, wbtauto, plf, gain, sharpness, backlight, focusauto, focus, saturation, pan, tilt, expabs, expauto, expautop;
    bool rotate;

    camera_info_manager::CameraInfoManager info_mgr;

    image_transport::Publisher pub;
    ros::Publisher info_pub;
    ros::Subscriber brightness_sub;
    ros::Subscriber contrast_sub;
    ros::Subscriber exposure_sub;
    ros::Subscriber exposureauto_sub;
    ros::Subscriber exposureautoprio_sub;
    ros::Subscriber backlight_sub;
    ros::Subscriber wbtauto_sub;
    ros::Subscriber wbt_sub;
    ros::Subscriber gain_sub;
    ros::Subscriber sharpness_sub;
    ros::Subscriber saturation_sub;  
    ros::Subscriber focusauto_sub;  
    ros::Subscriber focus_sub;  
    ros::Subscriber pan_sub;  
    ros::Subscriber tilt_sub;  
    uvc_cam::Cam *cam;
    boost::thread image_thread;
};

};


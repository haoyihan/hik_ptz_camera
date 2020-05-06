#if !defined(_HIK_CAMERA_H_)
#define _HIK_CAMERA_H_

#include <string>
#include <ros/ros.h>
#include <hik_ptz_camera/PtzCtrl.h>
#include <hik_ptz_camera/PtzStatus.h>
#include <hik_ptz_camera/DvrGetCfg.h>
#include <hik_ptz_camera/DvrSetCfg.h>
#include <hik_ptz_camera/FocusCtrl.h>
#include <hik_ptz_camera/FocusStatus.h>
#include <hik_ptz_camera/FastGo.h>


namespace hik_camera
{
using namespace std;

class HikCamera{

    public:
        HikCamera();
        ~HikCamera();
        void run();
    private:
        int _loginHandle;
        // long _playHandle;
        // DH_Channel_Info _dwUser;
        void initialize();
        bool login(string ip_address, short port, string user_name, string passwd);
        void logout();
        void cleanup();

        ros::NodeHandle _nh;
        ros::ServiceServer _ptz_ctrl_service;
        ros::ServiceServer _ptz_status_service;
        ros::ServiceServer _dvr_get_cfg_service;
        ros::ServiceServer _dvr_set_cfg_service;
        ros::ServiceServer _focus_status_service;
        ros::ServiceServer _focus_ctrl_service;
        ros::ServiceServer _fast_go_service;

        bool ptzCtrlCB(hik_ptz_camera::PtzCtrl::Request  &req, hik_ptz_camera::PtzCtrl::Response &res);
        bool ptzStatCB(hik_ptz_camera::PtzStatus::Request  &req, hik_ptz_camera::PtzStatus::Response &res);
        bool dvrGetCfgCB(hik_ptz_camera::DvrGetCfg::Request  &req, hik_ptz_camera::DvrGetCfg::Response &res);
        bool dvrSetCfgCB(hik_ptz_camera::DvrSetCfg::Request  &req, hik_ptz_camera::DvrSetCfg::Response &res);
        bool focusCtrlCB(hik_ptz_camera::FocusCtrl::Request  &req, hik_ptz_camera::FocusCtrl::Response &res);
        bool focusStatCB(hik_ptz_camera::FocusStatus::Request  &req, hik_ptz_camera::FocusStatus::Response &res);
        bool fastGoCB(hik_ptz_camera::FastGo::Request  &req, hik_ptz_camera::FastGo::Response &res);

};



}

#endif // _HIK_CAMERA_H_

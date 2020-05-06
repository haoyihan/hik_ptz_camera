#include "hik_camera.h"
#include "HCNetSDK.h"
#include <unistd.h>
#include <algorithm>


namespace hik_camera
{
    HikCamera::HikCamera(){
        initialize();
    }

    HikCamera::~HikCamera(){
        cleanup();
    }

    static std::string get_repo_home_path(){
        std::string pc_home_path = getenv("HOME");
        std::string repo_name_path = pc_home_path+"/.env/home_repo";
        FILE *fd = fopen ( repo_name_path.c_str(), "r" );
        char repo_name[50];
        fgets(repo_name,sizeof(repo_name),fd);
        repo_name[strlen(repo_name)-1]='\0';
        fclose(fd);
        return pc_home_path + "/" + repo_name;
    }

    static void Demo_SDK_Version()
    {
        unsigned int uiVersion = NET_DVR_GetSDKBuildVersion();

        char strTemp[1024] = {0};
        sprintf(strTemp, "HCNetSDK V%d.%d.%d.%d\n", \
            (0xff000000 & uiVersion)>>24, \
            (0x00ff0000 & uiVersion)>>16, \
            (0x0000ff00 & uiVersion)>>8, \
            (0x000000ff & uiVersion));
        printf(strTemp);
    }

    static int dec_to_hex(int param){
        int out = 0 , ind = 0;
        char buf[10] = {0};
        while(param){
            buf[ind] = param%10;
            ind++;
            param /= 10;
        }
        for(int i=ind-1;i>=0;i--){
            out = out *16 + buf[i];
        }
        return out;
    }

    static int hex_to_dec(int param){
        int out = 0 , ind = 0;
        char buf[10] = {0};
        while(param){
            buf[ind] = param%16;
            ind++;
            param /= 16;
        }
        for(int i=ind-1;i>=0;i--){
            out = out *10 + buf[i];
        }
        return out;
    }

    void HikCamera::initialize(){
        _loginHandle = -1;
        _ptz_ctrl_service = _nh.advertiseService("ptz_ctrl",&HikCamera::ptzCtrlCB,this);
        _ptz_status_service = _nh.advertiseService("ptz_status",&HikCamera::ptzStatCB,this);
        _dvr_get_cfg_service = _nh.advertiseService("dvr_status",&HikCamera::dvrGetCfgCB,this);
        _dvr_set_cfg_service = _nh.advertiseService("dvr_setting",&HikCamera::dvrSetCfgCB,this);

        _focus_status_service = _nh.advertiseService("focus_status",&HikCamera::focusStatCB,this);

        _fast_go_service = _nh.advertiseService("fast_go",&HikCamera::fastGoCB,this);

        NET_DVR_Init();
        Demo_SDK_Version();
        NET_DVR_SetLogToFile(3, "./sdkLog");
        std::string path = get_repo_home_path() + "/src/hik_ptz_camera/lib";

        char cryptoPath[2048] = {0};
        std::string crypto_path = path + "/libcrypto.so";
        sprintf(cryptoPath, crypto_path.c_str());
        NET_DVR_SetSDKInitCfg(NET_SDK_INIT_CFG_LIBEAY_PATH, cryptoPath);

        char sslPath[2048] = {0};
        std::string ssl_path = path + "/libssl.so";
        sprintf(sslPath, ssl_path.c_str());
        NET_DVR_SetSDKInitCfg(NET_SDK_INIT_CFG_SSLEAY_PATH, sslPath); 

        NET_DVR_LOCAL_SDK_PATH sdk_path;
        sprintf(sdk_path.sPath,path.c_str());
        NET_DVR_SetSDKInitCfg(NET_SDK_INIT_CFG_SDK_PATH,(void*)&sdk_path);

    }

    bool HikCamera::ptzCtrlCB(hik_ptz_camera::PtzCtrl::Request  &req, hik_ptz_camera::PtzCtrl::Response &res){
        NET_DVR_PTZPOS ptz_pos;
        NET_DVR_POINT_FRAME point_frame;
        int middle_point_x,middle_point_y;
        unsigned int errCode;
        int std_cols = 1920 , std_rows = 1080;

        switch(req.type){
            case (hik_ptz_camera::PtzCtrl::Request::FAST_GO):
                middle_point_x = req.param_1;
                middle_point_y = req.param_2;
                point_frame.xTop = (middle_point_x * 255 / std_cols );
                point_frame.xBottom = point_frame.xTop+1 ;// (int)((target_tmp.x + target_tmp.width) / std_cols * 255);
                point_frame.yTop = (middle_point_y * 255 / std_rows);
                point_frame.yBottom = point_frame.yTop+1  ;// (int)((target_tmp.y + target_tmp.height) / std_rows * 255);
                point_frame.bCounter = 1;
                
                if(NET_DVR_PTZSelZoomIn_EX(_loginHandle,1,&point_frame)){
                    res.success = true;
                }else{
                    errCode = NET_DVR_GetLastError();
                    printf("******************************************\n");
                    printf("errCode: %d\n",errCode);
                    printf("******************************************\n");
                    res.success = false;
                }
                res.status_message = "Fast GO";
                break;
            case (hik_ptz_camera::PtzCtrl::Request::EXACT_GO):
                ptz_pos.wAction = 1;
                ptz_pos.wPanPos = dec_to_hex(req.param_1);
                ptz_pos.wTiltPos = dec_to_hex(req.param_2);
                ptz_pos.wZoomPos = dec_to_hex(req.param_3);
                if(NET_DVR_SetDVRConfig(_loginHandle,NET_DVR_SET_PTZPOS,1,(void*)&ptz_pos,sizeof(NET_DVR_PTZPOS))){
                    res.success = true;
                }else{
                    errCode = NET_DVR_GetLastError();
                    printf("******************************************\n");
                    printf("errCode: %d\n",errCode);
                    printf("******************************************\n");
                    res.success = false;
                }
                res.status_message = "Exact GO";
                break;
            case (hik_ptz_camera::PtzCtrl::Request::PUSH_FOCUS):
                if(NET_DVR_FocusOnePush(_loginHandle,1)){
                    res.success = true;
                }else{
                    errCode = NET_DVR_GetLastError();
                    printf("******************************************\n");
                    printf("errCode: %d\n",errCode);
                    printf("******************************************\n");
                    res.success = false;
                }
                res.status_message = "PUSH FOCUS";
                break;
            default:
                return false;
        }
        return true;
    }

    bool HikCamera::ptzStatCB(hik_ptz_camera::PtzStatus::Request  &req, hik_ptz_camera::PtzStatus::Response &res){
        NET_DVR_PTZPOS ptz_pos;
        unsigned int ret_len = 0;
        unsigned int errCode;
        if(!NET_DVR_GetDVRConfig(_loginHandle,NET_DVR_GET_PTZPOS,1,(void*)&ptz_pos,sizeof(NET_DVR_PTZPOS),&ret_len))
            return false;
        if(!ret_len)
            return false;
        res.nPTZPan = hex_to_dec(ptz_pos.wPanPos);
        res.nPTZTilt = hex_to_dec(ptz_pos.wTiltPos);
        res.nPTZZoom = hex_to_dec(ptz_pos.wZoomPos);
        return true;
    }
    
    bool HikCamera::fastGoCB(hik_ptz_camera::FastGo::Request  &req, hik_ptz_camera::FastGo::Response &res){
        NET_DVR_POINT_FRAME point_frame;
        unsigned int errCode;
        int std_cols = 1920 , std_rows = 1080;
        point_frame.xTop = req.xmin * 255 / std_cols;
        point_frame.xBottom = req.xmax * 255 / std_cols;// (int)((target_tmp.x + target_tmp.width) / std_cols * 255);
        point_frame.yTop = req.ymin * 255 / std_rows;
        point_frame.yBottom = req.ymax * 255 / std_rows;// (int)((target_tmp.y + target_tmp.height) / std_rows * 255);
        point_frame.bCounter = 1;
        if(NET_DVR_PTZSelZoomIn_EX(_loginHandle,1,&point_frame)){
            res.success = true;
        }else{
            errCode = NET_DVR_GetLastError();
            printf("******************************************\n");
            printf("errCode: %d\n",errCode);
            printf("******************************************\n");
            res.success = false;
        }
        return true;
    }


    bool HikCamera::focusStatCB(hik_ptz_camera::FocusStatus::Request  &req, hik_ptz_camera::FocusStatus::Response &res){
        NET_DVR_FOCUSMODE_CFG focus_cfg;
        unsigned int ret_len = 0;
        unsigned int errCode = 0;
        if(NET_DVR_GetDVRConfig(_loginHandle,NET_DVR_GET_FOCUSMODECFG,1,(void*)&focus_cfg,sizeof(focus_cfg),&ret_len)){
                    printf("---------------------------------------------\n");
                    printf("focus_cfg.dwFocusPos: %d\n",focus_cfg.dwFocusPos);
                    printf("---------------------------------------------\n");
                    res.FocusMode = focus_cfg.byFocusMode;
                    res.AutoFocusMode = focus_cfg.byAutoFocusMode;
                    res.MinFocusDistance = focus_cfg.wMinFocusDistance;
                    res.ZoomSpeedLevel = focus_cfg.byZoomSpeedLevel;
                    res.FocusSpeedLevel = focus_cfg.byFocusSpeedLevel;
                    res.OpticalZoom = focus_cfg.byOpticalZoom;
                    res.DigtitalZoom = focus_cfg.byDigtitalZoom;
                    res.OpticalZoomLevel = focus_cfg.fOpticalZoomLevel;
                    res.FocusPos = focus_cfg.dwFocusPos;
                    res.FocusDefinitionDisplay = focus_cfg.byFocusDefinitionDisplay;
                    res.FocusSensitivity = focus_cfg.byFocusSensitivity;
                    res.RelativeFocusPos = focus_cfg.dwRelativeFocusPos;
                }else{
                    errCode = NET_DVR_GetLastError();
                    printf("******************************************\n");
                    printf("errCode: %d\n",errCode);
                    printf("******************************************\n");
                    return false;
                }
        return true;
    }

    bool HikCamera::dvrGetCfgCB(hik_ptz_camera::DvrGetCfg::Request  &req, hik_ptz_camera::DvrGetCfg::Response &res){
        NET_DVR_CAMERAPARAMCFG_EX basic_param;
        NET_DVR_ISP_CAMERAPARAMCFG isp_param;
        NET_DVR_FOCUSMODE_CFG focus_cfg;
        unsigned int ret_len,errCode;
        switch(req.type){
            case (hik_ptz_camera::DvrGetCfg::Request::DVR_CCDPARAMCFG_EX):
                if(NET_DVR_GetDVRConfig(_loginHandle,NET_DVR_GET_CCDPARAMCFG_EX,1,(void*)&basic_param,sizeof(basic_param),&ret_len)){
                    printf("struElectronicStabilization is_on: %d\n",basic_param.struElectronicStabilization.byEnable);
                    printf("struElectronicStabilization level: %d\n",basic_param.struElectronicStabilization.byLevel);
                    res.param_1 = basic_param.struElectronicStabilization.byEnable;
                    res.param_2 = basic_param.struElectronicStabilization.byLevel;
                    res.description = "param_1: basic_param.struElectronicStabilization.byEnable. | param_2: basic_param.struElectronicStabilization.byLevel.";
                    res.success = true;
                }else{
                    errCode = NET_DVR_GetLastError();
                    printf("******************************************\n");
                    printf("errCode: %d\n",errCode);
                    printf("******************************************\n");
                    res.success = false;
                }
                res.status_message = "DVR_CCDPARAMCFG_EX";
                break;
            case (hik_ptz_camera::DvrGetCfg::Request::DVR_ISP_CAMERAPARAMCFG):
                if(NET_DVR_GetDVRConfig(_loginHandle,NET_DVR_GET_ISP_CAMERAPARAMCFG,1,(void*)&isp_param,sizeof(isp_param),&ret_len)){
                    printf("isp byWorkType: %d\n",isp_param.byWorkType);
                    res.param_1 = isp_param.byWorkType;
                    res.description = "param_1: isp_param.byWorkType.";
                    res.success = true;
                }else{
                    errCode = NET_DVR_GetLastError();
                    printf("******************************************\n");
                    printf("errCode: %d\n",errCode);
                    printf("******************************************\n");
                    res.success = false;
                }
                res.status_message = "DVR_ISP_CAMERAPARAMCFG";
                break;
            case (hik_ptz_camera::DvrGetCfg::Request::DVR_WDR):
                if(NET_DVR_GetDVRConfig(_loginHandle,NET_DVR_GET_CCDPARAMCFG_EX,1,(void*)&basic_param,sizeof(basic_param),&ret_len)){
                    printf("struWdr byWDREnabled: %d\n",basic_param.struWdr.byWDREnabled);
                    printf("struWdr byWDRLevel1: %d\n",basic_param.struWdr.byWDRLevel1);
                    res.param_1 = basic_param.struWdr.byWDREnabled;
                    res.param_2 = basic_param.struWdr.byWDRLevel1;
                    res.description = "param_1: basic_param.struWdr.byWDREnabled. | param_2: basic_param.struWdr.byWDRLevel1.";
                    res.success = true;
                }else{
                    errCode = NET_DVR_GetLastError();
                    printf("******************************************\n");
                    printf("errCode: %d\n",errCode);
                    printf("******************************************\n");
                    res.success = false;
                }
                res.status_message = "DVR_WDR";
                break;
            case (hik_ptz_camera::DvrGetCfg::Request::DVR_FOCUS):
                if(NET_DVR_GetDVRConfig(_loginHandle,NET_DVR_GET_FOCUSMODECFG,1,(void*)&focus_cfg,sizeof(focus_cfg),&ret_len)){
                    printf("focus_cfg.byFocusMode: %d\n",focus_cfg.byFocusMode);
                    printf("focus_cfg.dwFocusPos: %d\n",focus_cfg.dwFocusPos);
                    res.param_1 = focus_cfg.byFocusMode;
                    res.param_2 = focus_cfg.dwFocusPos;
                    res.description = "param_1: focus_cfg.byFocusMode | param_2: focus_cfg.dwFocusPos.";
                    res.success = true;
                }else{
                    errCode = NET_DVR_GetLastError();
                    printf("******************************************\n");
                    printf("errCode: %d\n",errCode);
                    printf("******************************************\n");
                    res.success = false;
                }
                res.status_message = "DVR_FOCUS";
                break;
            default:
                return false;
        }
        return true;
    }

    bool HikCamera::dvrSetCfgCB(hik_ptz_camera::DvrSetCfg::Request  &req, hik_ptz_camera::DvrSetCfg::Response &res){
        NET_DVR_CAMERAPARAMCFG_EX basic_param;
        NET_DVR_ISP_CAMERAPARAMCFG isp_param;
        NET_DVR_FOCUSMODE_CFG focus_cfg;
        unsigned int ret_len,errCode;
        switch(req.type){
            case (hik_ptz_camera::DvrGetCfg::Request::DVR_CCDPARAMCFG_EX):
                if(NET_DVR_GetDVRConfig(_loginHandle,NET_DVR_GET_CCDPARAMCFG_EX,1,(void*)&basic_param,sizeof(basic_param),&ret_len)){
                    basic_param.struElectronicStabilization.byEnable = req.param_1;
                    basic_param.struElectronicStabilization.byLevel = req.param_2;
                    if(NET_DVR_SetDVRConfig(_loginHandle,NET_DVR_SET_CCDPARAMCFG_EX,1,(void*)&basic_param,sizeof(basic_param)))
                        res.success = true;
                    else 
                        res.success = false;
                }else{
                    errCode = NET_DVR_GetLastError();
                    printf("******************************************\n");
                    printf("errCode: %d\n",errCode);
                    printf("******************************************\n");
                    res.success = false;
                }
                res.status_message = "DVR_CCDPARAMCFG_EX";
                break;
            case (hik_ptz_camera::DvrGetCfg::Request::DVR_ISP_CAMERAPARAMCFG):
                if(NET_DVR_GetDVRConfig(_loginHandle,NET_DVR_GET_ISP_CAMERAPARAMCFG,1,(void*)&isp_param,sizeof(isp_param),&ret_len)){
                    isp_param.byWorkType = req.param_1;
                    if(NET_DVR_SetDVRConfig(_loginHandle,NET_DVR_GET_ISP_CAMERAPARAMCFG,1,(void*)&isp_param,sizeof(isp_param)))
                        res.success = true;
                    else 
                        res.success = false;
                }else{
                    errCode = NET_DVR_GetLastError();
                    printf("******************************************\n");
                    printf("errCode: %d\n",errCode);
                    printf("******************************************\n");
                    res.success = false;
                }
                res.status_message = "DVR_ISP_CAMERAPARAMCFG";
                break;
            case (hik_ptz_camera::DvrGetCfg::Request::DVR_WDR):
                if(NET_DVR_GetDVRConfig(_loginHandle,NET_DVR_GET_CCDPARAMCFG_EX,1,(void*)&basic_param,sizeof(basic_param),&ret_len)){
                    basic_param.struWdr.byWDREnabled = req.param_1;
                    basic_param.struWdr.byWDRLevel1 = req.param_2;
                    if(NET_DVR_SetDVRConfig(_loginHandle,NET_DVR_SET_CCDPARAMCFG_EX,1,(void*)&basic_param,sizeof(basic_param)))
                        res.success = true;
                    else 
                        res.success = false;
                }else{
                    errCode = NET_DVR_GetLastError();
                    printf("******************************************\n");
                    printf("errCode: %d\n",errCode);
                    printf("******************************************\n");
                    res.success = false;
                }
                res.status_message = "DVR_WDR";
                break;
            case (hik_ptz_camera::DvrGetCfg::Request::DVR_FOCUS):
                if(NET_DVR_GetDVRConfig(_loginHandle,NET_DVR_GET_FOCUSMODECFG,1,(void*)&focus_cfg,sizeof(focus_cfg),&ret_len)){
                    printf("focus_cfg.byFocusMode: %d\n",focus_cfg.byFocusMode);
                    printf("focus_cfg.dwFocusPos: %d\n",focus_cfg.dwFocusPos);
                    focus_cfg.byFocusMode = req.param_1;
                    focus_cfg.dwFocusPos = req.param_2;
                    if(NET_DVR_SetDVRConfig(_loginHandle,NET_DVR_GET_FOCUSMODECFG,1,(void*)&focus_cfg,sizeof(focus_cfg)))
                        res.success = true;
                    else 
                        res.success = false;
                }else{
                    errCode = NET_DVR_GetLastError();
                    printf("******************************************\n");
                    printf("errCode: %d\n",errCode);
                    printf("******************************************\n");
                    res.success = false;
                }
                res.status_message = "DVR_FOCUS";
                break;
            default:
                return false;
        }
        return true;
    }
    
    bool HikCamera::login(string ip_address="192.168.10.124", short port=8000, string user_name="admin", string passwd="Admin123"){
        //Login device
        NET_DVR_USER_LOGIN_INFO struLoginInfo = {0};
        NET_DVR_DEVICEINFO_V40 struDeviceInfoV40 = {0};
        struLoginInfo.bUseAsynLogin = false;

        struLoginInfo.wPort = port;
        memcpy(struLoginInfo.sDeviceAddress, ip_address.c_str(), NET_DVR_DEV_ADDRESS_MAX_LEN);
        memcpy(struLoginInfo.sUserName, user_name.c_str(), NAME_LEN);
        memcpy(struLoginInfo.sPassword, passwd.c_str(), NAME_LEN);

        _loginHandle = NET_DVR_Login_V40(&struLoginInfo, &struDeviceInfoV40);

        if (_loginHandle < 0){
            printf("pyd---Login error, %d\n", NET_DVR_GetLastError());
            cleanup();
            return false;
        }
        return true;
    }

    void HikCamera::cleanup() {
        logout();
        NET_DVR_Cleanup();
    }

    void HikCamera::logout() {
        if (_loginHandle >= 0){
            NET_DVR_Logout_V30(_loginHandle);
            _loginHandle = -1;
        }
    }

    void HikCamera::run(){
        if(!login()) 
            return;
        printf("login succeed!\n");
        ros::spin();
        logout();

    }


}

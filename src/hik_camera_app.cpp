#include "hik_camera.h"
#include <stdio.h>



int main(int argc, char *argv[]){
    ros::init(argc,argv,"hik_camera_app");
    hik_camera::HikCamera camera;
    printf("start run camera!\n");
    camera.run();
    printf("finish run camera!\n");
    return 0;
}
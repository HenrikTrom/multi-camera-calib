#include "load_pattern.h"

using namespace Calibration;

int main(){

    // loadPattern(&data_);
    std::string filepath = "/home/docker/workspace/workspace/multi-camera-calib/cfg/CameraCalibrationSettings.json";
    config_camera_calibration cfg;
    load_config_camera_calibration(filepath, cfg);
    Data data_(cfg);
    // calculateIntrinsics(&data_);
    calibrate(&data_);

    
    return 0;
}
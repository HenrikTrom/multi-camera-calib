#include "load_pattern.h"

using namespace Calibration;

int main(int argc, char* argv){

    std::string filepath = std::string(CONFIG_DIR)+"/CameraCalibrationSettings.json";
    config_camera_calibration cfg;
    cfg.input_dir = std::string(CONFIG_DIR)+"/../data/images";
    load_config_camera_calibration(filepath, cfg);
    Data data_(cfg);
    calibrate(&data_, cfg);

    
    return 0;
}
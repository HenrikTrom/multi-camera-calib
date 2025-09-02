#include "config_parser.hpp"

bool load_config_camera_calibration(std::string &filepath, config_camera_calibration &cfg){
    // TODO: schmeapath
    rapidjson::Document doc;
    if (!cpp_utils::load_json_with_schema(
        filepath,
        std::string(CONFIG_DIR)+"/CameraCalibrationSettings.schema.json",
        65536, doc
    )){
        return false;
    }

    cfg.pattern_size_first = doc["pattern_size_first"].GetInt();
    cfg.pattern_size_second = doc["pattern_size_second"].GetInt();
    cfg.charuco_params_border_size = doc["charuco_params_border_size"].GetFloat();
    cfg.charuco_params_dict = doc["charuco_params_dict"].GetInt();
    cfg.charuco_params_marker_border_pixel = doc["charuco_params_marker_border_pixel"].GetInt();
    cfg.charuco_params_marker_to_square_ratio = doc["charuco_params_marker_to_square_ratio"].GetFloat();
    cfg.charuco_params_square_size = doc["charuco_params_square_size"].GetFloat();
    cfg.savedir = doc["savedir"].GetString();
    cfg.main_cam_serial = doc["main_cam_serial"].GetString();

    for (const auto &sn : doc["serial_numbers"].GetArray()) {
        cfg.SNs.push_back(sn.GetString());
    }

    return true;
}
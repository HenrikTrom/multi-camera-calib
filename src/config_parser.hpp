#include <cpp_utils/jsontools.h>
#include "hardware_constants.h"

struct config_camera_calibration
{
    int pattern_type;
    int pattern_size_first;
    int pattern_size_second;
    float charuco_params_border_size;
    int charuco_params_dict;
    int charuco_params_marker_border_pixel;
    float charuco_params_marker_to_square_ratio;
    float charuco_params_square_size;
    std::string input_dir;
    std::string savedir;
};

bool load_config_camera_calibration(std::string &filepath, config_camera_calibration &cfg);
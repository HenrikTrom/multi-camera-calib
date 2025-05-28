#include "data.h"

using namespace std;
using namespace cv;

Calibration::Data::Data()
{
    //create calibration parameter list
    meta_calib_param_list = {
        Parameter(&pattern_type),
        Parameter(&pattern_size.first),
        Parameter(&pattern_size.second),
        Parameter(&charuco_params.border_size),
        Parameter(&charuco_params.dict),
        Parameter(&charuco_params.marker_border_pixel),
        Parameter(&charuco_params.marker_to_square_ratio),
        Parameter(&charuco_params.square_size)
    };
}

bool Calibration::Data::setParameter(std::string value, int param_code){
    bool success = true;

    if(param_code == PARAM_PATTERN_WIDTH){
        int val = std::stoi(value);
        if(success){
            this->pattern_size.first = val;
        }
    }
    else if(param_code == PARAM_PATTERN_HEIGHT){
        int val = std::stoi(value);
        if(success){
            this->pattern_size.second = val;
        }
    }
    else if(param_code == PARAM_CHARUCO_BORDER_PIXEL){
        float val = std::stof(value);
        if(success){
            this->charuco_params.marker_border_pixel = val;
        }
    }
    else if(param_code == PARAM_CHARUCO_SQUARE_SIZE){
        float val = std::stof(value);
        if(success){
            this->charuco_params.square_size = val;
        }
    }
    else if(param_code == PARAM_LM_MAX_ITERATIONS){
        int val = std::stoi(value);
        if(success){
            this->lm_params.max_iterations = val;
        }
    }
    else if(param_code == PARAM_LM_CHANGE_THRESHOLD){
        float val = std::stof(value);
        if(success){
            this->lm_params.change_threshold = val;
        }
    }

    return success;
}

bool Calibration::Detection::convertCorrespondencesToMat(){
    object_points.clear();
    image_points.clear();

    //convert detected Feature_Points to opencv mats
    for(int cam_idx = 0; cam_idx < num_cams; cam_idx++){
        vector_2D<cv::Mat> object_points_for_cam;
        vector_2D<cv::Mat> image_points_for_cam;

        for(int pattern_idx = 0; pattern_idx < num_pattern_positions; pattern_idx++){
            vector<cv::Mat> object_points_for_pattern;
            vector<cv::Mat> image_points_for_pattern;

            for(int rig_idx = 0; rig_idx < num_rig_positions; rig_idx++){
                cv::Mat object_points_for_rig(0,0,CV_64FC3);
                cv::Mat image_points_for_rig(0,0,CV_64FC2);

                if(correspondences[cam_idx][pattern_idx][rig_idx].size() != 0){
                    object_points_for_rig = cv::Mat(correspondences[cam_idx][pattern_idx][rig_idx].size(),1,CV_64FC3);
                    image_points_for_rig = cv::Mat(correspondences[cam_idx][pattern_idx][rig_idx].size(),1,CV_64FC2);

                    for(size_t point_idx = 0; point_idx < correspondences[cam_idx][pattern_idx][rig_idx].size(); point_idx++){
                        const Feature_Point& p = correspondences[cam_idx][pattern_idx][rig_idx][point_idx];
                        object_points_for_rig.at<cv::Vec3d>(point_idx,0) = p.corr;
                        image_points_for_rig.at<cv::Vec2d>(point_idx,0) = p.p;
                    }
                }

                object_points_for_pattern.push_back(object_points_for_rig);
                image_points_for_pattern.push_back(image_points_for_rig);
            }

            object_points_for_cam.push_back(object_points_for_pattern);
            image_points_for_cam.push_back(image_points_for_pattern);
        }
        object_points.push_back(object_points_for_cam);
        image_points.push_back(image_points_for_cam);
    }

    return true;
}

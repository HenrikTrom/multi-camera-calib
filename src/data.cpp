#include "data.h"

using namespace std;
using namespace cv;

Calibration::Data::Data(config_camera_calibration &cfg)
{
    this->pattern_size.first = cfg.pattern_size_first;
    this->pattern_size.second = cfg.pattern_size_second;
    this->charuco_params.border_size = cfg.charuco_params_border_size;
    this->charuco_params.dict = cfg.charuco_params_dict;
    this->charuco_params.marker_border_pixel = cfg.charuco_params_marker_border_pixel;
    this->charuco_params.marker_to_square_ratio = cfg.charuco_params_marker_to_square_ratio;
    this->charuco_params.square_size = cfg.charuco_params_square_size; 
    this->input_dir = cfg.input_dir;
    this->savedir = cfg.savedir;
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

#ifndef DATA_H
#define DATA_H
#pragma once

// #include "logger.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "config_parser.hpp"

namespace Calibration{

//////////////////////////   definition for constants   /////////////////////////////

//pattern types
const int CALIB_PATTERN_CHARUCO = 0;
const int CALIB_PATTERN_RANDOM = 1;

//Levenberg Marquardt termination criteria
const int LM_TERM_MAX_ITERATIONS = 0;
const int LM_TERM_CHANGE_THRESHOLD = 1;
const int LM_TERM_COMBINED = 2;

//camera types
const int CAMERA_RGB = 0;
const int CAMERA_IR = 1;

//camera setting modes
const int SET_CAMERA_INTRINSICS = 0;
const int SET_CAMERA_ALL = 1;
const int SET_CAMERA_RESOLUTION = 2;

//parameter codes for setter
const int PARAM_PATTERN_WIDTH = 0;
const int PARAM_PATTERN_HEIGHT = 1;
const int PARAM_CHARUCO_SQUARE_SIZE = 2;
const int PARAM_CHARUCO_BORDER_PIXEL = 3;
const int PARAM_LM_CHANGE_THRESHOLD = 4;
const int PARAM_LM_MAX_ITERATIONS = 5;
const int PARAM_IMAGE_INPUT_DIR = 6;

//parameter container codes
const int VAR_INT = 0;
const int VAR_FLOAT = 1;
const int VAR_DOUBLE = 2;
const int VAR_STRING = 3;

//intrinsics modes
const int INIT_INTRINSICS_NONE = 0;
const int INIT_INTRINSICS_FILE = 1;
const int INIT_INTRINSICS_WIDGET = 2;
const int REFINE_INTRINSICS_ALL = 0;
const int REFINE_INTRINSICS_GUI = 1;
const int REFINE_INTRINSICS_NONE = 2;
const int CALC_EXTRINSICS_ALL = 0;
const int CALC_EXTRINSICS_GUI = 1;
const int INIT_EXTRINSICS_FROM_PATTERNS = 0;
const int INIT_EXTRINSICS_FROM_GUI = 1;

//aruco codes
const std::vector<int> ARUCO_CODES = {
    cv::aruco::DICT_4X4_50,
    cv::aruco::DICT_4X4_100,
    cv::aruco::DICT_4X4_250,
    cv::aruco::DICT_4X4_1000,
    cv::aruco::DICT_5X5_50,
    cv::aruco::DICT_5X5_100,
    cv::aruco::DICT_5X5_250,
    cv::aruco::DICT_5X5_1000,
    cv::aruco::DICT_6X6_50,
    cv::aruco::DICT_6X6_100,
    cv::aruco::DICT_6X6_250,
    cv::aruco::DICT_6X6_1000,
    cv::aruco::DICT_7X7_50,
    cv::aruco::DICT_7X7_100,
    cv::aruco::DICT_7X7_250,
    cv::aruco::DICT_7X7_1000
};

//aliases for multidimensional vectors
template<typename T>
using vector_4D = std::vector<std::vector<std::vector<std::vector<T>>>>;

template<typename T>
using vector_3D = std::vector<std::vector<std::vector<T>>>;

template<typename T>
using vector_2D = std::vector<std::vector<T>>;

//aliases for pairs
using pairi = std::pair<int,int>;
using pairf = std::pair<float,float>;

///
/// \brief The Parameter struct is a container used to construct lists of variables of different types
///
struct Parameter{
    int type;
    void* pointer;

    Parameter(int* pointer){
        this->type = VAR_INT;
        this->pointer = (void*)pointer;
    }
    Parameter(float* pointer){
        this->type = VAR_FLOAT;
        this->pointer = (void*)pointer;
    }
    Parameter(double* pointer){
        this->type = VAR_DOUBLE;
        this->pointer = (void*)pointer;
    }
    Parameter(std::string* pointer){
        this->type = VAR_STRING;
        this->pointer = (void*)pointer;
    }
};

///
/// \brief The DistortionParameters struct
///
struct Distortion_Parameters{
    ///distortion parameters k1, k2, p1, p2 and k3 according to opencv documentation
    double k_1=0.0,k_2=0.0,k_3=0.0,p_1=0.0,p_2=0.0;
};

///
/// \brief The Camera struct contains extrinsic as well as intrinsic parameters for a camera as well as identifying information
///
struct Camera{
    std::string id = "";
    ///K matrix containing fx, fy and central point
    cv::Mat K = cv::Mat::eye(3,3,CV_64F);
    ///distortion parameters k1, k2, p1, p2 and k3 according to opencv documentation
    Distortion_Parameters distortion;
    ///extrinsics: contains rotation and translation wrt a reference camera
    cv::Mat pose = cv::Mat::eye(4,4,CV_64F);
    ///width of images from this camera
    int resolution_x = -1;
    ///height of images from this camera
    int resolution_y = -1;
    ///RGB or IR - this is relevant for the preprocessing of the images
    int type = -1;
    ///specifies whether the intrinsics of this camera are calculated by the intrinsics estimation method
    bool calculate_intrinsics = true;
    ///specifies whether the intrinsics are refined duriung the optimization
    bool refine_intrinsics = true;
    ///specifies whether the extrinsics of this camera are optimized
    bool optimize_extrinsics = true;


    cv::Mat getDistortion() const{
        cv::Mat distortion(1,5,CV_64F);
        distortion.at<double>(0,0) = this->distortion.k_1;
        distortion.at<double>(0,1) = this->distortion.k_2;
        distortion.at<double>(0,4) = this->distortion.k_3;
        distortion.at<double>(0,2) = this->distortion.p_1;
        distortion.at<double>(0,3) = this->distortion.p_2;
        return distortion;
    }

    void setDistortion(cv::Mat& dist){
        if(dist.rows != 1 || dist.cols != 5){
            return;
        }
        distortion.k_1 = dist.at<double>(0,0);
        distortion.k_2 = dist.at<double>(0,1);
        distortion.k_3 = dist.at<double>(0,4);
        distortion.p_1 = dist.at<double>(0,2);
        distortion.p_2 = dist.at<double>(0,3);
    }
};

///
/// \brief The Files struct contains a list of file paths and a 3D index list which associates the file paths to camera, pattern and rig postion
///
struct Files{
    ///list of file paths
    std::vector<std::string> file_paths;
    ///file_idx[i][j][k] is the file idx of cam i, pattern position j, rig position k and -1 if not available
    vector_3D<int> file_idx;
};

///
/// \brief The Feature_Point struct contains a 2d image point, the corresponding 3d pattern point (last coordinate is 0 for 2d patterns) and for
/// charuco feature points the id
///
struct Feature_Point{
    cv::Vec2d p;
    int id; //id of corresponding pattern point
    cv::Vec3d corr; //corresponding pattern point

    Feature_Point(cv::Vec2d p = cv::Vec2d(-1.0,-1.0), int id = -1, cv::Vec3d corr = cv::Vec3d(-1.0,-1.0,0.0)) : p(p), id(id), corr(corr){}
};

using Feature_List = std::vector<Feature_Point>;

///
/// \brief The Camera_Vertex struct is used for building a graph relating camera and pattern positions prior to the LM optimization
///
struct Camera_Vertex{
    int parent_pattern_idx = -1;
    std::vector<pairi> child_pattern_rig_idx;

    //the transform from the first camera to this camera
    cv::Mat pose = cv::Mat::eye(4,4,CV_64F);
    bool pose_calculated = false;

    int camera_idx;
};

///
/// \brief The Pattern_Vertex struct is used for building a graph relating camera and pattern positions prior to the LM optimization
///
struct Pattern_Vertex{
    int parent_camera_idx = -1;
    std::vector<pairi> child_camera_rig_idx;

    //the transform from pattern to the first camera
    cv::Mat pose = cv::Mat::eye(4,4,CV_64F);
    bool pose_calculated = false;

    int pattern_position;
};

///
/// \brief The Edge struct is used in the graph to relate different vertices to each other
///
struct Edge{
    int cam_vertex;
    int rig_position = 0;
    int pattern_vertex;
    cv::Mat transform;

    Edge(int cam_vertex, int pattern_vertex, int rig_position, cv::Mat transform){
        this->cam_vertex = cam_vertex;
        this->pattern_vertex = pattern_vertex;
        this->rig_position = rig_position;
        this->transform = transform;
    }
};

///
/// \brief The Graph struct
///
struct Graph{
    std::vector<Camera_Vertex> cam_vertices;
    std::vector<Pattern_Vertex> pattern_vertices;
    std::vector<Edge> edges;
};

///
/// \brief The ChArUco_Params struct contains charuco pattern specific parameters
///
struct ChArUco_Params{
    ///border-to-square ratio
    float border_size = 0.5;
    ///size of a checkerboard square in mm
    float square_size = 10.0;
    ///marker-to-square ratio determines the size of a marker wrt the checkerboard square size
    float marker_to_square_ratio = 0.5;
    ///the opencv code for the aruco dictionary
    int dict = cv::aruco::DICT_4X4_50;
    ///number of border pixels of a marker
    int marker_border_pixel = 1;
};

///
/// \brief The LM_Params struct contains the parameters for the Levenberg Marquardt optimization
///
struct LM_Params{
    ///termination criterium for Levenberg Marquardt optimization
    int term_criterium = LM_TERM_COMBINED;
    ///threshold for change termination
    double change_threshold = 0.001;
    ///maximum number of iterations for the Levenberg Marquardt optimization
    int max_iterations = 5;
};

///
/// \brief The Estimation struct contains the camera parameters and transforms estimated by the calibration algorithm
///
class Estimation
{

public:
    ///saves the estimated camera parameters
    std::vector<Camera> cams;
    //mapping from the camera names to corresponding indices in the vector above
    std::map<std::string, int> cam_id_to_idx_map;

    //index lists for the properties in the camera struct - used in the graph building and optimization
    std::vector<int> refine_idx;
    std::vector<int> calculation_idx;
    std::vector<int> calc_extrinsics_idx_list;

    vector_3D<cv::Mat> rotations;
    vector_3D<cv::Mat> translations;
    vector_3D<cv::Mat> transforms;
    ///saves the estimated rig axes for hand eye calibration
    cv::Mat x_axis = cv::Mat::eye(3,3,CV_64F).col(0);
    cv::Mat y_axis = cv::Mat::eye(3,3,CV_64F).col(1);

    void clearGeometry(){
        rotations.clear();
        translations.clear();
        transforms.clear();
    }

    void clear(){
        cams.clear();
        refine_idx.clear();
        calculation_idx.clear();
        x_axis = cv::Mat::eye(3,3,CV_64F).col(0);
        y_axis = cv::Mat::eye(3,3,CV_64F).col(1);
        clearGeometry();
    }

    void setCameras(
        const std::vector<Camera>& cameras, const int& set_mode = SET_CAMERA_ALL, 
        const bool& overwrite = true
    ){
        for(const Camera& new_cam : cameras){
            bool found = false;
            for(Camera& cam : cams){
                if(cam.id.compare(new_cam.id) == 0){
                    if(overwrite){
                        if(set_mode == SET_CAMERA_ALL){
                            cam = new_cam;
                        }
                        else if(set_mode == SET_CAMERA_INTRINSICS){
                            cam.resolution_x = new_cam.resolution_x;
                            cam.resolution_y = new_cam.resolution_y;
                            cam.distortion = new_cam.distortion;
                            new_cam.K.copyTo(cam.K);
                        }
                        else if(set_mode == SET_CAMERA_RESOLUTION){
                            cam.resolution_x = new_cam.resolution_x;
                            cam.resolution_y = new_cam.resolution_y;
                        }
                    }
                    found = true;
                    break;
                }
            }
            if(!found){
                //camera is not present yet
                Camera cam;
                if(set_mode == SET_CAMERA_ALL){
                    cam = new_cam;
                }
                else if(set_mode == SET_CAMERA_INTRINSICS){
                    cam.id = new_cam.id;
                    cam.resolution_x = new_cam.resolution_x;
                    cam.resolution_y = new_cam.resolution_y;
                    cam.distortion = new_cam.distortion;
                    new_cam.K.copyTo(cam.K);
                }
                else if(set_mode == SET_CAMERA_RESOLUTION){
                    cam.id = new_cam.id;
                    cam.resolution_x = new_cam.resolution_x;
                    cam.resolution_y = new_cam.resolution_y;
                }
                cams.push_back(cam);
                refine_idx.clear();
                calculation_idx.clear();
                calc_extrinsics_idx_list.clear();
                cam_id_to_idx_map[cam.id] = int(cams.size()-1);
            }
        }

        //check if new cameras were added and adjust the size of the geometry vectors, if these are already set, accordingly
        if(cams.size() > rotations.size() && rotations.size() > 0 && rotations[0].size() > 0 && rotations[0][0].size() > 0){
            unsigned int num_rig = rotations[0][0].size();
            unsigned int num_pattern = rotations[0].size();
            for(unsigned int i = rotations.size(); i < cams.size(); i++){
                rotations.push_back(vector_2D<cv::Mat>(num_pattern,std::vector<cv::Mat>(num_rig,cv::Mat(0,0,CV_64F))));
                translations.push_back(vector_2D<cv::Mat>(num_pattern,std::vector<cv::Mat>(num_rig,cv::Mat(0,0,CV_64F))));
                transforms.push_back(vector_2D<cv::Mat>(num_pattern,std::vector<cv::Mat>(num_rig,cv::Mat(0,0,CV_64F))));
            }
        }
    }

    int addCamera(const Camera& camera, const int& set_mode = SET_CAMERA_ALL, const bool& overwrite = true){
        int idx = -1;
        //search for camera id in existing cameras
        for(unsigned int i = 0; i < cams.size(); i++){
            Camera& cam = cams[i];
            if(cam.id.compare(camera.id) == 0){
                if(overwrite){
                    if(set_mode == SET_CAMERA_ALL){
                        cam = camera;
                    }
                    else if(set_mode == SET_CAMERA_INTRINSICS){
                        cam.resolution_x = camera.resolution_x;
                        cam.resolution_y = camera.resolution_y;
                        cam.distortion = camera.distortion;
                        camera.K.copyTo(cam.K);
                    }
                    else if(set_mode == SET_CAMERA_RESOLUTION){
                        cam.resolution_x = camera.resolution_x;
                        cam.resolution_y = camera.resolution_y;
                    }
                }
                idx = i;
                break;
            }
        }
        //add new camera
        if(idx == -1){
            //camera is not present yet
            Camera cam;
            if(set_mode == SET_CAMERA_ALL){
                cam = camera;
            }
            else if(set_mode == SET_CAMERA_INTRINSICS){
                cam.id = camera.id;
                cam.resolution_x = camera.resolution_x;
                cam.resolution_y = camera.resolution_y;
                cam.distortion = camera.distortion;
                camera.K.copyTo(cam.K);
            }
            else if(set_mode == SET_CAMERA_RESOLUTION){
                cam.id = camera.id;
                cam.resolution_x = camera.resolution_x;
                cam.resolution_y = camera.resolution_y;
            }
            cams.push_back(cam);
            refine_idx.clear();
            calculation_idx.clear();
            calc_extrinsics_idx_list.clear();
            cam_id_to_idx_map[cam.id] = int(cams.size()-1);

            if(rotations.size() > 0 && rotations[0].size() > 0 && rotations[0][0].size() > 0){
                unsigned int num_rig = rotations[0][0].size();
                unsigned int num_pattern = rotations[0].size();
                for(unsigned int i = rotations.size(); i < cams.size(); i++){
                    rotations.push_back(vector_2D<cv::Mat>(num_pattern,std::vector<cv::Mat>(num_rig,cv::Mat(0,0,CV_64F))));
                    translations.push_back(vector_2D<cv::Mat>(num_pattern,std::vector<cv::Mat>(num_rig,cv::Mat(0,0,CV_64F))));
                    transforms.push_back(vector_2D<cv::Mat>(num_pattern,std::vector<cv::Mat>(num_rig,cv::Mat(0,0,CV_64F))));
                }
            }
            idx = cams.size()-1;
            // emit updateCameras();
        }
        return idx;
    }

    std::vector<bool> getCalcIntrinsics(){
        std::vector<bool> result;
        for(const Camera& cam : cams){
            result.push_back(cam.calculate_intrinsics);
        }
        return result;
    }

    bool setCalcIntrinsics(const std::vector<bool>& calc){
        if(calc.size() != cams.size()){
            return false;
        }
        for(unsigned int i = 0; i < calc.size(); i++){
            cams[i].calculate_intrinsics = calc[i];
        }
        return true;
    }

};

///
/// \brief The Detection struct contains the detected pattern points and the corresponding image points
///
struct Detection{
    ///cam ids according to the file names or loaded correspondences
    std::vector<std::string> detected_cam_ids;
    ///number of cameras given by the filenames
    int num_cams = 0;
    ///number of pattern positions given by the filenames
    int num_pattern_positions = 0;
    ///number of rig positions given by the filenames
    int num_rig_positions = 0;

    ///correspondences[i][j][k][l] represents the l-th feature point in the image taken by camera i of pattern at position j and rig at position k
    vector_3D<Feature_List> correspondences;
    vector_3D<cv::Mat> object_points;
    vector_3D<cv::Mat> image_points;
    ///contains the rig positions as given by a configuration file
    std::vector<pairf> rig_positions;

    void clear(){
        num_cams = -1;
        num_pattern_positions = -1;
        num_rig_positions = -1;
        correspondences.clear();
        object_points.clear();
        image_points.clear();
        rig_positions.clear();
    }

    bool convertCorrespondencesToMat();
};

///
/// \brief The Optimization_IO struct
///
struct Optimization_Params{
    ///if true, the rig axes will be refined during the optimization
    bool refine_rig = false;
    ///
    int initial_intrinsics_mode = INIT_INTRINSICS_NONE;
    ///
    int refine_intrinsics_mode = REFINE_INTRINSICS_ALL;
    ///
    int calculate_extrinsics_mode = CALC_EXTRINSICS_ALL;
    ///
    int initial_extrinsics_mode = INIT_EXTRINSICS_FROM_PATTERNS;
};

///
/// \brief The Calibration_Data class contains all necessary data for the calibration, including io paths and pattern as well as optimization configurations
///
class Data{
public:

    ///
    /// \brief Calibration_Data is the standard constructor
    /// \param parent
    ///
    Data(config_camera_calibration &cfg);
    std::string savedir;
    /////////////////////////   calibration pattern parameters   ///////////////////////////

    /// saves current pattern type - currently supported are charuco and random patterns
    int pattern_type = CALIB_PATTERN_CHARUCO;
    ///size of pattern according to the opencv functions creating the patterns
    std::pair<int,int> pattern_size = std::pair<int,int>(8,5);
    ///holds image of current pattern
    cv::Mat pattern_image;
    ///charuco specific parameters
    ChArUco_Params charuco_params;

    ///////////////////////////////   optimization data   //////////////////////////////////

    ///detected feature points and numbers of cameras/pattern positions/rig positions
    Detection detection;
    ///estimated camera parameters and transformations
    Estimation estimation;
    ///parameters for the Levenberg Marquardt optimization
    LM_Params lm_params;

    /////////////////////////////////   IO parameters   ////////////////////////////////////

    ///image input dir
    std::string input_dir;
    ///list of file paths and a 3D index list which associates the file paths to camera, pattern and rig postion
    Files files;
    ///if true, the tool will load rig positions from a file in the image input dir or a user-specified dir
    bool use_rig_position_file = false;
    ///
    Optimization_Params opt_params;

    // bool setParameter(std::string value, int param_code);
    ///
    /// \brief patternWidth is a getter for the calibration pattern width
    /// \return calibration pattern width
    ///
    int patternWidth(){return pattern_size.first;}
    ///
    /// \brief patternHeight is a getter for the calibration pattern height
    /// \return calibration pattern height
    ///
    int patternHeight(){return pattern_size.second;}
};

}

#endif // CALIBRATION_DATA_H

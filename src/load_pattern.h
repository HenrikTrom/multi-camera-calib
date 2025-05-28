#include "data.h"
#include "spdlog/spdlog.h"
#include <fstream>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cpp_utils/utils.h>

using namespace std;
using namespace cv;

namespace Calibration{

const int DELIM_UNDERSCORE = 0;
const int DELIM_MINUS = 1;
const int DELIM_BOTH = 2;

bool loadParameters(const string& file_path, vector<Parameter>* parameters){

    ifstream file(file_path);
    if(file.is_open()){
        for(Parameter& param : *parameters){
            switch (param.type) {
            case VAR_INT:
                file >> *(int*)param.pointer;
                break;
            case VAR_FLOAT:
                file >> *(float*)param.pointer;
                break;
            case VAR_DOUBLE:
                file >> *(double*)param.pointer;
                break;
            case VAR_STRING:
                file >> *(string*)param.pointer;
                break;
            default:
                break;
            }
        }

        file.close();
    }

    return true;
};


void loadPattern(Data* data_){
    std::string file_path = "/home/docker/workspace/workspace/multi-camera-calib/data/pattern/board_pattern.MCC_Patt";

    //load config
    if(!loadParameters(file_path, &data_->meta_calib_param_list)){
        spdlog::error("Could not load parameters");
    }
};

bool splitString(const string& str, vector<string>* parts, const int delim_code){
    if(str.size() == 0 || parts == nullptr){
        return false;
    }

    parts->clear();

    vector<char> delimiters;
    if(delim_code == DELIM_MINUS || delim_code == DELIM_BOTH){
        delimiters.push_back('-');
    }
    if(delim_code == DELIM_UNDERSCORE || delim_code == DELIM_BOTH){
        delimiters.push_back('_');
    }

    int last_delim_position = -1;
    string substr;
    for(size_t i = 0; i < str.size(); i++){
        const char& c = str[i];
        for(char& delimiter : delimiters){
            if(c == delimiter){
                substr = str.substr(last_delim_position+1,i);
                parts->push_back(substr);

                last_delim_position = i;
                break;
            }
        }
        if(c == '.'){
            substr = str.substr(last_delim_position+1,i-last_delim_position-1);
            parts->push_back(substr);
            break;
        }
    }

    return true;
}


bool readFloatImage(const string file_path, Mat* image){
    if(file_path.size() == 0){
        spdlog::error("No valid file path given.");
        return false;
    }
    if(image == nullptr){
        spdlog::error("No valid image pointer given.");
        return false;
    }

    //check file type
    std::string type = file_path.substr(file_path.size()-3);
    if(type.compare("mat") == 0){
        std::ifstream file(file_path);
        if(file.is_open()){
            //read matrix dimensions
            //count items per row
            std::string line;
            std::getline(file,line);
            std::istringstream line_(line);
            int width = std::count(std::istreambuf_iterator<char>(line_), std::istreambuf_iterator<char>(),' ');
            //count rows
            int height = std::count(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>(),'\n')+1;
            //reset stream
            file.clear();
            file.seekg(0, std::ios::beg);

            if(width<=0 || height<=0){
                spdlog::error("The file {} does not contain sufficient data.", file_path);
                return false;
            }

            //read mat file values
            vector<float> data(height*width);
            for(int i=0;i<width*height;i++){
                file >> data[i];
            }
            *image = Mat(height,width,CV_32F,data.data()).clone();

        }
        else{
            spdlog::error("Unable to open file {}", file_path);
            return false;
        }
    }
    else if(type.compare("png") == 0){
        cv::Mat temp = cv::imread(file_path,cv::IMREAD_UNCHANGED);
        if(temp.type() != CV_8UC4){
            spdlog::error("Chosen image does not have 4 channels.");
            return false;
        }
        *image = Mat(temp.rows,temp.cols,CV_32F,temp.data);
    }
    else if(type.compare("exr") == 0){
        spdlog::error("exr is currently not supported.");
        return false;
    }
    else{
        spdlog::error("Invalid depth file format.");
        return false;
    }

    return true;
}

bool createImageList(const string& dir_path, const map<int,pairf>& rig_positions, Files* files, Detection* detection, vector<Camera>* cameras){
    if(files == nullptr || detection == nullptr){
        spdlog::error("No file list or detection struct given.");
        return false;
    }

    files->file_idx.clear();
    files->file_paths.clear();

    //check if image_dir exists
    if(dir_path.size() == 0){
        spdlog::error("No image dir specified.");
        return false;
    }
    if(!std::filesystem::exists(dir_path)){
        spdlog::error("Image dir does not exist., {}", dir_path);
        return false;
    }

    //get list of image files and wirte list to imagelist.xml

    std::vector<std::string> file_list;
    for (const auto& entry : std::filesystem::directory_iterator(dir_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".png") {
            file_list.push_back(entry.path().stem().string());
            spdlog::info("Found {}", entry.path().string());
        }
    }

    vector<string> camera_ids;
    vector<string> rig_pos_ids;
    vector<vector<int>> image_idx_list; //since we do not enforce continuous numbering, this list saves the indices corresponding to the ids above

    int num_rig_pos_ids = 0;

    for(int i = 0; i < file_list.size(); i++){
        const string file_name = file_list[i]+".png";
        const string file_path = dir_path+"/"+file_name;
        files->file_paths.push_back(file_path);

        //analyze file_name - assume naming according to $CAMID_$RIGPOSID.$FILETYPE
        vector<string> image_ids;
        if(!splitString(file_name,&image_ids, DELIM_BOTH) || image_ids.size() < 2){
            spdlog::warn("File name could not be analyzed correctly: {}", file_name);
            continue;
        }

        //calculate image idx
        vector<int> image_idx;
        int cam_idx = -1;
        if(cameras->size() > 0 && cameras->back().id.compare(image_ids[0]) != 0){
            for(unsigned int j = 0; j < cameras->size(); j++){
                if(cameras->at(j).id.compare(image_ids[0]) == 0){
                    cam_idx = int(j);
                }
            }
        }
        else{
            cam_idx = cameras->size()-1;
        }

        if(cam_idx == -1 || cam_idx == (int)camera_ids.size()){//new cam found
            detection->num_cams++;

            //create new camera
            Camera cam;
            cam.id = image_ids[0];

            //load image in order to get resolution
            Mat image;
            if(file_path.substr(file_path.size()-3).compare("mat") == 0){
                readFloatImage(file_path, &image);
            }
            else{
                image = imread(file_path);
            }
            cam.resolution_x = image.cols;
            cam.resolution_y = image.rows;
            cameras->push_back(cam);
            camera_ids.push_back(image_ids[0]);

            cam_idx = cameras->size()-1;

            detection->detected_cam_ids.push_back(image_ids[0]);
        }

        image_idx.push_back(cam_idx);

        int rig_pos_idx = distance(rig_pos_ids.begin(),find(rig_pos_ids.begin(),rig_pos_ids.end(),image_ids[1]));

        if(rig_pos_idx == -1 || rig_pos_idx == (int)rig_pos_ids.size()){//new rig position id found
            num_rig_pos_ids++;
            rig_pos_ids.push_back(image_ids[1]);
            image_idx.push_back(rig_pos_ids.size()-1);
        }
        else{
            image_idx.push_back(rig_pos_idx);
        }

        image_idx_list.push_back(image_idx);
    }

    detection->num_pattern_positions = int(float(num_rig_pos_ids) / float(detection->num_rig_positions));

    //fill the files->file_idx lookup table for cam/pattern position/rig position triples
    vector<int> indices_per_pattern(detection->num_rig_positions, -1);
    vector_2D<int> indices_per_cam;
    for(int i = 0 ; i < detection->num_pattern_positions; i++){
        indices_per_cam.push_back(indices_per_pattern);
    }
    for(int i = 0 ; i < detection->num_cams; i++){
        files->file_idx.push_back(indices_per_cam);
    }

    for(size_t i = 0 ; i < image_idx_list.size(); i++){
        if(image_idx_list[i].size()>0){
            files->file_idx[image_idx_list[i][0]][(int)((float)(image_idx_list[i][1])/(float)(detection->num_rig_positions))][image_idx_list[i][1] % detection->num_rig_positions] = i;
        }
    }

    //fill the detection->rig_positions vector
    detection->rig_positions.clear();
    if(rig_positions.size() != 0){
        pairf first_position;
        bool first_found = false;
        for(int rig_idx = 0; rig_idx < detection->num_rig_positions; rig_idx++){
            auto iterator = rig_positions.find(rig_idx);
            if(iterator != rig_positions.end()){
                if(!first_found){
                    //save first position
                    first_position = iterator->second;
                    first_found = true;
                }
                //calculate position relative to first rig position
                pairf relative_position = iterator->second;
                relative_position.first -= first_position.first;
                relative_position.second -= first_position.second;
                detection->rig_positions.push_back(relative_position);
            }
            else{
                spdlog::error("Rig position conversion failed.");
                return false;
            }
        }
    }

    return true;
}


bool convertFeaturelistsToMat(Detection* detection){
    detection->object_points.clear();
    detection->image_points.clear();

    //convert detected Feat_Points to opencv mats
    for(int cam_idx = 0; cam_idx < detection->num_cams; cam_idx++){
        vector_2D<cv::Mat> object_points_for_cam;
        vector_2D<cv::Mat> image_points_for_cam;

        for(int pat_idx = 0; pat_idx < detection->num_pattern_positions; pat_idx++){
            vector<cv::Mat> object_points_for_pattern;
            vector<cv::Mat> image_points_for_pattern;

            for(int rig_idx = 0; rig_idx < detection->num_rig_positions; rig_idx++){

                cv::Mat object_points_for_rig(0,0,CV_64FC3);
                cv::Mat image_points_for_rig(0,0,CV_64FC2);

                if(detection->correspondences[cam_idx][pat_idx][rig_idx].size() != 0){
                    object_points_for_rig = cv::Mat(detection->correspondences[cam_idx][pat_idx][rig_idx].size(),1,CV_64FC3);
                    image_points_for_rig = cv::Mat(detection->correspondences[cam_idx][pat_idx][rig_idx].size(),1,CV_64FC2);

                    for(size_t point_idx = 0; point_idx < detection->correspondences[cam_idx][pat_idx][rig_idx].size(); point_idx++){
                        const Feature_Point& p = detection->correspondences[cam_idx][pat_idx][rig_idx][point_idx];
                        object_points_for_rig.at<Vec3d>(point_idx,0) = p.corr;
                        image_points_for_rig.at<Vec2d>(point_idx,0) = p.p;
                    }
                }
                object_points_for_pattern.push_back(object_points_for_rig);
                image_points_for_pattern.push_back(image_points_for_rig);
            }
            object_points_for_cam.push_back(object_points_for_pattern);
            image_points_for_cam.push_back(image_points_for_pattern);
        }
        detection->object_points.push_back(object_points_for_cam);
        detection->image_points.push_back(image_points_for_cam);
    }

    return true;
}


bool detectChArUcoPatterns(const int width, const int height, const ChArUco_Params& charuco_params, const Files& files, Detection* detection){
    if(detection == nullptr){
        spdlog::error("No list given to save detected points in.");
        return false;
    }
    detection->correspondences.clear();

    //get dictionary
    auto dict = aruco::getPredefinedDictionary(charuco_params.dict);
    //create board
    // Ptr<aruco::CharucoBoard> board;
    cv::aruco::CharucoBoard board;
    try{
        board = aruco::CharucoBoard(cv::Size(width, height), charuco_params.square_size, charuco_params.marker_to_square_ratio*charuco_params.square_size, dict);
        board.setLegacyPattern(true);
        // board = aruco::CharucoBoard::create(width, height, charuco_params.square_size, charuco_params.marker_to_square_ratio*charuco_params.square_size, dict);
    }
    catch(...){
        spdlog::error("Unable to create marker board.");
        return false;
    }

    //function for calculating the image coordinates of a charuco corner
    auto charucoIdToImageCoordinates = [](int width, int id, double scale){
        double x = (double)(id % (width-1));
        double y = (double)((int)((double)(id) / (double)(width-1)));
        return Vec3d(scale*x,scale*y,0.0);
    };

    aruco::ArucoDetector aruco_detector = aruco::ArucoDetector(dict);
    aruco::CharucoDetector charuco_detector = aruco::CharucoDetector(board);

    for(int cam_idx = 0; cam_idx < detection->num_cams; cam_idx++){
        vector_3D<Feature_Point> features_for_cam;
        for(int pat_idx = 0; pat_idx < detection->num_pattern_positions; pat_idx++){
            vector<vector<Feature_Point>> features_for_pattern;
            for(int rig_idx = 0; rig_idx < detection->num_rig_positions; rig_idx++){
                vector<Feature_Point> features_for_rigpos;
                int file_path_idx = files.file_idx[cam_idx][pat_idx][rig_idx];
                if(file_path_idx != -1){
                    //load image
                    Mat image;
                    string file_path = files.file_paths[file_path_idx];
                    if(file_path.substr(file_path.size()-3).compare("mat") == 0){
                        //load float image (e.g. depth images)
                        if(!readFloatImage(file_path, &image)){
                            spdlog::error("Could not read float image {}", file_path);
                            return false;
                        }
                        else{
                            //scale and convert float image to 1 channel uchar image
                            cv::log(image,image);
                            image = 255.0*image/std::log(65536.0);
                            image.convertTo(image,CV_8UC1);
//                            cv::imshow("IR image2", image);
//                            cv::waitKey(0);
                        }
                    }
                    else{
                        //load color image
                        image = imread(file_path);
                    }

                    //find charuco markers
                    vector<vector<Point2f>> marker_corners;
                    vector<int> marker_ids;
                    aruco_detector.detectMarkers(image, marker_corners, marker_ids);

                    //find associated corners
                    if(marker_ids.size()>0){
                        vector<Vec2f> corners;
                        vector<int> ids;
                        charuco_detector.detectBoard(
                            image, 
                            corners,
                            ids, 
                            marker_corners,
                            marker_ids
                        );

                        //save feature points if at least 8 are found and corners are not in one line
                        if(ids.size()>7){
                            //check if corners are in one line by calculating angles -> degenerated cases are unusable for transformation estimations
                            bool degenerated = true;
                            for(size_t i = 0; i<ids.size() && degenerated; i++){
                                Vec2f& a = corners[i];
                                for(size_t j = 0; j<ids.size() && degenerated; j++){
                                    if(j != i){
                                        Vec2f& b = corners[j];
                                        for(size_t k = 0; k<ids.size() && degenerated; k++){
                                            if(k != i && k != j){
                                                Vec2f& c = corners[k];
                                                Vec2f ab = b-a;
                                                Vec2f ac = c-a;
                                                ab /= norm(ab);
                                                ac /= norm(ac);
                                                float angle = 180.0*acos(ab[0]*ac[0]+ab[1]*ac[1])/M_PI;
                                                if(std::fabs(angle)>45 && std::fabs(angle)<135){
                                                    degenerated = false;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            if(!degenerated){

                                //check if the points actually define a (distorted) regular grid - this is necessary since interpolateCornersCharuco
                                //does not filter the detected markers according to their plausability
                                bool is_grid = true;
                                vector<Vec2d> pattern_corners;
                                for(size_t i = 0; i<ids.size(); i++){
                                    Vec3d pattern_corner = charucoIdToImageCoordinates(width,ids[i],charuco_params.square_size);
                                    pattern_corners.push_back(Vec2d(pattern_corner[0],pattern_corner[1]));
                                }
                                Mat H = findHomography(corners,pattern_corners,RANSAC);
                                H.convertTo(H,CV_64F);
                                for(size_t i = 0; i<ids.size(); i++){
                                    Mat projected = H*Mat(Vec3d(corners[i][0],corners[i][1],1.0));
                                    Vec2d diff_2d = Vec2d(projected.at<double>(0,0)/projected.at<double>(2,0),projected.at<double>(1,0)/projected.at<double>(2,0)) - pattern_corners[i];
                                    if(cv::norm(diff_2d) > 20){
                                        is_grid = false;
                                        break;
                                    }
                                }
                                //save points
                                if(is_grid){
                                    for(size_t i = 0; i<ids.size(); i++){
                                        Vec2d corner(corners[i]);
                                        features_for_rigpos.push_back(Feature_Point(corner,ids[i],charucoIdToImageCoordinates(width,ids[i],charuco_params.square_size)));
                                    }
                                }
                                else{
                                    spdlog::warn("Pattern in image {}_{} was rejected - not a grid.", cam_idx, pat_idx);
//                                    aruco::drawDetectedCornersCharuco(image,corners,ids,Scalar(0,0,255));
//                                    aruco::drawDetectedMarkers(image,marker_corners,marker_ids,Scalar(0,255,0));
//                                    cv::imshow("Test",image);
//                                    cv::waitKey(0);
                                }
                            }
                            else{
                                spdlog::warn("Pattern in image {}_{} was rejected - degenerated case.", cam_idx, pat_idx);
                            }
                        }
                        spdlog::info("Found {} features in image {}_{}", features_for_rigpos.size(), cam_idx, pat_idx);
                    }
                    else{
                        spdlog::info("Found 0 features in image {}_{}", cam_idx, pat_idx);
                    }
                }
                features_for_pattern.push_back(features_for_rigpos);
            }
            features_for_cam.push_back(features_for_pattern);
        }
        detection->correspondences.push_back(features_for_cam);
    }

    return convertFeaturelistsToMat(detection);
}


void singleIntrinsicsEstimation(
    const Detection& detection, Estimation* estimation, const bool& use_initial_intrinsics, 
    const int& cam_idx_est, const int& cam_idx_det, Camera* cam, double* rms
    // , Safe_Counter* counter
){
    //convert detected Feat_Points to opencv compatible format
    vector<vector<Point3f>> object_points;
    vector<vector<Point2f>> image_points;
    bool correspondences_available = false;
    for(int pattern_idx = 0; pattern_idx < detection.num_pattern_positions; pattern_idx++){
        for(int rig_idx = 0; rig_idx < detection.num_rig_positions; rig_idx++){
            if(detection.correspondences[cam_idx_det][pattern_idx][rig_idx].size() != 0){
                vector<Point3f> object_points_for_image;
                vector<Point2f> image_points_for_image;

                for(size_t point_idx = 0; point_idx < detection.correspondences[cam_idx_det][pattern_idx][rig_idx].size(); point_idx++){
                    const Feature_Point& p = detection.correspondences[cam_idx_det][pattern_idx][rig_idx][point_idx];
                    object_points_for_image.push_back((Point3f)p.corr);
                    image_points_for_image.push_back((Point2f)p.p);
                }

                object_points.push_back(object_points_for_image);
                image_points.push_back(image_points_for_image);
                correspondences_available = true;
            }
        }
    }

    if(!correspondences_available){
        spdlog::error("No correspondences available for intrinsic estimation of camera {}", cam->id);
        // counter->up();
        return;
    }

    //prepare arguments for opencv intrinsic calibration
    Mat K;
    if(use_initial_intrinsics){
        K = cam->K.clone();
    }
    else{
        double K_data[9] = { 1.0, 0.0, (double)cam->resolution_x/2.0, 0.0, 1.0, (double)cam->resolution_y/2.0, 0.0, 0.0, 1.0 };
        K = Mat(3,3,CV_64F,K_data);
    }
    Mat distortion_coeffs;

    //intrinsic calibration
    if(estimation->cams[cam_idx_est].calculate_intrinsics){
        vector<Mat> rotations;
        vector<Mat> translations;
        cv::Size size(cam->resolution_x,cam->resolution_y);
        if(use_initial_intrinsics){
            *rms = cv::calibrateCamera(object_points, image_points, size, K, distortion_coeffs, rotations, translations, cv::CALIB_USE_INTRINSIC_GUESS);
        }
        else{
            *rms = cv::calibrateCamera(object_points, image_points, size, K, distortion_coeffs, rotations, translations);
        }

        //save camera matrix and estimated rotations/translations
        K.convertTo(cam->K, CV_64F);
        cv::Mat distortion_mat;
        distortion_coeffs.convertTo(distortion_mat, CV_64F);
        cam->setDistortion(distortion_mat);

        size_t tf_counter = 0;
        for(int pattern_idx = 0; pattern_idx < detection.num_pattern_positions; pattern_idx++){
            for(int rig_idx = 0; rig_idx < detection.num_rig_positions; rig_idx++){
                if(detection.correspondences[cam_idx_det][pattern_idx][rig_idx].size() != 0 && tf_counter < rotations.size()){
                    rotations[tf_counter].convertTo(estimation->rotations[cam_idx_est][pattern_idx][rig_idx],CV_64F);
                    translations[tf_counter].convertTo(estimation->translations[cam_idx_est][pattern_idx][rig_idx],CV_64F);
                    tf_counter++;
                }
            }
        }
    }
    else{
        K = cam->K.clone();
        distortion_coeffs = cam->getDistortion();
        Mat rotation, translation;
        for(int pattern_idx = 0; pattern_idx < detection.num_pattern_positions; pattern_idx++){
            for(int rig_idx = 0; rig_idx < detection.num_rig_positions; rig_idx++){
                if(detection.correspondences[cam_idx_det][pattern_idx][rig_idx].size() != 0){
                    *rms = cv::solvePnP(detection.object_points[cam_idx_det][pattern_idx][rig_idx], detection.image_points[cam_idx_det][pattern_idx][rig_idx], K, distortion_coeffs, rotation, translation);
                    rotation.convertTo(estimation->rotations[cam_idx_est][pattern_idx][rig_idx],CV_64F);
                    translation.convertTo(estimation->translations[cam_idx_est][pattern_idx][rig_idx],CV_64F);
                }
            }
        }
    }

    // counter->up();
    return;
}


bool intrinsicsEstimation(const Detection& detection, Estimation* estimation, const int& initial_intrinsics_mode){
    if(estimation->cams.size() != (size_t) detection.num_cams){
        spdlog::warn("Not all cameras have detected patterns available. Thus not all intrinsics can be calculated.");
    }

    //initialize estimation tables
    estimation->rotations = vector_3D<Mat>(detection.num_cams,vector_2D<Mat>(detection.num_pattern_positions,vector<Mat>(detection.num_rig_positions,Mat(0,0,CV_64F))));
    estimation->translations = vector_3D<Mat>(detection.num_cams,vector_2D<Mat>(detection.num_pattern_positions,vector<Mat>(detection.num_rig_positions,Mat(0,0,CV_64F))));
    estimation->transforms = vector_3D<Mat>(detection.num_cams,vector_2D<Mat>(detection.num_pattern_positions,vector<Mat>(detection.num_rig_positions,Mat(0,0,CV_64F))));

    //start intrinsics estimations in different threads
    vector<double> rmss(detection.num_cams, 0.0);

    for (int cam_idx = 0; cam_idx < detection.num_cams; cam_idx++){
        // single instrinsics estimation
        int cam_idx_est = estimation->cam_id_to_idx_map[detection.detected_cam_ids[cam_idx]];
        singleIntrinsicsEstimation(
            detection, estimation, (initial_intrinsics_mode > 0), 
            cam_idx_est, cam_idx, &(estimation->cams[cam_idx_est]), &(rmss[cam_idx])
        );
    }

    //delete threads and log calculated data
    for (int cam_idx = 0; cam_idx < detection.num_cams; cam_idx++){

        int cam_idx_est = estimation->cam_id_to_idx_map[detection.detected_cam_ids[cam_idx]];
        spdlog::info("Camera {} rms: {}", cam_idx_est, rmss[cam_idx]);
        std::ostringstream Koss, Doss;
        Koss << estimation->cams[cam_idx_est].K;
        Koss << estimation->cams[cam_idx_est].getDistortion();
        spdlog::info("K matrix:\n{}", Koss.str());
        spdlog::info("Distortion:\n{}", Doss.str());
    }

    // estimation->updateCameras();

    return true;
}


void detectPatterns(Data* data_){

    data_->input_dir = "/home/docker/workspace/workspace/multi-camera-calib/data/frames";
    //check if image input dir is set
    if(data_->input_dir.size() == 0){
        spdlog::error("Input image dir not set.");
        return;
    }

    //load rig position file if checked
    std::map<int,pairf> rig_positions;
    data_->detection.num_rig_positions = 1;
    data_->detection.rig_positions.clear();
    data_->detection.rig_positions.push_back(pairf(0.0,0.0));

    //create image list and initial cameras
    std::vector<Camera> cameras;
    if(!createImageList(data_->input_dir, rig_positions, &data_->files, &data_->detection, &cameras)){
        spdlog::error("Image list creation failed.");
        return;
    }
    data_->estimation.setCameras(cameras ,SET_CAMERA_ALL, false);

    //detect pattern
    if(data_->pattern_type == CALIB_PATTERN_CHARUCO){
        if(!detectChArUcoPatterns(data_->patternWidth(),data_->patternHeight(),data_->charuco_params,data_->files, &data_->detection)){
            spdlog::error("Unable to detect ChArUco patterns in images.");
            return;
        }
    }

    spdlog::info("Pattern detection complete.");
}


bool calculateIntrinsics(Data *data_){
    spdlog::info("Start intrinsics estimation. Please wait..");
    data_->estimation.clearGeometry();

    detectPatterns(data_);

    if(data_->detection.correspondences.size() == 0){
        spdlog::error("No correspondences available - either load them from a file or detect the patterns in the input images.");
        return false;
    }

    bool calc_transforms_only = true;
    for(const Camera& cam : data_->estimation.cams){
        if(cam.calculate_intrinsics){
            calc_transforms_only = false;
            break;
        }
    }

    if(!intrinsicsEstimation(data_->detection,&data_->estimation, data_->opt_params.initial_intrinsics_mode)){
        if(calc_transforms_only){
            spdlog::error("Transformation calculation failed.");
        }
        else{
            spdlog::error("Intrinsic calibration failed.");
        }
        return false;
    }
    if(calc_transforms_only){
        spdlog::info("Transformation successfully calculated.");
    }
    else{
        spdlog::info("Intrinsics successfully calculated.");
    }

    return true;
};

// --------------------------------------------


bool calculateJacobian(const Detection& detection, const Estimation& estimation, const Mat& extrinsics, const Mat& intrinsics, const Optimization_Params& opt_params, const Mat& rig, const Graph& graph, Mat* jacobian, Mat* error){
    bool refine_rig = !(rig.rows == 0);
    int num_params = (int)(extrinsics.total() + intrinsics.total() + rig.total());
    int num_edges = (int)graph.edges.size();

    //calculate start indices for every edge
    vector<int> start_idx(num_edges + 1, 0);
    for (int edge_idx = 0; edge_idx < num_edges; edge_idx++){
        int cam_idx = graph.cam_vertices[graph.edges[edge_idx].cam_vertex].camera_idx;
        int pattern_idx = graph.pattern_vertices[graph.edges[edge_idx].pattern_vertex].pattern_position;
        int rig_idx = graph.edges[edge_idx].rig_position;

        start_idx[edge_idx + 1] = start_idx[edge_idx] + 2* (int)detection.correspondences[cam_idx][pattern_idx][rig_idx].size();
    }

    *jacobian = Mat::zeros(start_idx[num_edges], num_params, CV_64F);
    *error = Mat::zeros(start_idx[num_edges], 1, CV_64F);

    for(int edge_idx = 0; edge_idx < num_edges; edge_idx++){
        const Edge& edge = graph.edges[edge_idx];

        int cam_idx = graph.cam_vertices[edge.cam_vertex].camera_idx;
        int pattern_idx = graph.pattern_vertices[edge.pattern_vertex].pattern_position;
        int rig_idx = edge.rig_position;

        Mat object_points = detection.object_points[cam_idx][pattern_idx][rig_idx];
        Mat image_points = detection.image_points[cam_idx][pattern_idx][rig_idx];

        //check if current cameras extrinsics should be optimized
        bool optimize_extrinsics = false;
        int optimization_index = 0;
        if(opt_params.calculate_extrinsics_mode == CALC_EXTRINSICS_ALL){
            optimize_extrinsics = true;
            optimization_index = edge.cam_vertex;
        }
        else{
            optimize_extrinsics = estimation.cams[cam_idx].optimize_extrinsics;
            optimization_index = estimation.calculation_idx[cam_idx];
        }

        //get rotations and translations of vertices, i.e. cam+pattern
        int offset = 6 * int(estimation.calc_extrinsics_idx_list.size()); //start idx of pattern parameters in extrinsics vector
        Mat Rod_pattern, T_pattern, Rod_cam, T_cam;
        Rod_pattern = extrinsics.colRange(offset+edge.pattern_vertex*6, offset+edge.pattern_vertex*6 + 3);
        Rod_pattern = Rod_pattern.reshape(1,3);
        T_pattern = extrinsics.colRange(offset+edge.pattern_vertex*6 + 3, offset+edge.pattern_vertex*6 + 6);
        T_pattern = T_pattern.reshape(1,3);

        if (edge.cam_vertex > 0 && optimize_extrinsics){
            Rod_cam = extrinsics.colRange((optimization_index-1)*6, (optimization_index-1)*6 + 3);
            Rod_cam = Rod_cam.reshape(1,3);
            T_cam = extrinsics.colRange((optimization_index-1)*6 + 3, (optimization_index-1)*6 + 6);
            T_cam = T_cam.reshape(1,3);
        }
        else{
            cv::Rodrigues(graph.cam_vertices[edge.cam_vertex].pose.colRange(0,3).rowRange(0,3), Rod_cam);
            graph.cam_vertices[edge.cam_vertex].pose.col(3).rowRange(0,3).copyTo(T_cam);
        }


        bool refine_intrinsics = false;
        int refine_idx = 0;
        Mat K = cv::Mat::eye(3,3,CV_64F);
        Distortion_Parameters d;
        if(opt_params.refine_intrinsics_mode == REFINE_INTRINSICS_ALL){
            refine_idx = edge.cam_vertex;
            refine_intrinsics = true;

            Mat intrinsic_cam = intrinsics.colRange(9*edge.cam_vertex, 9*edge.cam_vertex+9);
            K.at<double>(0,0) = intrinsic_cam.at<double>(0,0);
            K.at<double>(1,1) = intrinsic_cam.at<double>(0,1);
            K.at<double>(0,2) = intrinsic_cam.at<double>(0,2);
            K.at<double>(1,2) = intrinsic_cam.at<double>(0,3);
            d.k_1 = intrinsic_cam.at<double>(0,4);
            d.k_2 = intrinsic_cam.at<double>(0,5);
            d.p_1 = intrinsic_cam.at<double>(0,6);
            d.p_2 = intrinsic_cam.at<double>(0,7);
            d.k_3 = intrinsic_cam.at<double>(0,8);
        }
        else if(opt_params.refine_intrinsics_mode == REFINE_INTRINSICS_GUI && estimation.cams[cam_idx].refine_intrinsics){
            refine_idx = estimation.refine_idx[cam_idx];
            refine_intrinsics = true;

            Mat intrinsic_cam = intrinsics.colRange(9*refine_idx, 9*refine_idx+9);
            K.at<double>(0,0) = intrinsic_cam.at<double>(0,0);
            K.at<double>(1,1) = intrinsic_cam.at<double>(0,1);
            K.at<double>(0,2) = intrinsic_cam.at<double>(0,2);
            K.at<double>(1,2) = intrinsic_cam.at<double>(0,3);
            d.k_1 = intrinsic_cam.at<double>(0,4);
            d.k_2 = intrinsic_cam.at<double>(0,5);
            d.p_1 = intrinsic_cam.at<double>(0,6);
            d.p_2 = intrinsic_cam.at<double>(0,7);
            d.k_3 = intrinsic_cam.at<double>(0,8);
        }
        else{
            K = estimation.cams[cam_idx].K;
            d = estimation.cams[cam_idx].distortion;
        }

        Mat T_x_rig = detection.rig_positions[rig_idx].first*estimation.x_axis;
        Mat T_y_rig = detection.rig_positions[rig_idx].second*estimation.y_axis;

        if(refine_rig){
            T_x_rig = detection.rig_positions[rig_idx].first*rig.colRange(0,3).reshape(1,3);
            T_y_rig = detection.rig_positions[rig_idx].second*rig.colRange(3,6).reshape(1,3);
        }

        //calculate rotation matrices and derivatives of those wrt the rodrigues parameters
        Mat R_pattern, R_cam, dR_pattern_dRod_pattern, dR_cam_dRod_cam;
        cv::Rodrigues(Rod_pattern,R_pattern,dR_pattern_dRod_pattern);
        dR_pattern_dRod_pattern = dR_pattern_dRod_pattern.t();
        cv::Rodrigues(Rod_cam,R_cam,dR_cam_dRod_cam);
        dR_cam_dRod_cam = dR_cam_dRod_cam.t();

        //calculate helpful matrices
        Mat Id = Mat::eye(3,3,CV_64F);
        Mat df_dR_cam, dR_c_R_p_y_dR_c, dR_c_R_p_y_dR_p;
        Mat dR_c_T_p_dR_c = cv::Mat::zeros(3,9,CV_64F);
        for(int col = 0; col < 9; col++){
            int col_idx = (int) ((float)(col)/3.0);
            Id.col(col_idx).copyTo(dR_c_T_p_dR_c.col(col));
            dR_c_T_p_dR_c.col(col) *= T_pattern.at<double>(col%3,0);
        }

        Mat J_proj, J_dist, J_img;
        Mat df_dRod_cam_proj, df_dT_cam_proj, df_dRod_pattern_proj, df_dT_pattern_proj, df_ddistortion, df_dK, df_dT_x_rig, df_dT_y_rig;
        Mat x, y, f_y_proj, f_y, f_y_dist, f_y_img, proj_error, R_p_y;

        //calculate derivatives of 3D-to-camera projection f
        //1. df/dt_cam = Id
        Mat df_dT_cam =  Id;
        //2. df/dt_pattern = R_cam
        Mat df_dT_pattern =  R_cam;
        //3. rotational derivatives
        Mat df_dRod_cam, df_dRod_pattern;
        for(size_t i = 0; i < object_points.total(); i++){
            //get object point
            y = object_points.row(i);
            y = y.reshape(1,3);
            x = image_points.row(i);
            x = x.reshape(1,2);

            //calculate 3d projection coordinates f(y)
            f_y = R_cam*(R_pattern*y+T_pattern)+T_cam+T_x_rig+T_y_rig;

            //calculate 2d projection
            f_y_proj = Mat::zeros(2,1,CV_64F);
            f_y_proj.at<double>(0,0) = f_y.at<double>(0,0)/f_y.at<double>(2,0);
            f_y_proj.at<double>(1,0) = f_y.at<double>(1,0)/f_y.at<double>(2,0);

            //calculate Jacobian for 3d-to-2d projection
            J_proj = Mat::zeros(2,3,CV_64F);
            J_proj.at<double>(0,0) = 1.0/f_y.at<double>(2,0);
            J_proj.at<double>(1,1) = 1.0/f_y.at<double>(2,0);
            J_proj.at<double>(0,2) = -f_y.at<double>(0,0)/pow(f_y.at<double>(2,0),2.0);
            J_proj.at<double>(1,2) = -f_y.at<double>(1,0)/pow(f_y.at<double>(2,0),2.0);


            //distort projection
            double x_ = f_y_proj.at<double>(0,0);
            double y_ = f_y_proj.at<double>(1,0);
            double r_ = x_*x_+y_*y_;
            f_y_dist = Mat::zeros(2,1,CV_64F);
            f_y_dist.at<double>(0,0) = x_*(1+d.k_1*r_+d.k_2*r_*r_+d.k_3*r_*r_*r_)+2.0*d.p_1*x_*y_+d.p_2*(r_+2.0*x_*x_);
            f_y_dist.at<double>(1,0) = y_*(1+d.k_1*r_+d.k_2*r_*r_+d.k_3*r_*r_*r_)+2.0*d.p_2*x_*y_+d.p_1*(r_+2.0*y_*y_);

            //calculate Jacobian for distortion
            J_dist = Mat::zeros(2,2,CV_64F);
            J_dist.at<double>(0,0) = 1+d.k_1*(2.0*x_*x_+r_)+d.k_2*(r_*r_+4.0*x_*x_*r_)+d.k_3*(r_*r_*r_+6.0*x_*x_*r_*r_)+2.0*d.p_1*y_+6.0*x_*d.p_2;
            J_dist.at<double>(0,1) = 2.0*y_*d.k_1*x_+4.0*d.k_2*(x_*y_*r_)+6.0*d.k_3*(x_*y_*r_*r_)+2.0*d.p_1*x_+2.0*d.p_2*y_;
            J_dist.at<double>(1,0) = J_dist.at<double>(0,1);
            J_dist.at<double>(1,1) = 1+d.k_1*(2.0*y_*y_+r_)+d.k_2*(r_*r_+4.0*y_*y_*r_)+d.k_3*(r_*r_*r_+6.0*y_*y_*r_*r_)+2.0*d.p_2*x_+6.0*y_*d.p_1;

            //transfer to image coordinates
            f_y_img = Mat::zeros(2,1,CV_64F);
            f_y_img.at<double>(0,0) = K.at<double>(0,0)*f_y_dist.at<double>(0,0)+K.at<double>(0,2);
            f_y_img.at<double>(1,0) = K.at<double>(1,1)*f_y_dist.at<double>(1,0)+K.at<double>(1,2);

            //calculate Jacobian for coordinate change
            J_img = Mat::zeros(2,2,CV_64F);
            K.rowRange(0,2).colRange(0,2).copyTo(J_img);

            //calculate Jacobians for intrinsic refinement
            if(refine_intrinsics){
                Mat ddist_ddistortion = Mat::zeros(2,5,CV_64F);
                //ddist/dk_1
                ddist_ddistortion.at<double>(0,0) = x_*r_;
                ddist_ddistortion.at<double>(1,0) = y_*r_;
                //ddist/dk_2
                ddist_ddistortion.at<double>(0,1) = x_*r_*r_;
                ddist_ddistortion.at<double>(1,1) = y_*r_*r_;
                //ddist/dp_1
                ddist_ddistortion.at<double>(0,2) = 2.0*x_*y_;
                ddist_ddistortion.at<double>(1,2) = r_+2.0*y_*y_;
                //ddist/dp_2
                ddist_ddistortion.at<double>(0,3) = r_+2.0*x_*x_;
                ddist_ddistortion.at<double>(1,3) = 2.0*x_*y_;
                //ddist/dk_3
                ddist_ddistortion.at<double>(0,4) = x_*r_*r_*r_;
                ddist_ddistortion.at<double>(1,4) = y_*r_*r_*r_;

                df_ddistortion = J_img * ddist_ddistortion;

                df_dK = cv::Mat::zeros(2,4,CV_64F);
                //df/dfx
                df_dK.at<double>(0,0) = f_y_dist.at<double>(0,0);
                df_dK.at<double>(1,1) = f_y_dist.at<double>(1,0);
                df_dK.at<double>(0,2) = 1.0;
                df_dK.at<double>(1,3) = 1.0;
            }


            //save error
            proj_error = x-f_y_img;
            proj_error.copyTo(error->rowRange(start_idx[edge_idx]+2*i,start_idx[edge_idx]+2*i+2));

            //calculate helpful matrices which depend on the detected points
            R_p_y = R_pattern * y;
            dR_c_R_p_y_dR_c = cv::Mat::zeros(3,9,CV_64F);
            for(int col = 0; col < 9; col++){
                int col_idx = (int) ((float)(col)/3.0);
                Id.col(col_idx).copyTo(dR_c_R_p_y_dR_c.col(col));
                dR_c_R_p_y_dR_c.col(col) *= R_p_y.at<double>(col%3,0);
            }

            dR_c_R_p_y_dR_p = cv::Mat::zeros(3,9,CV_64F);
            for(int col = 0; col < 9; col++){
                int col_idx = (int) ((float)(col)/3.0);
                Id.col(col_idx).copyTo(dR_c_R_p_y_dR_p.col(col));
                dR_c_R_p_y_dR_p.col(col) *= y.at<double>(col%3,0);
            }

            //3.1. df/dRod_cam = df/dR_cam*dR_cam/dRod_cam = (dR_cam_R_pattern_y/dR_cam + dR_camT_pattern/dR_cam)*dR_cam/dRod_cam
            df_dR_cam = dR_c_R_p_y_dR_c+dR_c_T_p_dR_c;
            df_dRod_cam = df_dR_cam * dR_cam_dRod_cam;

            //3.2. df/dRod_pattern = df/dR_pattern*dR_pattern/dRod_pattern = dR_camR_pattern_y/dR_pattern*dR_pattern/dRod_pattern
            df_dRod_pattern = dR_c_R_p_y_dR_p * dR_pattern_dRod_pattern;

            //save derivatives to jacobian
            df_dRod_cam_proj = J_img*J_dist*J_proj*df_dRod_cam;
            df_dT_cam_proj = J_img*J_dist*J_proj*df_dT_cam;
            df_dRod_pattern_proj = J_img*J_dist*J_proj*df_dRod_pattern;
            df_dT_pattern_proj = J_img*J_dist*J_proj*df_dT_pattern;

            if(edge.cam_vertex > 0 && optimize_extrinsics){
                df_dRod_cam_proj.copyTo(jacobian->rowRange(start_idx[edge_idx]+2*i,start_idx[edge_idx]+2*i+2).colRange((optimization_index-1)*6, (optimization_index-1)*6 + 3));
                df_dT_cam_proj.copyTo(jacobian->rowRange(start_idx[edge_idx]+2*i,start_idx[edge_idx]+2*i+2).colRange((optimization_index-1)*6 + 3, (optimization_index-1)*6 + 6));
            }
            df_dRod_pattern_proj.copyTo(jacobian->rowRange(start_idx[edge_idx]+2*i,start_idx[edge_idx]+2*i+2).colRange(offset+edge.pattern_vertex*6, offset+edge.pattern_vertex*6 + 3));
            df_dT_pattern_proj.copyTo(jacobian->rowRange(start_idx[edge_idx]+2*i,start_idx[edge_idx]+2*i+2).colRange(offset+edge.pattern_vertex*6 + 3, offset+edge.pattern_vertex*6 + 6));

            if(refine_intrinsics){
                int offset_intr = extrinsics.total();
                df_dK.copyTo(jacobian->rowRange(start_idx[edge_idx]+2*i,start_idx[edge_idx]+2*i+2).colRange(offset_intr+refine_idx*9, offset_intr+refine_idx*9+4));
                df_ddistortion.copyTo(jacobian->rowRange(start_idx[edge_idx]+2*i,start_idx[edge_idx]+2*i+2).colRange(offset_intr+refine_idx*9+4, offset_intr+refine_idx*9+9));
            }

            if(refine_rig){
                int offset_rig = extrinsics.total()+intrinsics.total();
                df_dT_x_rig = detection.rig_positions[rig_idx].first*J_img*J_dist*J_proj*Mat::eye(3,3,CV_64F);
                df_dT_y_rig = detection.rig_positions[rig_idx].second*J_img*J_dist*J_proj*Mat::eye(3,3,CV_64F);
                df_dT_x_rig.copyTo(jacobian->rowRange(start_idx[edge_idx]+2*i,start_idx[edge_idx]+2*i+2).colRange(offset_rig, offset_rig+3));
                df_dT_y_rig.copyTo(jacobian->rowRange(start_idx[edge_idx]+2*i,start_idx[edge_idx]+2*i+2).colRange(offset_rig+3, offset_rig+6));
            }
        }
    }

    return true;
}

bool optimizeExtrinsicsLM(const Detection& detection, Estimation* estimation, Graph* graph, double* error, const Calibration::LM_Params& params, const Optimization_Params& opt_params){
    //prepare initial solution for LM optimization
    int num_cams_to_optimize = 0;
    estimation->calc_extrinsics_idx_list.clear();
    for(unsigned int i = 1; i < graph->cam_vertices.size(); i++){
        const Camera_Vertex& cam_vertex = graph->cam_vertices[i];
        if(estimation->cams[cam_vertex.camera_idx].optimize_extrinsics){
            estimation->calc_extrinsics_idx_list.push_back(i);
            num_cams_to_optimize++;
        }
    }
    std::cerr << num_cams_to_optimize;

    int num_vertices = int(estimation->calc_extrinsics_idx_list.size()) + int(graph->pattern_vertices.size());
    Mat extrinsics(1, num_vertices*6, CV_64F);
    int offset = 0;
    for (size_t i = 0; i < estimation->calc_extrinsics_idx_list.size(); i++){
        //get rodrigues rotation and translation from pose saved in vertex
        Mat rotation, translation;
        cv::Rodrigues(graph->cam_vertices[estimation->calc_extrinsics_idx_list[i]].pose.rowRange(0,3).colRange(0, 3), rotation);
        graph->cam_vertices[estimation->calc_extrinsics_idx_list[i]].pose.rowRange(0,3).col(3).copyTo(translation);
        rotation.reshape(1, 1).copyTo(extrinsics.colRange(offset, offset + 3));
        translation.reshape(1, 1).copyTo(extrinsics.colRange(offset+3, offset +6));
        offset += 6;
    }
    for (size_t i = 0; i < graph->pattern_vertices.size(); i++){
        //get rodrigues rotation and translation from pose saved in vertex
        Mat rotation, translation;
        cv::Rodrigues(graph->pattern_vertices.at(i).pose.rowRange(0,3).colRange(0, 3), rotation);
        graph->pattern_vertices.at(i).pose.rowRange(0,3).col(3).copyTo(translation);
        rotation.reshape(1, 1).copyTo(extrinsics.colRange(offset, offset + 3));
        translation.reshape(1, 1).copyTo(extrinsics.colRange(offset+3, offset +6));

        offset += 6;
    }

    //prepare initial intrinsics vector for refinement
    Mat intrinsics;
    int intrinsics_size = 0;
    if(opt_params.refine_intrinsics_mode == REFINE_INTRINSICS_GUI){
        for (const Camera& cam : estimation->cams){
            if(cam.refine_intrinsics){
                intrinsics_size++;
            }
        }
    }
    else if(opt_params.refine_intrinsics_mode == REFINE_INTRINSICS_ALL){
        intrinsics_size = (int)(graph->cam_vertices.size());
    }

    if(opt_params.refine_intrinsics_mode != REFINE_INTRINSICS_NONE && intrinsics_size > 0){
        intrinsics = cv::Mat(1, intrinsics_size*9, CV_64F);
        offset = 0;
        for (size_t i = 0; i < graph->cam_vertices.size(); i++){
            if(estimation->cams[i].refine_intrinsics || opt_params.refine_intrinsics_mode == REFINE_INTRINSICS_ALL){
                Mat fx_fy_cx_cy(1,4, CV_64F);
                fx_fy_cx_cy.at<double>(0,0) = estimation->cams[graph->cam_vertices[i].camera_idx].K.at<double>(0,0);
                fx_fy_cx_cy.at<double>(0,1) = estimation->cams[graph->cam_vertices[i].camera_idx].K.at<double>(1,1);
                fx_fy_cx_cy.at<double>(0,2) = estimation->cams[graph->cam_vertices[i].camera_idx].K.at<double>(0,2);
                fx_fy_cx_cy.at<double>(0,3) = estimation->cams[graph->cam_vertices[i].camera_idx].K.at<double>(1,2);
                Mat distortion = estimation->cams[graph->cam_vertices[i].camera_idx].getDistortion();

                fx_fy_cx_cy.copyTo(intrinsics.colRange(offset,offset+4));
                distortion.copyTo(intrinsics.colRange(offset+4, offset+9));

                offset += 9;
            }
        }
    }
    else{
        intrinsics = cv::Mat(0,0,CV_64F);
    }

    //prepare initial rig vector for refinement
    Mat rig;
    if(opt_params.refine_rig){
        rig = cv::Mat(1, 6, CV_64F);
        rig.colRange(0,3) = estimation->x_axis.t();
        rig.colRange(3,6) = estimation->y_axis.t();
    }
    else{
        rig = cv::Mat(0,0,CV_64F);
    }

    int num_extrinsics = extrinsics.total();
    int num_intrinsics = intrinsics.total();
    int num_rig = rig.total();

    //Levenberg Marquardt optimization
    double change = 1.0;
    double lambda = 1e-5; //initial step size
    *error = 1e+100;
    double current_error;
    Mat J, J_last, Error, Error_last, DtD, JtJ_step, step, JtJ;
    bool last_failed = false;

    for(int iter = 0; ; iter++){
        //check termination criteria
        if ((params.term_criterium == LM_TERM_MAX_ITERATIONS && iter >= params.max_iterations) ||
                (params.term_criterium == LM_TERM_CHANGE_THRESHOLD && change <= params.change_threshold) ||
                (params.term_criterium == LM_TERM_COMBINED && (change <= params.change_threshold || iter >= params.max_iterations))){
            break;
        }

        //if the error increased in the last iteration, reset the jacobian etc. (but not the stepsize!)
        if(last_failed){
            last_failed = false;
            spdlog::info("Iteration {} failed. Resetting.",iter-1);

            J = J_last.clone();
            Error = Error_last.clone();
            JtJ =  J.t() * J;
            DtD = cv::Mat::eye(J.cols,J.cols,CV_64F).mul(JtJ);
            JtJ_step = JtJ + lambda * DtD;
            solve(JtJ_step,J.t() * Error,step);
            step = step.reshape(1,1);
            extrinsics = extrinsics + step.colRange(0,num_extrinsics);
            if(opt_params.refine_intrinsics_mode != REFINE_INTRINSICS_NONE && intrinsics_size > 0){
                intrinsics = intrinsics + step.colRange(num_extrinsics,num_extrinsics+num_intrinsics);
            }
            if(opt_params.refine_rig){
                rig = rig + step.colRange(num_extrinsics+num_intrinsics,num_extrinsics+num_intrinsics+num_rig);
            }
            iter--;
            continue;
        }

        //calculate jacobian
        calculateJacobian(detection, *estimation, extrinsics, intrinsics, opt_params, rig, *graph, &J, &Error);

        current_error = 0.0;
        for(int point_idx = 0; point_idx < Error.rows/2; point_idx++){
            current_error += pow(Error.at<double>(2*point_idx,0),2.0)+pow(Error.at<double>(2*point_idx+1,0),2.0);
        }
        current_error = sqrt(current_error/(double)(Error.rows/2));

        spdlog::info("Iteration {} Error: {} Stepsize: ",iter, current_error, lambda);

        //calculate change
        change = *error - current_error;
        //adjust stepsize accordingly
        if(current_error > *error){
            if(iter == 0){
                spdlog::error("Initial LM error for extrinsic calibration too high.");
                return false;
            }

            step = step.reshape(1,1);
            extrinsics = extrinsics - step.colRange(0,num_extrinsics);
            if(opt_params.refine_intrinsics_mode != REFINE_INTRINSICS_NONE && intrinsics_size > 0){
                intrinsics = intrinsics - step.colRange(num_extrinsics,num_extrinsics+num_intrinsics);
            }
            if(opt_params.refine_rig){
                rig = rig - step.colRange(num_extrinsics+num_intrinsics,num_extrinsics+num_intrinsics+num_rig);
            }

            lambda *= 10.0;
            last_failed = true;
            change = 1.0;
            continue;
        }
        else{
            // TODO: time
            *error = current_error;
            // time.start();
            JtJ = J.t()*J;
            // int elapsed = time.elapsed();
            // spdlog::info("Time elapsed: "+QString::number(elapsed)+" ms");

            DtD = cv::Mat::eye(J.cols,J.cols,CV_64F).mul(JtJ);
            JtJ_step = JtJ + lambda * DtD;
            solve(JtJ_step,J.t() * Error,step);
            step = step.reshape(1,1);
            extrinsics = extrinsics + step.colRange(0,num_extrinsics);
            if(opt_params.refine_intrinsics_mode != REFINE_INTRINSICS_NONE && intrinsics_size > 0){
                intrinsics = intrinsics + step.colRange(num_extrinsics,num_extrinsics+num_intrinsics);
            }
            if(opt_params.refine_rig){
                rig = rig + step.colRange(num_extrinsics+num_intrinsics,num_extrinsics+num_intrinsics+num_rig);
            }

            J_last = J.clone();
            Error_last = Error.clone();

            lambda /= 10.0;
        }
    }

    //calculate final rotations and translations from optimized poses
    step = step.reshape(1,1);
    extrinsics = extrinsics - step.colRange(0,num_extrinsics);
    if(opt_params.refine_intrinsics_mode != REFINE_INTRINSICS_NONE && intrinsics_size > 0){
        intrinsics = intrinsics - step.colRange(num_extrinsics,num_extrinsics+num_intrinsics);
    }
    if(opt_params.refine_rig){
        rig = rig - step.colRange(num_extrinsics+num_intrinsics,num_extrinsics+num_intrinsics+num_rig);
    }

    //save camera pose results and intrinsics to estimated cameras
    vector<Vec3d> rotations, translations;
    for (int i = 0; i < extrinsics.cols; i+=6){
        rotations.push_back(Vec3d(extrinsics.colRange(i, i + 3)));
        translations.push_back(Vec3d(extrinsics.colRange(i + 3, i + 6)));
    }
    vector<Mat> fx_fy_cx_cy, distortions;
    if(opt_params.refine_intrinsics_mode != REFINE_INTRINSICS_NONE && intrinsics_size > 0){
        for (int i = 0; i < intrinsics.cols; i+=9){
            fx_fy_cx_cy.push_back(Mat(intrinsics.colRange(i, i + 4)));
            distortions.push_back(Mat(intrinsics.colRange(i + 4, i + 9)));
        }
    }

    int intrinsics_counter = 0;
    int extrinsics_counter = 0;
    for (size_t i = 0; i < graph->cam_vertices.size(); i++){
        if(i!=0 && estimation->cams[graph->cam_vertices[i].camera_idx].optimize_extrinsics){
            Mat pose = Mat::eye(4,4,CV_64F);
            Mat R;
            Rodrigues(rotations[extrinsics_counter], R);
            R.copyTo(pose.rowRange(0,3).colRange(0,3));
            Mat(translations[extrinsics_counter]).reshape(1, 3).copyTo(pose.rowRange(0, 3).col(3));
            pose.copyTo(estimation->cams[graph->cam_vertices[i].camera_idx].pose);
            extrinsics_counter++;
        }

        if(opt_params.refine_intrinsics_mode != REFINE_INTRINSICS_NONE && intrinsics_size > 0 && estimation->cams[i].refine_intrinsics){
            Mat dist = distortions[intrinsics_counter].reshape(1,1);
            estimation->cams[graph->cam_vertices[i].camera_idx].setDistortion(dist);
            Mat K = cv::Mat::eye(3,3,CV_64F);
            K.at<double>(0,0) = fx_fy_cx_cy[intrinsics_counter].at<double>(0,0);
            K.at<double>(1,1) = fx_fy_cx_cy[intrinsics_counter].at<double>(0,1);
            K.at<double>(0,2) = fx_fy_cx_cy[intrinsics_counter].at<double>(0,2);
            K.at<double>(1,2) = fx_fy_cx_cy[intrinsics_counter].at<double>(0,3);
            K.copyTo(estimation->cams[graph->cam_vertices[i].camera_idx].K);
            intrinsics_counter++;
        }
    }

    for(size_t i = 0; i < graph->pattern_vertices.size(); i++){
        cv::Mat rot;
        cv::Rodrigues(rotations[i+graph->cam_vertices.size()-1],rot);
        graph->pattern_vertices[i].pose = cv::Mat::eye(4,4,CV_64F);
        rot.copyTo(graph->pattern_vertices[i].pose.colRange(0,3).rowRange(0,3));
        Mat(translations[i+graph->cam_vertices.size()-1]).reshape(1,3).copyTo(graph->pattern_vertices[i].pose.rowRange(0,3).col(3));
    }

    //save pattern transformations
    for (size_t i = 0; i < graph->edges.size(); i++){
        Edge& edge = graph->edges[i];
        int cam_idx = graph->cam_vertices[edge.cam_vertex].camera_idx;
        int pattern_idx = graph->pattern_vertices[edge.pattern_vertex].pattern_position;

        Mat transform = estimation->cams[cam_idx].pose*graph->pattern_vertices[edge.pattern_vertex].pose;

        cv::Mat rod;
        cv::Rodrigues(transform.colRange(0,3).rowRange(0,3),rod);
        cv::Mat t = transform.rowRange(0,3).col(3);

        estimation->rotations[cam_idx][pattern_idx][edge.rig_position] = rod.clone();
        estimation->translations[cam_idx][pattern_idx][edge.rig_position] = t.clone();
    }

    //save rig parameters
    if(opt_params.refine_rig){
        rig = rig.reshape(1,6);
        rig.rowRange(0,3).copyTo(estimation->x_axis);
        rig.rowRange(3,6).copyTo(estimation->y_axis);
    }

    //calculate per camera error
    vector<double> error_per_cam = vector<double>(graph->cam_vertices.size(),0.0);
    vector<int> entries_per_cam = vector<int>(graph->cam_vertices.size(),0);
    int current_offset = 0;
    for(const Edge& edge : graph->edges){
        int cam_idx = graph->cam_vertices[edge.cam_vertex].camera_idx;
        int pattern_idx = graph->pattern_vertices[edge.pattern_vertex].pattern_position;
        int rig_idx = edge.rig_position;
        int interval = int(detection.correspondences[cam_idx][pattern_idx][rig_idx].size());

        for(int point_idx = 0; point_idx < interval; point_idx++){
            error_per_cam[edge.cam_vertex] += pow(Error.at<double>(current_offset+2*point_idx,0),2.0)+pow(Error.at<double>(current_offset+2*point_idx+1,0),2.0);
        }
        entries_per_cam[edge.cam_vertex] += interval;
        current_offset += 2*interval;
    }
    for(unsigned int i = 0 ; i < error_per_cam.size(); i++){
        error_per_cam[i] = sqrt(error_per_cam[i] / double(entries_per_cam[i]));
        spdlog::info("Error for cam {} : {}", estimation->cams[graph->cam_vertices[i].camera_idx].id, error_per_cam[i]);
    }

    return true;
};


bool buildGraph(const Detection& detection, Estimation* estimation, const int& initial_extrinsics_mode, Graph* graph){

    if(graph ==  nullptr){
        spdlog::error("Invalid parameters for graph building.");
        return false;
    }
    if(detection.num_cams == 0 || detection.num_pattern_positions == 0 || detection.num_rig_positions == 0){
        spdlog::error("In order to build the optimization graph, the camera estimations have to be initialized.");
        return false;
    }

    graph->cam_vertices.clear();
    graph->pattern_vertices.clear();
    graph->edges.clear();

    //create vertices
    vector<int> idx_for_cam_vert;
    vector<int> idx_for_pattern_vert;
    for(int i = 0; i<detection.num_cams; i++){
        graph->cam_vertices.push_back(Camera_Vertex());
        graph->cam_vertices.back().camera_idx = i;
        idx_for_cam_vert.push_back(i);
        if(i == 0){
            graph->cam_vertices.back().pose_calculated = true;
        }
    }
    for(int i = 0; i<detection.num_pattern_positions; i++){
        graph->pattern_vertices.push_back(Pattern_Vertex());
        graph->pattern_vertices.back().pattern_position = i;
        idx_for_pattern_vert.push_back(i);
    }

    //create edges and calculate transforms
    for(int cam_idx = 0; cam_idx < detection.num_cams; cam_idx++){
        for(int pattern_idx = 0; pattern_idx < detection.num_pattern_positions; pattern_idx++){
            for(int rig_idx = 0; rig_idx < detection.num_rig_positions; rig_idx++){

                Mat rotation = estimation->rotations[cam_idx][pattern_idx][rig_idx];
                Mat translation = estimation->translations[cam_idx][pattern_idx][rig_idx];

                if(rotation.rows == 0 || translation.rows == 0){
                    estimation->transforms[cam_idx][pattern_idx][rig_idx] = cv::Mat(0,0,CV_64F).clone();
                    continue;
                }

                Mat transform = Mat::eye(4, 4, CV_64F);
                Mat R, T;
                Rodrigues(rotation, R);
                T = translation.reshape(1, 3);
                R.copyTo(transform.rowRange(0, 3).colRange(0, 3));
                T.copyTo(transform.rowRange(0, 3).col(3));

                transform.copyTo(estimation->transforms[cam_idx][pattern_idx][rig_idx]);

                graph->edges.push_back(Edge(cam_idx, pattern_idx, rig_idx, transform));
            }
        }
    }

    //precalculate rig transforms
    vector<Mat> rig_transform;
    for(int rig_idx = 0; rig_idx < detection.num_rig_positions; rig_idx++){
        Mat transform = Mat::eye(4, 4, CV_64F);
        Mat translation = detection.rig_positions[rig_idx].first*estimation->x_axis + detection.rig_positions[rig_idx].second*estimation->y_axis;
        translation.copyTo(transform.rowRange(0,3).col(3));
        rig_transform.push_back(transform);
    }

    //traverse graph in order to calculate initial poses for all vertices
    vector<bool> cams_visited(graph->cam_vertices.size(),false);
    vector<bool> patterns_visited(graph->pattern_vertices.size(),false);

    set<int> queue_cam = {0};
    set<int> queue_pattern = {};

    while(queue_cam.size() != 0 || queue_pattern.size() != 0){

        bool is_cam_vertex = true;
        int vert_idx;

        //check if current vertex is cam certex or pattern vertex
        if(queue_cam.size() != 0){
            vert_idx = *queue_cam.begin();
            queue_cam.erase(queue_cam.begin());
        }
        else{
            vert_idx = *queue_pattern.begin();
            queue_pattern.erase(queue_pattern.begin());
            is_cam_vertex = false;
        }

        //check if vertex has already been visited
        if(is_cam_vertex && cams_visited[vert_idx]){
            continue;
        }
        else if(!is_cam_vertex && patterns_visited[vert_idx]){
            continue;
        }

        //add children to queue if unvisited and calculate initial poses
        if(is_cam_vertex){
            for(int pattern_idx = 0; pattern_idx < detection.num_pattern_positions; pattern_idx++){
                for(int rig_idx = 0; rig_idx < detection.num_rig_positions; rig_idx++){
                    if(estimation->transforms[vert_idx][pattern_idx][rig_idx].rows != 0){
                        if(!patterns_visited[pattern_idx] && !graph->pattern_vertices.at(pattern_idx).pose_calculated){
                            //set parent to child
                            graph->pattern_vertices.at(pattern_idx).parent_camera_idx = vert_idx;
                            //calculate pose for child
                            graph->pattern_vertices.at(pattern_idx).pose = graph->cam_vertices.at(vert_idx).pose.inv() * rig_transform[rig_idx] * estimation->transforms[vert_idx][pattern_idx][rig_idx];
                            graph->pattern_vertices.at(pattern_idx).pose_calculated = true;
                            //add pattern to queue
                            queue_pattern.insert(pattern_idx);
                        }
                        //add (child index,rig position) pair to the current vertex's child list
                        graph->cam_vertices.at(vert_idx).child_pattern_rig_idx.push_back(pairi(pattern_idx,rig_idx));
                    }
                }
            }
        }
        else{
            for(int cam_idx = 0; cam_idx < detection.num_cams; cam_idx++){
                for(int rig_idx = 0; rig_idx < detection.num_rig_positions; rig_idx++){
                    if(estimation->transforms[cam_idx][vert_idx][rig_idx].rows != 0){
                        if(!cams_visited[cam_idx] && !graph->cam_vertices.at(cam_idx).pose_calculated){
                            //set parent to child
                            graph->cam_vertices.at(cam_idx).parent_pattern_idx = vert_idx;
                            //calculate pose for child
                            graph->cam_vertices.at(cam_idx).pose = rig_transform[rig_idx] * estimation->transforms[cam_idx][vert_idx][rig_idx] * graph->pattern_vertices.at(vert_idx).pose.inv();
                            graph->cam_vertices.at(cam_idx).pose_calculated = true;
                            //add pattern to queue
                            queue_cam.insert(cam_idx);
                        }
                        //add (child index,rig position) pair to the current vertex's child list
                        graph->pattern_vertices.at(vert_idx).child_camera_rig_idx.push_back(pairi(cam_idx,rig_idx));
                    }
                }
            }
        }
    }

    //delete vertices without or too few connections
    bool changed = true;
    while(changed){
        changed = false;

        bool is_cam_vertex = true;
        int vert_idx = -1;

        //search for vertex to delete
        for(size_t i = 0; i < graph->cam_vertices.size(); i++){
            if(graph->cam_vertices.at(i).child_pattern_rig_idx.size() == 0){//camera vertex without connections
                vert_idx = i;
                break;
            }
        }
        if(vert_idx == -1){
            for(size_t i = 0; i < graph->pattern_vertices.size(); i++){
                if(graph->pattern_vertices.at(i).child_camera_rig_idx.size() <= 1){//pattern vertex with no or only one connection
                    vert_idx = i;
                    is_cam_vertex = false;
                    break;
                }
            }
        }
        if(vert_idx == -1){
            continue;
        }

        //delete vertex and all corresponding entries in edges and other vertices
        changed = true;
        if(is_cam_vertex){
            int vert_pos = graph->cam_vertices[vert_idx].camera_idx;
            spdlog::warn(
                "Camera {} does not have sufficient correspondences with other cameras to be optimized.",
                estimation->cams[graph->cam_vertices[idx_for_cam_vert[vert_pos]].camera_idx].id
            );
            // QThread::sleep(2);
            graph->cam_vertices.erase(graph->cam_vertices.begin()+idx_for_cam_vert[vert_pos]); //delete vertex
            idx_for_cam_vert[vert_pos] = -1; //set idx
            for(size_t i = vert_pos+1; i < idx_for_cam_vert.size(); i++){
                idx_for_cam_vert[i]--;
            }

            //delete corresponding edges
            for(size_t i = 0; i < graph->edges.size(); i++){
                if(graph->edges.at(i).cam_vertex == vert_pos){
                    graph->edges.erase(graph->edges.begin()+i);
                    i--;
                }
            }
            //delete from child lists of pattern vertices
            for(size_t i = 0; i < graph->pattern_vertices.size(); i++){
                for(size_t j = 0; j < graph->pattern_vertices.at(i).child_camera_rig_idx.size(); j++){
                    if(graph->pattern_vertices.at(i).child_camera_rig_idx[j].first == vert_pos){
                        graph->pattern_vertices.at(i).child_camera_rig_idx.erase(graph->pattern_vertices.at(i).child_camera_rig_idx.begin()+j);
                        j--;
                    }
                }
            }
        }
        else{
            int vert_pos = graph->pattern_vertices[vert_idx].pattern_position;
            graph->pattern_vertices.erase(graph->pattern_vertices.begin()+idx_for_pattern_vert[vert_pos]); //delete vertex
            idx_for_pattern_vert[vert_pos] = -1; //set idx
            for(size_t i = vert_pos+1; i < idx_for_pattern_vert.size(); i++){
                idx_for_pattern_vert[i]--;
            }

            //delete corresponding edges
            for(size_t i = 0; i < graph->edges.size(); i++){
                if(graph->edges.at(i).pattern_vertex == vert_pos){
                    graph->edges.erase(graph->edges.begin()+i);
                    i--;
                }
            }
            //delete from child lists of cam vertices
            for(size_t i = 0; i < graph->cam_vertices.size(); i++){
                for(size_t j = 0; j < graph->cam_vertices.at(i).child_pattern_rig_idx.size(); j++){
                    if(graph->cam_vertices.at(i).child_pattern_rig_idx[j].first == vert_pos){
                        graph->cam_vertices.at(i).child_pattern_rig_idx.erase(graph->cam_vertices.at(i).child_pattern_rig_idx.begin()+j);
                        j--;
                    }
                }
            }
        }
    }

    //reset vertex indices in edges to match the vertex lists after the deletion process
    for(Edge& edge : graph->edges){
        edge.cam_vertex = idx_for_cam_vert[edge.cam_vertex];
        edge.pattern_vertex = idx_for_pattern_vert[edge.pattern_vertex];
    }

    //set idx lists
    estimation->refine_idx = vector<int>(estimation->cams.size(), -1);
    estimation->calculation_idx = vector<int>(estimation->cams.size(), -1);
    int refine_counter = 0;
    int calc_counter = 0;
    for(const Camera_Vertex& cam : graph->cam_vertices){
        if(estimation->cams[cam.camera_idx].refine_intrinsics){
            estimation->refine_idx[cam.camera_idx] = refine_counter;
            refine_counter++;
        }
        if(estimation->cams[cam.camera_idx].optimize_extrinsics){
            estimation->calculation_idx[cam.camera_idx] = calc_counter;
            calc_counter++;
        }
    }

    if(initial_extrinsics_mode == INIT_EXTRINSICS_FROM_GUI){
        for(Camera_Vertex& cam : graph->cam_vertices){
            estimation->cams[cam.camera_idx].pose.copyTo(cam.pose);
        }
    }

    return true;
};


bool optimizeExtrinsics(const Detection& detection, Estimation* estimation, const LM_Params& params, const Optimization_Params& opt_params){
    //build graph
    spdlog::info("Building graph..");
    Graph graph;

    if(!buildGraph(detection, estimation, opt_params.initial_extrinsics_mode, &graph)){
        spdlog::error("Graph construction failed.");
        return false;
    }

    spdlog::info("Initial camera poses:");
    for(size_t i = 0; i < estimation->cams.size(); i++){
        std::ostringstream oss;
        oss << estimation->cams[i].pose;
        spdlog::info("Camera {}: {}", i ,oss.str());
    }

    //optimize extrinsics
    spdlog::info("Start extrinsics optimization..");
    double error;
    if(!optimizeExtrinsicsLM(detection, estimation, &graph, &error, params, opt_params)){
        spdlog::error("Optimization failed.");
        return false;
    }

    spdlog::info("Final camera poses:");
    for(size_t i = 0; i < estimation->cams.size(); i++){
        std::ostringstream oss;
        oss << estimation->cams[i].pose;
        spdlog::info("Camera {}: {}", i ,oss.str());
    }
    spdlog::info("Optimization finished with an error of {}", error);
    return true;
};

void calibrate(Data *data_){
    calculateIntrinsics(data_);

    //start optimization
    if(!optimizeExtrinsics(data_->detection, &data_->estimation, data_->lm_params, data_->opt_params)){
        spdlog::error("Optimization of extrinsic parameters not successful.");
        return;
    }
    // TODO: save directly in the right format 
    // saveCameras();
}

}
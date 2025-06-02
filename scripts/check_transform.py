import pyflircam
import cv2
import os
from rich import print
import json
import numpy as np
import sys

# Stolen from anipose in trinagulation.py
# @jit(nopython=True, parallel=True)
def triangulate_simple_dlt(points, camera_mats):
    """triangulates from N-Cams with DLT
    Args:
        points (list of (x,y)): 
        camera_mats (list of projection matices): 
    Returns:
        np.array: 3d point
    """
    num_cams = len(camera_mats)
    A = np.zeros((num_cams * 2, 4))
    for i in range(num_cams):
        x, y = points[i]
        mat = camera_mats[i]
        A[(i * 2):(i * 2 + 1)] = x * mat[2] - mat[0]
        A[(i * 2 + 1):(i * 2 + 2)] = y * mat[2] - mat[1]
    u, s, vh = np.linalg.svd(A, full_matrices=True)
    p3d = vh[-1]
    p3d = p3d[:3] / p3d[3]
    return p3d

def get_images(cam_settings_file) -> dict:
    fcam = pyflircam.FlirMultiCameraHandler(cam_settings_file)
    fcam.start()
    imgs_ = fcam.get_images()
    imgs = [img.copy() for img in imgs_]
    fcam.stop()
    return {sn: img for sn, img in zip(os.environ['FLIR_CAMERA_SERIAL_NUMBERS'].split(","), imgs)}
    
def load_calibration(filepath):
    with open(filepath) as json_file:
        data = json.load(json_file)
    
    cameras = data['CAMERAS']
    cam_data = {}
    for cam in cameras:
        cam_data[cam["SerialNumber"]] = {
            "Distortion" : np.array(cam["Distortion"]),
            "Intrinsic" : np.reshape(cam["Intrinsic"], (3, 3)),
            "Extrinsic" : np.reshape(cam["Extrinsic"], (4, 4)),
        }
        cam_data[cam["SerialNumber"]]["Extrinsic"][:, -1] /= 1000
        cam_data[cam["SerialNumber"]]["Extrinsic"][-1][-1] = 1

    print(f"Loaded {filepath}")
    return cam_data

def detect_patterns(imgs: dict, settings: dict, script_dir: str):
    chdict = cv2.aruco.getPredefinedDictionary(settings["charuco_params_dict"])
    board = cv2.aruco.CharucoBoard(
        (settings["pattern_size_first"], settings["pattern_size_second"]),
        settings["charuco_params_square_size"], 
        settings["charuco_params_square_size"]*settings["charuco_params_marker_to_square_ratio"],
        chdict
    )
    detector = cv2.aruco.CharucoDetector(board)

    img_corners, img_ids = [], []
    for sn, img in imgs.items():
        corners, ids, marker_corners, marker_ids = detector.detectBoard(img)
        cv2.aruco.drawDetectedCornersCharuco(img, corners)
        cv2.aruco.drawDetectedMarkers(img, marker_corners, marker_ids)
        np.squeeze(marker_ids)
        img_corners.append(marker_corners)
        img_ids.append([int(id) for id in marker_ids])
        fp = f"{script_dir}/../test/{sn}_detected.png"
        cv2.imwrite(fp, img)
        print(f"Saved {fp}")

    ids_intersection = list(set(img_ids[0]).intersection(*map(set, img_ids[1:])))
    
    # corner for each view
    corner_list = []
    for rid in ids_intersection:
        corner_list_tmp = [[], [], [], []]
        for idx, sn in enumerate(imgs.items()):
            for id, mcorners in zip(img_ids[idx], img_corners[idx]):
                if id == rid:
                    for j in range(4):
                        corner_list_tmp[j].append((mcorners[0, j, 0], mcorners[0, j, 1]))

        for j in range(4):   
            corner_list.append(corner_list_tmp[j])
             
    return corner_list

def back_project_0(imgs, corner_list, cam_data, script_dir):
    marker_colors = [
        (0, 255, 0),
        (255, 0, 0),
        (0, 0, 255),
    ]
    markerType = cv2.MARKER_CROSS
    markerSize = 20
    thickness = 1
    top_cam_sn = os.environ['FLIR_TOP_CAM_SERIAL']
    top_cam_idx = list(imgs.keys()).index(top_cam_sn)
    intrinsic_top_cam = cam_data[top_cam_sn]["Intrinsic"]
    top_cam_img = imgs[top_cam_sn]
    
    camera_mats = [
        data["Intrinsic"] @ data["Extrinsic"][:3, :] for _, data in cam_data.items()
    ]
    
    intrinsic_top_inv = np.linalg.inv(intrinsic_top_cam)
    error = 0
    for detection in corner_list:
        point3d = triangulate_simple_dlt(detection, camera_mats)
        bp = intrinsic_top_cam @ point3d
        bp = bp[:2]/bp[2]
        cv2.drawMarker(
            top_cam_img, (int(bp[0]), int(bp[1])), 
            marker_colors[2], markerType, markerSize, thickness
        )
        error += abs(np.linalg.norm(bp-np.array(detection[top_cam_idx])))

    cv2.imwrite(f"{script_dir}/../test/back_projeced3d.jpg", top_cam_img)
    print(f"Mean back-projection error: {error/len(corner_list)} pixels")
     
def main():
    W = sys.argv[1]
    H = sys.argv[2]
    script_path = os.path.abspath(__file__)
    script_dir = os.path.dirname(script_path)
    settings_path = f"{script_dir}/../cfg/CameraCalibrationSettings.json"
    with open(settings_path) as json_file:
        settings = json.load(json_file)
    cam_settings_file = f"{script_dir}/../cfg/record_video_Settings{W}x{H}.json"
    imgs = get_images(cam_settings_file)
    print("Got images")
    corner_list = detect_patterns(imgs, settings, script_dir)
    print("Detected pattern")
    savedir = settings["savedir"]
    cam_data = load_calibration(
        f"{savedir}/Calibration{W}x{H}.json"
    )
    print("Got calibration")
    back_project_0(imgs, corner_list, cam_data, script_dir)
    
if __name__ == "__main__":
    main()
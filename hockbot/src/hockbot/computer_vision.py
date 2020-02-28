#!/usr/bin/env python
#
#   computer_vision.py
#
#   Run a Cascade Object Detector in OpenCV.
#!/usr/bin/env python
#
# computer_vision.py
import cv2
import cv2.aruco as aruco
import numpy as np


def process_image(gray_im):
    corners = locate_aruco_corners(gray_im)
    target_corners = np.float32(
        [[800, 480],
         [800, 0],
         [0, 0],
         [0, 480]])
    persp_mat = cv2.getPerspectiveTransform(corners, target_corners)
    gray_im = cv2.warpPerspective(gray_im, persp_mat, (800, 480))

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 10
    params.maxThreshold = 200

    # Filter by Color
    params.filterByColor = False
    params.filterByColor = 50

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 2000
    params.maxArea = 4000

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.80

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.80
        
    # Filter by Inertia
    params.filterByInertia = False
    params.minInertiaRatio = 0.001

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs.
    keypoints = detector.detect(gray_im)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
    # the size of the circle corresponds to the size of blob

    im_with_keypoints = cv2.drawKeypoints(gray_im, keypoints, np.array([]),
     (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Show blobs
    return im_with_keypoints


def locate_aruco_corners(gray_im):
    marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    corners, ids, _ = aruco.detectMarkers(
        gray_im,
        marker_dict)
    id_to_corner = {int(i): c for c, i in zip(corners, ids)}
    ordered_corners = np.array([id_to_corner[i] for i in [1, 2, 3, 4]])
    ordered_centers = np.average(ordered_corners, axis=2)
    return ordered_centers

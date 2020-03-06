#!/usr/bin/env python
#
#   computer_vision.py
#
#   Run a Cascade Object Detector in OpenCV.
#!/usr/bin/env python
#
# computer_vision.py
from threading import Lock
import cv2
import cv2.aruco as aruco
import numpy as np

MARKER_WIDTH = 1.697
MARKER_LENGTH = 1.255
EXTENDED_TABLE_LENGTH = 1.8
PIXELS_PER_METER = 100.0


class PuckDetector(object):
    def __init__(self):
        self._image_count = 0
        self._last_corners = None
        self._lock = Lock()

    def process_image(self, gray_im):
        corners = self.locate_aruco_corners(gray_im)
        if corners is None:
            return None, None
        marker_image_width = MARKER_WIDTH*PIXELS_PER_METER
        marker_image_length = MARKER_LENGTH*PIXELS_PER_METER
        extended_image_length = EXTENDED_TABLE_LENGTH*PIXELS_PER_METER
        marker_image_width_pix = int(marker_image_width)
        marker_image_length_pix = int(marker_image_length)
        extended_image_length_pix = int(extended_image_length)
        target_corners = np.float32(
            [[0, 0],
             [marker_image_width, 0],
             [marker_image_width, marker_image_length],
             [0, marker_image_length]])
        persp_mat = cv2.getPerspectiveTransform(corners, target_corners)
        gray_im = cv2.warpPerspective(gray_im, persp_mat, (marker_image_width_pix, extended_image_length_pix))

        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.thresholdStep = 5
        params.minThreshold = 10
        params.maxThreshold = 100

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 40 # 75
        params.maxArea = 140 # 250

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.8

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.9

        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.8

        # Create a detector with the parameters
        detector = cv2.SimpleBlobDetector_create(params)

        # Detect blobs.
        keypoints = detector.detect(gray_im)
        if not keypoints:
            return cv2.cvtColor(gray_im, cv2.COLOR_GRAY2BGR), None
        center_points = [np.array(k.pt) for k in keypoints]
        transform = np.array(
                [[1/PIXELS_PER_METER, 0],
                 [0, 1/PIXELS_PER_METER]])
        center_points_metric = np.dot(center_points, transform)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
        # the size of the circle corresponds to the size of blob

        im_with_keypoints = cv2.drawKeypoints(gray_im, keypoints, np.array([]),
         (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        print(center_points_metric)
        # Show blobs
        return im_with_keypoints, center_points_metric

    def locate_aruco_corners(self, gray_im):
        with self._lock:
            self._image_count += 1
            if self._image_count % 10 != 1:
                return self._last_corners
        marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        detector_params = aruco.DetectorParameters_create()
        # detector_params.minMarkerPerimeterRate = 0.01
        corners, ids, _ = aruco.detectMarkers(
            gray_im,
            marker_dict,
            parameters=detector_params)
        if ids is None or set(ids[:, 0].tolist()) != set([1, 2, 3, 4]):
            return self._last_corners
        id_to_corner = {int(i): c for c, i in zip(corners, ids)}
        ordered_corners = np.array([id_to_corner[i] for i in [1, 2, 3, 4]])
        ordered_centers = np.average(ordered_corners, axis=2)
        self._last_corners = ordered_centers
        return self._last_corners

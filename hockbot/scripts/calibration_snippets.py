#
#   calibration_snippets.py
#
#   These are useful openCV code snippets for the camera calibration
#   and processing.  Please cut and paste and place into the
#   appropriate context.
#

# Imports.  These are generally necessary.
import rospy
import cv2

import numpy as np

from sensor_msgs.msg import Image


#
#   Intrinstic Parameters
#
#   Get the intrinsic camara parameters!  Note most USB cameras set up
#   a camera_info topic.  The RealSense may not do this, in which case
#   hard-coding the parameters may be the easiest solution.
#
from sensor_msgs.msg import CameraInfo

def get_intrinsic_parameters:
    def from_calibration_file/topic:
        # Grab a camera_info message, change topic as needed.
        msg = rospy.wait_for_message('/camera/camera_info', CameraInfo)

        # Check/grab the camera matrix.
        if (msg.K[0] == 0 or msg.K[1] != 0 or msg.K[2] == 0 or
            msg.K[3] != 0 or msg.K[4] == 0 or msg.K[5] == 0 or
            msg.K[6] != 0 or msg.K[7] != 0 or msg.K[8] != 1):
            rospy.logerr("Camera Intrinsic Parameters strangely formatted!")
            rospy.signal_shutdown("Camera incorrectly calibrated")
            return
        K = np.float64(msg.K).reshape(3,3)

        # Check/grab the distortion model.
        D = np.float64(msg.D)

    def hardcoded:
        fx = 650
        fy = 650
        u0 = 320
        v0 = 240
        K = np.float64([fx, 0, u0, 0, fy, v0, 0, 0, 1]).reshape(3,3)
        D = np.float64([0, 0, 0, 0, 0])


#
#   Get Checkboard pixels (and declare points).
#
#   Note you will probably want to set the x/y/z coordinates
#   appropriate to your approach.
#
from camera_calibration.calibrator import ChessboardInfo
from camera_calibration.calibrator import Calibrator

def get_checkerboard_data:
    def __init__(self):
        # Define the Checkerboard.  Note the OpenCV detector
        # apparently assumes more columns than rows.
        board = ChessboardInfo()
        board.n_cols = 8
        board.n_rows = 6
        board.dim = 0.0254

        # Instantiate a Calibrator, to extract corners etc.
        self.calibrator = Calibrator([board])

    def callback(self, image):
        # Test for the presense of a checkerboard and pull out the
        # corners as a list of (u,v) data.
        gray = self.calibrator.mkgray(image)
        (ok, corners, board) = self.calibrator.get_corners(gray)
        if not ok:
            print("No matching checkboard...")
            return
        corners = corners.reshape(-1,2)

        # Set the (X,Y,Z) data for each corner.  This presumes the
        # checkerboard is on the Z=0 plane and centered in X/Y!
        xyz = np.zeros((len(corners), 3))
        for r in xrange(board.n_rows):
            for c in xrange(board.n_cols):
                i = r*board.n_cols + c
                xyz[i][0] = board.dim * (c - (board.n_cols-1)/2.0)
                xyz[i][1] = board.dim * ((board.n_rows-1)/2.0 - r)
                xyz[i][2] = 0

        # Really these are lists of (u,v) and (x,y,z)
        uvlist  = corners
        xyzlist = xyz


#
#   Affine Mapping
#
def affine:
    def calibrate:
        # Set (or get from some input) the matching pixel and
        # coordinates.
        uvlist = np.float32([[160, 360], [320, 120], [480, 360]])
        xylist = np.float32([[-0.2, -0.2], [0.0, 0.2], [0.2, -0.2]])

        # Compute the mapping.
        M = cv2.getAffineTransform(uvlist, xylist)
        print(M)

    def apply:
        # Pick some pixel values (or grab from somewhere):
        uv = np.float32([320, 240])

        # Transform.  Note the code assumes a set of lists of
        # coordinate.  Reshape accordingly.
        xy = cv2.transform(uv.reshape(1,-1,2), M).reshape(2)
        print(xy)


#
#   Perspective Mapping
#
#   Note findHomography() takes more time, but does a better fit to 4
#   or more data points.  getPerspectiveTransform() takes only 4 pts.
#
def perspective:
    def calibrate_from_4_not_so_good:
        # Set (or get from some input) the matching pixel and
        # coordinates.
        uvlist = np.float32([[160, 360], [160, 120], [480, 120], [480, 360]])
        xylist = np.float32(0.1 * [[-2, -2], [-3, 3], [3, 3], [2, -2]])

        # Compute the mapping.
        M = cv2.getPerspectiveTransform(uvlist, xylist)
        print(M)

    def calibrate:
        # Set (or get from some input) the matching pixel and
        # coordinates.  4 points work.
        uvlist = np.float32([[160, 360], [160, 120], [480, 120], [480, 360]])
        xylist = np.float32(0.1 * [[-2, -2], [-3, 3], [3, 3], [2, -2]])

        # More points are better...
        uvlist = np.float32([[160, 360], [160, 120], [480, 120], [480, 360], [320, 240]])
        xylist = 0.1 * np.float32([[-2, -2], [-3, 3], [3, 3], [2, -2], [0,0]])

        # Compute the mapping.
        (M, _) = cv2.findHomography(uvlist, xylist)
        print(M)

    def apply:
        # Pick some pixel values (or grab from somewhere):
        uv = np.float32([320, 240])

        # Transform.  Note the code assumes a set of lists of
        # coordinate.  Reshape accordingly.
        xy = cv2.perspectiveTransform(uv.reshape(1,-1,2), M).reshape(2)
        print(xy)


#
#   Determine the camera position/orientation
#
#   Note in the vision world, folks are interested in where objects
#   are relative to the camera.  So we will need to invert the
#   orientation/position to get the camera w.r.t. world (which is our
#   object).
#
def locate_camera:
    # Use the camera matrix K and distortion D already loaded.  See above.
    K = np.float64(...)
    D = np.float64(...)

    # Compute the world frame w.r.t. camera.
    ok, rvec, tvec = cv2.solvePnP(xyz, corners, K, D)
    if not ok:
        print("Problem locating the camera!")
        return
    (R_world_wrt_cam, _) = cv2.Rodrigues(rvec)
    x_world_wrt_cam      = tvec

    # Convert into the camera frame w.r.t. world.
    R_cam_wrt_world = R_world_wrt_cam.transpose()
    x_cam_wrt_world = - np.matmul(R_cam_wrt_world, x_world_wrt_cam)

    # Report.
    # print(R_cam_wrt_world)
    # print(x_cam_wrt_world)
    print("Cam loc: %6.3f, %6.3f, %6.3f" % tuple(x_cam_wrt_world.reshape(3)))



#
#   Undistort
#
#   Compute the normalized (image) coordinates from the pixels
#
def undistort:
    # Use the camera matrix K and distortion D already loaded.  See above.
    K = np.float64(...)
    D = np.float64(...)

    # Pick a (u,v) pair.  I used the top-left corner for
    # testing, which is (-3.5, 2.5) * 0.0254
    uv = corners[0]

    # Map to the normalized (image) coordinates.  As above, the API
    # assume a set of lists of points, so reshape accordingly.
    xybar = cv2.undistortPoints(uv.reshape(1,-1,2), K, D).reshape(2)
    print(xybar)

    # Now map into the world.  Here I am assuming zw = 0...
    Rc = R_cam_wrt_world
    xc = x_cam_wrt_world.reshape(3)

    lam = -xc[2] / (Rc[2][0]*xybar[0] + Rc[2][1]*xybar[1] + Rc[2][2])
    xw = lam*(Rc[0][0]*xybar[0] + Rc[0][1]*xybar[1] + Rc[0][2]) + xc[0]
    yw = lam*(Rc[1][0]*xybar[0] + Rc[1][1]*xybar[1] + Rc[1][2]) + xc[1]

    # Check the location in number of squares...
    nw = xw / 0.0254
    nw = yw / 0.0254
    print([nw, nw])

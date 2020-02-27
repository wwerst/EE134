import cv2
import numpy as np

def process_image(gray_im):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 10
    params.maxThreshold = 200

    # Filter by Color
    params.filterByColor = True
    params.filterByColor = 50

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 1400
    params.maxArea = 1500

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.01

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.87
        
    # Filter by Inertia
    params.filterByInertia = True
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
    cv2.imshow("Keypoints", im_with_keypoints)


def main():
    cap = cv2.VideoCapture(0)
    while cv2.waitKey(1) & 0xFF != ord('q'):
        ret, frame = cap.read()
        gray_im = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        process_image(gray_im)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

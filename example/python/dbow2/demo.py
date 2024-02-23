import cv2
import numpy as np


def group_keypoints_by_level(keypoints, nlevels):
    """
    Groups keypoints by their octave levels.

    :param keypoints: List of keypoints detected by a feature detector.
    :param nlevels: Number of levels in the scale space.
    :return: A dictionary where the keys are the octave levels and the values are lists of keypoints
        corresponding to each level.
    """
    grouped_keypoints = {level: [] for level in range(nlevels)}
    for kp in keypoints:
        octave = kp.octave & 0xFF  # Extracting the octave number
        if octave < nlevels:
            grouped_keypoints[octave].append(kp)
    return grouped_keypoints


orb = cv2.ORB_create()
print(orb.getNLevels())
print(orb.getDefaultName())
print(orb.getEdgeThreshold())
print(orb.getFastThreshold())
print(orb.getMaxFeatures())
print(orb.getScaleFactor())

for i in range(1):
    img = cv2.imread('images/image{}.png'.format(i))
    if img is not None:
        # In OpenCV, when you detect keypoints using feature detectors like SIFT, ORB,
        # or others that employ an image pyramid, the coordinates (x, y) of
        # the detected key points are typically given in the coordinate system of
        # the original image scale, not the scaled images in the image pyramid.
        keypoints, descriptors = orb.detectAndCompute(img, None)
        assert len(keypoints) == len(descriptors)
        print(descriptors.shape if descriptors is not None else "No descriptors")
        grouped_keypoints = group_keypoints_by_level(keypoints, orb.getNLevels())
        cv2.imshow('Level={}, Image {}'.format(0, i),
                   cv2.drawKeypoints(img, grouped_keypoints[0], None, color=(0, 0, 255)))
        cv2.imshow('Level={}, Image {}'.format(1, i),
                   cv2.drawKeypoints(img, grouped_keypoints[1], None, color=(0, 255, 255)))
        cv2.imshow('Level={}, Image {}'.format(7, i),
                   cv2.drawKeypoints(img, grouped_keypoints[7], None, color=(255, 255, 0)))
        cv2.waitKey(0)
    else:
        print(f"Failed to load images/image{i}.png")

cv2.destroyAllWindows()

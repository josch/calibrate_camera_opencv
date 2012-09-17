#!/usr/bin/python
import cv
import sys

if __name__ == "__main__":
    cv.NamedWindow("win")
    filename = sys.argv[1]
    im = cv.LoadImage(filename, cv.CV_LOAD_IMAGE_GRAYSCALE)
    im3 = cv.LoadImage(filename, cv.CV_LOAD_IMAGE_COLOR)
    board_width, board_height = (4, 6)
    corner_count = board_width * board_height

    found_all, corners = cv.FindChessboardCorners( im, (board_width, board_height))
    print found_all, corners

    if found_all and len(corners) == corner_count:
        cv.DrawChessboardCorners( im3, (board_width, board_height), corners, found_all )

        camera_intrinsic_mat = cv.CreateMat(3, 3, cv.CV_32FC1)
        distortion_mat = cv.CreateMat(5, 1, cv.CV_32FC1)
        rotation_vecs = cv.CreateMat(1, 3, cv.CV_32FC1)
        translation_vecs = cv.CreateMat(1, 3, cv.CV_32FC1)
        object_points = cv.CreateMat(corner_count, 3, cv.CV_32FC1)
        image_points = cv.CreateMat(corner_count, 2, cv.CV_32FC1)
        point_counts = cv.CreateMat(1, 1, cv.CV_32SC1)
        for jj in range(corner_count):
            cv.Set2D(object_points, jj, 0, float(jj/board_width))
            cv.Set2D(object_points, jj, 1, float(jj%board_width))
            cv.Set2D(object_points, jj, 2, float(0.0))
            cv.Set2D(image_points, jj, 0, corners[jj][0])
            cv.Set2D(image_points, jj, 1, corners[jj][1])
        cv.Set1D(point_counts, 0, corner_count)
        cv.CalibrateCamera2(object_points, image_points, point_counts, cv.GetSize(im3), camera_intrinsic_mat, distortion_mat, rotation_vecs, translation_vecs)

        undistorted = cv.CloneImage(im3);

        cv.Undistort2(im3, undistorted, camera_intrinsic_mat, distortion_mat)

        cv.ShowImage("win", undistorted);
        cv.WaitKey()
    else:
        print "not found"

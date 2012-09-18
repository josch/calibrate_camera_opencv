#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

#include <fstream>
#include <iostream>

using namespace std;
using namespace cv;

enum {INTERACTIVE_MODE, PRESPECIFIED_MODE};

#define KEY_ESCAPE 1048603
#define KEY_SPACE 1048608
#define KEY_CLOSE_WINDOW -1

#define COUNT_SQUARES_X 4
#define COUNT_SQUARES_Y 6

int main(int argc, char** argv)
{
	int user_mode;
	int specified_boards;
	if (argc > 2) 
		cerr << "Usage: ./calibrate [number of frames]\n";
	else if (argc == 1) 
	{
		user_mode = INTERACTIVE_MODE;
		cout << "Camera calibration using interactive behavior. Press SPACE to grab frame, ESC to quit.\n";
	} 
	else 
	{
		user_mode = PRESPECIFIED_MODE;
		stringstream ss; ss << argv[1]; 
		ss >> specified_boards;
		cout << "Camera calibration using " << specified_boards << " frames.\n";
	}

	VideoCapture capture(0); // open the default camera
	if (!capture.isOpened())  // check if opening camera stream succeeded
	{
		cerr << "Camera could not be found. Exiting.\n";
		return -1;
	}
	// set frame width and height by hand, defaults to 160x120
	capture.set (CV_CAP_PROP_FRAME_WIDTH, 640);
	capture.set (CV_CAP_PROP_FRAME_HEIGHT, 480);

	// show camera image in a separate window
	namedWindow("Camera Image", CV_WINDOW_KEEPRATIO);
	// store object points (world coords) and image points (image coords) for use in calibrateCamera 
	vector< vector <Point3f> > object_points;
  vector< vector <Point2f> > image_points;

	/// current frame, its grayscale counterpart and storage for the first frame
	Mat frame, gray_frame, first_frame;
	// flag for determining whether pattern was detected in at least one of the camera grabs
	bool acquired_samples = false;

	// loop indefinitely and keep frame counter
	for(int count_frames = 0; ; ++count_frames)
	{
	  capture >> frame; // get a new frame from camera
		cvtColor(frame, gray_frame, CV_BGR2GRAY); // convert current frame to grayscale
	  imshow("Camera Image", frame); // update camera image

		int key_pressed = waitKey(0); // get user key press
		if (key_pressed == KEY_CLOSE_WINDOW || key_pressed == KEY_ESCAPE) break;

		if (user_mode == INTERACTIVE_MODE && key_pressed == KEY_SPACE)
		{
			Size pattern_size(COUNT_SQUARES_X, COUNT_SQUARES_Y); // number of squares in the pattern, a.k.a, interior number of corners
			vector <Point2f> corners; // storage for the detected corners in findChessboardConrners
			bool pattern_found = findChessboardCorners( gray_frame, pattern_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		
			if (pattern_found) 
			{
				if (count_frames == 0) first_frame = frame;
				// if corners are detected, they are further refined by calculating subpixel corners from the grayscale image
				// this iterative process terminates after the given number of iterations and error epsilon
				cornerSubPix(gray_frame, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.1));
				// draw the detected corners for debugging purposes
				drawChessboardCorners(frame, pattern_size, Mat(corners), pattern_found);
				// show detected corners in a different window
				imshow("Detected pattern", frame);

				// build a grid of 3D points (z component is 0 because the pattern is in one plane) to fit the square pattern area (COUNT_SQUARES_X * COUNT_SQUARES_Y)
				vector< Point3f > pattern_points;
			  for(int j = 0; j < COUNT_SQUARES_X * COUNT_SQUARES_Y; ++j)
		      pattern_points.push_back(Point3f(j/COUNT_SQUARES_X, j%COUNT_SQUARES_X, 0.0f));

				// populate image points with corners and object points with grid points
				image_points.push_back(corners);
        object_points.push_back(pattern_points);
				cout << "Frame " << count_frames << " grabbed.\n";
				acquired_samples = true;
			}
		}

	}

	// if at least one video capture contains the pattern, perform calibration
	if (acquired_samples) 
	{
		Mat intrinsic = Mat(3, 3, CV_32FC1);
		Mat distCoeffs;
		vector<Mat> rvecs;
		vector<Mat> tvecs;

		// perform calibration, obtain instrinsic parameters and distortion coefficients
		cout << "\nCalibrating...\n";
		calibrateCamera(object_points, image_points, frame.size(), intrinsic, distCoeffs, rvecs, tvecs);

/*
		Mat grab2 = imread("./data/grab2.jpg"), grab2_undistorted;
		Mat grab4 = imread("./data/grab4.jpg"), grab4_undistorted;
		undistort(grab2, grab2_undistorted, intrinsic, distCoeffs);
		undistort(grab4, grab4_undistorted, intrinsic, distCoeffs);

		imwrite("./data/grab2_undistorted.jpg", grab2_undistorted);
		imwrite("./data/grab4_undistorted.jpg", grab4_undistorted);
*/
		
		Mat undistorted_frame;
		// apply the calibration transformation to the first frame and store image on disk
		undistort(first_frame, undistorted_frame, intrinsic, distCoeffs);
		imwrite("capture.jpg", first_frame);
		imwrite("undistorted_frame.jpg", undistorted_frame);
		cout << "Saved first frame to undistorted_frame.jpg\n";
		cout << "Intrinsic parameters: \n" << intrinsic << "\n";
		cout << "Distortion coefficients: \n" << distCoeffs << "\n";
	}
	else 
	{
		cerr << "No frames grabbed!\n";
	}

	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

#include <fstream>
#include <iostream>

using namespace std;
using namespace cv;

enum { INTERACTIVE_MODE, PRESPECIFIED_MODE };

#define KEY_ESCAPE 1048603
#define KEY_SPACE 1048608
#define KEY_CLOSE_WINDOW -1

#define COUNT_SQUARES_X 4
#define COUNT_SQUARES_Y 6

/**
 * Method for detecting pattern in current frame. 
 * Returns world and image coordinates of detected corners via out parameters.
 */
bool detectPattern(Mat frame, vector< vector<Point3f> >& object_points, vector<vector<Point2f> >& image_points)
{
  // number of squares in the pattern, a.k.a, interior number of corners
  Size pattern_size(COUNT_SQUARES_X, COUNT_SQUARES_Y);
  // storage for the detected corners in findChessboardConrners
  vector<Point2f> corners;

  bool pattern_found = findChessboardCorners(
            frame, pattern_size, corners,
            CALIB_CB_ADAPTIVE_THRESH +
            CALIB_CB_NORMALIZE_IMAGE +
            CALIB_CB_FAST_CHECK);

  if (!pattern_found)
    return false;

  // if corners are detected, they are further refined by calculating subpixel corners from the grayscale image
  // this iterative process terminates after the given number of iterations and error epsilon
  cornerSubPix(frame, corners, Size(11, 11), Size(-1, -1),
               TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100,
               0.15));
  // draw the detected corners as sanity check
  drawChessboardCorners(frame, pattern_size, Mat(corners),
                        pattern_found);

  // show detected corners in a different window
  imshow("Detected pattern", frame);

  // build a grid of 3D points (z component is 0 because the pattern is in one plane) to fit the square pattern area (COUNT_SQUARES_X * COUNT_SQUARES_Y)
  vector<Point3f> pattern_points;
  for (int j = 0; j < COUNT_SQUARES_X * COUNT_SQUARES_Y; ++j)
    pattern_points.push_back(Point3f(j / COUNT_SQUARES_X, j % COUNT_SQUARES_X, 0.0f));

  // populate image points with corners and object points with grid points
  object_points.push_back(pattern_points);
  image_points.push_back(corners);

  return true;
}

/**
 * Main method for interactive behavior. 
 * Requires user to present calibration pattern in front of camera. 
 * By pressing any key an image is grabbed from the camera stream, pressing ESC finishes calibration.
 */
int runInteractive()
{
  cout << "Camera calibration using interactive behavior. Press any key to grab frame, ESC to perform calibration.\n";
  // show camera image in a separate window
  namedWindow("Camera Image", CV_WINDOW_KEEPRATIO);

  // current camera frame and first captured frame 
  Mat frame, first_frame;
  // storage for object points (world coords) and image points (image coords) for use in calibrateCamera 
  vector< vector< Point3f > > object_points;
  vector< vector< Point2f > > image_points;
	// calibration parameters
  Mat intrinsic = Mat(3, 3, CV_32FC1);
  Mat distCoeffs;
  vector<Mat> rvecs, tvecs;

  // open the default camera
  VideoCapture capture(0);

  // check if opening camera stream succeeded
  if (!capture.isOpened()) 
  {
    cerr << "Camera could not be found. Exiting.\n";
    return -1;
  }

  // set frame width and height by hand, defaults to 160x120
  capture.set (CV_CAP_PROP_FRAME_WIDTH, 640);
  capture.set (CV_CAP_PROP_FRAME_HEIGHT, 480);

  // flag for determining whether pattern was detected in at least one of the camera grabs
  bool calibration_ready = false;
  // flag for detecting first frame with pattern
	bool first_flag = true;

  for (int count_frames = 0; ; ++count_frames) 
  {
    cout << "Frame: " << count_frames << endl;

    capture >> frame; // get a new frame from camera
    Mat gray_frame;
    cvtColor(frame, gray_frame, CV_BGR2GRAY); // convert current frame to grayscale
    imshow("Camera Image", frame); // update camera image

    int key_pressed = waitKey(0); // get user key press
    if (key_pressed == KEY_CLOSE_WINDOW || key_pressed == KEY_ESCAPE)
      break;
    else 
    {
      if (detectPattern(gray_frame, object_points, image_points)) 
      {
        if (first_flag)
        {
          first_frame = frame; // save first frame for later use
          first_flag = false;
        }

        cout << "Frame " << count_frames << " grabbed." << endl;
				calibration_ready = true;
      } 
      else cout << "Pattern not found" << endl;
    } 

  }
  
  // if at least one video capture contains the pattern, perform calibration
  if (calibration_ready) 
  {
    // perform calibration, obtain instrinsic parameters and distortion coefficients
    cout << "Calibrating..." << endl;
    calibrateCamera(object_points, image_points, frame.size(), intrinsic,
                    distCoeffs, rvecs, tvecs);

    Mat undistorted_frame;
    // apply the calibration transformation to the first frame and store image on disk
    undistort(first_frame, undistorted_frame, intrinsic, distCoeffs);
    imwrite("first_frame.jpg", first_frame);
    imwrite("undistorted_frame.jpg", undistorted_frame);
    cout << "Applied undistortion to first frame (first_frame.jpg) and saved to undistorted_frame.jpg" << endl;
    cout << "Intrinsic parameters:" << endl << intrinsic << endl;
    cout << "Distortion coefficients:" << endl << distCoeffs << endl;
  }
  else 
  {
    cerr << "No pattern found in any video capture. Exiting." << endl;
  }

	// release camera
  capture.release();
}

int runPrespecified(int argc, char **argv)
{
  // current camera frame and first captured frame 
  Mat frame, first_frame;
  // storage for object points (world coords) and image points (image coords) for use in calibrateCamera 
  vector<vector<Point3f> > object_points;
  vector<vector<Point2f> > image_points;
	// calibration parameters
  Mat intrinsic = Mat(3, 3, CV_32FC1);
  Mat distCoeffs;
  vector<Mat> rvecs, tvecs;

  // flag for determining whether pattern was detected in at least one of the images
  bool calibration_ready = false;
  // flag for detecting first frame with pattern
  bool first_flag = true;

	// loop through the specified images
  for (int i = 0; i < argc; i++) 
  {
    Mat gray_frame;
		cerr << endl << argv[i] << endl;
    frame = imread(argv[i], CV_LOAD_IMAGE_COLOR);
    cvtColor(frame, gray_frame, CV_BGR2GRAY);
    imshow("Camera Image", frame);
		// apply pattern detection
    if (detectPattern(gray_frame, object_points, image_points))
    { 
      if (first_flag) 
      {
        first_frame = frame;
        first_flag = false;
      }
      cout << "Frame " << i << " grabbed." << endl;
      calibration_ready = true;
    }
    else 
      cout << "Pattern not found in frame " << i << endl;
  }

  // if at least one image contains the pattern, perform calibration
	if (calibration_ready) 
  {
    // perform calibration, obtain instrinsic parameters and distortion coefficients
    cout << "Calibrating..." << endl;
    calibrateCamera(object_points, image_points, frame.size(), intrinsic,
                    distCoeffs, rvecs, tvecs);

    Mat undistorted_frame;
    // apply the calibration transformation to the first frame and store image on disk
    undistort(first_frame, undistorted_frame, intrinsic, distCoeffs);
    imwrite("first_frame.jpg", first_frame);
    imwrite("undistorted_frame.jpg", undistorted_frame);
    cout << "Applied undistortion to first frame (first_frame.jpg) and saved to undistorted_frame.jpg" << endl;
    cout << "Intrinsic parameters:" << endl << intrinsic << endl;
    cout << "Distortion coefficients:" << endl << distCoeffs << endl;
  }
  else 
  {
    cerr << "No pattern found in any of the images. Exiting." << endl;
  }

}

int main(int argc, char **argv)
{
  int user_mode;
  int specified_boards;

  /* no arguments means interactive mode
   * one or more arguments are image filenames */
  if (argc == 1) {
      runInteractive();
  } else {
      runPrespecified(argc-1, ++argv);
  }

  return 0;
}

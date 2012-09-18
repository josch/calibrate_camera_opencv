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

int detect_pattern(Mat frame, vector< vector<Point3f> >& object_points, vector<vector<Point2f> >& image_points)
{
  // interior number of corners
  Size pattern_size(COUNT_SQUARES_X, COUNT_SQUARES_Y);
  // will be filled by the detected corners
  vector<Point2f> corners;

  bool pattern_found = findChessboardCorners(
            frame, pattern_size, corners,
            CALIB_CB_ADAPTIVE_THRESH +
            CALIB_CB_NORMALIZE_IMAGE +
            CALIB_CB_FAST_CHECK);

  if (!pattern_found)
    return -1;

  // to tweak params: http://bit.ly/QyoU3k
  cornerSubPix(frame, corners, Size(11, 11), Size(-1, -1),
               TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100,
               0.1));
  drawChessboardCorners(frame, pattern_size, Mat(corners),
                        pattern_found);
  vector<Point3f> obj;
  for (int j = 0; j < COUNT_SQUARES_X * COUNT_SQUARES_Y; ++j)
    obj.push_back(Point3f(j / COUNT_SQUARES_X, j % COUNT_SQUARES_X, 0.0f));

  object_points.push_back(obj);
  image_points.push_back(corners);
  return 0;
}

int run_interactive()
{
  namedWindow("Camera Image", CV_WINDOW_KEEPRATIO);

  int count = 0;
  Mat frame;
  vector<vector<Point3f> > object_points;
  vector<vector<Point2f> > image_points;
  Mat intrinsic = Mat(3, 3, CV_32FC1);
  Mat distCoeffs;
  vector<Mat> rvecs, tvecs;

  // open the default camera
  VideoCapture capture(-1);

  // check if opening camera stream succeeded
  if (!capture.isOpened()) {
    cerr << "Camera could not be found. Exiting.\n";
    return -1;
  }

  for (;;) {
    cout << "Frame: " << count << endl;
    Mat gray_frame;
    capture >> frame;        // get a new frame from camera
    cvtColor(frame, gray_frame, CV_BGR2GRAY);
    imshow("Camera Image", frame);

    int key_pressed = waitKey(0);
    if (key_pressed == KEY_CLOSE_WINDOW || key_pressed == KEY_ESCAPE)
      break;

    if (key_pressed == KEY_SPACE) {
      if (detect_pattern(gray_frame, object_points, image_points) == 0) {
        cout << "Frame " << count << " grabbed." << endl;
      } else {
        cout << "pattern not found" << endl;
      }
    }
  }

  calibrateCamera(object_points, image_points, frame.size(), intrinsic,
                  distCoeffs, rvecs, tvecs);

  for (;;) {
    capture >> frame;
    Mat imageUndistorted;
    undistort(frame, imageUndistorted, intrinsic, distCoeffs);
    imshow("win1", frame);
    imshow("win2", imageUndistorted);
    waitKey(1);
  }

  capture.release();
}

int run_prespecified(int argc, char **argv)
{
  int count = 0;
  Mat frame;
  vector<vector<Point3f> > object_points;
  vector<vector<Point2f> > image_points;
  Mat intrinsic = Mat(3, 3, CV_32FC1);
  Mat distCoeffs;
  vector<Mat> rvecs, tvecs;

  for (int i = 0; i < argc; i++) {
    Mat gray_frame;
		cerr << endl << argv[i] << endl;
    frame = imread(argv[i], CV_LOAD_IMAGE_COLOR);
    cvtColor(frame, gray_frame, CV_BGR2GRAY);
    imshow("Camera Image", frame);
    if (detect_pattern(gray_frame, object_points, image_points) == 0) {
      cout << "Frame " << i << " grabbed." << endl;
    } else {
      cout << "Pattern not found in frame " << i << endl;
    }
  }

	cout << "Calibrating..." << endl;
  calibrateCamera(object_points, image_points, frame.size(), intrinsic,
                  distCoeffs, rvecs, tvecs);

  for (int i = 0; i < argc; i++) {
    frame = imread(argv[i], CV_LOAD_IMAGE_COLOR);
    Mat imageUndistorted;
    undistort(frame, imageUndistorted, intrinsic, distCoeffs);
    imshow("win1", frame);
    imshow("win2", imageUndistorted);
    waitKey(0);
  }

  return 0;
}

int main(int argc, char **argv)
{
  int user_mode;
  int specified_boards;

  /* no arguments means interactive mode
   * one or more arguments are image filenames */
  if (argc == 1) {
      run_interactive();
  } else {
      run_prespecified(argc-1, ++argv);
  }

  return 0;
}

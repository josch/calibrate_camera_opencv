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

int main(int argc, char **argv)
{
  int user_mode;
  int specified_boards;
  if (argc > 2)
    cerr << "Usage: ./calibrate [number of frames]\n";
  else if (argc == 1) {
    user_mode = INTERACTIVE_MODE;
    cout <<
	"Camera calibration using interactive behavior. Press SPACE to grab frame.\n";
  } else {
    user_mode = PRESPECIFIED_MODE;
    stringstream ss;
    ss << argv[1];
    ss >> specified_boards;
    cout << "Camera calibration using " << specified_boards <<
	" frames.\n";
  }

  VideoCapture capture(-1);	// open the default camera
  if (!capture.isOpened())	// check if opening camera stream succeeded
  {
    cerr << "Camera could not be found. Exiting.\n";
    return -1;
  }

  namedWindow("Camera Image", CV_WINDOW_KEEPRATIO);
  vector < vector < Point3f > >object_points;
  vector < vector < Point2f > >image_points;

  Mat frame, gray_frame;
  for (int count = 0;; ++count) {
    cout << "Frame: " << count << "\n";
    capture >> frame;		// get a new frame from camera
    cvtColor(frame, gray_frame, CV_BGR2GRAY);
    imshow("Camera Image", frame);

    int key_pressed = waitKey(0);
    if (key_pressed == KEY_CLOSE_WINDOW || key_pressed == KEY_ESCAPE)
      break;

    if ((user_mode == INTERACTIVE_MODE && key_pressed == KEY_SPACE) ||
	user_mode == PRESPECIFIED_MODE && count < specified_boards) {
      Size pattern_size(COUNT_SQUARES_X, COUNT_SQUARES_Y);	//interior number of corners
      vector < Point2f > corners;	//this will be filled by the detected corners
      bool pattern_found =
	  findChessboardCorners(gray_frame, pattern_size, corners,
				CALIB_CB_ADAPTIVE_THRESH +
				CALIB_CB_NORMALIZE_IMAGE +
				CALIB_CB_FAST_CHECK);

      if (pattern_found)	// to tweak params: http://bit.ly/QyoU3k
      {
	cornerSubPix(gray_frame, corners, Size(11, 11), Size(-1, -1),
		     TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30,
				  0.1));
	drawChessboardCorners(frame, pattern_size, Mat(corners),
			      pattern_found);

	vector < Point3f > obj;
	for (int j = 0; j < COUNT_SQUARES_X * COUNT_SQUARES_Y; ++j)
	  obj.push_back(Point3f
			(j / COUNT_SQUARES_X, j % COUNT_SQUARES_X, 0.0f));

	image_points.push_back(corners);
	object_points.push_back(obj);
	cout << "Frame " << count << " grabbed.\n";
      }
    }

  }


  Mat intrinsic = Mat(3, 3, CV_32FC1);
  Mat distCoeffs;
  vector < Mat > rvecs;
  vector < Mat > tvecs;

  calibrateCamera(object_points, image_points, frame.size(), intrinsic,
		  distCoeffs, rvecs, tvecs);

  Mat imageUndistorted;
  while (1) {
    capture >> frame;
    undistort(frame, imageUndistorted, intrinsic, distCoeffs);

    imshow("win1", frame);
    imshow("win2", imageUndistorted);

    waitKey(1);
  }

  capture.release();
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}

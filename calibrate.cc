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

int main(int argc, char** argv)
{
	int user_mode;
	if (argc > 2) 
		cerr << "Usage: ./calibrate [number of frames]\n";
	else 
	if (argc == 1) 
	{
		user_mode = INTERACTIVE_MODE;
		cout << "Camera calibration using interactive behavior. Press SPACE to grab frame.\n";
	} 
	else 
	{
		user_mode = PRESPECIFIED_MODE;
		cout << "Camera calibration using " << argv[1] << " frames.\n";
	}

	VideoCapture cap(-1); // open the default camera
	if(!cap.isOpened())  // check if opening camera stream succeeded
	{
		cerr << "Camera could not be found. Exiting.\n";
		return -1;
	}

	namedWindow("Camera Image", CV_WINDOW_KEEPRATIO);
	for(int count = 0; ; ++count)
	{
		cout << "Frame: " << count << "\n";		
	  Mat frame;
	  cap >> frame; // get a new frame from camera
	  imshow("Camera Image", frame);
		int key_pressed = waitKey(0);
		if (key_pressed == KEY_CLOSE_WINDOW || key_pressed == KEY_ESCAPE) break;
		else 
		if (user_mode == INTERACTIVE_MODE) 
		{		
			if (key_pressed == KEY_SPACE) 
			{
				Size pattern_size(4, 6); //interior number of corners
				vector<Point2f> corners; //this will be filled by the detected corners
				bool pattern_found = findChessboardCorners( frame, pattern_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
			
				if(pattern_found) // to tweak params: http://bit.ly/QyoU3k
					cornerSubPix(frame, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

				drawChessboardCorners(frame, pattern_size, Mat(corners), pattern_found);
				imshow("Calibrated", frame);
				waitKey(30);
			}		
		}
	}

	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}

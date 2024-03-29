#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/ximgproc.hpp>
#include <stdio.h>

using namespace std;
using namespace cv;
using namespace cv::ximgproc;

int main()
{
	//Load in stereo images
	Mat left_img=imread("left.png");
	Mat left = left_img;					//Mat left will be resized to help the image displays to fit the screen
	Mat right_img = imread("right.png");
	Mat right = left_img;					//Mat right will be resized to help the image displays to fit the screen
	Mat combined,left_gray,right_gray;

	//Convert images to gray scale
	cvtColor(left_img, left_gray, COLOR_BGR2GRAY);
	cvtColor(right_img, right_gray, COLOR_BGR2GRAY);

	//Resize original images and place side to side to make it easier to view
	resize(left, left, Size(), 0.5, 0.5);
	resize(right, right, Size(), 0.5, 0.5);
	vector<Mat> matrices = { left,right };
	hconcat(matrices, combined);
	imshow("Original Left and Right Images", combined);

	Mat left_disp, right_disp,filtered_disp,solved_filtered_disp;
	
	//Initialize the stereo matcher
	Ptr<StereoBM> left_matcher=StereoBM::create(80,13);
	Ptr<DisparityWLSFilter> wls_filter = createDisparityWLSFilter(left_matcher);
	Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
	
	//Compute the disparity maps
	left_matcher->compute(left_gray, right_gray, left_disp);
	right_matcher->compute(right_gray, left_gray, right_disp);

	//Use the filter to clean
	wls_filter->setLambda(20000);
	wls_filter->setSigmaColor(1.2);
	wls_filter->setLRCthresh(5);
	wls_filter->setDepthDiscontinuityRadius(0.05);
	wls_filter->filter(left_disp, left_img, filtered_disp, right_disp);
	Mat conf_map = wls_filter->getConfidenceMap();
	
	//Get the disparity visualization for both the raw and filtered disparity map
	Mat raw_disp_vis,combined_disparity;
	float vis_mult = 3.0f;
	getDisparityVis(left_disp, raw_disp_vis, vis_mult);
	Mat filtered_disp_vis;
	getDisparityVis(filtered_disp, filtered_disp_vis, vis_mult);

	//Resize both images to fit them onto screen and put them side by side
	resize(raw_disp_vis, raw_disp_vis, Size(), 0.5, 0.5);
	resize(filtered_disp_vis, filtered_disp_vis, Size(), 0.5, 0.5);
	matrices = { raw_disp_vis,filtered_disp_vis };
	hconcat(matrices, combined_disparity);
	imshow("Raw and Filtered Disparities", combined_disparity);

	//Save images to file
	imwrite("left_right_original.jpg", combined);
	imwrite("raw_filtered_disparities(BM).jpg",combined_disparity);

	while (true) {
		char k = waitKey(50);
		if (k == 27) {
			break;
		}
	}

	return 0;
}


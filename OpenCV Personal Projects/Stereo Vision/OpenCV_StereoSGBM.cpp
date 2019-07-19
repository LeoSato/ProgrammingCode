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
	Mat left_img = imread("left.png");
	Mat left = left_img;					//Mat left will be resized to help the image displays to fit the screen
	Mat right_img = imread("right.png");
	Mat right = left_img;					//Mat right will be resized to help the image displays to fit the screen
	Mat combined, left_gray, right_gray;

	//Convert images to gray scale
	cvtColor(left_img, left_gray, COLOR_BGR2GRAY);
	cvtColor(right_img, right_gray, COLOR_BGR2GRAY);

	//Resize original images and place side to side to make it easier to view
	resize(left, left, Size(), 0.5, 0.5);
	resize(right, right, Size(), 0.5, 0.5);
	vector<Mat> matrices = { left,right };
	hconcat(matrices, combined);
	imshow("Original Left and Right Images", combined);

	Ptr<StereoSGBM> left_matcher_SGBM = StereoSGBM::create(0, 80, 3, 100, 400, 1, 16, 10, 100, 1, 0);
	Ptr<DisparityWLSFilter> wls_filter_SGBM = createDisparityWLSFilter(left_matcher_SGBM);
	Ptr<StereoMatcher> right_matcher_SGBM = createRightMatcher(left_matcher_SGBM);

	//Compute the disparity maps
	Mat left_disp, right_disp, filtered_disp, solved_filtered_disp;
	left_matcher_SGBM->compute(left_gray, right_gray, left_disp);
	right_matcher_SGBM->compute(right_gray, left_gray, right_disp);

	//Use the filter to clean
	wls_filter_SGBM->setLambda(16000);
	wls_filter_SGBM->setSigmaColor(2);
	//wls_filter_SGBM->setLRCthresh(5);
	wls_filter_SGBM->setDepthDiscontinuityRadius(0.05);
	wls_filter_SGBM->filter(left_disp, left_img, filtered_disp, right_disp);

	//Get the disparity visualization for both the raw and filtered disparity map
	Mat raw_disp_vis, combined_disparity, filtered_disp_vis;
	float vis_mult = 3.5f;
	getDisparityVis(left_disp, raw_disp_vis, vis_mult);
	getDisparityVis(filtered_disp, filtered_disp_vis, vis_mult);

	//Resize both images to fit them onto screen and put them side by side
	resize(raw_disp_vis, raw_disp_vis, Size(), 0.5, 0.5);
	resize(filtered_disp_vis, filtered_disp_vis, Size(), 0.5, 0.5);
	matrices = { raw_disp_vis,filtered_disp_vis };
	hconcat(matrices, combined_disparity);
	imshow("Raw and Filtered Disparities", combined_disparity);

	//Save images to file
	imwrite("left_right_original.jpg", combined);
	imwrite("raw_filtered_disparities(SGBM).jpg", combined_disparity);

	while (true) {
		char k = waitKey(50);
		if (k == 27) {
			break;
		}
	}

	return 0;
}


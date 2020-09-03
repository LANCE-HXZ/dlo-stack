#include "dlo.h"
using namespace cv;


 // ==========    Main    ==========
Mat skeleton(cv::Mat input, string save_address, int channels)
{	
	cv::Mat src;
	int c;
	if(channels == 3)
	{
		//若输入为三通道图像, 则分离通道
		std::vector<cv::Mat>SrcMatpart(input.channels());
		cv::split(input,SrcMatpart);
		src = SrcMatpart[0];
	}
	else if(channels == 1){
		src = input;
	}
	
	src/=255;
	src*=255;
	// pre_erode(src, 3, 1); // 腐蚀去除白离群点
	pre_dilate(src, 3, 2); // 膨胀去除黑离群点
	// cv::imshow("pre_dilate", src);  // === 显示图片 ===
	cv::imwrite("pic_buffer/4_B2_dilate.png", src);
	// cv::waitKey();
	// cv::destroyWindow("pre_dilate");  // === 显示图片 ===
	// pre_erode(src, 3, 5); // 腐蚀去除白离群点
	// cv::imshow("pre_erode", src);  // === 显示图片 ===
	// cv::imwrite("pic_buffer/4_B3_erode.png", src);
	// cv::waitKey();
	// cv::destroyWindow("pre_erode");  // === 显示图片 ===
	// ========== Zhang-Suen ========== 
	clock_t start, end; // time
	start = clock(); // time
	thinImage(src); // 图像细化
	end = clock(); // time
	double endtime = (double)(end - start) / CLOCKS_PER_SEC; // time
	// cout << endtime << endl; // time
	cv::imwrite(save_address, src);
	// namedWindow("Skeleton", WINDOW_AUTOSIZE);  // === 显示图片 === //WINDOW_AUTOSIZE:系统默认,显示自适应
	// imshow("Skeleton", src);
	// moveWindow("Skeleton", 0, 360);
	// waitKey();
	// cv::destroyWindow("Skeleton");// === 显示图片 ===
	return src;
	// ========== Zhang-Suen ========== 
	
	// ==========    LAB     ==========
	// clock_t start, end; // time
	// start = clock(); // time
	// //cv::Mat src = cv::imread("C:/Users/Lance/Desktop/thinning_Zhan-Suen-master/test/test0.png", 0);
	// cv::Mat dst;
	// thinning(src, dst);
	// end = clock(); // time
	// double endtime = (double)(end - start) / CLOCKS_PER_SEC; // time
	// cout << endtime << endl; // time
	// cv::namedWindow("Skeleton", WINDOW_AUTOSIZE);
	// cv::imshow("Skeleton", dst);
	// cv::imwrite(save_address, src);
	// cv::waitKey();
	// cv::destroyWindow("Skeleton");
	// return dst;
	// ==========    LAB     ==========
}// ==========    Main    ==========


 // 腐蚀  膨胀
 // ==========   dilate   ==========
void pre_dilate(Mat & srcImg, int size, int times){
	Mat element = getStructuringElement(MORPH_CROSS, Size(size, size));
	for (size_t i = 0; i < times; i++)
	{
		dilate(srcImg, srcImg, element);
	}
}
 // ==========   dilate   ==========
 // ==========   Erode    ==========
void pre_erode(Mat & srcImg, int size, int times){
	Mat element = getStructuringElement(MORPH_CROSS, Size(size, size));
	for (size_t i = 0; i < times; i++)
	{
		erode(srcImg, srcImg, element);
	}
}// ==========   Erode    ==========


 // 图像细化
 // ========== Zhang-Suen ==========
void thinImage(Mat & srcImg) {
	vector<Point> deleteList;
	int neighbourhood[9];
	int nl = srcImg.rows;
	int nc = srcImg.cols;
	bool inOddIterations = true;
	while (true) {
		for (int j = 1; j < (nl - 1); j++) {
			uchar* data_last = srcImg.ptr<uchar>(j - 1);
			uchar* data = srcImg.ptr<uchar>(j);
			uchar* data_next = srcImg.ptr<uchar>(j + 1);
			for (int i = 1; i < (nc - 1); i++) {
				if (data[i] == 255) {
					int whitePointCount = 0;
					neighbourhood[0] = 1;
					if (data_last[i] == 255) neighbourhood[1] = 1;
					else  neighbourhood[1] = 0;
					if (data_last[i + 1] == 255) neighbourhood[2] = 1;
					else  neighbourhood[2] = 0;
					if (data[i + 1] == 255) neighbourhood[3] = 1;
					else  neighbourhood[3] = 0;
					if (data_next[i + 1] == 255) neighbourhood[4] = 1;
					else  neighbourhood[4] = 0;
					if (data_next[i] == 255) neighbourhood[5] = 1;
					else  neighbourhood[5] = 0;
					if (data_next[i - 1] == 255) neighbourhood[6] = 1;
					else  neighbourhood[6] = 0;
					if (data[i - 1] == 255) neighbourhood[7] = 1;
					else  neighbourhood[7] = 0;
					if (data_last[i - 1] == 255) neighbourhood[8] = 1;
					else  neighbourhood[8] = 0;
					for (int k = 1; k < 9; k++) {
						whitePointCount += neighbourhood[k];
					}
					if ((whitePointCount >= 2) && (whitePointCount <= 6)) {
						int ap = 0;
						if ((neighbourhood[1] == 0) && (neighbourhood[2] == 1)) ap++;
						if ((neighbourhood[2] == 0) && (neighbourhood[3] == 1)) ap++;
						if ((neighbourhood[3] == 0) && (neighbourhood[4] == 1)) ap++;
						if ((neighbourhood[4] == 0) && (neighbourhood[5] == 1)) ap++;
						if ((neighbourhood[5] == 0) && (neighbourhood[6] == 1)) ap++;
						if ((neighbourhood[6] == 0) && (neighbourhood[7] == 1)) ap++;
						if ((neighbourhood[7] == 0) && (neighbourhood[8] == 1)) ap++;
						if ((neighbourhood[8] == 0) && (neighbourhood[1] == 1)) ap++;
						if (ap == 1) {
							if (inOddIterations && (neighbourhood[3] * neighbourhood[5] * neighbourhood[7] == 0)
								&& (neighbourhood[1] * neighbourhood[3] * neighbourhood[5] == 0)) {
								deleteList.push_back(Point(i, j));
							}
							else if (!inOddIterations && (neighbourhood[1] * neighbourhood[5] * neighbourhood[7] == 0)
								&& (neighbourhood[1] * neighbourhood[3] * neighbourhood[7] == 0)) {
								deleteList.push_back(Point(i, j));
							}
						}
					}
				}
			}
		}
		if (deleteList.size() == 0)
			break;
		for (size_t i = 0; i < deleteList.size(); i++) {
			Point tem;
			tem = deleteList[i];
			uchar* data = srcImg.ptr<uchar>(tem.y);
			data[tem.x] = 0;
		}
		deleteList.clear();
		inOddIterations = !inOddIterations;
	}

}// ========== Zhang-Suen ==========
 // https://blog.csdn.net/FunnyWhiteCat/article/details/80670332


 // ==========    LAB     ==========
void thinningIteration(cv::Mat& img, int iter)
{
	CV_Assert(img.channels() == 1);
	CV_Assert(img.depth() != sizeof(uchar));
	CV_Assert(img.rows > 3 && img.cols > 3);

	cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);

	int nRows = img.rows;
	int nCols = img.cols;

	if (img.isContinuous()) {
		nCols *= nRows;
		nRows = 1;
	}

	int x, y;
	uchar *pAbove;
	uchar *pCurr;
	uchar *pBelow;
	uchar *nw, *no, *ne;    // north (pAbove)
	uchar *we, *me, *ea;
	uchar *sw, *so, *se;    // south (pBelow)

	uchar *pDst;

	// initialize row pointers
	pAbove = NULL;
	pCurr = img.ptr<uchar>(0);
	pBelow = img.ptr<uchar>(1);

	for (y = 1; y < img.rows - 1; ++y)
	{
		// shift the rows up by one
		pAbove = pCurr;
		pCurr = pBelow;
		pBelow = img.ptr<uchar>(y + 1);

		pDst = marker.ptr<uchar>(y);

		// initialize col pointers
		no = &(pAbove[0]);
		ne = &(pAbove[1]);
		me = &(pCurr[0]);
		ea = &(pCurr[1]);
		so = &(pBelow[0]);
		se = &(pBelow[1]);

		for (x = 1; x < img.cols - 1; ++x)
		{
			// shift col pointers left by one (scan left to right)
			nw = no;
			no = ne;
			ne = &(pAbove[x + 1]);
			we = me;
			me = ea;
			ea = &(pCurr[x + 1]);
			sw = so;
			so = se;
			se = &(pBelow[x + 1]);

			int A = (*no == 0 && *ne == 1) + (*ne == 0 && *ea == 1) +
				(*ea == 0 && *se == 1) + (*se == 0 && *so == 1) +
				(*so == 0 && *sw == 1) + (*sw == 0 && *we == 1) +
				(*we == 0 && *nw == 1) + (*nw == 0 && *no == 1);
			int B = *no + *ne + *ea + *se + *so + *sw + *we + *nw;
			int m1 = iter == 0 ? (*no * *ea * *so) : (*no * *ea * *we);
			int m2 = iter == 0 ? (*ea * *so * *we) : (*no * *so * *we);

			if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
				pDst[x] = 1;
		}
	}
	img &= ~marker;
}
void thinning(const cv::Mat& src, cv::Mat& dst)
{
	dst = src.clone();
	dst /= 255;         // convert to binary image

	cv::Mat prev = cv::Mat::zeros(dst.size(), CV_8UC1);
	cv::Mat diff;

	do {
		thinningIteration(dst, 0);
		thinningIteration(dst, 1);
		cv::absdiff(dst, prev, diff);
		dst.copyTo(prev);
	} while (cv::countNonZero(diff) > 200);
	//while (cv::countNonZero(diff) > 0);

	dst *= 255;
}// ==========    LAB     ==========


// ==========    查表法    ==========
Mat ImgSkeletonization(Mat &input_src,Mat & output_dst, int number)
{
	output_dst = input_src.clone();
	int search_array[]= { 0,0,1,1,0,0,1,1,1,1,0,1,1,1,0,1,\
		1,1,0,0,1,1,1,1,0,0,0,0,0,0,0,1,\
		0,0,1,1,0,0,1,1,1,1,0,1,1,1,0,1,\
		1,1,0,0,1,1,1,1,0,0,0,0,0,0,0,1,\
		1,1,0,0,1,1,0,0,0,0,0,0,0,0,0,0,\
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
		1,1,0,0,1,1,0,0,1,1,0,1,1,1,0,1,\
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
		0,0,1,1,0,0,1,1,1,1,0,1,1,1,0,1,\
		1,1,0,0,1,1,1,1,0,0,0,0,0,0,0,1,\
		0,0,1,1,0,0,1,1,1,1,0,1,1,1,0,1,\
		1,1,0,0,1,1,1,1,0,0,0,0,0,0,0,0,\
		1,1,0,0,1,1,0,0,0,0,0,0,0,0,0,0,\
		1,1,0,0,1,1,1,1,0,0,0,0,0,0,0,0,\
		1,1,0,0,1,1,0,0,1,1,0,1,1,1,0,0,\
		1,1,0,0,1,1,1,0,1,1,0,0,1,0,0,0 };
	for (size_t i = 0; i < number; i++)
	{
		ImgSkeletonization_H(output_dst, &search_array[0]);
		ImgSkeletonization_V(output_dst, &search_array[0]);
		
	}
	return output_dst;
}
Mat ImgSkeletonization_H(Mat &input_src, int *search_arr)
{
	int h = input_src.rows;
	int w = input_src.cols;
	bool NEXT = true;
	for (size_t j = 1; j < w - 1; j++)//注意边界问题！！！！！！
	{
		for (size_t i = 1; i < h - 1; i++)
		{
			if (!NEXT)
				NEXT = true;
			else
			{
				int judge_value;
				if (1 <i < h - 1)
					judge_value = input_src.at<uchar>(i - 1, j) + input_src.at<uchar>(i, j) + input_src.at<uchar>(i + 1, j);
				else
					judge_value = 1;
				if (input_src.at<uchar>(i, j) == 0 && judge_value != 0)
				{
					int a[9] = { 1,1,1,1,1,1,1,1,1};
					for (size_t m = 0; m < 3; m++)
					{
						for (size_t n = 0; n < 3; n++)
						{
							if ((0 <= (i - 1 + m) < h) && (0 <= (j - 1 + n) < w) && input_src.at<uchar>(i - 1 + m, j - 1 + n) == 0)
								a[m * 3 + n] = 0;
						}
					}
					int sum_value = a[0] * 1 + a[1] * 2 + a[2] * 4 + a[3] * 8 + a[5] * 16 + a[6] * 32 + a[7] * 64 + a[8] * 128;
					input_src.at<uchar>(i, j) = search_arr[sum_value] * 255;
					if (search_arr[sum_value] == 1)
						NEXT = false;
				}
			}
		}
	}
	return input_src;
}
Mat ImgSkeletonization_V(Mat &input_src, int *search_arr)
{
	int h = input_src.rows;
	int w = input_src.cols;
	bool NEXT = true;
	for (size_t i = 1; i < h - 1; i++)//注意边界问题！！！！！！
	{
		for (size_t j = 1; j < w - 1; j++)
		{
			if (!NEXT)
				NEXT = true;
			else
			{
				int judge_value;
				if (1 < j <w - 1)
					judge_value = input_src.at<uchar>(i, j - 1) + input_src.at<uchar>(i, j) + input_src.at<uchar>(i, j + 1);
				else
					judge_value = 1;
				if (input_src.at<uchar>(i, j) == 0 && judge_value != 0)
				{
					int a[9] = {1,1,1,1,1,1,1,1,1 };
					for (size_t m = 0; m < 3; m++)
					{
						for (size_t n = 0; n < 3; n++)
						{
							if ((0 <= (i - 1 + m) < h) && (0 <= (j - 1 + n) < w) && input_src.at<uchar>(i - 1 + m, j - 1 + n) == 0)
								a[m * 3 + n] = 0;
						}
					}
					int sum_value = a[0] * 1 + a[1] * 2 + a[2] * 4 + a[3] * 8 + a[5] * 16 + a[6] * 32 + a[7] * 64 + a[8] * 128;
					input_src.at<uchar>(i, j) = search_arr[sum_value] * 255;
					if (search_arr[sum_value] == 1)
						NEXT = false;
				}
			}
		}
	}
	return input_src;
}// ==========    查表法    ==========
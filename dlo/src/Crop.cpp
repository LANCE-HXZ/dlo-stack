#include "pch.h"
#include<math.h>

cv::Mat init, rot, cut;

 // ==========    Main    ==========
void crop(cv::Mat src)
{
	cv::namedWindow("src", cv::WINDOW_AUTOSIZE);
	src.copyTo(init);
	cv::setMouseCallback("src", crop_callback, 0);
	cv::imshow("src", src);
	cv::waitKey(0);
}// ==========    Main    ==========

 // ======= 鼠标事件回调函数 =======
void crop_callback(int event, int x, int y, int flags, void* ustc)
{
	static int x1, y1;
	//clock_t start, end; // time
	//start = clock(); // time
	if (event == CV_EVENT_RBUTTONDOWN)
	{
		x1 = x, y1 = y;
	}
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		int height = 60, width = 60;
		init.copyTo(rot);
		cv::Point center = cv::Point(x1, y1);
		circle(init, center, 3, cv::Scalar(0, 0, 255), -1);

		float dx = x - x1; float dy = y - y1;
		double angle = -atan(dx / (dy + 0.00000001)) * 180 / 3.14159265;
		if (dy < 0)
		{
			angle += 180;
		}
		rot = RotateImage(rot, angle, center);// 逆时针angle为正
		circle(rot, cv::Point(x1, y1), 3, cv::Scalar(0, 0, 255), -1);
		cv::Rect m_select = cv::Rect(x1 - width / 2, y1 - height / 2, width, height);
		cut = rot(m_select);
		imshow("src", init);
		imshow("rot", rot);
		imshow("cut", cut);
		rot.copyTo(init);//确保画线操作是在src上进行
	}
	//end = clock(); // time
	//double endtime = (double)(end - start) / CLOCKS_PER_SEC; // time	
	//cout << endtime << endl; // time
}// ======= 鼠标事件回调函数 =======

 // =========   旋转图片   =========
cv::Mat RotateImage(cv::Mat src, double angle, cv::Point center)
{
	cv::Mat copy;
	src.copyTo(copy);
	try
	{
		//float scale = 200.0/ src.rows;//缩放因子    
		//cv::resize(src, src, cv::Size(), scale, scale, cv::INTER_LINEAR);    	    	
		//输出图像的尺寸与原图一样    
		cv::Size dst_sz(copy.cols, copy.rows);

		//指定旋转中心      
		//cv::Point2f center(static_cast<float>(src.cols / 2.), static_cast<float>(src.rows / 2.));

		//获取旋转矩阵（2x3矩阵）      
		cv::Mat rot_mat = cv::getRotationMatrix2D(center, angle, 1.0);
		//设置选择背景边界颜色   
		/*cv::Scalar borderColor = Scalar(0, 238, 0);*/
		/*cv::warpAffine(src, dst, rot_mat, src.size(), INTER_LINEAR, BORDER_CONSTANT, borderColor);*/
		//复制边缘填充
		cv::warpAffine(copy, copy, rot_mat, dst_sz, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
	}
	catch (cv::Exception e)
	{
	}
	return copy;
}// =========   旋转图片   =========
#include "dlo.h"
using namespace cv;
using namespace std;

Mat traversal_visualization(Mat src){
    // 每根线涂一种颜色
	int line_index = 1; int cpt_index = 0; int color_size = 7;
	for(int n = 0; n < point_num; ++n){
		int ept_index = 2*line_index-1;
		Scalar color_value = getcolor(n, ept_index, line_index);
		circle(src, pt[n], color_size/2, color_value, -1);
		if(n == ept[ept_index])	
			++line_index;
		if(n == cpt[cpt_index])	
			++cpt_index;
		if(line_index > line_num)	break;
	}
    return src;
}

void visualization()
{
    // Mat visual_img = imread("pic_buffer/init/black.png");  // 可视化背景
	// resize(visual_img, visual_img, Size(800, 640));
	Mat visual_img = cv::Mat::zeros(640, 800, CV_8UC3);
	int line_index = 1; int line_num = ept_num/2; int cpt_index = 0;
	int color_size = 7; int end_count = 0, cross_count = 0;
	cout << '\t' << "LINE" << '\t' << "INDEX" << '\t' << "CROSS" << '\t' << "CLASS" << endl;
	// cout << '\t' << end_count << '\t' << line_index << '\t' << cross.size()+end_count++ << '\t' << "E" << endl;
	for(int n = 0; n < point_num; ++n){
		int ept_index = 2*line_index-1;
		Scalar color_value = getcolor(n, ept_index, line_index);
        if(n != ept[ept_index-1] && n != cpt[cpt_index])
			// colorline(line_index, n, 7, color_value);
			line(visual_img, pt[n], pt[n-1], color_value, color_size);
		if(n == cpt[cpt_index]){
			cout << '\t' << line_index  << '\t' << cpt_index << '\t' << g_vnCrossList[cpt_index] << '\t' << g_vnClassList[cpt_index] << endl;
			if(1 == g_vnClassList[cpt_index])
			{
				// // 条线连接交叉处
				// line(visual_img, pt[cpt[cpt_index]], pt[cpt[cpt_index]-1], color_value, color_size);
				// // 两条线连接交叉处
				line(visual_img, pt[cpt[cpt_index]], cross[g_vnCrossList[cpt_index]], color_value, color_size);
				line(visual_img, cross[g_vnCrossList[cpt_index]], pt[cpt[cpt_index]-1], color_value, color_size);
				// // 四条线连接交叉处 p5 p6还要琢磨一下
				// Point p0, p1, p2, p3, p4, p5, p6;
				// p4 = pt[cpt[cpt_index]]; 
				// p0 = pt[cpt[cpt_index]-1];
				// p2 = cross[crosslist[cpt_index]];
				// p1 = (p0+p2)/2; p3 = (p4+p2)/2;
				// p5 = 2*p1-(3*p0+p2+p4)/5; p6 = 2*p3-(3*p4+p2+p0)/5;
				// line(visual_img, p0, p5, color_value, color_size);
				// line(visual_img, p5, p2, color_value, color_size);
				// line(visual_img, p2, p6, color_value, color_size);
				// line(visual_img, p6, p4, color_value, color_size);
			}
			else
			{
				Point p0, p1, p2, p3, p4;
				p4 = pt[cpt[cpt_index]]; 
				p0 = pt[cpt[cpt_index]-1];
				p2 = cross[g_vnCrossList[cpt_index]];
				p1 = 2*p0/3+p2/3; p3 = 2*p4/3+p2/3;
				line(visual_img, p1, p0, color_value, color_size);
				line(visual_img, p4, p3, color_value, color_size);
			}
			++cpt_index;
		}
		// if(cpt_index == cpt_num)	break;
		if(n == ept[ept_index]){
			// cout << '\t' << end_count << '\t' << line_index << '\t' << cross.size()+end_count++ << '\t' << "E" << endl << endl;
			++line_index;
			// if(line_index != line_num)cout << '\t' << end_count << '\t' << line_index << '\t' << cross.size()+end_count++ << '\t' << "E" << endl;
		}	
	}
	// namedWindow("visual_img", WINDOW_AUTOSIZE);// === 显示图片 ===
	// imshow("visual_img", visual_img);
	// waitKey();
	// cv::destroyWindow("visual_img");// === 显示图片 ===
	cv::imwrite("pic_buffer/7_V.png", visual_img);
}

Scalar getcolor(int n, int ept_index, int line_index)
{
	int color_v = (n-ept[ept_index-1])*255/(ept[ept_index]-ept[ept_index-1]);
	if(1 == line_index){
		return Scalar(255, color_v, 0);
	}
	else if(2 == line_index){
		return Scalar(0, 255, color_v);
	}
	else if(3 == line_index){
		return Scalar(color_v, 128, 255);
	}
	else if(4 == line_index){
		return Scalar(255, 0, color_v);
	}
	else if(5 == line_index){
		return Scalar(64, 64, color_v);
	}
}
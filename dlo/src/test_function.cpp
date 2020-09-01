#include "dlo.h"
using namespace cv;
vector<Point> whiteptset;

// ros::NodeHandle node_handle; 
//   int a;
//   node_handle.setParam("qiangqiang", 3);
//   node_handle.getParam("qiangqiang", a);
//   cout << a << endl;

void rgb2binary(cv::Mat &src, cv::Mat &dst){
	int c = src.cols, r = src.rows;
	for(int ic = 0; ic < c; ++ic){
		for(int ir = 0; ir < r; ++ir){
			if(src.at<Vec3b>(ir, ic)[2] > 200 && src.at<Vec3b>(ir, ic)[0] < 55)
				dst.at<Vec3b>(ir, ic)[0] = dst.at<Vec3b>(ir, ic)[1] = dst.at<Vec3b>(ir, ic)[2] = 0;
			else
				dst.at<Vec3b>(ir, ic)[0] = dst.at<Vec3b>(ir, ic)[1] = dst.at<Vec3b>(ir, ic)[2] = 255;
		}
	}
}

void new_ske(cv::Mat &src, cv::Mat &dst, cv::Mat &bw, vector<cv::Point> &endpoint){
	int c = src.cols;
	int r = src.rows;
	dst*=255;
	// imshow("dst", dst);
	// moveWindow("dst", 840, 360);
	Mat search;
	Mat new_skeleton;
	src.copyTo(search);
	search *= 0;
	search.copyTo(new_skeleton);
	vector<int> flag(endpoint.size());
	for(int i = 0; i < endpoint.size(); ++i){
		if(flag[i])	continue;
		else	flag[i] = 1;
		bool reachend = 0;
		Point nextp = get_first_current_point(dst, endpoint[i], 10, 1, 1);
		Point currp = endpoint[i];
		cout << " --- " << endl;
		int t = 0;
		while(t < 50){
			circle(dst, currp, 1, Scalar(225), -1);
			circle(dst, nextp, 1, Scalar(0), -1);
			Point sdir = per_dir(currp, nextp, 10);
			// cout << currp << nextp << sdir << endl;
			Point startp = nextp+Point(-sdir.y,sdir.x), endp = nextp+Point(sdir.y,-sdir.x);
			DrawArc(search, bw, nextp, nextp+sdir, endp, 2);
			DrawArc(search, bw, nextp, startp, nextp+sdir, 2);
			// bool foundnext = 0;
			Point avg = 20 * (nextp+sdir);
			imshow("search", search);
			moveWindow("search", 840, 840);
			int count = 20;
			for(int ir = 0; ir < r; ++ir){
				for(int ic = 0; ic < c; ++ic){
					if(search.at<uchar>(ir, ic)){
						if(bw.at<uchar>(ir, ic)){	
						avg += Point(ic, ir);  
						++count;
						cout << "found  " << ic << " " << ir << endl;
						imshow("search", search);
						moveWindow("search", 840, 740);
						// waitKey();
						}
					}
				}
			}
			line(new_skeleton, currp, nextp, Scalar(255, 255, 255), 1);
			currp = nextp;
			// cout << whiteptset.size() << endl;
			// nextp = whiteptset[whiteptset.size()/2];
			nextp = avg/count;///////////////////////////////这里的方法会走进黑色区域///////////////////////////////
			search*=0;
			// DrawArc(dst, nextp, startp, endp, 1);
			imshow("ske", new_skeleton);
			imshow("dst", dst);
			moveWindow("dst", 0, 360);
			waitKey();
			++t;
			whiteptset.clear();
			// break;
		}
	}
	
}

void DrawArc(Mat &src, Mat &bw, Point ArcCenter, Point StartPoint, Point EndPoint, int Fill)
{
	if (Fill <= 0) return;
 
	vector<Point> Dots;
	double Angle1 = atan2((StartPoint.y - ArcCenter.y), (StartPoint.x - ArcCenter.x));
	double Angle2 = atan2((EndPoint.y - ArcCenter.y), (EndPoint.x - ArcCenter.x));
	double Angle = Angle1 - Angle2;
	Angle = Angle * 180.0 / CV_PI;
 
	if (Angle < 0) Angle = 360 + Angle;
	if (Angle == 0) Angle = 360;
	int  brim = floor(Angle / 10); // 向下取整
 
	Dots.push_back(StartPoint);
	for (int i = 0; i < brim; i++)
	{
		double dSinRot = sin(-(10 * (i + 1)) * CV_PI / 180);
		double dCosRot = cos(-(10 * (i + 1)) * CV_PI / 180);
		int x = ArcCenter.x + dCosRot * (StartPoint.x - ArcCenter.x) - dSinRot * (StartPoint.y - ArcCenter.y);
		int y = ArcCenter.y + dSinRot * (StartPoint.x - ArcCenter.x) + dCosRot * (StartPoint.y - ArcCenter.y);
		Dots.push_back(Point(x, y));
		if(src.at<uchar>(y, x) && bw.at<uchar>(y, x)){
			whiteptset.push_back(Point(x, y));
		}	
	}
	Dots.push_back(EndPoint);
	RNG &rng = theRNG();
	Scalar color = Scalar(255,255,255);
	for (int i = 0; i < Dots.size() - 1; i++) {
		line(src, Dots[i], Dots[i + 1], color, Fill);
	}
	Dots.clear();
}


cv::Mat new_bw = cv::imread("/home/lance/Data/3-WhiteLine/1.png");
// class Ve {
// public:
// 	vector<Mat> ReadImage(cv::String pattern);
// };
// // 批量处理图片
	// cv::String pattern = "/home/lance/Data/test/2-替换背景/0-rgb/*.png";
	// Ve ve;
	// vector<Mat> img = ve.ReadImage(pattern);
	// string read_address = "/home/lance/Data/0-RGBimg/1.png";
	// Mat rgb_img = imread(read_address);

// // 图像细化
	// string read_address = "/home/lance/Workspaces/vs_ws/pic/test.png";
	// string save_address = "/home/lance/Workspaces/vs_ws/pic/test_s.png";
	// Mat src = imread(read_address, 0);
	// skeleton(src, save_address);

	// // 遍历曲线
	// //vector<cv::String> fn;
	// string read_address = "/home/lance/Workspaces/vs_ws/pic/test_s.png";
	// Mat src = imread(read_address);
	// traversal(src);
	
	// // 旋转裁剪
	// string read_address = "/home/lance/Workspaces/vs_ws/pic/test.png";
	// Mat src = imread(read_address);
	// crop(src);


	// 删除背景
	// string read_address = "/home/lance/Data/test/2-替换背景/0-rgb.png";
	// Mat rgb = imread(read_address);
	// read_address = "/home/lance/Data/test/2-替换背景/bg2.png";
	// Mat background = imread(read_address);
	// read_address = "/home/lance/Data/test/2-替换背景/1-whiteline.png";
	// Mat whiteline = imread(read_address);
	// // Mat cableonly_img = remove_background(rgb_img, bw_img);
	// // imwrite("/home/lance/Workspaces/hxz_ws/src/dlo/data/cableonly_img.bmp", cableonly_img);
	// pre_erode(whiteline, 3, 2); // 腐蚀图像
	// Mat rebackground = change_background(rgb, background, whiteline);
	// imwrite("/home/lance/Data/test/2-替换背景/4-rebackground.png", rebackground);




//// 批量骨骼化
//class Ve {
//public:
//	vector<Mat> ReadImage(cv::String pattern);
//};
//int test()
//{
//	cv::String pattern = "G:/Desktop/Project/test/WhiteLine/*.bmp";
//	Ve ve;
//	vector<Mat> img = ve.ReadImage(pattern);
//	return 0;
//}
//vector<Mat> Ve::ReadImage(cv::String pattern)
//{
//	vector<cv::String> fn;
//	glob(pattern, fn, false);
//	vector<Mat> images;
//	size_t count = fn.size(); //number of png files in images folder
//	for (size_t i = 0; i < count; i++)
//	{
//		images.emplace_back(cv::imread(fn[i]));
//		string save_address = "G:/Desktop/Project/test/Skeleton/" + fn[i].substr(34, fn[i].length() - 38) + ".png";
//		skeleton(imread(fn[i], 0), save_address);
//		cout << save_address << endl;
//	}
//	return images;
//}
//
//
//class Ve {
//public:
//	vector<Mat> ReadImage(cv::String pattern);
//};




// // 批量裁剪交点
// class Ve {
// public:
// 	vector<Mat> ReadImage(cv::String pattern);
// };
// int test()
// {
// 	cv::String pattern = "/home/lance/Workspaces/hxz_ws/src/dlo/python/mynet/checkpoint/S*";
// 	Ve ve;
// 	vector<Mat> img = ve.ReadImage(pattern);
// 	return 0;
// }
// //批量遍历裁剪交叉点
// vector<Mat> Ve::ReadImage(cv::String pattern)
// {
// 	vector<cv::String> fn;
// 	glob(pattern, fn, false);
// 	vector<Mat> images;
// 	size_t count = fn.size(); //number of png files in images folder
// 	for (size_t i = 0; i < count; i++) {
// 		Mat src = imread(fn[i]);
// 		copyMakeBorder(src, src, 100, 100, 100, 100, BORDER_CONSTANT, 0);
// 		string img_num = fn[i].substr(53, fn[i].length() - 57);
// 		cout << img_num << endl;
// 		traversal(src, img_num);
// 		const char *deletepath = fn[i].c_str();
// 		if(remove(deletepath)==0)
// 		{
// 			cout << img_num << ".png" << '\t' << "删除成功" << endl;
// 			cout << "---========================---" << endl;
// 		}
// 		else{
// 			cout << img_num << ".png" << '\t' << "删除失败" << endl;
// 			cout << "---========================---" << endl;
// 		}
// 	}
// 	return images;
// }




// // 批量换背景
// vector<Mat> Ve::ReadImage(cv::String pattern)
// {
	
// 	int nr, nc;
// 	// nr = new_bw.rows;
// 	// nc = new_bw.cols;
// 	// for(int i=0; i<nr; i++){
// 	// 	uchar* new_bw_pix = new_bw.ptr<uchar>(i);
// 	// 	for(int j=0; j<nc; j++){
// 	// 		new_bw_pix[3*j] = 0;
// 	// 		new_bw_pix[3*j+1] = 0;
// 	// 		new_bw_pix[3*j+2] = 0;
// 	// 	}
// 	// }
// 	vector<cv::String> fn;
// 	glob(pattern, fn, false);
// 	vector<Mat> images;
// 	size_t count = fn.size(); //number of png files in images folder
// 	for(int y = 370; y < 500; y+=50){
// 		cv::resize(new_bw, new_bw, Size(y*2, y), 0, 0, cv::INTER_CUBIC);
// 		// Mat *p = &new_bw;
// 		for(int j = 0; j < 3; j++){
// 			for (size_t i = 0; i < count; i++) {
// 				cv::Mat rebackground;
// 				Mat src = imread(fn[i]);
// 				//copyMakeBorder(src, src, 100, 100, 100, 100, BORDER_CONSTANT, 0);
// 				string img_num = fn[i].substr(43, fn[i].length() - 47);
// 				// cout << img_num << endl;
// 				// 删除背景
// 				string read_address = "/home/lance/Data/test/2-替换背景/0-rgb/" + img_num + ".png";
// 				Mat rgb = imread(read_address);
				
// 				read_address = "/home/lance/Data/test/2-替换背景/1-bw/" + img_num + ".png";
// 				Mat whiteline = imread(read_address);

// 				read_address = "/home/lance/Data/test/2-替换背景/2-bg/bg" + to_string(j) + ".png";
// 				Mat background = imread(read_address);

// 				// Mat cableonly_img = remove_background(rgb_img, bw_img);
// 				// imwrite("/home/lance/Workspaces/hxz_ws/src/dlo/data/cableonly_img.bmp", cableonly_img);
// 				pre_erode(whiteline, 3, 2); // 腐蚀图像
// 				rebackground= change_background(rgb, background, whiteline, y);
// 				imwrite("/home/lance/Data/test/2-替换背景/3-newrgb/" + to_string(y) + "_" + img_num + "_" + to_string(j) + ".png", rebackground);
// 				cout << y << "_" << j << "_" << i << "/" << count << endl;
// 			cv::resize(new_bw, new_bw, rgb.size(), 0, 0, cv::INTER_CUBIC);
// 			imwrite("/home/lance/Data/test/2-替换背景/4-newbw/" + to_string(y) + "_" + img_num + ".png", new_bw);
// 			nr = new_bw.rows;
// 			nc = new_bw.cols;
// 			for(int i=0; i<nr; i++){
// 				uchar* new_bw_pix = new_bw.ptr<uchar>(i);
// 				for(int j=0; j<nc; j++){
// 					new_bw_pix[3*j] = 0;
// 					new_bw_pix[3*j+1] = 0;
// 					new_bw_pix[3*j+2] = 0;
// 					}
// 				}
// 			}
// 		}
// 	}
// 	return images;
// }
#include "pch.h"
using namespace cv;


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

// 批量裁剪交点
class Ve {
public:
	vector<Mat> ReadImage(cv::String pattern);
};
int test()
{
	cv::String pattern = "/media/lance/新加卷/Desktop/Project/test/Skeleton/*.png";
	Ve ve;
	vector<Mat> img = ve.ReadImage(pattern);
	return 0;
}
vector<Mat> Ve::ReadImage(cv::String pattern)
{
	vector<cv::String> fn;
	glob(pattern, fn, false);
	vector<Mat> images;
	size_t count = fn.size(); //number of png files in images folder
	for (size_t i = 0; i < count; i++) {
		Mat src = imread(fn[i]);
		copyMakeBorder(src, src, 100, 100, 100, 100, BORDER_CONSTANT, 0);
		string img_num = fn[i].substr(53, fn[i].length() - 57);
		cout << img_num << endl;
		traversal(src, img_num);
		const char *deletepath = fn[i].c_str();
		if(remove(deletepath)==0)
		{
			cout << img_num << ".png" << '\t' << "删除成功" << endl;
			cout << "---========================---" << endl;
		}
		else{
			cout << img_num << ".png" << '\t' << "删除失败" << endl;
			cout << "---========================---" << endl;
		}
	}
	return images;
}
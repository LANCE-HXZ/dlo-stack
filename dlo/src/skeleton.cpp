#include "dlo.h"
using namespace cv;
Mat glo;
int glocols;
Point left = Point(-1,0), right_ = Point(1,0), up = Point(0,-1), down = Point(0,1);
bool foundback = 0;

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
	pre_dilate(src, 3, 3); // 膨胀去除黑离群点
	// cv::imshow("pre_dilate", src);  // === 显示图片 ===
	cv::imwrite("/home/lance/Workspaces/hxz_ws/pic_buffer/S_dilate.png", src);
	// cv::waitKey();
	// cv::destroyWindow("pre_dilate");  // === 显示图片 ===
	// pre_erode(src, 3, 5); // 腐蚀去除白离群点
	// cv::imshow("pre_erode", src);  // === 显示图片 ===
	// cv::imwrite("/home/lance/Workspaces/hxz_ws/pic_buffer/S_erode.png", src);
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
// 膨胀
void pre_dilate(Mat & srcImg, int size, int times)
{
	Mat element = getStructuringElement(MORPH_CROSS, Size(size, size));
	for (size_t i = 0; i < times; i++)
	{
		dilate(srcImg, srcImg, element);
	}
}
 // ==========   Erode    ==========
void pre_erode(Mat & srcImg, int size, int times)
{
	Mat element = getStructuringElement(MORPH_CROSS, Size(size, size));
	for (size_t i = 0; i < times; i++)
	{
		erode(srcImg, srcImg, element);
	}
}// ==========   Erode    ==========

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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void thinImage(Mat & src, Mat & dst)
{
    int width = src.cols;
    int height = src.rows;
    src.copyTo(dst);
    vector<uchar *> mFlag; //用于标记需要删除的点    
    while (true)
    {
        //步骤一   
        for (int i = 0; i < height; ++i)
        {
            uchar * p = dst.ptr<uchar>(i);
            for (int j = 0; j < width; ++j)
            {
                //获得九个点对象，注意边界问题
                uchar p1 = p[j];
                if (p1 != 1) continue;
                uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
                uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
                uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
                uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
                uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
                uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);
                uchar p8 = (j == 0) ? 0 : *(p + j - 1);
                uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
				if((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) == 0)
					mFlag.push_back(p + j);
                if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)//条件1判断
                {
                    //条件2计算
                    int ap = 0;
                    if (p2 == 0 && p3 == 1) ++ap;
                    if (p3 == 0 && p4 == 1) ++ap;
                    if (p4 == 0 && p5 == 1) ++ap;
                    if (p5 == 0 && p6 == 1) ++ap;
                    if (p6 == 0 && p7 == 1) ++ap;
                    if (p7 == 0 && p8 == 1) ++ap;
                    if (p8 == 0 && p9 == 1) ++ap;
                    if (p9 == 0 && p2 == 1) ++ap;
                    //条件2、3、4判断
                    if (ap == 1 && p2 * p4 * p6 == 0 && p4 * p6 * p8 == 0)
                    {
                        //标记    
                        mFlag.push_back(p + j);
                    }
                }
            }
        }
        //将标记的点删除    
        for (vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
        {
            **i = 0;
        }
        //直到没有点满足，算法结束    
        if (mFlag.empty())
        {
            break;
        }
        else
        {
            mFlag.clear();//将mFlag清空    
        }

        //步骤二，根据情况该步骤可以和步骤一封装在一起成为一个函数
        for (int i = 0; i < height; ++i)
        {
            uchar * p = dst.ptr<uchar>(i);
            for (int j = 0; j < width; ++j)
            {
                //如果满足四个条件，进行标记    
                //  p9 p2 p3    
                //  p8 p1 p4    
                //  p7 p6 p5    
                uchar p1 = p[j];
                if (p1 != 1) continue;
                uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
                uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
                uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
                uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
                uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
                uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);
                uchar p8 = (j == 0) ? 0 : *(p + j - 1);
                uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
                if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
                {
                    int ap = 0;
                    if (p2 == 0 && p3 == 1) ++ap;
                    if (p3 == 0 && p4 == 1) ++ap;
                    if (p4 == 0 && p5 == 1) ++ap;
                    if (p5 == 0 && p6 == 1) ++ap;
                    if (p6 == 0 && p7 == 1) ++ap;
                    if (p7 == 0 && p8 == 1) ++ap;
                    if (p8 == 0 && p9 == 1) ++ap;
                    if (p9 == 0 && p2 == 1) ++ap;
                    if (ap == 1 && p2 * p4 * p8 == 0 && p2 * p6 * p8 == 0)
                    {
                        //标记    
                        mFlag.push_back(p + j);
                    }
                }
            }
        }
        //将标记的点删除    
        for (vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
        {
            **i = 0;
        }
        //直到没有点满足，算法结束    
        if (mFlag.empty())
        {
            break;
        }
        else
        {
            mFlag.clear();//将mFlag清空    
        }
    }
}

void fix_cross_error(Mat & src)
{
    int width = src.cols;
    int height = src.rows;
	src.copyTo(glo);
	glo *= 255;
	// imshow("glo", glo);
	glo /= 255;
	// moveWindow("glo", 1200, 360);
	// waitKey();
	glocols = width;
    vector<Point> error_cross;
	vector<Point> cross;
	int times = 0;
	uchar *pointer;
	while(1){
    //遍历骨骼化后的图像，找到端点和交叉点，分别放入容器中
		for (int i = 0; i < height; ++i)
		{
			uchar * p = glo.ptr<uchar>(i);
			for (int j = 0; j < width; ++j)
			{
				//获得九个点对象，注意边界问题
				uchar p1 = p[j];
				if (p1 != 1) continue;
				uchar p2 = (i == 0) ? 0 : *(p - glo.step + j);
				uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - glo.step + j + 1);
				uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
				uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + glo.step + j + 1);
				uchar p6 = (i == height - 1) ? 0 : *(p + glo.step + j);
				uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + glo.step + j - 1);
				uchar p8 = (j == 0) ? 0 : *(p + j - 1);
				uchar p9 = (i == 0 || j == 0) ? 0 : *(p - glo.step + j - 1);
				int ap = 0;
				if (p2 == 0 && p3 == 1) ++ap;
				if (p3 == 0 && p4 == 1) ++ap;
				if (p4 == 0 && p5 == 1) ++ap;
				if (p5 == 0 && p6 == 1) ++ap;
				if (p6 == 0 && p7 == 1) ++ap;
				if (p7 == 0 && p8 == 1) ++ap;
				if (p8 == 0 && p9 == 1) ++ap;
				if (p9 == 0 && p2 == 1) ++ap;
				if (ap == 3)
				{
					error_cross.push_back(Point(j, i));
				}
				if(ap == 4)		cross.push_back(Point(j, i));
			}
		}
		if(error_cross.size() < 2)	break;
		// vector<bool> pair[error_cross.size()] = {0};
		int opt_index = times%2;
		int min = 50; int ni = -1;
		while(1){
			for(int k = 0; k < error_cross.size(); ++k){
				int xy_dir = abs(error_cross[k].x - error_cross[opt_index].x) + abs(error_cross[k].y - error_cross[opt_index].y);
				// cout << xy_dir << endl;
				if(min >= xy_dir && xy_dir){
					min = xy_dir;
					ni = k;
				}
			}
			cout << "ni: " << ni << endl;
			if(ni != -1)	break;//////////////////////ni=-1error
			else	++opt_index;
		}
		cout << "error_cross: " << error_cross[opt_index] << "  -  " << error_cross[ni] << endl;
		Point fixdir = (error_cross[ni] - error_cross[opt_index]);
		cout << "fixdir: " << fixdir << endl;
		
		int xdir = fixdir.x?fixdir.x/abs(fixdir.x):0; // 确定x搜索方向
		int ydir = fixdir.y?fixdir.y/abs(fixdir.y):0; // 确定y搜索方向
		cout << "xdir: " << xdir << '\t' << "ydir: " << ydir << endl;
		
		Point fixstep = fixdir/2;  // 每次跳步距离
		if(!fixstep.x && fixdir.x)	fixstep.x = fixdir.x;
		if(!fixstep.y && fixdir.y)	fixstep.y = fixdir.y;
		cout << "fixstep: " << fixstep << endl;
		
		Point opt_p = error_cross[opt_index];
			pointer = glo.ptr<uchar>(opt_p.y);
			pointer[d(opt_p, fixstep)] = 1;		// 移动操作点
			Point opt1, opt2, flag1, flag2;
			int step1, step2;
			bool found1 = 0, found2 = 0;
			if(fixdir.x && fixdir.y){
				// if(pointer[d(opt_p, Point(0, ydir))]){
				// 	found1 = 1;
				// 	opt1 = Point(0, ydir);
				// 	step1 = abs(fixstep.y) - 1;
				// }
				// else if(pointer[d(opt_p, Point(xdir, 0))]){
				// 	found2 = 1;
				// 	opt2 = Point(xdir, 0);
				// 	step2 = abs(fixstep.x) - 1;
				// }
				for(int i = 0; i < abs(fixstep.x); ++i){
					for(int j = 0; j < abs(fixstep.y); ++j){
						pointer[d(opt_p, Point(i*xdir, j*ydir))] = 0;
						pointer[d(opt_p, Point(fixstep.x, j*ydir))] = 1;
					}
					pointer[d(opt_p, Point(i*xdir, fixstep.y))] = 1;
				}
				// if(found1){
				// 	pointer[d(opt_p, Point(0, ydir))] = 1;
				// 	pointer[d(opt_p, Point(0, 0))] = 1;
				// }
				// else if(found2){
				// 	pointer[d(opt_p, Point(xdir, 0))] = 1;
				// 	pointer[d(opt_p, Point(0, 0))] = 1;
				// }
				for(int i = 1; i >= -1; --i){
					if(!found1 && pointer[d(opt_p, Point(-xdir, i*ydir))]){  // 沿y向
						opt1 = Point(-xdir, i*ydir);
						step1 = abs(fixstep.y);
						found1 = 1;				
					}
					if(!found2 && pointer[d(opt_p, Point(i*xdir, -ydir))]){  // 沿x向
						opt2 = Point(i*xdir, -ydir);
						step2 = abs(fixstep.x);
						found2 = 1;
					}
				}
				if(!found1){
					step1 = abs(fixstep.y)-1;
					for(int i = 2; i < 10; ++i){
						if(pointer[d(opt_p, Point(-xdir, i*ydir))]){
							opt1 = Point(-xdir, i*ydir);
							step1 = abs(fixstep.y);
							found1 = 1;			
						}
						--step1;
					}
					if(!found1)
						cout << "Error opt1 in y direction" << endl;
				}
				if(!found2){
					step2 = abs(fixstep.x)-1;
					for(int i = 2; i < 10; ++i){
						if(pointer[d(opt_p, Point(i*xdir, -ydir))]){
							opt2 = Point(i*xdir, -ydir);
							step2 = abs(fixstep.x);
							found2 = 1;	
						}
						--step2;
					}
					if(!found2)
						cout << "Error opt1 in x direction" << endl;
				}
				if(step2>0)
					move(opt_p, opt2, step2, pointer, Point(xdir, 0), xdir, ydir);  // 递归函数
				cout << opt_p << " " << opt2 << " " << step2 << " " << Point(xdir, 0) << " " << xdir << " " << ydir << endl;
				if(step1>0)
					move(opt_p, opt1, step1, pointer, Point(0, ydir), xdir, ydir);
			}
			else if(fixdir.x){
				if(pointer[d(opt_p, Point(-xdir, 0))])	{
					opt1 = Point(-xdir, 0);
					found1 = 1;
				}
				else{
					for(int i = 0; i < abs(fixstep.x); ++i){
						pointer[d(opt_p, Point(i*xdir, 0))] = 0;
					}
				}
				if(!found1){
					if(pointer[d(opt_p, Point(0, -1))])	opt1 = Point(0, -1);
					else if(pointer[d(opt_p, Point(-xdir, -1))])	opt1 = Point(-1, -1);
				}
				if(pointer[d(opt_p, Point(0, 1))])	opt2 = Point(0, 1);
				else if(pointer[d(opt_p, Point(-xdir, 1))])	opt2 = Point(-xdir, 1);
				else if(pointer[d(opt_p, Point(0, -1))])	opt2 = Point(0, -1);
				else if(pointer[d(opt_p, Point(-xdir, -1))])	opt2 = Point(-xdir, -1);
				move(opt_p, opt1, abs(fixstep.x), pointer, Point(xdir, 0), xdir, 1);
				move(opt_p, opt2, abs(fixstep.x), pointer, Point(xdir, 0), xdir, -opt2.y);
			}
			else if(fixdir.y){
				if(pointer[d(opt_p, Point(0, -ydir))])	{
					opt1 = Point(0, -ydir);
					found1 = 1;
					foundback = 1;
				}
				else{
					for(int i = 0; i < abs(fixstep.y); ++i){
						pointer[d(opt_p, Point(0, i*ydir))] = 0;
					}
				}
				if(!found1){
					if(pointer[d(opt_p, Point(-1, 0))])	opt1 = Point(-1, 0);
					else if(pointer[d(opt_p, Point(-1, -ydir))])	opt1 = Point(-1, -ydir);
				}
				if(pointer[d(opt_p, Point(1, 0))])	opt2 = Point(1, 0);
				else if(pointer[d(opt_p, Point(1, -ydir))])	opt2 = Point(1, -ydir);
				else if(pointer[d(opt_p, Point(-1, 0))])	opt2 = Point(-1, 0);
				else if(pointer[d(opt_p, Point(-1, -ydir))])	opt2 = Point(-1, -ydir);
				move(opt_p, opt1, abs(fixstep.y), pointer, Point(0, ydir), 1, ydir);
				move(opt_p, opt2, abs(fixstep.y), pointer, Point(0, ydir), -opt2.x, ydir);
			}
		circle(src, error_cross[opt_index], 5, Scalar(255));
		circle(src, error_cross[ni], 5, Scalar(255));
		src*=255;
		imshow("src", src);
		moveWindow("src", 840, 740);
		glo *= 255;
		// imshow("glo", glo);
		glo /= 255;
		// moveWindow("glo", 840, 360);
		waitKey();
		error_cross.clear();  
		cross.clear();
		glo.copyTo(src);
		thinImage(src, glo);
		++times; 
		cout << endl;
	}
	glo *= 255;
	// imshow("glo", glo);
	glo /= 255;
	// moveWindow("glo", 800, 840);
	waitKey();
}

void move(Point opt_p, Point opt, int step, uchar *pointer, Point dirflag, int xdir, int ydir){
	pointer[d(opt_p, opt)] = 0; 					//
	pointer[d(opt_p, opt + step*dirflag)] = 1;  	// 移动opt
	bool foundnext = 0;
	if(pointer[d(opt_p, opt - dirflag)]){
		pointer[d(opt_p, opt + (step-1)*dirflag)] = 1;
		opt -= dirflag;
		cout << "反向: " << opt_p << " " << opt << " " << step-1 << " " << dirflag << " " << xdir << " " << ydir << endl;
		if(--step > 0)
			move(opt_p, opt, step, pointer, dirflag, xdir, ydir);
	}
	else if(dirflag.x && (pointer[d(opt_p, opt + Point(xdir, -ydir))] || pointer[d(opt_p, opt + Point(0, -ydir))])){
		opt = pointer[d(opt_p, opt + Point(xdir, -ydir))] ? opt + Point(xdir, -ydir) : opt + Point(0, -ydir);
		cout << "上横前角/垂向: "<< opt_p << " " << opt << " " << step-1 << " " << dirflag << " " << xdir << " " << ydir << endl;
		if(--step > 0)
			move(opt_p, opt, step, pointer, dirflag, xdir, ydir);
	}
	else if(dirflag.y && (pointer[d(opt_p, opt + Point(-xdir, ydir))] || pointer[d(opt_p, opt + Point(-xdir, 0))])){
		opt = pointer[d(opt_p, opt + Point(-xdir, ydir))] ? opt + Point(-xdir, ydir) : opt + Point(-xdir, 0);
		cout << "纵前角/垂向: "<< opt_p << " " << opt << " " << step-1 << " " << dirflag << " " << xdir << " " << ydir << endl;
		if(--step > 0)
			move(opt_p, opt, step, pointer, dirflag, xdir, ydir);
	}
	else if(pointer[d(opt_p, opt + Point(-xdir, -ydir))]){
		opt += Point(-xdir, -ydir);
		cout << "后角 "<< opt_p << " " << opt << " " << step-1 << " " << dirflag << " " << xdir << " " << ydir << endl;
		if(step > 0)
			move(opt_p, opt, step, pointer, dirflag, xdir, ydir);
	}
	else if(dirflag.x && (pointer[d(opt_p, opt + Point(xdir, ydir))] || pointer[d(opt_p, opt + Point(0, ydir))])){
		opt = pointer[d(opt_p, opt + Point(xdir, ydir))] ? opt + Point(xdir, ydir) : opt + Point(0, ydir);
		cout << "上横前角/垂向: "<< opt_p << " " << opt << " " << step-1 << " " << dirflag << " " << xdir << " " << -ydir << endl;
		if(--step > 0)
			move(opt_p, opt, step, pointer, dirflag, xdir, ydir);
	}
	else if(dirflag.y && (pointer[d(opt_p, opt + Point(xdir, ydir))] || pointer[d(opt_p, opt + Point(xdir, 0))])){
		opt = pointer[d(opt_p, opt + Point(xdir, ydir))] ? opt + Point(xdir, ydir) : opt + Point(xdir, 0);
		cout << "纵前角/垂向: "<< opt_p << " " << opt << " " << step-1 << " " << dirflag << " " << -xdir << " " << ydir << endl;
		if(--step > 0)
			move(opt_p, opt, step, pointer, dirflag, xdir, ydir);
	}
	else if(dirflag.x && pointer[d(opt_p, opt + Point(-xdir, ydir))]){
		opt += Point(-xdir, ydir);
		cout << "后角 "<< opt_p << " " << opt << " " << step-1 << " " << dirflag << " " << xdir << " " << -ydir << endl;
		if(step > 0)
			move(opt_p, opt, step, pointer, dirflag, xdir, ydir);
	}
	else if(dirflag.y && pointer[d(opt_p, opt + Point(xdir, -ydir))]){
		opt += Point(xdir, -ydir);
		cout << "后角 "<< opt_p << " " << opt << " " << step-1 << " " << dirflag << " " << -xdir << " " << ydir << endl;
		if(step > 0)
			move(opt_p, opt, step, pointer, dirflag, xdir, ydir);
	}
}

int d(Point opt, Point dir){
	return opt.x+dir.x+dir.y*glocols;
}


void endPointAndintersectionPointDetection(Mat & src, vector<Point> &endpoint)
{
    int width = src.cols;
    int height = src.rows;
    // vector<CvPoint> endpoint;
    vector<CvPoint> intersectionPoint;
    //遍历骨骼化后的图像，找到端点和交叉点，分别放入容器中
    for (int i = 0; i < height; ++i)
    {
        uchar * p = src.ptr<uchar>(i);
        for (int j = 0; j < width; ++j)
        {
            //获得九个点对象，注意边界问题
            uchar p1 = p[j];
            if (p1 != 1) continue;
            uchar p2 = (i == 0) ? 0 : *(p - src.step + j);
            uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - src.step + j + 1);
            uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
            uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + src.step + j + 1);
            uchar p6 = (i == height - 1) ? 0 : *(p + src.step + j);
            uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + src.step + j - 1);
            uchar p8 = (j == 0) ? 0 : *(p + j - 1);
            uchar p9 = (i == 0 || j == 0) ? 0 : *(p - src.step + j - 1);

            // if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) == 1)//端点判断
            // {
			// 	int sum = 0, e = 4;
			// 	Point opt_p = Point(j, i);
			// 	uchar * pointer = glo.ptr<uchar>(opt_p.y);
			// 	for(int m = -e; m <= e; ++m){
			// 		for(int n = -e; n <= e; ++n){
			// 			sum += pointer[d(opt_p, Point(m, n))];
			// 		}
			// 	}
			// 	if(sum <= 2*e-1){
			// 		printf("端点：%d %d\n", j, i);
			// 		endpoint.push_back(cvPoint(j, i));
			// 	}
            // }
			if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) == 1)//端点判断
            {
				printf("端点：%d %d\n", j, i);
				endpoint.push_back(Point(j, i));
            }
            else //交叉点判断
            {
                int ap = 0;
                if (p2 == 0 && p3 == 1) ++ap;
                if (p3 == 0 && p4 == 1) ++ap;
                if (p4 == 0 && p5 == 1) ++ap;
                if (p5 == 0 && p6 == 1) ++ap;
                if (p6 == 0 && p7 == 1) ++ap;
                if (p7 == 0 && p8 == 1) ++ap;
                if (p8 == 0 && p9 == 1) ++ap;
                if (p9 == 0 && p2 == 1) ++ap;
                if (ap > 3)
                {
                    printf("交叉点：%d %d\n", j, i);
                    intersectionPoint.push_back(cvPoint(j, i));
                }
				if (p2 && p3 && p4){
					printf("交叉点：%d %d\n", j, i);
                    intersectionPoint.push_back(cvPoint(j, i));
				}
            }
        }
    }
    // //画出端点
    // for (vector<Point>::iterator i = endpoint.begin(); i != endpoint.end(); ++i)
    // {
    //     circle(src, Point(i->x, i->y), 5, Scalar(255), -1);
    // }
    // //画出交叉点
    // for (vector<CvPoint>::iterator i = intersectionPoint.begin(); i != intersectionPoint.end(); ++i)
    // {
    //     circle(src, cvPoint(i->x, i->y), 5, Scalar(255));
    // }
    // endpoint.clear();//数据回收 
    intersectionPoint.clear();   
}
////////////////////////////////////////////////////////////////////////////////////////////////////////

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

// ==========    黑白反置    ==========
void bw_change(Mat &src){
	int r = src.rows;
	int c = src.cols;
	for(int i = 0; i < r; ++i){
		for(int j = 0; j < c; ++j){
			src.at<uchar>(i, j) = !src.at<uchar>(i, j);
		}
	}
}
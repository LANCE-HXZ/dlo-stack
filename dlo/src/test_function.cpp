#include "dlo.h"
using namespace cv;
vector<Point> whiteptset;

// ros::NodeHandle node_handle; 
//   int a;
//   node_handle.setParam("qiangqiang", 3);
//   node_handle.getParam("qiangqiang", a);
//   cout << a << endl;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Mat glo;
int glocols;
Point left = Point(-1,0), right_ = Point(1,0), up = Point(0,-1), down = Point(0,1);
bool foundback = 0;

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
	// // imwrite("src/dlo/data/cableonly_img.bmp", cableonly_img);
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
// 	cv::String pattern = "src/dlo/python/mynet/checkpoint/S*";
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
// 				// imwrite("src/dlo/data/cableonly_img.bmp", cableonly_img);
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
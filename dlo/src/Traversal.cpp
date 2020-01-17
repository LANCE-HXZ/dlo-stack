#include "pch.h"
using namespace cv;
using namespace std;

Mat src, dst, show, RGBimg;
int col;
char temp_1[20];
string img_num;
//int cross_area[] = {411, 59, 464, 107, 
//					331, 112, 384, 154, 
//					446, 119, 498, 177, 
//					336, 224, 391, 275};
//Point center[sizeof(cross_area)];


 // ==========    Main    ==========
void traversal(Mat src, string imgnum)
{
	img_num = imgnum;
	string rgb_address = "/media/lance/新加卷/Desktop/Project/test/RGBimg/" + img_num + ".bmp";
	RGBimg = imread(rgb_address);
	copyMakeBorder(RGBimg, RGBimg, 100, 100, 100, 100, BORDER_CONSTANT, 0);
	/*for (int sqr_num = 0; sqr_num < sizeof(cross_area) / 16; sqr_num++)
	{
		center[sqr_num] = Point((cross_area[4 * sqr_num] + cross_area[4 * sqr_num + 2]) / 2,
								(cross_area[4 * sqr_num + 1] + cross_area[4 * sqr_num + 3]) / 2);
	}*/
	//注意：这一步必须要有，不然进行不了鼠标操作
	namedWindow("src", WINDOW_AUTOSIZE);//WINDOW_AUTOSIZE:系统默认,显示自适应
	src.copyTo(dst);
	//cout << cross_area[0] << endl << cross_area[1];
	setMouseCallback("src", traversal_callback, 0);
	//setMouseCallback("src", get_point, 0);
	imshow("src", src);
	waitKey();
}
void traversal(Mat src)
{
	/*for (int sqr_num = 0; sqr_num < sizeof(cross_area) / 16; sqr_num++)
	{
		center[sqr_num] = Point((cross_area[4 * sqr_num] + cross_area[4 * sqr_num + 2]) / 2,
								(cross_area[4 * sqr_num + 1] + cross_area[4 * sqr_num + 3]) / 2);
	}*/
	copyMakeBorder(src, src, 100, 100, 100, 100, BORDER_CONSTANT, 0);
	//注意：这一步必须要有，不然进行不了鼠标操作
	namedWindow("src", WINDOW_AUTOSIZE);//WINDOW_AUTOSIZE:系统默认,显示自适应
	src.copyTo(dst);
	//cout << cross_area[0] << endl << cross_area[1];
	setMouseCallback("src", traversal_callback, 0);
	//setMouseCallback("src", get_point, 0);
	imshow("src", src);
	waitKey();
}// ==========    Main    ==========

 // ======= 鼠标事件回调函数 =======
void traversal_callback(int event, int x, int y, int flags, void* ustc)
{
	static int center_num = 0, L_down_num = 0;
	static Point center[20];
	if (event == CV_EVENT_RBUTTONDOWN)
	{
		center[center_num] = Point(x, y);
		center_num++;
		std::cout << center_num << "  " << center[center_num - 1] << endl;
	}

	if (event == CV_EVENT_LBUTTONDOWN)
	{
		cout << "Start - ";
		L_down_num++;
		clock_t now, start, end; // time
		start = clock(); // time
		Point pre_pt, curr_pt, next_pt;
		Point pt[250];
		dst.copyTo(src);
		pre_pt = Point(x, y);
		sprintf(temp_1, "x:%d,y:%d", x, y);
		pt[0] = pre_pt;
		col = src.cols * 3; int row = src.rows;
		int per_step = 5; int smaller_cross = 15; int jump_step = 4; 
		int crop_size = 50; int dir_size = 20; 
		int point_num = 1;
		int ddx, ddy; float d;
		bool incross[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		bool crossing[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		curr_pt = get_first_current_point(pre_pt, per_step);
		pt[point_num] = curr_pt;
		for (int i = 0; i < 250; i++){
			point_num++;
			//sprintf_s(temp_1, "%d", point_num);
			//putText(src, temp_1, curr_pt, FONT_HERSHEY_SIMPLEX, 0.1, Scalar(150, 55, 245));
			int dx = curr_pt.x - pre_pt.x; int dy = curr_pt.y - pre_pt.y;
			int nx = curr_pt.x + dx; int ny = curr_pt.y + dy;
			// 旋转裁剪
			//for (int j = 0; j < sizeof(cross_area) / 16; j++){
			for (int j = 0; j < center_num; j++) {
				if (in_square(center[j], nx, ny, dir_size)){
					if (!incross[j]){
						double angle = -atan(dx / (dy+0.00000001)) * 180 / 3.14159 - 180;
						if (dy < 0) angle += 180;
						Mat rotation = RotateImage(RGBimg, angle, Point(curr_pt.x, curr_pt.y));
						cv::Rect select = cv::Rect(curr_pt.x - crop_size/2, curr_pt.y - crop_size, crop_size, crop_size);
						Mat croped = rotation(select);
						string nj = to_string(j); string ni = to_string(i); 
						now = clock(); string nt = to_string(now);
						string dir_savecroped = "/media/lance/新加卷/Desktop/Project/test/" + img_num + "_" + nt + "_" + ni + "_" + nj + ".png";
						//cout << dir_savecroped << endl;
						imwrite(dir_savecroped, croped);
						incross[j] = 1;
						break;
					}
				}
				else 
					{incross[j] = 0;}
			}
			//遇到交叉点
			//for (int j = 0; j < sizeof(cross_area) / 16; j++) {
			for (int j = 0; j < center_num; j++) {
				if (in_square(center[j], nx, ny, dir_size)) {
					if (!crossing[j]) {
						next_pt = get_opposite_point(center[j], curr_pt.x, curr_pt.y);
						pre_pt = curr_pt; curr_pt = next_pt;
						pt[point_num] = curr_pt;
						point_num++;
						dx = curr_pt.x - pre_pt.x; dy = curr_pt.y - pre_pt.y;
						if (abs(dx) >= abs(dy))
						{
							ddx = per_step * dx / abs(dx);
							d = float(dx) / float(ddx);
							ddy = dy / d;
						}
						else
						{
							ddy = per_step * dy / abs(dy);
							d = float(dy) / float(ddy);
							ddx = dx / d;
						}
						dx = ddx; dy = ddy;
						cout << j << " - ";
						crossing[j] = 1;
						break;
					}
				}
				else{
					crossing[j] = 0;
				}
			}

			/*for (int j = 0; j < sizeof(cross_area) / 16; j++){
				if ((nx > cross_area[4 * j] + smaller_cross) && (nx < cross_area[4 * j + 2] - smaller_cross)
					&& (ny > cross_area[4 * j + 1] + smaller_cross) && (ny < cross_area[4 * j + 3] - smaller_cross)){
					for (int k = 0; k < 1; k++){
						next_pt = get_next_point(curr_pt, dx, dy, per_step);
						if (next_pt.x == curr_pt.x && next_pt.y == curr_pt.y) 
							{break;}
						pre_pt = curr_pt; curr_pt = next_pt;
						pt[point_num] = curr_pt;
						point_num++;
						dx = curr_pt.x - pre_pt.x; dy = curr_pt.y - pre_pt.y;
					}
					curr_pt = get_next_point(curr_pt, jump_step*dx, jump_step*dy, jump_step*per_step);
					pt[point_num] = curr_pt;
					point_num++;
					cout << "Cross_area" << j << endl;
				}
			}*/
			next_pt = get_next_point(curr_pt, dx, dy, per_step);
			if (next_pt.x == curr_pt.x && next_pt.y == curr_pt.y) 
			{
				break;
			}
			pre_pt = curr_pt; curr_pt = next_pt;
			pt[point_num] = curr_pt;
			//circle(src, curr_pt, 3, Scalar(255, 255, 0), -1); // 画圆(图, 坐标, 半径, 颜色, -1表示实心)
			imshow("src", src);
		}
		if (L_down_num == 2) {
			center_num = 0;
			L_down_num = 0;
		}
		for (int n = 0; n < point_num; n++){
			circle(src, pt[n], 3, Scalar(255, 2*n, 0), -1); // 画圆(图, 坐标, 半径, 颜色, -1表示实心)
			imshow("show", src);

		}
		//src.copyTo(dst);//确保画线操作是在src上进行
		end = clock(); // time
		double endtime = (double)(end - start) / CLOCKS_PER_SEC; // time
		//cout << endtime << endl; // time
	}
} // ======= 鼠标事件回调函数 =======

 // = 用相对curr_pt的dx,dy定义指针 =
int dxdy2currentpoint(Point curr_pt, int dx, int dy)
{
	uchar *pointer = src.ptr<uchar>(curr_pt.y);
	return pointer[3 * (curr_pt.x + dx) + col * dy + 1];
}// = 用相对curr_pt的dx,dy定义指针 =

bool in_square(Point center, int nx, int ny, int sqr_size)
{
	bool if_in_sqr = false;
	if ((nx > center.x - sqr_size) && (nx < center.x + sqr_size) && (ny > center.y - sqr_size) && (ny < center.y + sqr_size))
	{
		if_in_sqr = true;
	}
	return if_in_sqr;
}

 // =========    寻找起点   =========
Point get_first_current_point(Point pre_pt, int per_step)
{
	bool endfind = 0;
	Point curr_pt;
	int dx = per_step; int dy = per_step;
	for (dx; dx > -per_step; dx--)
	{
		if (dxdy2currentpoint(pre_pt, dx, dy) == 255)
		{
			curr_pt = Point(pre_pt.x + dx, pre_pt.y + dy);
			endfind = 1;
			break;
		}
	}
	if (!endfind)
	{
		for (dy; dy > -per_step; dy--)
		{
			if (dxdy2currentpoint(pre_pt, dx, dy) == 255)
			{
				curr_pt = Point(pre_pt.x + dx, pre_pt.y + dy);
				endfind = 1;
				break;
			}
		}
	}
	if (!endfind)
	{
		for (dx; dx < per_step; dx++)
		{
			if (dxdy2currentpoint(pre_pt, dx, dy) == 255)
			{
				curr_pt = Point(pre_pt.x + dx, pre_pt.y + dy);
				endfind = 1;
				break;
			}
		}
	}
	if (!endfind)
	{
		for (dy; dy < per_step; dy++)
		{
			if (dxdy2currentpoint(pre_pt, dx, dy) == 255)
			{
				curr_pt = Point(pre_pt.x + dx, pre_pt.y + dy);
				endfind = 1;
				break;
			}
		}
	}
	if (!endfind)
		cout << "Error starting point" << endl;
	return curr_pt;
}// =========   寻找起点   =========

 // =========   寻找下一点  =========
Point get_next_point(Point curr_pt, int dx, int dy, int per_step)
{
	int search_times = 2 * per_step;
	Point dir_s, dir_n, next_pt = Point(curr_pt.x + dx, curr_pt.y + dy);
	Point s_pt = next_pt; Point n_pt = next_pt;
	for (int i = 0; i < search_times; i++)
	{
		if (dxdy2currentpoint(s_pt, 0, 0) == 255)
		{
			next_pt = s_pt;
			break;
		}
		if (dxdy2currentpoint(n_pt, 0, 0) == 255)
		{
			next_pt = n_pt;
			break;
		}
		int dx_s = (s_pt.x - curr_pt.x); int dy_s = (s_pt.y - curr_pt.y);
		int dx_n = (n_pt.x - curr_pt.x); int dy_n = (n_pt.y - curr_pt.y);
		if (abs(dx_s) == abs(dy_s))
		{
			dir_s.x = -(dx_s + dy_s) / (2 * per_step);
			dir_s.y = (dx_s - dy_s) / (2 * per_step);
		}
		else
		{
			dir_s.x = -dy_s / per_step;
			dir_s.y = dx_s / per_step;
		}
		if (abs(dx_n) == abs(dy_n))
		{
			dir_n.x = -(dx_n - dy_n) / (2 * per_step);
			dir_n.y = -(dx_n + dy_n) / (2 * per_step);
		}
		else
		{
			dir_n.x = dy_n / per_step;
			dir_n.y = -dx_n / per_step;
		}
		s_pt += dir_s;
		n_pt += dir_n;
		if (i == search_times - 1)
		{
			cout << "End" << endl;
			//circle(src, curr_pt, 3, Scalar(0, 255, 255), -1);
			imshow("src", src);
			next_pt = curr_pt;
		}
	}
	return next_pt;
}// =========   寻找下一点  =========

 // ===   跳过交叉点寻找对面的点  ===
Point get_opposite_point(Point center, int nx, int ny)
{
	int dx_s = nx - center.x, dy_s = ny - center.y;
	int half_sqr_size = (abs(dx_s) > abs(dy_s)) ? abs(dx_s) : abs(dy_s);
	int search_times = 8 * half_sqr_size, count = 0, last_i = 0, pass_length = 5;
	Point dir_s, next_pt = Point(nx, ny);
	Point s_pt = Point(nx, ny);
	for (int i = 0; i < search_times - pass_length; i++) {
		if (dxdy2currentpoint(s_pt, 0, 0) == 255) {
			if (i - last_i > pass_length)
			{
				count++;
			}
			if (count == 2) {
				next_pt = s_pt;
				//break;
			}
			last_i = i;
		}
		dx_s = (s_pt.x - center.x); dy_s = (s_pt.y - center.y);
		if (abs(dx_s) == abs(dy_s)) {
			dir_s.x = -(dx_s + dy_s) / (2 * half_sqr_size);
			dir_s.y = (dx_s - dy_s) / (2 * half_sqr_size);
		}
		else {
			dir_s.x = -dy_s / half_sqr_size;
			dir_s.y = dx_s / half_sqr_size;
		}
		s_pt += dir_s;
		/*if (i == search_times - 1) {
			cout << "Ending point" << endl;
			circle(src, center, 3, Scalar(0, 255, 255), -1);
			imshow("src", src);
			next_pt = center;
		}*/
	}
	return next_pt;
}

 // ====   获得鼠标点击点的坐标  ====
void get_point(int event, int x, int y, int flags, void* ustc)
{
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		Point pre_pt, curr_pt, next_pt;
		dst.copyTo(src);
		pre_pt = Point(x, y);
		sprintf(temp_1, "x:%d,y:%d", x, y);
		circle(src, pre_pt, 3, Scalar(0, 255, 0), -1); // 画圆(图, 坐标, 半径, 颜色, -1表示实心)
		cout << x << "," << y << endl;
	}
}// ====   获得鼠标点击点的坐标  ====
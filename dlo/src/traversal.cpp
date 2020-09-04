#include "dlo.h"
#include "dlo_global.h"
using namespace cv;
using namespace std;

Mat src, dst, RGBimg;
int col;
string img_num;
vector<Point> pt, endpoint, cross, dir;
vector<int> cpt, ept, start;
int point_num = 0, cpt_num = 0, ept_num = 0, line_num = 0;
//  ==========    Main    ==========
vector<string> traversal(string thin_img_address, Mat rgb_img, dlo::BoundingBoxes::ConstPtr boxes)
{
	pt.clear(); endpoint.clear(); cross.clear(); dir.clear(); cpt.clear(); ept.clear(); start.clear();
	vector<string> crop_file_name;
	int expand = 0, save_index = 0; // 图像边缘扩大尺寸/保存时的序号按遍历顺序
	int end_num = 0, cross_num = 0; // 端点和交叉点计数
	bool end_used[10] = {0}, incross[20] = {0}, crossing[20] = {0}; // 一些判断条件(端点已用/进入交叉点/在交叉点内)
	point_num = 0; cpt_num = 0; ept_num = 0; line_num = 0;
	for(int i=0; i<boxes->bounding_boxes.size(); ++i)
	{
		if("endpoint" == boxes->bounding_boxes[i].Class)
		{
			endpoint.push_back(Point(boxes->bounding_boxes[i].xmin+expand, boxes->bounding_boxes[i].ymin+expand));
			std::cout << '\t' << "Endpoint" << end_num << '\t' << "[" << endpoint[end_num].x-expand << ", " << endpoint[end_num].y-expand << "]" << endl;
			end_num++;
		}
		else if("cross" == boxes->bounding_boxes[i].Class)
		{
			cross.push_back(Point(boxes->bounding_boxes[i].xmin+expand, boxes->bounding_boxes[i].ymin+expand));
			std::cout << '\t' << "Cross" << cross_num << '\t' << '\t' << "[" << cross[cross_num].x-expand << ", " << cross[cross_num].y-expand << "]" << endl;
			cross_num++;
		}
		else
		{
			cout << "ERROR: UNKNOWN BOXES CLASS" << endl;
		}
		
	}

	if(0 == end_num)	{cout << "ERROR: NO ENDPOINT DETECTED" << endl; return crop_file_name;}
	else if(0 != end_num%2)	{cout << "ERROR: ENDPOINTS TOTAL NUMBER ERROR: " << end_num << endl;}
	Mat thin_img = readImg(thin_img_address);
	convertTo3Channels(thin_img);
	copyMakeBorder(thin_img, dst, expand, expand, expand, expand, BORDER_CONSTANT, 0);
	copyMakeBorder(rgb_img, rgb_img, expand, expand, expand, expand, BORDER_CONSTANT, 0);
	RGBimg = rgb_img;
	
	for(int e=0; e<2*end_num; ++e)
	{
		int end_index; bool no_used_end = 0;
		for(int i=0; i<end_num; ++i)
		{
			if(end_used[i])
				continue;
			else
				end_index = i;
				end_used[i] = 1;
				no_used_end = 1;
				break;
		}
		if(!no_used_end){break;}
		cout << '\t' << "Start - ";
		// clock_t now, start, end; // time
		// start = clock(); // time
		Point pre_pt, curr_pt, next_pt;
		dst.copyTo(src);
		pre_pt = endpoint[end_index];
		ept.push_back(point_num);
		ept_num++;
		start.push_back(cpt_num);
		line_num++;
		pt.push_back(pre_pt);
		dir.push_back(Point(0, 0));
		point_num++;
		// pt[point_num++] = pre_pt;
		col = src.cols * 3; int row = src.rows;
		int per_step = 5; int smaller_cross = 15; int jump_step = 4; 
		int crop_size = 50; int dir_size = 20;  // 一些遍历参数
		
		int ddx, ddy; float d;
		curr_pt = get_first_current_point(src, pre_pt, 2*per_step, 3, 3);
		pt.push_back(curr_pt);
		point_num++;
		for (int i = 0; i < 500; i++){
			Point curr_dir = per_dir(pre_pt, curr_pt, per_step);
			int dx = curr_dir.x; int dy = curr_dir.y;
			dir.push_back(curr_dir);
			int nx = curr_pt.x + dx; int ny = curr_pt.y + dy;
			// 旋转裁剪
			for (int j = 0; j < cross_num; j++) {
				if (in_square(cross[j], nx, ny, dir_size)){  //====
					if (!incross[j]){
						cpt.push_back(point_num);
						cpt_num++;
						double angle = -atan(dx / (dy+0.00000001)) * 180 / 3.14159 - 180;
						if (dy < 0) angle += 180;
						Mat rotation = RotateImage(RGBimg, angle, Point(curr_pt.x, curr_pt.y));
						cv::Rect select = cv::Rect(curr_pt.x - crop_size/2, curr_pt.y - crop_size, crop_size, crop_size);
						Mat croped = rotation(select);
						string str_j = to_string(j); string str_save_index = to_string(save_index++); 
						string dir_savecroped = "pic_buffer/C/" + str_save_index + "_" + str_j + ".png";
						crop_file_name.push_back(dir_savecroped);
						imwrite(dir_savecroped, croped);
						incross[j] = 1;
						break;
					}
				}
				else 
					{incross[j] = 0;}
			}
			//遇到交叉点跳跃
			for (int j = 0; j < cross_num; j++) {
				if (in_square(cross[j], nx, ny, dir_size)) {
					if (!crossing[j]) {
						next_pt = get_opposite_point(cross[j], curr_pt.x, curr_pt.y);
						pre_pt = curr_pt; curr_pt = next_pt;
						pt.push_back(curr_pt);
						Point curr_dir = per_dir(pre_pt, curr_pt, per_step);
						dx = curr_dir.x; dy = curr_dir.y;
						dir.push_back(curr_dir);
						point_num++;
						cout << j << " - ";
						crossing[j] = 1;
						break;
					}
				}
				else{
					crossing[j] = 0;
				}
			}
			next_pt = get_next_point(curr_pt, dx, dy, per_step);
			if (next_pt.x == curr_pt.x && next_pt.y == curr_pt.y) 
			{
				ept.push_back(point_num);
				ept_num++;
				start.push_back(cpt_num-1);
				pt.push_back(curr_pt);
				dir.push_back(Point(0, 0));
				point_num++;
				for (int j = 0; j < end_num; j++) {
					if (in_square(endpoint[j], curr_pt.x, curr_pt.y, dir_size)){
						end_used[j] = 1;
						break;
					}
					else if(in_square(endpoint[j], curr_pt.x+5*dx, curr_pt.y+5*dy, dir_size)){
						end_used[j] = 1;
						break;
					}
					else if(in_square(endpoint[j], curr_pt.x+9*dx, curr_pt.y+9*dy, dir_size)){
						end_used[j] = 1;
						break;
					}
					else if(in_square(endpoint[j], curr_pt.x+13*dx, curr_pt.y+13*dy, dir_size)){
						end_used[j] = 1;
						break;
					}
					else if(in_square(endpoint[j], curr_pt.x+17*dx, curr_pt.y+17*dy, dir_size)){
						end_used[j] = 1;
						break;
					}
				}
				break;
			}
			pre_pt = curr_pt; curr_pt = next_pt;
			pt.push_back(curr_pt);
			point_num++;
		}
	}

	src = traversal_visualization(src);
	Mat tvs = src(cv::Rect(expand, expand, thin_img.cols, thin_img.rows));
	// namedWindow("Traversal", WINDOW_AUTOSIZE);  // === 显示图片 ===
	// imshow("Traversal", tvs);
	// waitKey();
	// cv::destroyWindow("Traversal");  // === 显示图片 ===
	cv::imwrite("pic_buffer/6_T.png", src);	
	return crop_file_name;
}

void traversal(string thin_img_address, Mat rgb_img)
{
	Mat thin_img = readImg(thin_img_address);
	copyMakeBorder(thin_img, dst, 100, 100, 100, 100, BORDER_CONSTANT, 0);
	copyMakeBorder(rgb_img, rgb_img, 100, 100, 100, 100, BORDER_CONSTANT, 0);
	RGBimg = rgb_img;
	/*for (int sqr_num = 0; sqr_num < sizeof(cross_area) / 16; sqr_num++)
	{
		center[sqr_num] = Point((cross_area[4 * sqr_num] + cross_area[4 * sqr_num + 2]) / 2,
								(cross_area[4 * sqr_num + 1] + cross_area[4 * sqr_num + 3]) / 2);
	}*/
	//注意：这一步必须要有，不然进行不了鼠标操作
	namedWindow("Skeleton", WINDOW_AUTOSIZE);//WINDOW_AUTOSIZE:系统默认,显示自适应
	// thin_img.copyTo(dst);
	//cout << cross_area[0] << endl << cross_area[1];
	setMouseCallback("Skeleton", traversal_callback, 0);
	//setMouseCallback("src", get_point, 0);
	imshow("Skeleton", thin_img);
	waitKey();
	cv::destroyWindow("Skeleton");
	cv::destroyWindow("Traversal");
}
 // ==========    Main    ==========
void traversal(Mat src, string imgnum)
{
	img_num = "1";
	string rgb_address = "/home/lance/Data/0-RGBimg/" + img_num + ".png";
	RGBimg = readImg(rgb_address);
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
	namedWindow("Skeleton", WINDOW_AUTOSIZE);//WINDOW_AUTOSIZE:系统默认,显示自适应
	src.copyTo(dst);
	//cout << cross_area[0] << endl << cross_area[1];
	setMouseCallback("Skeleton", traversal_callback, 0);
	//setMouseCallback("src", get_point, 0);
	imshow("Skeleton", src);
	waitKey();
}// ==========    Main    ==========

 // ======= 鼠标事件回调函数 =======
void traversal_callback(int event, int x, int y, int flags, void* ustc)
{
	static int center_num = 0, L_down_num = 0;
	static Point center[20];
	if (event == CV_EVENT_RBUTTONDOWN)
	{
		center[center_num] = Point(x+100, y+100);
		center_num++;
		std::cout << center_num << "  " << center[center_num - 1].x-100 << ", " << center[center_num - 1].y-100 << endl;
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
		pre_pt = Point(x+100, y+100);
		pt[0] = pre_pt;
		col = src.cols * 3; int row = src.rows;
		int per_step = 5; int smaller_cross = 15; int jump_step = 4; 
		int crop_size = 50; int dir_size = 20; 
		int point_num = 1;
		int ddx, ddy; float d;
		bool incross[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		bool crossing[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		curr_pt = get_first_current_point(src, pre_pt, per_step, 5);
		pt[point_num] = curr_pt;
		for (int i = 0; i < 250; i++){
			point_num++;
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
						string dir_savecroped = "pic_buffer/C/" + ni + "_" + nj + ".png";
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
			// imshow("src", src);
		}
		if (L_down_num == 2) {
			center_num = 0;
			L_down_num = 0;
		}
		for (int n = 0; n < point_num; n++){
			circle(src, pt[n], 3, Scalar(255, 2*n, 0), -1); // 画圆(图, 坐标, 半径, 颜色, -1表示实心)
		}
		Mat img = src(cv::Rect(100, 100, 640, 320));
		imshow("Traversal", img);
		//src.copyTo(dst);//确保画线操作是在src上进行
		end = clock(); // time
		double endtime = (double)(end - start) / CLOCKS_PER_SEC; // time
		//cout << endtime << endl; // time
	}
} // ======= 鼠标事件回调函数 =======

 // = 用相对curr_pt的dx,dy定义指针 =
int dxdy2currentpoint(Mat src_, Point curr_pt, int dx, int dy, int channel)
{
	uchar *pointer = src_.ptr<uchar>(curr_pt.y);
	int c = src_.cols;
	return pointer[channel * (curr_pt.x + dx) + channel * c * dy];
	// return pointer[(curr_pt.x + dx) + col * dy];
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
Point get_first_current_point(Mat srcf, Point pre_pt, int per_step, int round, int channel)
{
	bool endfind = 0;
	int per_add = per_step;
	Point curr_pt;
	for(int r=0; r<round; ++r)
	{
		int dx = per_step; int dy = per_step;
		if (!endfind)
		{
			for (dx; dx > -per_step; dx--)
			{
				// circle(src, Point(pre_pt.x+dx, pre_pt.y+dy), 1, Scalar(255, 0, 0), -1);
				if (dxdy2currentpoint(srcf, pre_pt, dx, dy, channel) == 255)
				{
					curr_pt = Point(pre_pt.x + dx, pre_pt.y + dy);
					endfind = 1;
					break;
				}
			}
		}
		if (!endfind)
		{
			for (dy; dy > -per_step; dy--)
			{
				// circle(src, Point(pre_pt.x+dx, pre_pt.y+dy), 1, Scalar(255, 0, 0), -1);
				if (dxdy2currentpoint(srcf, pre_pt, dx, dy, channel) == 255)
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
				// circle(src, Point(pre_pt.x+dx, pre_pt.y+dy), 1, Scalar(255, 0, 0), -1);
				if (dxdy2currentpoint(srcf, pre_pt, dx, dy, channel) == 255)
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
				// circle(src, Point(pre_pt.x+dx, pre_pt.y+dy), 1, Scalar(255, 0, 0), -1);
				if (dxdy2currentpoint(srcf, pre_pt, dx, dy, channel) == 255)
				{
					curr_pt = Point(pre_pt.x + dx, pre_pt.y + dy);
					endfind = 1;
					break;
				}
			}
		}
		per_step += per_add;
	}
	if (!endfind)
		cout << "ERROR STARTING POINT" << endl;
	return curr_pt;
}// =========   寻找起点   =========

 // =========   寻找下一点  =========
Point get_next_point(Point curr_pt, int dx, int dy, int per_step)
{
	int search_times = 3 * per_step;
	Point dir_s, dir_n, next_pt = Point(curr_pt.x + dx, curr_pt.y + dy);
	Point s_pt = next_pt; Point n_pt = next_pt;
	for (int i = 0; i < search_times; i++)
	{
		if (dxdy2currentpoint(src, s_pt, 0, 0) == 255)
		{
			next_pt = s_pt;
			break;
		}
		if (dxdy2currentpoint(src, n_pt, 0, 0) == 255)
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
			// imshow("src", src);
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
	Point dir_s, next_pt = Point(nx, ny), n2, n1;
	Point s_pt = Point(nx, ny);
	for (int i = 0; i < search_times - pass_length; i++) {
		if (dxdy2currentpoint(src, s_pt, 0, 0) == 255) {
			if (i - last_i > pass_length)
			{
				count++;
			}
			if (count == 1) {
				n1 = s_pt;
			}
			else if (count == 2) {
				n2 = s_pt;
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
	if(count == 3)	next_pt = n2;
	else if(count == 2)	next_pt = n1;
	else
	{
		cout << "【ERROR】 Find opposite point error" << endl;
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
		circle(src, pre_pt, 3, Scalar(0, 255, 0), -1); // 画圆(图, 坐标, 半径, 颜色, -1表示实心)
		cout << x << "," << y << endl;
	}
}// ====   获得鼠标点击点的坐标  ====


Mat convertTo3Channels(const Mat& binImg)
{
    Mat three_channel = Mat::zeros(binImg.rows,binImg.cols,CV_8UC3);
    vector<Mat> channels;
    for (int i=0;i<3;i++)
    {
        channels.push_back(binImg);
    }
    merge(channels,three_channel);
    return three_channel;
}

Point per_dir(Point pre_pt, Point curr_pt, int per_step){
	int dx = curr_pt.x - pre_pt.x; 
	int dy = curr_pt.y - pre_pt.y;
	int ddx, ddy;
	float d;
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
	return Point(dx, dy);
}
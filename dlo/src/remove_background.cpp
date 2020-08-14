#include "dlo.h"

cv::Mat remove_background(cv::Mat rgb_img, cv::Mat bw_img)
{
    cv::Mat cableonly_img;
	bw_img.copyTo(cableonly_img);
    int nr = bw_img.rows;
	int nc = bw_img.cols;
    for(int i=0; i<nr; i++){
        uchar* bw_pix = bw_img.ptr<uchar>(i);
        uchar* rgb_pix = rgb_img.ptr<uchar>(i);
		uchar* cableonly_pix = cableonly_img.ptr<uchar>(i);
        for(int j=0; j<nc; j++){
            if(bw_pix[3*j]==255){
				cableonly_pix[3*j] = rgb_pix[3*j];
				cableonly_pix[3*j+1] = rgb_pix[3*j+1];
				cableonly_pix[3*j+2] = rgb_pix[3*j+2];
			}
		}
    }
	imshow("cableonly_img", cableonly_img);
	cv::waitKey();
    return cableonly_img;
}

cv::Mat change_background(cv::Mat rgb_img, cv::Mat bg_img, cv::Mat bw_img, int y)
{
	int dx, dy;
	int x = y*2;
	cv::resize(bg_img, bg_img, cv::Size(x, y), 0, 0, cv::INTER_CUBIC);
	cv::resize(new_bw, new_bw, cv::Size(x, y), 0, 0, cv::INTER_CUBIC);
    string save_address = "/home/lance/Data/test/2-替换背景/3-new_bw.png";
	cv::Mat rebackground_img;
	bg_img.copyTo(rebackground_img);
	int nr = bw_img.rows;
	int nc = bw_img.cols;
	dx = (x-nc)/2;
	dy = (y-nr)/2;
    for(int i=0; i<nr; i++){
        uchar* bw_pix = bw_img.ptr<uchar>(i);
        uchar* rgb_pix = rgb_img.ptr<uchar>(i);
		uchar* rebackground_pix = rebackground_img.ptr<uchar>(i+dy);
		uchar* new_bw_pix = new_bw.ptr<uchar>(i+dy);
        for(int j=0; j<nc; j++){
            if(bw_pix[3*j]==255){
				rebackground_pix[3*(j+dx)] = rgb_pix[3*j];
				rebackground_pix[3*(j+dx)+1] = rgb_pix[3*j+1];
				rebackground_pix[3*(j+dx)+2] = rgb_pix[3*j+2];
				new_bw_pix[3*(j+dx)] = 255;
				new_bw_pix[3*(j+dx)+1] = 255;
				new_bw_pix[3*(j+dx)+2] = 255;
			}
		}
    }

	cv::resize(rebackground_img, rebackground_img, rgb_img.size(), 0, 0, cv::INTER_CUBIC);
	// cv::resize(new_bw, new_bw, rgb_img.size(), 0, 0, cv::INTER_CUBIC);
	// imshow("rebackground_img", rebackground_img);
	// imshow("new_bw_img", new_bw_img);
	//cv::imwrite(save_address, new_bw_img);
	cv::waitKey();
    return rebackground_img;
}

cv::Mat change_background(cv::Mat rgb_img, cv::Mat bg_img, cv::Mat bw_img)
{
	cv::Mat rebackground_img;
	bg_img.copyTo(rebackground_img);
	int nr = bw_img.rows;
	int nc = bw_img.cols;
	for(int i=0; i<nr; i++){
        uchar* bw_pix = bw_img.ptr<uchar>(i);
        uchar* rgb_pix = rgb_img.ptr<uchar>(i);
		uchar* rebackground_pix = rebackground_img.ptr<uchar>(i);
        for(int j=0; j<nc; j++){
            if(bw_pix[3*j]==255){
				rebackground_pix[3*(j)] = rgb_pix[3*j];
				rebackground_pix[3*(j)+1] = rgb_pix[3*j+1];
				rebackground_pix[3*(j)+2] = rgb_pix[3*j+2];
			}
		}
    }
    return rebackground_img;
}
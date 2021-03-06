#include "strategy.h"

using namespace std;

Scalar red = Scalar(0, 0, 255), green = Scalar(0, 255, 0), 
		blue = Scalar(255, 0, 0), yellow = Scalar(0, 255, 255),
		purple = Scalar(255, 0, 255), cyan = Scalar(255, 255, 0);
Mat result_img;
int nCheckReturn;
vector<int> c0, c1;

CStrategy::CStrategy(){
}
CStrategy::~CStrategy(){
}

SOperation CStrategy::strategy(){
	SOperation oprt_rt;
	
	Point add_text = Point(5,5);
	c0.clear(); c1.clear();
    result_img = readImg("pic_buffer/1_R.png");
	int line_index = 1;
	int ept_index = 2*line_index-1;
	int cpt_index = 0;
	for(int n = 0; n < point_num; ++n){ // 标记遍历轨迹
		int ept_index = 2*line_index-1;
		Scalar color_value = getcolor(n, ept_index, line_index);
		circle(result_img, pt[n], 2, 0, -1);
		circle(result_img, pt[n], 1, color_value, -1);
		if(n == ept[ept_index])	
			++line_index;
		if(cpt_num && n == cpt[cpt_index])	
			++cpt_index;
		if(line_index > line_num)	break;
	}
	for(int i=0; i<ept.size(); ++i){ // 标记端点序号
		int end_i = cross.size() + i;
		string endtext = to_string(end_i);
		Point textpoint = Point(pt[ept[i]].x-4, pt[ept[i]].y+5);
		putText(result_img, endtext, textpoint, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
	}
	for(int i=0; i<cross.size(); ++i){ // 标记交叉点序号
		string i_text = to_string(i);
		Point textpoint = Point(cross[i].x-4, cross[i].y+5);
		putText(result_img, i_text, textpoint, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
	}
	line_index = 1;
	ept_index = 2*line_index-1;
	for(int i = 0; i < cpt.size(); ++i){ // 标记交叉点信息
		if(cpt[i] >= ept[ept_index])	++line_index;
		ept_index = 2*line_index-1;
		Scalar color_value = getcolor(cpt[i], ept_index, line_index);
		string classtext;
		if(g_vnClassList[i])	classtext = "U";
		else 	classtext = "D";
		Point textpoint = pt[cpt[i]];
		putText(result_img, classtext, textpoint, FONT_HERSHEY_SIMPLEX, 0.5, color_value, 2);
		string cross_text = to_string(cpt[i]);
		textpoint = Point(pt[cpt[i]].x, pt[cpt[i]+1].y+15);
		putText(result_img, cross_text, textpoint, FONT_HERSHEY_SIMPLEX, 0.4, color_value, 1);
	}
	// 记录序号为i的交叉点的1类和0类在crosslist[]里的序号
	for(int i = 0; i < cross.size(); ++i){ 
		c0.push_back(-1);
		c1.push_back(-1);
	}
	for(int i = 0; i < g_vnClassList.size(); ++i){
		if(g_vnClassList[i])	c1[g_vnCrossList[i]] = i;
		else 					c0[g_vnCrossList[i]] = i;
	}

	
	Point front_dir, back_dir, target;
	int opt_index, ref_index;
	vector<bool> is_end;
	oprt_rt.strOperationType = "N";		//	标记操作类型, 初始为 N 表示 None
	bool bGetLRindexOfOptionI = 0;
	int opt1_index, opt2_index;
	if(checklist()==1){
		exit(0);
		oprt_rt.strOperationType = "E";		//	Error
		return oprt_rt;
	}
	

	// === 无交叉点时将线缆拉出视野 I型　===
	if(cross.size() == 0){
		oprt_rt.strOperationType = "I";
		for(int i = 0; i < start.size(); i+=2){		//	先找出两个端点至少有一个不在视野边缘的一根线缆, 排除掉已经挑出的线缆
			Point ptEnd1 = pt[ept[i]+1], ptEnd2 = pt[ept[i+1]-1];
			if(IsPointInMatrix(ptEnd1) || IsPointInMatrix(ptEnd2)){
				opt1_index = ept[i]+13;
				opt2_index = ept[i+1]-13;
				bGetLRindexOfOptionI = 1;
				break;
			}
			else if(ept[i+1] - ept[i] > 150){
				opt1_index = ept[i]+20;
				opt2_index = ept[i+1]-20;
				bGetLRindexOfOptionI = 1;
				g_nEndLeft--;
				g_nEndRight--;
				break;
			}
		}
		if(bGetLRindexOfOptionI == 0){
			cout << "\n\n\t\t=====  【END】  =====\n\n\n";
			exit(0);
		}
	}

	// === 检测最上层的独立线缆 I型 ===
	if(oprt_rt.strOperationType == "N"){
		for(int i = 0; i < start.size(); i+=2){
			/*	检查start[i]开始的线缆上的交叉点类型是否都为1
				全为1表示该线缆在视野最上层, 循环结束时 mul=1	*/
			bool mul = 1;
			for(int j = start[i]; j <= start[i+1]; ++j){
				mul *= g_vnClassList[j];
				if(!mul){
					if(c1[g_vnCrossList[j]] >= start[i] && c1[g_vnCrossList[j]] <= start[i+1]){
						bool mul2 = 1;
						for(int k = j+1; k < c1[g_vnCrossList[j]]; ++k){
							mul2 *= g_vnClassList[k];
						}
						if(mul2)
							mul = 1;
						else
							break;
					}
					else
				 		break;
				}
			}
			if(start[i] > start[i+1])	continue;		//	表示为全程无交叉的独立线缆
			if(mul){
				oprt_rt.strOperationType = "I";
				opt1_index = ept[i]+20;
				opt2_index = ept[i+1]-20;
				bGetLRindexOfOptionI = 1;
				break;
			}
		}
	}

	//	===	I型返回值计算	===
	if(bGetLRindexOfOptionI){
		//	检查左右目标点是否距离过近
		while(abs(pt[opt1_index].x - pt[opt2_index].x) < 150 && abs(pt[opt1_index].y - pt[opt2_index].y) < 150){
			opt1_index+=10;				opt2_index-=10;
			if(abs(opt1_index - opt2_index) < 20){
				cout << "\n\n===== 【NO OPRATION POINT CAN USE IN TYPE I】 " << " =====\n";
				oprt_rt.strOperationType = "N";
				break;
			}
		}
		int rightindex, leftindex;
		if(pt[opt1_index].x < pt[opt2_index].x){	//	遍历先经过的点在右手工作区域
			rightindex = opt1_index;	leftindex = opt2_index;
			oprt_rt.vdAddInfo.push_back((pt[leftindex+1].x >= pt[leftindex].x) ? 0 : 1);
			oprt_rt.vdAddInfo.push_back((pt[rightindex+1].x >= pt[rightindex].x) ? 0 : 1);
		}
		else{										//	遍历先经过的点在左手工作区域
			rightindex = opt2_index;	leftindex = opt1_index;
			oprt_rt.vdAddInfo.push_back((pt[leftindex+1].x < pt[leftindex].x) ? 0 : 1);
			oprt_rt.vdAddInfo.push_back((pt[rightindex+1].x < pt[rightindex].x) ? 0 : 1);
		}
		oprt_rt.vptPoint.push_back(pt[leftindex]);
		oprt_rt.vptPoint.push_back(pt[rightindex]);

		draw_point(pt[leftindex], "OPT_L", red);
		draw_point(pt[rightindex], "OPT_R", red);
		oprt_rt.vdGripperDir.push_back(draw_grip_direction(leftindex));
		oprt_rt.vdGripperDir.push_back(draw_grip_direction(rightindex));
	}
    

	// === 检测D类交叉 ===
	if(oprt_rt.strOperationType == "N"){
		for(int i = 0; i < g_vnClassList.size()-1; ++i){
			if (g_vnClassList[i] * g_vnClassList[i+1]) {
				/*	检查当前交叉D型是否为跨端点的错误识别类型	
					若结束循环时 is_end=1, 则表示跨端点, 应跳过	*/
				bool is_end = 0;
				for(int k = 0; k < start.size(); ++k){
					if(i == start[k] || i+1 == start[k]){
						is_end = 1;
						break;
					}
				}
				
				// if (!is_end && abs(c0[g_vnCrossList[i]] - c0[g_vnCrossList[i + 1]]) == 1) {
				if (abs(c0[g_vnCrossList[i]] - c0[g_vnCrossList[i + 1]]) == 1) {
					oprt_rt.strOperationType = "D";
					
					/*	打印交叉信息	*/
					cout << endl << "OPT-D:" << endl;
					cout << '\t' << '\t' << "INDEX" << '\t' << "CROSS" << '\t' << "CLASS" << endl;
					cout << '\t' << '\t' << i << '\t' << g_vnCrossList[i] << '\t' << g_vnClassList[i] << endl;
					cout << '\t' << '\t' << i + 1 << '\t' << g_vnCrossList[i + 1] << '\t' << g_vnClassList[i + 1] << endl;
					if (c0[g_vnCrossList[i]] < c0[g_vnCrossList[i + 1]]) {
						cout << '\t' << '\t' << c0[g_vnCrossList[i]] << '\t' << g_vnCrossList[c0[g_vnCrossList[i]]] << '\t' << g_vnClassList[c0[g_vnCrossList[i]]] << endl;
						cout << '\t' << '\t' << c0[g_vnCrossList[i + 1]] << '\t' << g_vnCrossList[c0[g_vnCrossList[i + 1]]] << '\t' << g_vnClassList[c0[g_vnCrossList[i + 1]]] << endl;
					}
					else {
						cout << '\t' << '\t' << c0[g_vnCrossList[i + 1]] << '\t' << g_vnCrossList[c0[g_vnCrossList[i + 1]]] << '\t' << g_vnClassList[c0[g_vnCrossList[i + 1]]] << endl;
						cout << '\t' << '\t' << c0[g_vnCrossList[i]] << '\t' << g_vnCrossList[c0[g_vnCrossList[i]]] << '\t' << g_vnClassList[c0[g_vnCrossList[i]]] << endl;
					}

					opt_index = (cpt[i] + cpt[i+1])/2;
					oprt_rt.vptPoint.push_back(pt[opt_index]);
					oprt_rt.vdGripperDir.push_back(draw_grip_direction(opt_index));
					draw_point(pt[opt_index], "opt", blue);

					ref_index = (cpt[c0[g_vnCrossList[i]]] + cpt[c0[g_vnCrossList[i + 1]]])/2;
					draw_point(pt[ref_index], "ref", green);
					target = pt[ref_index] + per_dir(pt[opt_index], pt[ref_index], 30);
					draw_point(target, "target", yellow);

					oprt_rt.vptPoint.push_back(target);
					oprt_rt.vdGripperDir.push_back(draw_grip_direction(opt_index));

					break;
                }
			}
		}
	}

	// === 检测T类交叉 ===
	if(oprt_rt.strOperationType == "N"){
		for(int i = 0; i < start.size(); ++i){
			// int nEptX = pt[ept[i]].x, nEptY = pt[ept[i]].y;
			// if(nEptX > 160 && nEptX < 640 && nEptY > 160 && nEptY < 480){	//	端点不在图像边缘, 即悬空端点, 可拖动
			if(IsPointInMatrix(pt[ept[i]])){
				oprt_rt.strOperationType = "T";

				/*	打印交叉信息	*/
				cout << endl << "CLASS-T:" << endl;
				cout << '\t' << "INDEX" << '\t' << "CROSS" << '\t' << "CLASS" << endl;
				cout << '\t' << start[i] << '\t' << g_vnCrossList[start[i]] << '\t' << g_vnClassList[start[i]] << endl;
				int j = g_vnClassList[start[i]] ? c0[g_vnCrossList[start[i]]] : c1[g_vnCrossList[start[i]]];
				cout << '\t' << j << '\t' << g_vnCrossList[j] << '\t' << g_vnClassList[j] << endl;

				int ept_opt = i;
				int k = start[i];
				int add = cpt[k]>ept[ept_opt]?1:-1;
				while(g_vnClassList[k])	k+=add;
				opt_index = cpt[k]>ept[ept_opt]?cpt[k]+1:cpt[k]-1;		//	夹取点取相对于交叉点与端点不同边的一端
				Point cross_t = cross[g_vnCrossList[k]];
				// int fix_index = cpt[j]+2; // 固定点
			
				oprt_rt.vptPoint.push_back(pt[opt_index]);
				oprt_rt.vdGripperDir.push_back(draw_grip_direction(opt_index));
				draw_point(pt[opt_index], "opt", blue);
				draw_point(cross_t, "ref", green);
				Point ptTarget = pt[opt_index] + per_dir(cross_t, pt[opt_index], 30);
				oprt_rt.vptPoint.push_back(ptTarget);
				oprt_rt.vdGripperDir.push_back(draw_grip_direction(opt_index));
				draw_point(ptTarget, "tar", yellow);
				// draw_point(pt[fix_index], "fix", purple);
				int fakelength = abs(opt_index-ept[i]);
				oprt_rt.vdAddInfo.push_back(fakelength/2.5);

				break;
			}
		}
		if(oprt_rt.strOperationType == "N"){
			// === 检测V类交叉 ===
			for(int i = 0; i < start.size(); ++i){
				for(int j = i+1; j < start.size(); ++j){
					if(g_vnCrossList[start[i]] == g_vnCrossList[start[j]]){
						oprt_rt.strOperationType = "T";
						cout << endl << "CLASS-X:" << endl;
						cout << '\t' << "INDEX" << '\t' << "CROSS" << '\t' << "CLASS" << endl;
						cout << '\t' << start[i] << '\t' << g_vnCrossList[start[i]] << '\t' << g_vnClassList[start[i]] << endl;
						cout << '\t' << start[j] << '\t' << g_vnCrossList[start[j]] << '\t' << g_vnClassList[start[j]] << endl;

						int ept_opt = g_vnClassList[start[i]] ? i : j;
						int ept_ref = g_vnClassList[start[j]] ? i : j;
						opt_index = (cpt[c1[g_vnCrossList[start[j]]]]+ept[ept_opt])/2;
						ref_index = (cpt[c0[g_vnCrossList[start[j]]]]+ept[ept_ref])/2;

						oprt_rt.vptPoint.push_back(pt[opt_index]);
						oprt_rt.vdGripperDir.push_back(draw_grip_direction(opt_index));
						draw_point(pt[opt_index], "opt", blue);
						draw_point(pt[ref_index], "ref", green);
						target = pt[ref_index] + per_dir(pt[opt_index], pt[ref_index], 50);
						oprt_rt.vptPoint.push_back(target);
						oprt_rt.vdGripperDir.push_back(draw_grip_direction(opt_index));
						draw_point(target, "target", yellow);
						break;
					}
				}
				if(oprt_rt.strOperationType == "T")		break;
			}
		}
	}

	// === 检测Q类交叉 === 
	if(oprt_rt.strOperationType == "N"){
		for(int i = 0; i < g_vnClassList.size()-1; ++i){
			/*	检查当前交叉Q型是否为跨端点的错误识别类型	
				若结束循环时 twolines_ends=1, 则表示跨端点, 应跳过	*/
			bool twolines_ends = 0;
			for(int k = 0; k < start.size(); ++k){
				if(i == start[k] && i+1 == start[k+1]){
					if(k/2!=(k+1)/2){
						twolines_ends = 1;
						break;
					}
				}
			}

			if (!twolines_ends && g_vnCrossList[i] == g_vnCrossList[i+1]) {
				oprt_rt.strOperationType = "Q";

				/*	打印交叉信息	*/
				cout << endl << "OPT-Q:" << endl;
				cout << '\t' << '\t' << "INDEX" << '\t' << "CROSS" << '\t' << "CLASS" << endl;
				cout << '\t' << '\t' << i << '\t' << g_vnCrossList[i] << '\t' << g_vnClassList[i] << endl;
				cout << '\t' << '\t' << i + 1 << '\t' << g_vnCrossList[i + 1] << '\t' << g_vnClassList[i + 1] << endl;
			
				/*	抓取位置位于两个自交叉点间曲线段的中间位置附近	*/
				opt_index = (cpt[i] + cpt[i+1])/2;
				oprt_rt.vptPoint.push_back(pt[opt_index]);
				oprt_rt.vdGripperDir.push_back(draw_grip_direction(opt_index));
				draw_point(pt[opt_index], "opt", blue);

				/*	对应的自交叉点	*/
				Point cross_o = cross[g_vnCrossList[i]];
				ref_index = cpt[c1[g_vnCrossList[i]]];
				cout << "i: " << i << endl;
				draw_point(cross_o, "ref", green);

				/*	自交叉点位置位于上方的线缆切线方向	*/
				front_dir = cross_o+5*(dir[ref_index+1]+dir[ref_index-1]);
				back_dir = cross_o-5*(dir[ref_index+1]+dir[ref_index-1]);
				line(result_img, back_dir, front_dir, Scalar(0, 255, 0), 2);

				/*	自交叉计算旋转方向的参考角度	*/
				line(result_img, pt[opt_index], cross_o, Scalar(0, 255, 255), 2);
				putText(result_img, "rotation_dir", front_dir+add_text, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
				double angle_o = angle(pt[opt_index], front_dir, cross_o);
				putText(result_img, to_string(angle_o), cross_o+Point(20, -20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
				oprt_rt.vptPoint.push_back(cross_o);
				oprt_rt.vdGripperDir.push_back(angle_o);
				cout << "  angle_o: " << angle_o << endl;

				break;
			}
		}
	}

	// === 检测IX类交叉 ===
	if(oprt_rt.strOperationType == "N"){
		for(int i = 0; i < g_vnCrossList.size(); ++i){
			if(g_vnClassList[i]*g_vnClassList[i+1]){
				bool found = 0;
				int p = c0[g_vnCrossList[i]];
				Point cross_y;
				if(!g_vnClassList[p-1]){
					if(1 == abs(c0[g_vnCrossList[i+1]] - c1[g_vnCrossList[p-1]])){
						cout << endl << "OPT-IX:" << endl;
						cout_cross(i); cout_cross(i+1); cout_cross(p-1);cout_cross(p); 
						cout_cross(c0[g_vnCrossList[i+1]]); cout_cross(c1[g_vnCrossList[p-1]]);
						cross_y = cross[g_vnCrossList[p-1]];
						found = 1;
					}
				}
				if(!found && !g_vnClassList[p+1]){
					if(1 == abs(c0[g_vnCrossList[i+1]] - c1[g_vnCrossList[p+1]])){
						cout << endl << "OPT-IX:" << endl;
						cout_cross(i); cout_cross(i+1); cout_cross(p);cout_cross(p+1); 
						cout_cross(c0[g_vnCrossList[i+1]]); cout_cross(c1[g_vnCrossList[p+1]]);
						cross_y = cross[g_vnCrossList[p+1]];
						found = 1;
					}
				}
				if(!found){
					p = c0[g_vnCrossList[i+1]];
					if(!g_vnClassList[p-1]){
						if(1 == abs(c0[g_vnCrossList[i]] - c1[g_vnCrossList[p-1]])){
							cout << endl << "OPT-IX:" << endl;
							cout_cross(i); cout_cross(i+1); cout_cross(p-1);cout_cross(p); 
							cout_cross(c0[g_vnCrossList[i+1]]); cout_cross(c1[g_vnCrossList[p-1]]);
							cross_y = cross[g_vnCrossList[p-1]];
							found = 1;
						}
					}
					if(!found && !g_vnClassList[p+1]){
						if(1 == abs(c0[g_vnCrossList[i]] - c1[g_vnCrossList[p+1]])){
							cout << endl << "OPT-IX:" << endl;
							cout_cross(i); cout_cross(i+1); cout_cross(p);cout_cross(p+1); 
							cout_cross(c0[g_vnCrossList[i+1]]); cout_cross(c1[g_vnCrossList[p+1]]);
							cross_y = cross[g_vnCrossList[p+1]];
							found = 1;
						}
					}
				}
				if(found){
					oprt_rt.strOperationType = "IX";

					opt_index = (cpt[i]+cpt[i+1])/2;
					// ref_index = cpt[ref_i];
					oprt_rt.vptPoint.push_back(pt[opt_index]);
					oprt_rt.vdGripperDir.push_back(draw_grip_direction(opt_index));
					draw_point(pt[opt_index], "opt", blue);
					draw_point(cross_y, "ref", green);
					Point ptTarget = cross_y + per_dir(pt[opt_index], cross_y, 75);
					oprt_rt.vptPoint.push_back(ptTarget);
					oprt_rt.vdGripperDir.push_back(draw_grip_direction(opt_index));
					draw_point(ptTarget, "target", yellow);
					break;
				}
			}	
		}
	}



	cout << "======= " << oprt_rt.strOperationType << " ========" << endl;
	cv::imwrite("pic_buffer/8_Result.png", result_img);
	// namedWindow("Result", WINDOW_AUTOSIZE); // === 显示图片 ===
	// imshow("Result", result_img);
	// cv::destroyWindow("Result"); // === 显示图片 ===
	return oprt_rt;
	// // === 检测I类交叉 ===
	// if(!end_strategy){
	// 	for(int i = 0; i < start.size(); i+=2){
	// 		int sum = 0;
	// 		for(int j = start[i]; j <= start[i+1]; ++j){
	// 			sum += classlist[j];
	// 			if(sum) break;
	// 		}
	// 		if(!sum){
	// 			end_strategy = 1;
	// 			int length1 = abs(ept[i] - cpt[start[i]]);
	// 			int length2 = abs(ept[i+1] - cpt[start[i+1]]);
	// 			int index = length1 > length2 ? i : i+1;
	// 			opt_index = cpt[start[index]]>ept[index]? cpt[start[index]]-1:cpt[start[index]]+1;
	// 			draw_point(pt[opt_index], "OPT1", blue);
	// 			draw_grip_direction(opt_index);
	// 			ref_index = length1 > length2 ? ept[i]: ept[i+1];
	// 			draw_point(pt[ref_index], "OPT2", green);
	// 			break;
	// 		}
	// 	}
	// }

	// // === 检测X类交叉 ===
    // if(!end_strategy){
    //     for(int i = 0; i < start.size(); ++i){
    //         for(int j = i+1; j < start.size(); ++j){
    //             if(crosslist[start[i]] == crosslist[start[j]]){
    //                 end_strategy = 1;
    //                 cout << endl << "CLASS-X:" << endl;
    //                 cout << '\t' << "INDEX" << '\t' << "CROSS" << '\t' << "CLASS" << endl;
    //                 cout << '\t' << start[i] << '\t' << crosslist[start[i]] << '\t' << classlist[start[i]] << endl;
    //                 cout << '\t' << start[j] << '\t' << crosslist[start[j]] << '\t' << classlist[start[j]] << endl;

    //                 int ept_opt = classlist[start[i]] ? i : j;
    //                 int ept_ref = classlist[start[j]] ? i : j;
    //                 opt_index = (cpt[c1[crosslist[start[j]]]]+ept[ept_opt])/2;
    //                 ref_index = (cpt[c0[crosslist[start[j]]]]+ept[ept_ref])/2;
                
    //                 draw_grip_direction(opt_index);
	// 				draw_point(pt[opt_index], "opt", blue);
	// 				draw_point(pt[ref_index], "ref", green);
    //                 // circle(result_img, pt[opt_index], 5, Scalar(255, 0, 0), -1);
    //                 // putText(result_img, "opt", pt[opt_index]+add_text, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);
    //                 // circle(result_img, pt[ref_index], 5, Scalar(0, 255, 0), -1);
    //                 // putText(result_img, "ref", pt[ref_index]+add_text, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
                    
    //                 target = pt[ref_index] + per_dir(pt[opt_index], pt[ref_index], 30);
	// 				draw_point(target, "target", yellow);
	// 				// circle(result_img, target, 5, Scalar(0, 255, 255), -1);
    //                 // putText(result_img, "target", target+add_text, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
    //                 break;
    //             }
    //         }
    //         if(end_strategy)	break;
    //     }
    // }
}

double CStrategy::draw_grip_direction(int opt_index)
{
    Point front_dir = pt[opt_index]+5*(dir[opt_index+1]+dir[opt_index+2]); // 画近似切线 -- 方向矢量
    Point back_dir = pt[opt_index]-5*(dir[opt_index+1]+dir[opt_index+2]);
	double dy = front_dir.y - pt[opt_index].y;
	double dx = front_dir.x - pt[opt_index].x + 1e-10;
	// double dDirAngle = angle(pt[opt_index]+Point(20, 0), front_dir, pt[opt_index]);
    line(result_img, back_dir, front_dir, red, 2);
    putText(result_img, "grip_dir", front_dir+add_text, FONT_HERSHEY_SIMPLEX, 0.5, red, 1);
	double dAngle = atan(dy/dx)*180/3.141592658;
	// if(dAngle >= 0)
	// 	return dAngle-90;
	return dAngle+90;
	// return -dDirAngle;
}
double CStrategy::draw_grip_direction(Point ptTarget, Point ptTargetDir)
{
	line(result_img, ptTarget-ptTargetDir, ptTarget+ptTargetDir, yellow, 2);
	double dDirAngle = angle(ptTarget+Point(20, 0), ptTargetDir, ptTarget);
	double dy = ptTargetDir.y;
	double dx = ptTargetDir.x + 1e-10;
	double dAngle = atan(dy/dx)*180/3.141592658;
	// if(dAngle >= 0)
	// 	return dAngle-90;
	return dAngle+90;
	// return -dDirAngle;
}

void CStrategy::draw_point(Point pt, string pt_text, Scalar color, float text_size, int text_thick)
{
    circle(result_img, pt, 5, color, -1);
	putText(result_img, pt_text, pt+add_text, FONT_HERSHEY_SIMPLEX, text_size, color, text_thick);
}

void CStrategy::cout_cross(int i){
	cout << '\t' << '\t' << i << '\t' << g_vnCrossList[i] << '\t' << g_vnClassList[i] << endl;
}

int CStrategy::checklist(){
	int wrong = 0;
	
	//	检查每个交叉点是否有且仅有经过两次
	vector<int> crosspasstimes;
	int n = cross.size();
	while(n--)
		crosspasstimes.push_back(2);
	for(int i = 0; i < g_vnCrossList.size(); ++i){
		crosspasstimes[g_vnCrossList[i]]--;
	}
	for(int i = 0; i < cross.size(); ++i){
		if(crosspasstimes[i]){
			cout << "\n\n===== 【ERROR TRAVERSAL PATH】 " << " =====\n\n\n";
			return 1;
		}
	}

	//	检查交叉点列表是否是偶数
	int sumcross = 0;
	if(g_vnCrossList.size()%2){
		cout << "\n\n===== 【ERROR CROSSLIST LENGTH】 LIST LENGTH: " << g_vnCrossList.size() << " =====\n\n\n";
		wrong = 2;
	}

	//	检查两类交叉点数量是否相等
	for(int i = 0; i < g_vnCrossList.size(); ++i){
		sumcross += g_vnClassList[i];
	}
	if(cross.size() != sumcross){
		cout << "\n\n===== 【ERROR CROSSCLASS PREDICTED IMBALANCED】 " << " =====\n";
		cout << '\t' << "CLASS 0: " << 2*cross.size() - sumcross << endl;
		cout << '\t' << "CLASS 1: " << sumcross << "\n\n\n";
		wrong = 2;
	}

	// //	检查是否有未修改的c0[i]或c1[i]
	// for(int i = 0; i < cross.size(); ++i){
	// 	if(c0[i] == -1 || c1[i] == -1)	continue;
	// 	Mat imgTrainCross = imread(m_vstrCropDir[i])
	// 	// if(c0[i] < 0 || c1[i] < 0){
	// 	// 	cout << "\n\n===== 【ERROR CLASS WHILE PREDICTING CROSS】 " << " =====\n\n\n";
	// 	// 	wrong = 2;
	// 	// 	break;
	// 	// }
	// }
	return wrong;
}
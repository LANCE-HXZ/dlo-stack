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
		if(n == cpt[cpt_index])	
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

	nCheckReturn = checklist();

	bool end_strategy = 0;
	Point front_dir, back_dir, target;
	int opt_index, ref_index;
	vector<bool> is_end;
	oprt_rt.strOperationType = "N";		//	标记操作类型, 初始为 N 表示 None
	bool bMoveLeft = 0, bMoveRight = 0;

	// === 检测最上层的独立线缆 I型 ===
    for(int i = 0; i < start.size(); i+=2){
        bool mul = 1;
        for(int j = start[i]; j <= start[i+1]; ++j){	//	/*
			mul *= g_vnClassList[j];					//		检查start[i]开始的线缆上的交叉点类型是否都为1
			if(!mul) 	break;							//		全为1表示该线缆在视野最上层
        }												//												*/
        if(mul){
            end_strategy = 1;		oprt_rt.strOperationType = "I";
			
			int opt1_index = ept[i]+10;
			int opt2_index = ept[i+1]-10;
			int rightindex = pt[opt1_index].x < pt[opt2_index].x ? opt1_index : opt2_index;
			int leftindex = pt[opt1_index].x < pt[opt2_index].x ? opt2_index : opt1_index;
			oprt_rt.vptPoint.push_back(pt[leftindex]);
			oprt_rt.vptPoint.push_back(pt[rightindex]);

			draw_point(pt[rightindex], "OPT_R", red);
			draw_point(pt[leftindex], "OPT_L", red);
            oprt_rt.vdGripperDir.push_back(draw_grip_direction(leftindex));
			oprt_rt.vdGripperDir.push_back(draw_grip_direction(rightindex));

			int target_y = PIC_WIDTH + 1.0/2 * EDGE; 			//	目标位置为图像下边缘
			Point target_dir = Point(10, 0.000001);				//	移动到目标位置的抓取方向
			int nStepLength = abs(rightindex - leftindex);		// 	两个抓取点中间的遍历步数
			Point target_left = Point(pt[leftindex-nStepLength*1].x, target_y);
			Point target_right = Point(pt[rightindex+nStepLength*1].x, target_y);
			oprt_rt.vptPoint.push_back(target_right);
			oprt_rt.vptPoint.push_back(target_left);

			draw_point(target_left, "target_left", yellow);
			draw_point(target_right, "target_right", yellow);
			oprt_rt.vdGripperDir.push_back(draw_grip_direction(target_left, target_dir));
			oprt_rt.vdGripperDir.push_back(draw_grip_direction(target_right, target_dir));
        }
    }
    

	// === 检测D类交叉 ===
	if(!end_strategy){
		for(int i = 0; i < g_vnClassList.size()-1; ++i){
			bool is_end = 0;
			if (g_vnClassList[i] * g_vnClassList[i+1]) {
				for(int k = 0; k < start.size(); ++k){
					if(i == start[k] || i+1 == start[k]){
						is_end = 1;
						break;
					}
				}
				// if (!is_end && abs(c0[g_vnCrossList[i]] - c0[g_vnCrossList[i + 1]]) == 1) {
				if (abs(c0[g_vnCrossList[i]] - c0[g_vnCrossList[i + 1]]) == 1) {
					end_strategy = 1;
					oprt_rt.strOperationType = "D";
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
					double dGripDir = draw_grip_direction(opt_index);
					draw_point(pt[opt_index], "opt", blue);
					ref_index = (cpt[c0[g_vnCrossList[i]]] + cpt[c0[g_vnCrossList[i + 1]]])/2;
					draw_point(pt[ref_index], "ref", green);
					target = pt[ref_index] + per_dir(pt[opt_index], pt[ref_index], 30);
					draw_point(target, "target", yellow);

					string strGroup = "R";
					if(pt[opt_index].x >= 400)	strGroup = "L";
					// km.SetLeftPose(pt[opt_index]-Point(EDGE, EDGE), 0.94, {1.57+dGripDir, 0, 0});
					// km.SetRightPose(pt[opt_index]-Point(EDGE, EDGE), 0.94, {1.57-0.523-dGripDir, 0, 0});
                    // km.ExecuteGroup(strGroup);
					// km.MoveDdxdydz(0, 0, 0.1);
					// gc.Dual_Gripper_anypose("220", "220");
					// km.MoveDdxdydz(0, 0, -0.1);
					// km.SetLeftPose(target-Point(EDGE, EDGE), 0.94, {1.57+dGripDir, 0, 0});
					// km.SetRightPose(target-Point(EDGE, EDGE), 0.94, {1.57-0.523-dGripDir, 0, 0});
                    // km.ExecuteGroup(strGroup);
					// km.MoveDdxdydz(0, 0, 0.1);
					// gc.Dual_Gripper_anypose("0", "0");
					// km.GoHome(D_GROUP);

					break;
                }
			}
		}
	}

	// === 检测Q类交叉 === 
	if(!end_strategy){
		for(int i = 0; i < g_vnClassList.size()-1; ++i){
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
				end_strategy = 1;
				oprt_rt.strOperationType = "Q";
				cout << endl << "OPT-Q:" << endl;
				cout << '\t' << '\t' << "INDEX" << '\t' << "CROSS" << '\t' << "CLASS" << endl;
				cout << '\t' << '\t' << i << '\t' << g_vnCrossList[i] << '\t' << g_vnClassList[i] << endl;
				cout << '\t' << '\t' << i + 1 << '\t' << g_vnCrossList[i + 1] << '\t' << g_vnClassList[i + 1] << endl;
			
				opt_index = (cpt[i] + cpt[i+1])/2;
				double dGripDir = draw_grip_direction(opt_index);
				draw_point(pt[opt_index], "opt", blue);

				Point cross_o = cross[g_vnCrossList[i]];
				ref_index = cpt[c1[g_vnCrossList[i]]];
				cout << "i: " << i << endl;
				draw_point(cross_o, "ref", green);
				front_dir = cross_o+5*(dir[ref_index+1]+dir[ref_index-1]); // 画近似切线 -- 方向矢量
				back_dir = cross_o-5*(dir[ref_index+1]+dir[ref_index-1]);
				line(result_img, back_dir, front_dir, Scalar(0, 255, 0), 2);
				line(result_img, pt[opt_index], cross_o, Scalar(0, 255, 255), 2);
				putText(result_img, "rotation_dir", front_dir+add_text, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
				double angle_o = angle(pt[opt_index], front_dir, cross_o);
				string angle_text = to_string(angle_o);
				putText(result_img, angle_text, cross_o+Point(20, -20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
				cout << "  angle_o: " << angle_o << endl;

				string strGroup = "R";
				if(pt[opt_index].x >= 400)	strGroup = "L";
				// // km.SetLeftPose(pt[opt_index]-Point(EDGE, EDGE), 0.945, {1.57+dGripDir, 0, 0});
				// km.SetRightPose(pt[opt_index]-Point(EDGE, EDGE), 0.945, {3.14-0.523-dGripDir, 0, 0});
				// km.ExecuteGroup(strGroup);
				// km.MoveRdxdydz(0, 0, 0.1);
				// gc.Gripper_anypose('R', "220");
				// km.MoveRdxdydz(0, 0, -0.2);
				// // km.SetLeftPose(pt[ref_index]-Point(EDGE, EDGE), 0.745, {1.57+dGripDir, 0, 0});
				// // km.SetRightPose(pt[ref_index]-Point(EDGE, EDGE), 0.745, {3.14-0.523-dGripDir, 0, 0});
				// // km.ExecuteGroup(strGroup);
				// km.MoveOneRightJointIncrease(6, -5);
				// // km.MoveOneLeftJointIncrease(6, -3.14);
				// // km.SetLeftPose(pt[opt_index]-Point(EDGE, EDGE), 0.945, {-1.57+dGripDir, 0, 0});
				// // km.SetRightPose(pt[opt_index]-Point(EDGE, EDGE), 0.945, {-3.14-0.523-dGripDir, 0, 0});
				// // km.ExecuteGroup(strGroup);
				// km.MoveRdxdydz(0, 0, 0.1);
				// gc.Gripper_anypose('R', "0");
				// km.GoHome(D_GROUP);

				break;
			}
		}
	}

	// === 检测IX类交叉 ===
	if(!end_strategy){
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
					end_strategy = 1;
					oprt_rt.strOperationType = "IX";
					opt_index = (cpt[i]+cpt[i+1])/2;
					// ref_index = cpt[ref_i];
					draw_grip_direction(opt_index);
					draw_point(pt[opt_index], "opt", blue);
					draw_point(cross_y, "ref", green);
					target = cross_y + per_dir(pt[opt_index], cross_y, 30);
					draw_point(target, "target", yellow);
					break;
				}
			}	
		}
	}

	// === 检测T类交叉 ===
	if(!end_strategy){
		for(int i = 0; i < start.size(); ++i){
			end_strategy = 1;
			oprt_rt.strOperationType = "T";
			cout << endl << "CLASS-T:" << endl;
			cout << '\t' << "INDEX" << '\t' << "CROSS" << '\t' << "CLASS" << endl;
			cout << '\t' << start[i] << '\t' << g_vnCrossList[start[i]] << '\t' << g_vnClassList[start[i]] << endl;
			int j = g_vnClassList[start[i]] ? c0[g_vnCrossList[start[i]]] : c1[g_vnCrossList[start[i]]];
			cout << '\t' << j << '\t' << g_vnCrossList[j] << '\t' << g_vnClassList[j] << endl;

			int ept_opt = i;
			opt_index = cpt[start[i]]>ept[ept_opt]?cpt[start[i]]+1:cpt[start[i]]-1;
			int fix_index = cpt[j]+2; // 固定点
			Point cross_t = cross[g_vnCrossList[j]];
		
			draw_grip_direction(opt_index);
			draw_point(pt[opt_index], "opt", blue);
			draw_point(pt[fix_index], "fix", purple);
			draw_point(cross_t, "ref", green);
			break;
		}
	}
	cout << "--" << oprt_rt.strOperationType << "--" << endl;
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
	double dy = front_dir.y - back_dir.y;
	double dx = front_dir.y - back_dir.x + 0.000000001;
    line(result_img, back_dir, front_dir, red, 2);
    putText(result_img, "grip_dir", front_dir+add_text, FONT_HERSHEY_SIMPLEX, 0.5, red, 1);
	return atan(dy/dx);
}
double CStrategy::draw_grip_direction(Point ptTarget, Point ptTargetDir)
{
	line(result_img, ptTarget-ptTargetDir, ptTarget+ptTargetDir, yellow, 2);
	double dy = ptTargetDir.y;
	double dx = ptTargetDir.x + 0.000000001;
	return atan(dy/dx);
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
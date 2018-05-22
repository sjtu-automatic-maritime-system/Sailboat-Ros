#include "avoidance_v2.h"
#include "rtwtypes.h"
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "sailboat_vpp/CDynamicVPP.h"


#include <fstream>  
#include <sstream>  
#include <string>  
#include <stdlib.h>
#include <math.h>

using namespace std;

extern scanningModelClass collision_avoidance_Obj;


double obs_radius = 5.0;

//angles in rad: i_angle target_information.angle
//angles in deg: static_wind_information.wind_angle

double rad2deg(double rad){
	return rad * 180 / PI;
}

double deg2rad(double deg){
	return deg * PI / 180;
}

double formalize_angle(double angle){
	while (angle > 360)
		angle -= 360;

	while (angle < 0)
		angle += 360;

	return angle;
}

double formalize_rad(double angle){
	while (angle > 2*PI)
		angle -= 2*PI;

	while (angle < 0)
		angle += 2*PI;

	return angle;
}

string Trim(string& str)  
{  
    //str.find_first_not_of(" \t\r\n"),在字符串str中从索引0开始，返回首次不匹配"\t\r\n"的位置  
    str.erase(0,str.find_first_not_of(" \t\r\n"));  
    str.erase(str.find_last_not_of(" \t\r\n") + 1);  
    return str;  
}  


double ** read_vpp(){
	//read vpp file
    ifstream fin("/home/boxian/polardiagram.csv"); 
    
    
    string line; 
    double **vpps;
    vpps = new double * [121];
    int counter = 0;
    while (getline(fin, line))   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取  
    {  
        //cout <<"原始字符串："<< line << endl; //整行输出  
        istringstream sin(line); //将整行字符串line读入到字符串流istringstream中  
        vector<string> fields; //声明一个字符串向量  
        string field;  
        while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符  
        {  
            fields.push_back(field); //将刚刚读取的字符串添加到向量fields中  
        } 
        vpps[counter] = new double[10];
        for (int i = 0; i < 10; i++){
        	vpps[counter][i] = atof(Trim(fields[i]).c_str());
        }

        counter++;

        //cout <<"处理之后的："<< vpps[0][0]<<endl;   
    } 

    /*
    //print vpp
    for (int i = 0; i < 121; i++){
    	for (int j = 0; j < 10; j++)
    		cout << vpps[i][j]<<' ';
    	cout <<endl;
    }
    */

    /*
    for (int i = 0; i < 121; i++){
    	delete [] vpps[i];
    }
    delete vpps;
    */

	return vpps;
}

double interp_vpp(double v_wind, double input_angle, double **vpps)  
{

    //interpolation
    double vel = v_wind;
    double angle = formalize_angle(input_angle);
    //cout << atof(argv[1]) << atof(argv[2]) << endl;

    int cel_vel = ceil(vel)-1;
    int flo_vel = floor(vel)-1;

    int cel_angle = ceil(angle/3);
    int flo_angle = floor(angle/3);
    //cout << "Cel_Angle is "<<cel_angle<<endl;

    double result_tmp_flo;
    double result_tmp_cel;
    double result;

    if (vel < 1){
    	if (cel_angle == flo_angle)
    		result = vpps[flo_angle][0]*vel;
    	else
    		result = vel*(vpps[cel_angle][0] - vpps[flo_angle][0]) * ((angle/3) - flo_angle)/(cel_angle - flo_angle);
    }

    //extrapolate
    else if (vel > 10){
    	if (cel_angle == flo_angle){
    		//cout << vpps[flo_angle][9] << ' '<<vpps[flo_angle][8]<<endl;
    		result =  vpps[flo_angle][9] + (vpps[flo_angle][8] - vpps[flo_angle][9]) * (vel - 10);
    	}
    	else{
    		result_tmp_flo = vpps[flo_angle][9] + (vpps[flo_angle][9] - vpps[flo_angle][8]) * (vel - 10);
	    	result_tmp_cel = vpps[cel_angle][9] + (vpps[cel_angle][9] - vpps[cel_angle][8]) * (vel - 10);
	    	result = result_tmp_flo + (result_tmp_cel - result_tmp_flo) * ((angle/3) - flo_angle)/(cel_angle - flo_angle);
    	}
    }

	//interpolate
    else{
	    if (cel_vel == flo_vel){
	    	if (cel_angle == flo_angle)
	    		result = vpps[flo_angle][flo_vel];
	    	else
	    		result = vpps[flo_angle][flo_vel] + (vpps[cel_angle][flo_vel] - vpps[flo_angle][flo_vel]) * ((angle/3) - flo_angle)/(cel_angle - flo_angle);
	    }
	    else{
	    	result_tmp_flo = vpps[flo_angle][flo_vel] + (vel - flo_vel)/(cel_vel - flo_vel)*(vpps[flo_angle][cel_vel] - vpps[flo_angle][flo_vel]);
	    	result_tmp_cel = vpps[cel_angle][flo_vel] + (vel - flo_vel)/(cel_vel - flo_vel)*(vpps[cel_angle][cel_vel] - vpps[cel_angle][flo_vel]);

		    //cout <<"The result_tmp_cel is "<<result_tmp_cel<<endl;
		    //cout <<"The result_tmp_flo is "<<result_tmp_flo<<endl;
	    	if (cel_angle == flo_angle)
	    		result = result_tmp_flo;
	    	else
	    		result = result_tmp_flo + (result_tmp_cel - result_tmp_flo) * ((angle/3) - flo_angle)/(cel_angle - flo_angle);
	    }

	}
    
    cout << "The result is "<< result << endl;

    return result;  
}  

void scanningModelClass::avoidance_algo(){
	//std::cout << "obstacle size is " << obstacle_information.data.size() << '\n'; //for debugging
	// if no obstacle information, warn and return
	if (collision_avoidance_Obj.obstacle_information.data.empty()){
		ROS_INFO("[Input] No obstacle information input!");
		// default angle is target angle;
		collision_avoidance_Obj.output.angle = collision_avoidance_Obj.target_information.angle;
		ROS_INFO("[Output] The output angle is %f", collision_avoidance_Obj.output.angle);
		return ;
	}

	int data_size = obstacle_information.data.size();

	double obs_pos_x = collision_avoidance_Obj.obstacle_pose.x;
	double obs_pos_y = collision_avoidance_Obj.obstacle_pose.y;

	double obj_pos_x = collision_avoidance_Obj.collision_avoidance_U.north;
    double obj_pos_y = collision_avoidance_Obj.collision_avoidance_U.east;

	double distance_obs = sqrt((obs_pos_x - obj_pos_x)*(obs_pos_x - obj_pos_x) + (obs_pos_y - obj_pos_y)*(obs_pos_y - obj_pos_y));

	//ROS_INFO("[Algo] Obstacle_msg sub : [%f %f %f %f %f %f %f %f] ", collision_avoidance_Obj.obstacle_information.data[0], collision_avoidance_Obj.obstacle_information.data[1], collision_avoidance_Obj.obstacle_information.data[2], collision_avoidance_Obj.obstacle_information.data[3], collision_avoidance_Obj.obstacle_information.data[4], collision_avoidance_Obj.obstacle_information.data[5], collision_avoidance_Obj.obstacle_information.data[6], collision_avoidance_Obj.obstacle_information.data[7]);
	//cout << "run here" << endl;
	double ** vpps = read_vpp();


	std::vector<double> v_vec;
	v_vec.clear();
	for (int i=0; i<data_size; ++i){
		double origin_dist = collision_avoidance_Obj.obstacle_information.data.at(i);
		double i_angle = i * (2 * PI /data_size);
		double q; 			//obstacle-quality factor
		if (origin_dist > MAX_DISTANCE)
			q = 1.0;
		else if (origin_dist < MIN_DISTANCE)
			q = 0.0;
		else 
			q = (origin_dist - MIN_DISTANCE)/(MAX_DISTANCE - MIN_DISTANCE);

		//dealing with crash near obstacles: when the sailboat is near the obstacle (within 5m), avoid tacking!
		if (distance_obs < (obs_radius + 5) && formalize_rad(fabs(i_angle-collision_avoidance_Obj.output.angle)) > 1.0)
			q = 0.0;


		double v_b;		//velocity of sailboat
		//VPP
		v_b = interp_vpp(collision_avoidance_Obj.collision_avoidance_U.Airmar_wind_speed, rad2deg(i_angle)-collision_avoidance_Obj.collision_avoidance_U.Airmar_yaw-collision_avoidance_Obj.collision_avoidance_U.Airmar_wind_angle, vpps);

		/*
		CDynamicVPP vpp(i_angle, collision_avoidance_Obj.collision_avoidance_U.Airmar_yaw, collision_avoidance_Obj.collision_avoidance_U.Airmar_wind_speed, collision_avoidance_Obj.collision_avoidance_U.Airmar_wind_angle + collision_avoidance_Obj.collision_avoidance_U.Airmar_yaw);
		vpp.run(0.1);
		*/

		cout << "v_b is " << v_b << endl;
		double v_t;		//velocity of sailboat towards target
		v_t = q * v_b * cos(i_angle - collision_avoidance_Obj.target_information.angle);
		cout << "angle is " << rad2deg(i_angle - collision_avoidance_Obj.target_information.angle)  << endl;
		cout << "cos is " << cos(i_angle - collision_avoidance_Obj.target_information.angle) << endl;
		cout << "v_t is " << v_t << endl;
		v_vec.push_back(v_t);
			
	}

	//find the angle for max velocity
	double v_max = v_vec.at(0);
	int angle_max = 0;
	int count = 0;
	for (std::vector<double> ::iterator iter= v_vec.begin();iter!=v_vec.end();iter++){
		if (*iter > v_max){
			v_max = *iter;
			angle_max = count;
		}
		count++;
	}

	//output v_vec
	ROS_INFO("[Algo] v_vec is %f %f %f %f %f %f %f %f", v_vec.at(0), v_vec.at(1), v_vec.at(2), v_vec.at(3), v_vec.at(4), v_vec.at(5), v_vec.at(6), v_vec.at(7));	

	double output_angle = 360.0/data_size * angle_max ;
	
	ROS_INFO("[Output] The output angle is %f", output_angle);

	collision_avoidance_Obj.output.angle = deg2rad(output_angle);
}

void scanningModelClass::initialize(){

	//initialize default input
	collision_avoidance_U.roll=0;
	collision_avoidance_U.roll_rate=0;                  // '<Root>/roll_rate'
	collision_avoidance_U.camera_confidence=0;          // '<Root>/camera_confidence'
	collision_avoidance_U.north=0;                      // '<Root>/north'
	collision_avoidance_U.east=0;                       // '<Root>/east'
	collision_avoidance_U.Airmar_yaw=0;                 // '<Root>/Airmar_yaw'
	collision_avoidance_U.roll=0;                       // '<Root>/roll'
	collision_avoidance_U.Airmar_wind_speed=0;          // '<Root>/Airmar_wind_speed'
	collision_avoidance_U.Airmar_wind_angle=0;          // '<Root>/Airmar_wind_angle'
	collision_avoidance_U.yaw_rate=0; 					 // '<Root>/yaw_rate'

	//initialize obstacle information
	obstacle_information.data.clear();
	/* //initialize obstacle distance to infinity
	for (int i=0; i<8; ++i)
		obstacle_information.data.push_back(INFINITY_DOUBLE);
	*/

	//initialize target information
	target_information.angle = collision_avoidance_U.Airmar_yaw;

}
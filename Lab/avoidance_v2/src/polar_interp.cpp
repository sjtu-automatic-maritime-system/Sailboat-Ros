#include <iostream>  
#include <fstream>  
#include <sstream>  
#include <string>  
#include <vector>  
#include <stdlib.h>
#include <math.h>

using namespace std;

string Trim(string& str)  
{  
    //str.find_first_not_of(" \t\r\n"),在字符串str中从索引0开始，返回首次不匹配"\t\r\n"的位置  
    str.erase(0,str.find_first_not_of(" \t\r\n"));  
    str.erase(str.find_last_not_of(" \t\r\n") + 1);  
    return str;  
}  

double formalize_angle(double angle){
	while (angle > 360)
		angle -= 360;

	while (angle < 0)
		angle += 360;

	return angle;
}

int main(int argc,char *argv[])  
{
	//read vpp file
    ifstream fin("polardiagram.csv"); 
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

        //cout <<"处理之后的："<< a[0]<<' '<<a[1]<<' '<<a[2]<<' '<<a[3]<<' '<< a[4]<<' '<<endl;   
    } 

    /*
    for (int i = 0; i < 121; i++){
    	for (int j = 0; j < 10; j++)
    		cout << vpps[i][j]<<' ';
    	cout <<endl;
    }
    */

    //interpolation
    double vel = atof(argv[1]);
    double angle = formalize_angle(atof(argv[2]));
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

    for (int i = 0; i < 121; i++){
    	delete [] vpps[i];
    }
    delete vpps;

    return 0;  
}  
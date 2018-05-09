#include "particle_filter.h"

ParticleFilter::ParticleFilter(){
    ball_num = 0;
    sigma1 = 4;
    sigma2 = 16;
    setReFrom = 100;
}

ParticleFilter::~ParticleFilter(){

}

double ParticleFilter::calDistance(double x1,double y1,double x2,double y2){
    return std::sqrt(std::pow(x1-x2,2)+std::pow(y1-y2,2));
}

void ParticleFilter::createParticles(double x, double y){
    //std::cout<<"new createParticles"<<std::endl;
    ball_num += 1;
    ball_point_set.push_back(pointArray());
    int num = ball_point_set.size();
    //pointArray *point_array = new pointArray();
    for (int i = 0; i < 200; i++ ){
        ball_point_set[num-1].pointDataArray.push_back(pointData());
        // pointData *point_data = new pointData();
        // todo
        ball_point_set[num-1].pointDataArray[i].x = x + (double)rand()/RAND_MAX * 10 - 5;
        ball_point_set[num-1].pointDataArray[i].y = y + (double)rand()/RAND_MAX * 10 - 5;
        ball_point_set[num-1].pointDataArray[i].probability = 1.0/200;
        //std::cout << "x = " << ball_point_set[num-1].pointDataArray[i].x <<" y = " << ball_point_set[num-1].pointDataArray[i].y <<std::endl;
        
    }
    ball_point_set[num-1].detectNum = 0;
    estimate(num-1);
    //ball_point_set.push_back(pointArray());

}

int ParticleFilter::inputDetectBall(double x, double y){
    if ( ball_num != ball_point_set.size()){
        std::cout << "error" << std::endl;
        return -1;
    }

    if ( ball_point_set.size() == 0 ){
        createParticles(x, y);
        return ball_point_set.size() - 1;
    }
    else{
        int index = 0;
        if (isDetectBallInArray(x,y,index)){
            return index;
        }
        else{
            createParticles(x, y);
            return ball_point_set.size() - 1;
        }
    }
}

bool ParticleFilter::isDetectBallInArray(double x, double y, int &index){
    index = 0;
    for (int i = 0; i < ball_point_set.size(); i++ ){
        double best_x = ball_point_set[i].bestX;
        double best_y = ball_point_set[i].bestY;
        double dis = std::sqrt(std::pow(x - best_x, 2) + std::pow(y - best_y, 2));
        if (dis < 10){
            index = i;
            return true;
        }
    }
    return false;
}

void ParticleFilter::updateWeight(double x, double y, double yaw,int index){
    double sum = 0;
    for (int i = 0; i < ball_point_set[index].pointDataArray.size(); i++){
        double posX = ball_point_set[index].pointDataArray[i].x;
        double posY = ball_point_set[index].pointDataArray[i].y;
        double k = std::tan(yaw);
        double d1 = std::fabs(-k*posX + posY + k*x - y)/std::sqrt(1+ std::pow(k,2));
        double l = std::sqrt(std::pow(x-posX,2)+std::pow(y-posY,2));
        double d2 = std::sqrt(std::pow(l,2)-std::pow(d1,2));
        double weight = std::exp(-(std::pow(d1,2))/(2*sigma1)) * std::exp(-(std::pow(d2,2))/(2*sigma2));
        ball_point_set[index].pointDataArray[i].probability = weight;
        sum += weight;
    }
    
    double guiyi = sum;

    for (int i = 0; i < ball_point_set[index].pointDataArray.size(); i++){
        ball_point_set[index].pointDataArray[i].probability = ball_point_set[index].pointDataArray[i].probability / guiyi;
    }
    ball_point_set[index].detectNum += 1;
}

void ParticleFilter::estimate(int index){
    double bestX = 0;
    double bestY = 0;
    for (int i = 0; i < ball_point_set[index].pointDataArray.size(); i++){
        double posX = ball_point_set[index].pointDataArray[i].x;
        double posY = ball_point_set[index].pointDataArray[i].y;
        double weight = ball_point_set[index].pointDataArray[i].probability;
        bestX += posX * weight;
        bestY += posY * weight; 
    } 
    ball_point_set[index].bestX = bestX;
    ball_point_set[index].bestY = bestY;
}

double ParticleFilter::neff(int index){
    double sum = 0;
    for (int i = 0; i < ball_point_set[index].pointDataArray.size(); i++){
        sum += 1/std::pow(ball_point_set[index].pointDataArray[i].probability,2);
    }
    //std::cout<<"neff sum = "<< sum <<std::endl;
    return sum;
}

double ParticleFilter::resampleFromIndex(double x, double y,double yaw, int index){
    std::cout<<"resampleFromIndex"<<std::endl;
    int del_num = 0;
    int size = ball_point_set[index].pointDataArray.size();
    std::sort(ball_point_set[index].pointDataArray.begin(),ball_point_set[index].pointDataArray.end(),SortByM1); 
    // for (int i = 0; i < size/5; i++ ){
    //     ball_point_set[index].pointDataArray[i].x = ball_point_set[index].pointDataArray[size-1-i].x;
    //     ball_point_set[index].pointDataArray[i].y = ball_point_set[index].pointDataArray[size-1-i].y;
    //     ball_point_set[index].pointDataArray[i].probability = ball_point_set[index].pointDataArray[size-1-i].probability;
    // }
    for (int i = 0; i<size/5; i++){
        double dis = (double)rand()/RAND_MAX * 10 - 5;
        double yaw_dis = (double)rand()/RAND_MAX * 0.1 - 0.05;
        ball_point_set[index].pointDataArray[i].x = x + dis*std::cos(yaw+yaw_dis);
        ball_point_set[index].pointDataArray[i].y = y + dis*std::sin(yaw+yaw_dis);
        //std::cout << "probability : " << ball_point_set[index].pointDataArray[i].probability << std::endl;
    }
}


bool ParticleFilter::SortByM1( const pointData &v1, const pointData &v2)//注意：本函数的参数的类型一定要与vector中元素的类型一致  
{  
    return v1.probability < v2.probability;//升序排列  
}


void ParticleFilter::run(double x, double y, double yaw, double &bestX, double &bestY, double &last_yaw){
    
    int index;
    index = inputDetectBall(x,y);
    int size = ball_point_set[index].pointDataArray.size();

    if (setReFrom >= 100){
        resampleFromIndex(x,y,yaw, index);
        ball_point_set[index].last_yaw = yaw;
        setReFrom = 0;
        deleteObjet();
    }

    updateWeight(x,y,yaw, index);
    estimate(index);
    //std::cout << "size = " << ball_point_set.size()<<std::endl;
    bestX = ball_point_set[index].bestX;
    bestY = ball_point_set[index].bestY;
    last_yaw = ball_point_set[index].last_yaw;

    
    setReFrom += 1;
}


void ParticleFilter::publish(objectPoseArray &object_pose_array){
    for (int i = 0; i<ball_point_set.size(); i++){
        object_pose_array.poseArray.push_back(pointData());
        object_pose_array.poseArray[i].x =  ball_point_set[i].bestX ;
        object_pose_array.poseArray[i].y =  ball_point_set[i].bestY ;
        object_pose_array.poseArray[i].probability =  ball_point_set[i].detectNum ;
        std::cout << "obj X = " << ball_point_set[i].bestX << " obj Y = " << ball_point_set[i].bestY<< " detectNum = " << ball_point_set[i].detectNum<<std::endl;
    }
}


void ParticleFilter::deleteObjet(){
    if (ball_point_set.size() >= 2){
        while (true){
            for (int i = 0; i <  ball_point_set.size()-1; i++){
                for(int j = i+1; j < ball_point_set.size(); j++){
                    double dis = calDistance(ball_point_set[i].bestX,ball_point_set[i].bestY,ball_point_set[j].bestX,ball_point_set[j].bestY);
                    if (dis < 5){
                        ball_point_set.erase(ball_point_set.begin() + j);
                        break;
                    }
                }
            }
            break;
        }

    }
}


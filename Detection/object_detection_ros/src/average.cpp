#include "average.h"

Average::Average(){
    ball_num = 0;
}

Average::~Average(){

}

void Average::addObeject(double x, double y){
    //std::cout<<"new createParticles"<<std::endl;
    ball_num += 1;
    objectPoseArray.push_back(pointData2());
    int num = objectPoseArray.size();

    objectPoseArray[num-1].x = x;
    objectPoseArray[num-1].y = y;
}

int Average::inputDetectBall(double x, double y){
    if ( ball_num != objectPoseArray.size()){
        std::cout << "error" << std::endl;
        return -1;
    }

    if ( objectPoseArray.size() == 0 ){
        addObeject(x, y);
        return objectPoseArray.size() - 1;
    }
    else{
        int index = 0;
        if (isDetectBallInArray(x,y,index)){
            return index;
        }
        else{
            addObeject(x, y);
            return objectPoseArray.size() - 1;
        }
    }
}

bool Average::isDetectBallInArray(double x, double y, int &index){
    index = 0;
    for (int i = 0; i < objectPoseArray.size(); i++ ){
        double best_x = objectPoseArray[i].x;
        double best_y = objectPoseArray[i].y;
        double dis = std::sqrt(std::pow(x - best_x, 2) + std::pow(y - best_y, 2));
        if (dis < 10){
            index = i;
            return true;
        }
    }
    return false;
}


void Average::run(double x, double y,double yaw, double &bestX, double &bestY){
    int index;
    index = inputDetectBall(x,y);

    objectPoseArray[index].x = (objectPoseArray[index].x + x)/2;
    objectPoseArray[index].y = (objectPoseArray[index].y + y)/2;

    bestX = objectPoseArray[index].x;
    bestY = objectPoseArray[index].y;
}

void Average::publish(){
    for (int i = 0; i<objectPoseArray.size(); i++){
        std::cout << "obj X = " << objectPoseArray[i].x << "obj Y = " << objectPoseArray[i].y<<std::endl;
    }
}
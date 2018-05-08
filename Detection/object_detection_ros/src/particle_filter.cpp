#include "particle_filter.h"

ParticleFilter::ParticleFilter(){
    ball_num = 0;
    sigma = 4;
}

ParticleFilter::~ParticleFilter(){

}

void ParticleFilter::createParticles(double x, double y){
    std::cout<<"new createParticles"<<std::endl;
    ball_num += 1;
    ball_point_set.push_back(pointArray());
    int num = ball_point_set.size();
    //pointArray *point_array = new pointArray();
    for (int i = 0; i < 100; i++ ){
        ball_point_set[num-1].pointDataArray.push_back(pointData());
        // pointData *point_data = new pointData();
        // todo
        ball_point_set[num-1].pointDataArray[i].x = x + (double)rand()/RAND_MAX * 10 - 5;
        ball_point_set[num-1].pointDataArray[i].y = y + (double)rand()/RAND_MAX * 10 - 5;
        ball_point_set[num-1].pointDataArray[i].probability = 1/100;
        //std::cout << "x = " << ball_point_set[num-1].pointDataArray[i].x <<" y = " << ball_point_set[num-1].pointDataArray[i].y <<std::endl;
        
    }
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

void ParticleFilter::updateWeight(double x, double y, int index){
    double sum = 0;
    for (int i = 0; i < ball_point_set[index].pointDataArray.size(); i++){
        double posX = ball_point_set[index].pointDataArray[i].x;
        double posY = ball_point_set[index].pointDataArray[i].y;
        double weight = std::exp(-(std::pow(posX-x,2)+std::pow(posY-y,2))/(2*sigma));
        ball_point_set[index].pointDataArray[i].probability = weight;
        sum += weight;
    }
    
    double guiyi = sum;

    for (int i = 0; i < ball_point_set[index].pointDataArray.size(); i++){
        ball_point_set[index].pointDataArray[i].probability = ball_point_set[index].pointDataArray[i].probability / guiyi;
    }
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
    return sum;
}

double ParticleFilter::resampleFromIndex(int index){
    int del_num = 0;
    int size = ball_point_set[index].pointDataArray.size();
    std::sort(ball_point_set[index].pointDataArray.begin(),ball_point_set[index].pointDataArray.end(),SortByM1); 
    for (int i = 0; i < size/10; i++ ){
        ball_point_set[index].pointDataArray[i].x = ball_point_set[index].pointDataArray[size-1-i].x;
        ball_point_set[index].pointDataArray[i].y = ball_point_set[index].pointDataArray[size-1-i].y;
        ball_point_set[index].pointDataArray[i].probability = ball_point_set[index].pointDataArray[size-1-i].probability;
    }
}


bool ParticleFilter::SortByM1( const pointData &v1, const pointData &v2)//注意：本函数的参数的类型一定要与vector中元素的类型一致  
{  
    return v1.probability < v2.probability;//升序排列  
}


void ParticleFilter::run(double x, double y, double &bestX, double &bestY){
    int index;
    index = inputDetectBall(x,y);
    int size = ball_point_set[index].pointDataArray.size();

    updateWeight(x,y,index);
    estimate(index);
    if (neff(index) < size/2){
        resampleFromIndex(index);
    }

    std::cout << "size = " << ball_point_set.size()<<std::endl;

    // for (int i = 0; i < ball_point_set[index].pointDataArray.size(); i++){
    //     std::cout << "x = " << ball_point_set[index].pointDataArray[i].x <<" y = " << ball_point_set[index].pointDataArray[i].y <<" w = " << ball_point_set[index].pointDataArray[i].probability<<std::endl;
    
    // }

    bestX = ball_point_set[index].bestX;
    bestY = ball_point_set[index].bestY;
}


void ParticleFilter::publish(){
    for (int i = 0; i<ball_point_set.size(); i++){
        std::cout << "obj X = " << ball_point_set[i].bestX << "obj Y = " << ball_point_set[i].bestY<<std::endl;
    }
}
//
// Created by jianyun on 17-10-18.
//

#include <math.h>
#include "Astar.h"
#include <iostream>

#define _DEBUG_

double wrapAngle(double angle) {
    while (angle < -M_PI) { angle += 2 * M_PI; }
    while (angle > M_PI) { angle -= 2 * M_PI; }
    return angle;
}


void Astar::InitAstar(std::vector<std::vector<int>> &_maze, double _windAngle, double _init_heading) {
    maze = _maze;
    windAngle = _windAngle;
    init_heading = _init_heading;
}

int Astar::calcG(Point *temp_start, Point *point) {
    double stepAngle = M_PI - atan2((double) (point->y - temp_start->y), (double) (point->x - temp_start->x));
    // 风向损失
    double deltaAngle = windAngle - stepAngle;
#ifdef _DEBUG_
    deltaAngle = wrapAngle(deltaAngle);
//    std::cout << "point: " << point->x << ", " << point->y << " deltaAngle: " << deltaAngle * 57.3 << std::endl;
#endif
    double windG = exp((-cos(deltaAngle) - 0.25) * 8);
    // 转向损失
    double deltaAngle2 = temp_start->heading - stepAngle;
    double turnG = exp((-cos(deltaAngle2)) * 3);
//    std::cout << "point: " << point->x << ", " << point->y << " deltaAngle2: " << wrapAngle(deltaAngle2) * 57.3 << std::endl;

    int extraG = (int) windG + (int) turnG;
    int parentG = temp_start->G;
//    int parentG = point->parent == NULL ? 0 : point->parent->G; //如果是初始节点，则其父节点是空
    return parentG + extraG;

}

int Astar::calcH(Point *point, Point *end) {
    //用简单的欧几里得距离计算H，这个H的计算是关键，还有很多算法，没深入研究^_^
    return sqrt((double) (end->x - point->x) * (double) (end->x - point->x) +
                (double) (end->y - point->y) * (double) (end->y - point->y)) * kCost1;
}


double Astar::calcHeading(Point *point) {
    return M_PI - atan2((double) (point->y - point->parent->y), (double) (point->x - point->parent->x));
}

int Astar::calcF(Point *point) {
    return point->G + point->H;
}

Point *Astar::getLeastFpoint() {
    if (!openList.empty()) {
        auto resPoint = openList.front();
        for (auto &point:openList)
            if (point->F < resPoint->F)
                resPoint = point;
        return resPoint;
    }
    return NULL;
}

Point *Astar::findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner) {
    Point *new_start = new Point(startPoint.x, startPoint.y);
    new_start->heading = init_heading;
    openList.push_back(new_start); //置入起点,拷贝开辟一个节点，内外隔离
    while (!openList.empty()) {
#ifdef _DEBUG_
        std::cout << "\nopenList: " << std::endl;
        for (auto tmp1:openList) {
            std::cout << "x: " << tmp1->x << ", " << "y: " << tmp1->y << ", " << "G: " << tmp1->G << ", " << "Heading: "
                      << wrapAngle(tmp1->heading)*57.3 << std::endl;
        }
#endif
        auto curPoint = getLeastFpoint(); //找到F值最小的点
        openList.remove(curPoint); //从开启列表中删除
        closeList.push_back(curPoint); //放到关闭列表
        //1,找到当前周围八个格中可以通过的格子
        auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
#ifdef _DEBUG_
        std::cout << "\ncurPoint: " << curPoint->x << " ," << curPoint->y << std::endl;
#endif
        for (auto &target:surroundPoints) {
            //2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H
            auto p = isInList(openList, target); //不在list返回NULL，在list返回列表中的该点
            if (!p) {
                target->parent = curPoint;

                target->G = calcG(curPoint, target);
                target->H = calcH(target, &endPoint);
                target->F = calcF(target);
                target->heading = calcHeading(target);

                openList.push_back(target);
            }
                //3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
            else {
                target = p; //浅拷贝openList中的p给target，这样子能够修改openList
                int tempG = calcG(curPoint, target);
                if (tempG < target->G) {
                    target->parent = curPoint;

                    target->G = tempG;
                    target->F = calcF(target);
                    target->heading = calcHeading(target);
                }
            }
#ifdef _DEBUG_
            std::cout << "surPoint: " << target->x << " ," << target->y << std::endl;
            std::cout << "G: " << target->G << std::endl;
#endif
            Point *resPoint = isInList(openList, &endPoint);
            if (resPoint)
                return resPoint; //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝
        }
    }

    return NULL;
}

std::list<Point *> Astar::GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner) {
    Point *result = findPath(startPoint, endPoint, isIgnoreCorner);
    std::list<Point *> path;
    //返回路径，如果没找到路径，返回空链表
    while (result) {
        path.push_front(result);
        result = result->parent;
    }
    return path;
}

Point *Astar::isInList(const std::list<Point *> &list, const Point *point) const {
    //判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标
    for (auto p:list)
        if (p->x == point->x && p->y == point->y)
            return p;
    return NULL;
}

bool Astar::isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const {
    if (target->x < 0 || target->x > maze.size() - 1
        || target->y < 0 || target->y > maze[0].size() - 1
        || maze[target->x][target->y] == 1
        || target->x == point->x && target->y == point->y
        || isInList(closeList, target)) //如果点与当前节点重合、超出地图、是障碍物、或者在关闭列表中，返回false
        return false;
    else {
        if (abs(point->x - target->x) + abs(point->y - target->y) == 1) //非斜角可以
            return true;
        else {
            //斜对角要判断是否绊住
            if (maze[point->x][target->y] == 0 && maze[target->x][point->y] == 0)
                return true;
            else
                return isIgnoreCorner;
        }
    }
}

std::vector<Point *> Astar::getSurroundPoints(const Point *point, bool isIgnoreCorner) const {
    std::vector<Point *> surroundPoints;

    for (int x = point->x - 1; x <= point->x + 1; x++)
        for (int y = point->y - 1; y <= point->y + 1; y++)
            if (isCanreach(point, new Point(x, y), isIgnoreCorner))
                surroundPoints.push_back(new Point(x, y));

    return surroundPoints;
}
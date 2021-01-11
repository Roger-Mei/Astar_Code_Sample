#pragma once

#include <vector>
#include <list>
 
const int kCost = 10; //Cost of moving a single step
//const int kCost2 = 14;
 
struct Point
{
    int x, y; //point coordinates, x represents row number, y represents column number
    int F, G, H; //F = G + H
    Point* parent;
    Point(int _x, int _y)
        :x(_x), y(_y), F(0), G(0), H(0), parent(NULL) {}//initilize the class
};
 
class Astar
{
public:
    void InitAstar(std::vector<std::vector<int>> &_maze);
    std::list<Point*> GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
 
private:
    Point* findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
    
    std::vector<Point*> getSurroundPoints(const Point* point) const;
    
    //Justify whether the target point can be used for search
    bool isCanreach(const Point* point, const Point* target) const;
    
    //Justify whether the point is included in openList or closedList
    Point* isInList(const std::list<Point*> &list, const Point* point) const;
    
    //Get the node with the smallest F value
    Point* getLeastFpoint();
    
    //Calculate the value of F, G, H
    int calcG(Point* temp_start, Point* point);
    int calcH(Point* point, Point* end);
    int calcF(Point* point);
    
private:
    std::vector<std::vector<int>> maze;
    std::list<Point*> openList;
    std::list<Point*> closeList;
};

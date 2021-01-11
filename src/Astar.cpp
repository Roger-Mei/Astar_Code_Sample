#include <math.h>
#include "Astar.h"
 
void Astar::InitAstar(std::vector<std::vector<int>> &_maze)
{
    maze = _maze;
}
 
int Astar::calcG(Point* temp_start, Point* point)
{
    // Force the agent to obey the walking rule
    int extraG = abs(temp_start->x - point->x) + abs(temp_start->y - point->y) == 1? kCost : 0;
    // if the node is root node, then its parent G value should be zero
    int parentG = point->parent == NULL ? 0 : point->parent->G;
    return parentG + extraG;
}
 
int Astar::calcH(Point* point, Point* end)
{
    // Use Euclidian distance as Heuristic
    return sqrt((double)(end->x - point->x)*(double)(end->x - point->x) + (double)(end->y - point->y)*(double)(end->y - point->y))*kCost;
}
 
int Astar::calcF(Point* point)
{
    return point->G + point->H;
}
 
Point* Astar::getLeastFpoint()
{
    if (!openList.empty())
    {
        auto resPoint = openList.front();
        for (auto &point : openList){
            if (point->F<resPoint->F) resPoint = point;
        }
        
        return resPoint;
    }
    return NULL;
}
 
Point* Astar::findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
    //Copy the start point and insert it into the openList
    openList.push_back(new Point(startPoint.x, startPoint.y));
    
    while (!openList.empty())
    {
        auto curPoint = getLeastFpoint(); //Find the point with smallest F value
        openList.remove(curPoint); //Remove the point from openList
        closeList.push_back(curPoint); //put the point into the closeList
        
        //*First, get the available points surrounding the current point
        auto surroundPoints = getSurroundPoints(curPoint);
        
        for (auto &target : surroundPoints)
        {
            //*Second, if the point is not in openList, put it in the openList, set its parent and calculate the value of F, G, H
            if (!isInList(openList, target))
            {
                target->parent = curPoint;
 
                target->G = calcG(curPoint, target);
                target->H = calcH(target, &endPoint);
                target->F = calcF(target);
 
                openList.push_back(target);
            }
            //*Third, if the point is in openList, then we calculate the G value and compare with the existing one. If it is larger, we skip. If not, we set its parent and update the G and F.
            else
            {
                int tempG = calcG(curPoint, target);
                if (tempG < target->G)
                {
                    target->parent = curPoint;
 
                    target->G = tempG;
                    target->F = calcF(target);
                }
            }
            // The reason why I didn't return the original endpoint pointer is because there is deepcopy happened here
            Point* resPoint = isInList(openList, &endPoint);
            if (resPoint)
                return resPoint;
        }
    }
 
    return NULL;
}
 
std::list<Point*> Astar::GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
    Point* result = findPath(startPoint, endPoint, isIgnoreCorner);
    std::list<Point*> path;
    // return the path, if the path is empty, return an empty list
    while (result)
    {
        path.push_front(result);
        result = result->parent;
    }
 
    // Clean the temperary openList and closeList, jsut in case of any unexpected error when we run GetPath for several times
    openList.clear();
    closeList.clear();
 
    return path;
}
 
Point* Astar::isInList(const std::list<Point*> &list, const Point* point) const
{
    for (auto p : list)
    if (p->x == point->x&&p->y == point->y)
        return p;
    return NULL;
}
 
bool Astar::isCanreach(const Point *point, const Point *target) const
{
    // If target point follows one of the following condition:
    //      1. exceed the map size
    //      2. is obstacle
    //      3. overlapping with the current point
    //      4. is in the closeList
    // we will return false, meaning, the target point cannot be used for path search
    if (target->x<0 || target->x>maze.size() - 1
        || target->y<0 || target->y>maze[0].size() - 1
        || maze[target->x][target->y] == 1
        || (target->x == point->x && target->y == point->y)
        || isInList(closeList, target))
        return false;
    else
    {
        return true;
    }
}
 
std::vector<Point*> Astar::getSurroundPoints(const Point* point) const
{
    std::vector<Point*> surroundPoints;
    std::vector<int> dirc = {1,0,-1,0,1};
    
    for (int i = 0; i < 4; ++i){
        int x = point->x + dirc[i];
        int y = point->y + dirc[i+1];
        if (isCanreach(point, new Point(x, y)))
            surroundPoints.push_back(new Point(x, y));
    }
 
    return surroundPoints;
}

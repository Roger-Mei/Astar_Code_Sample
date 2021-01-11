#include <iostream>
#include <sstream>
#include <fstream>

#include "Astar.h"

using namespace std;
 
int main()
{   
    //Initialize the 2-D grid map, where 1 represents obstacles and 0
    //represents the available path. (Please feel free to use your own map!)
    vector<vector<int>> maze;
    
    // Create an input filestream
    ifstream inFile;
    inFile.open("/home/roger/Desktop/Astar_Code_Sample/src/map.txt");
    if(!inFile.is_open()) throw std::runtime_error("Could not open file!!!");
    string tmp;
    
    while(std::getline(inFile, tmp, '\n')){
        int n = tmp.length();
        vector<int> vec;
        for (int i = 0; i < n; ++i){
            if (isdigit(tmp[i])){
                vec.push_back(tmp[i]);
            }
        }
        maze.push_back(vec);
    }
    
    Astar astar;
    astar.InitAstar(maze);
 
    //Set up the init point and the end point
    Point start(1, 1);
    Point end(6, 10);
    //Use A* to search for the path
    list<Point*> path = astar.GetPath(start, end, false);

    //Print out the result
    for (auto &p : path){
        cout << '(' << p->x << ',' << p->y << ')' << endl;
    }
    cout << "Search Ends!" << endl;     
 
    cin.get();
    return 0;
}

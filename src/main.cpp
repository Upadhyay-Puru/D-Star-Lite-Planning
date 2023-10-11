#include <iostream>

#include <SFML/Graphics.hpp>
#include "d_star_lite.h"


int main()
{
    unsigned int rows = 20;
    unsigned int cols = 30;

    unsigned int cellSize = 40;

    sf::RenderWindow window(sf::VideoMode(cols * cellSize, rows * cellSize), "SFML Grid");

    DStarLite planner(&window, rows, cols, cellSize);

    std::vector<Node> path;

    std::pair<int,int> start = {0,0};
    std::pair<int,int> goal = {10,10};

    planner.plan(start, goal, path, {{0,1}, {1,1}});


}
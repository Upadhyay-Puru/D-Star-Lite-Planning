#ifndef D_STAR_LITE_H
#define D_STAR_LITE_H

#include <iostream>
#include <SFML/Graphics.hpp>
#include "nodes.h"
#include "global_planner.h"
#include "Priority_Queue.h"

#define _INF_ 0x2fffffff

typedef std::pair<int, int> Key;
#endif

class DStarLite: public global_planner::GlobalPlanner
{
    public: 
        DStarLite(sf::RenderWindow* window, unsigned int rows, unsigned int cols, unsigned int cellSize);

        virtual bool plan(std::pair<int,int> start, std::pair<int,int> goal, std::vector<Node>& path , std::unordered_set<std::pair<int,int>, hashFunction> obstacles);

        void ComputeShortestPath();

        Key calculateKey(const LNode* nide);

        int heuristic(const LNode* s1, const LNode* s2);

        void updateVertex(const LNode &s);

        int computeRHS(const LNode &s);

        void updateVertex( LNode* s);

        int computeRHS(const LNode* s);

        vector<LNode*> neighborStates(const LNode* s);

        int cost(const LNode* s1, const LNode* s2);

        vector<LNode*> getPath();

        LNode* peekNext(LNode* s);

    private:
        sf::RenderWindow* window_;
        unsigned int rows_;
        unsigned int cols_;
        unsigned int cellSize_;

        double km_; 
        PriorityQueue<Key, LNode*> U;
        std::vector<LNode*> LNode_Map_;
        LNode* s_start;
        LNode* s_goal;
        LNode* s_last;

        // vector<std::pair<int,int>> actions {{-1, 0}, {0, 1}, {1, 0}, {0, -1}};
        vector<std::pair<int,int>> actions {{-1, 0}, {-1, 1}, {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}};
        std::unordered_set<std::pair<int,int>, hashFunction> obstacles_;
        
};
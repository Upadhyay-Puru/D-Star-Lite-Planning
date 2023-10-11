#include "d_star_lite.h"


DStarLite::DStarLite(sf::RenderWindow* window, unsigned int rows, unsigned int cols, unsigned int cellSize)
    :window_(window), rows_(rows), cols_(cols), global_planner::GlobalPlanner(cols, rows)
{
    LNode_Map_.resize(ns_);

    for(int i=0;i<ns_;i++)
    {
        int x = i % nx_;
        int y = i / nx_;
        LNode_Map_[i] = new LNode(x, y);
        LNode_Map_[i]->g_ = _INF_;
        LNode_Map_[i]->rhs = _INF_;
    }
}


bool DStarLite::plan(std::pair<int,int> start, std::pair<int,int> goal, std::vector<Node>& path , std::unordered_set<std::pair<int,int>, hashFunction> obstacles)
{
    obstacles_ = obstacles;
    int start_index = grid2Index(start.first, start.second);
    int goal_index = grid2Index(goal.first, goal.second);

    s_start = LNode_Map_[start_index];

    s_goal = LNode_Map_[goal_index];

    s_last = s_start;

    s_goal->rhs = 0;

    U.push(s_goal, {heuristic(s_start, s_goal), 0});

    ComputeShortestPath();

    std::vector<LNode*> path_ = getPath();

    std::cout<<"Path Size: "<<path.size()<<std::endl;

    for(auto node: path_)
    {
        std::cout<<"x: "<<node->x_<<" "<<"y: "<<node->y_<<std::endl;
    }

    obstacles_ = {{0,1}, {1,0}};

    int temp1_index = grid2Index(0,1);
    int temp2_index = grid2Index(1,1);

    LNode* temp1 = LNode_Map_[temp1_index];
    LNode* temp2 = LNode_Map_[temp2_index];

    updateVertex(temp1);
    for (const auto &neighbor: neighborStates(temp1))
    {
        updateVertex(neighbor);
    }

    updateVertex(temp2);
    for (const auto &neighbor: neighborStates(temp2))
    {
        updateVertex(neighbor);
    }

    ComputeShortestPath();
    
    path_ = getPath();
    std::cout<<"Path Size: "<<path_.size()<<std::endl;
    for(auto node: path_)
    {
        std::cout<<"x: "<<node->x_<<" "<<"y: "<<node->y_<<std::endl;
    }

    return true;
}


Key DStarLite::calculateKey(const LNode* node)
{
    auto min = std::min(node->h_, node->rhs);
    return {min + heuristic(s_start, node) + km_, min};
}


int DStarLite::heuristic(const LNode* s1, const LNode* s2) 
{
  auto diff_x = s1->x_ - s2->x_;
  auto diff_y = s1->y_ - s2->y_;
  return std::abs(diff_x) + std::abs(diff_y);
  // return std::sqrt(std::pow(diff.first, 2) + std::pow(diff.second, 2));
}


vector<LNode*> DStarLite::neighborStates(const LNode* s)
{
  vector<LNode*> neighbors;
  for (const auto &a: actions)
  {
    auto next_x = s->x_ + a.first;
    auto next_y = s->y_ + a.second;

    int next_index = grid2Index(next_x, next_y);

    if (next_index < 0 || next_index >= ns_)
        continue;

    LNode* nextNode = LNode_Map_[next_index];


    if (obstacles_.find({nextNode->x_, nextNode->y_}) != obstacles_.end())
        continue;

    neighbors.push_back(nextNode);
  }
  return neighbors;
}


int DStarLite::cost(const LNode* s1, const LNode* s2)
{
  return (obstacles_.find({s1->x_, s1->y_}) != obstacles_.end() || obstacles_.find({s2->x_, s2->y_}) != obstacles_.end()) ? _INF_ : 1;
}


int DStarLite::computeRHS(const LNode* s)
{
    double RHS = _INF_;
    for (const auto &neighbor: neighborStates(s))
    {
        RHS = std::min(RHS, neighbor->g_ + cost(s, neighbor));
    }
    return RHS;
}


void DStarLite::updateVertex(LNode* s)
{
    if (s != s_goal)
    {
        s->rhs = computeRHS(s);
    }

    if (s->g_ != s->rhs)
    {
        U.update(s, calculateKey(s));
    }
    else
    {
        U.remove(s);
    }
}

LNode* DStarLite::peekNext(LNode* s)
{
  if (s == s_goal)
    return s;

  LNode* s_min = s;
  for (const auto &neighbor: neighborStates(s))
  {
    if (neighbor->g_ < s_min->g_)
    {
      s_min = neighbor;
    }
  }
  return s_min;
}

std::vector<LNode*> DStarLite::getPath()
{
  vector<LNode*> path = {s_last};
  LNode* s = s_last;

  while (s != s_goal)
  {
    LNode* s_next = peekNext(s);
    if (s_next == s)
        break;
    s = s_next;
    path.push_back(s);
  }

  return path;
}


void DStarLite::ComputeShortestPath()
{
    while (U.size())
    {
        auto [kOld, u] = U.top();

        if ((kOld >= calculateKey(s_start)) and (s_start->rhs == s_start->g_))
            break;

        U.pop();
        auto kNew = calculateKey(u);
        if (kOld < kNew)
        {
            U.push(u, kNew);
            continue;
        }

        if (u->g_ > u->rhs)
        {
            u->g_ = u->rhs;
        }
        else
        {
            u->g_ = _INF_;
            updateVertex(u);
        }
        for (const auto &neighbor: neighborStates(u))
        {
            updateVertex(neighbor);
        }
    }
}
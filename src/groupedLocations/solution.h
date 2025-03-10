#pragma once
#include <map>
#include <queue>
#include <algorithm>
#include "../instance.h"
#include "../orders/solution.h"

class GLEvent {
    public:
    int timestamp;
    int vehicle;

    GLEvent(int ts, int r): timestamp(ts), vehicle(r) {}
};

class GLEventCmp
{
    public:
  
    bool operator() (const GLEvent & lhs, const GLEvent & rhs) const {
        return lhs.timestamp > rhs.timestamp;
    }
};

typedef std::priority_queue<GLEvent,std::vector<GLEvent>,GLEventCmp> GLEventQueue;

class GLVehicle {
    public:

    GLVehicle(): noOfOrders(0) {}

    std::vector<int> sequence;
    std::map<int,std::vector<int>> orderMap;
    int noOfOrders;
};

typedef std::vector<GLVehicle> GLSolutionData;

// orders are distributed among vehicles, but orders in vehicle are grouped by locations
class GLSolution {
    public:

    Instance * inst;
    GLSolutionData solution;
    GoalFunction goalFunction;

    GLSolution(Instance * _inst): inst(_inst) {
        for (int r = 0; r < inst->v; ++r)
            solution.push_back(GLVehicle());
    }

    bool evaluate() {
        OSolution oSolution(inst);

        for (int r = 0; r < inst->v; ++r) {
            for (int k : solution[r].sequence) {
                for (int i : solution[r].orderMap[k])
                    oSolution.addOrder(i, r);
            }
        }

        if (oSolution.evaluate()) {
            goalFunction = oSolution.goalFunction;
            return true;
        }
        else
            return false;
    }

    void addOrder(int i, int r) {
        int loc = inst->locations[i];

        if (solution[r].orderMap.find(loc) == solution[r].orderMap.end())
            solution[r].sequence.push_back(loc);

        solution[r].orderMap[loc].push_back(i);
        solution[r].noOfOrders++;
    }

    void insertOrder(int i, int pos, int r) {
        int loc = inst->locations[i];

        if (solution[r].orderMap.find(loc) == solution[r].orderMap.end()) {
            int pos2;
            if (solution[r].noOfOrders == 0)
                pos2 = 0;
            else
                pos2 = solution[r].sequence.size() * pos / solution[r].noOfOrders;
            solution[r].sequence.insert(solution[r].sequence.begin() + pos2, loc);
        }
        
        solution[r].orderMap[loc].push_back(i);
        solution[r].noOfOrders++;
    }

    void removeAndInsertBatch(int p1, int p2, int r) {
        int b = solution[r].sequence[p1];
        solution[r].sequence.erase(solution[r].sequence.begin() + p1);
        solution[r].sequence.insert(solution[r].sequence.begin() + p2, b);
    }    

    void removeOrder(int i, int r) {
        int loc = inst->locations[i];

        solution[r].orderMap[loc].erase(std::remove(solution[r].orderMap[loc].begin(), solution[r].orderMap[loc].end(), i), solution[r].orderMap[loc].end());
        solution[r].noOfOrders--;

        if (solution[r].orderMap[loc].size() == 0) {
            solution[r].orderMap.erase(loc);
            solution[r].sequence.erase(std::remove(solution[r].sequence.begin(), solution[r].sequence.end(), loc), solution[r].sequence.end());
        }
    }

    void print() {
        for (int r = 0; r < inst->v; ++r) {
            GLVehicle & veh = solution[r];
            printf("%d: ", r);
            for (int k = 0; k < veh.sequence.size(); ++k) {
                printf("%d [", veh.sequence[k]);
                for (int i : veh.orderMap[veh.sequence[k]])
                    printf("%d ", i);
                printf("]   ");
            }
            printf("\n");
        }
    }

    static void tryAddToFront(std::vector<GLSolution> & front, GLSolution & sol) {
        for (int i = 0; i < front.size(); ++i) {
            if (front[i].goalFunction.dominates(sol.goalFunction) || (front[i].goalFunction.crit1 == sol.goalFunction.crit1 && front[i].goalFunction.crit2 == sol.goalFunction.crit2))
                return;
        }

        std::vector<GLSolution> remaining;
        for (int i = 0; i < front.size(); ++i) {
            if (! sol.goalFunction.dominates(front[i].goalFunction))
                remaining.push_back(front[i]);
        }
        front.swap(remaining);
        front.push_back(sol);
    }

    static void addFrontToFronts(std::vector<std::vector<GoalFunction>> & fronts, std::vector<GLSolution> & front) {
        std::vector<GoalFunction> f;
        for (int i = 0; i < front.size(); ++i)
            f.push_back(front[i].goalFunction);
        fronts.push_back(f);
    }    
};
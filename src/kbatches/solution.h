#pragma once
#include <map>
#include <queue>
#include <algorithm>
#include "../instance.h"
#include "../orders/solution.h"

class KBEvent {
    public:
    int timestamp;
    int vehicle;

    KBEvent(int ts, int r): timestamp(ts), vehicle(r) {}
};

class KBEventCmp
{
    public:
  
    bool operator() (const KBEvent & lhs, const KBEvent & rhs) const {
        return lhs.timestamp > rhs.timestamp;
    }
};

typedef std::priority_queue<KBEvent,std::vector<KBEvent>,KBEventCmp> KBEventQueue;

class KBVehicle {
    public:

    std::vector<std::vector<int>> sequence;
};

typedef std::vector<KBVehicle> KBSolutionData;

// batches of orders (from the same location) are distributed among vehicles, batch size is limited
class KBSolution {
    public:

    Instance * inst;
    KBSolutionData solution;
    GoalFunction goalFunction;
    
    KBSolution(Instance * _inst): inst(_inst) {
        for (int r = 0; r < inst->v; ++r)
            solution.push_back(KBVehicle());
    }

    bool evaluate() {
        OSolution oSolution(inst);

        for (int r = 0; r < inst->v; ++r) {
            for (int i = 0; i < solution[r].sequence.size(); ++i) {
                for (int j : solution[r].sequence[i])
                    oSolution.addOrder(j, r);
            }
        }

        if (oSolution.evaluate()) {
            goalFunction = oSolution.goalFunction;
            return true;
        }
        else
            return false;
    }
    
    void print() {
        for (int r = 0; r < inst->v; ++r) {
            KBVehicle & veh = solution[r];
            printf("%d: ", r);
            for (int i = 0; i < veh.sequence.size(); ++i) {
                for (int j = 0; j < veh.sequence[i].size(); ++j) {
                    printf("%d ", veh.sequence[i][j]);
                }
            }
            printf("\n");
        }
    }

    void addBatch(std::vector<int> & orders, int r) {
        solution[r].sequence.push_back(orders);
    }

    void insertBatch(int i, std::vector<int> & orders, int r) {
        solution[r].sequence.insert(solution[r].sequence.begin() + i, orders);
    }

    void removeBatch(int i, int r) {
        solution[r].sequence.erase(solution[r].sequence.begin() + i);
    }    

    static void tryAddToFront(std::vector<KBSolution> & front, KBSolution & sol) {
        for (int i = 0; i < front.size(); ++i) {
            if (front[i].goalFunction.dominates(sol.goalFunction) || (front[i].goalFunction.crit1 == sol.goalFunction.crit1 && front[i].goalFunction.crit2 == sol.goalFunction.crit2))
                return;
        }

        std::vector<KBSolution> remaining;
        for (int i = 0; i < front.size(); ++i) {
            if (! sol.goalFunction.dominates(front[i].goalFunction))
                remaining.push_back(front[i]);
        }
        front.swap(remaining);
        front.push_back(sol);
    }

    static void addFrontToFronts(std::vector<std::vector<GoalFunction>> & fronts, std::vector<KBSolution> & front) {
        std::vector<GoalFunction> f;
        for (int i = 0; i < front.size(); ++i)
            f.push_back(front[i].goalFunction);
        fronts.push_back(f);
    }    
};
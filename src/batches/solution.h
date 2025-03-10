#pragma once
#include <map>
#include <queue>
#include <algorithm>
#include "../instance.h"
#include "../orders/solution.h"

class BEvent {
    public:
    int timestamp;
    int vehicle;

    BEvent(int ts, int r): timestamp(ts), vehicle(r) {}
};

class BEventCmp
{
    public:
  
    bool operator() (const BEvent & lhs, const BEvent & rhs) const {
        return lhs.timestamp > rhs.timestamp;
    }
};

typedef std::priority_queue<BEvent,std::vector<BEvent>,BEventCmp> BEventQueue;

class BVehicle {
    public:

    std::vector<int> sequence;
    std::map<int,std::vector<int>> orderMap;
};

typedef std::vector<BVehicle> BSolutionData;

// locations are distributed among vehicles as batches (locations with too many orders are split among multiple batches)
class BSolution {
    public:

    Instance * inst;
    BSolutionData solution;
    GoalFunction goalFunction;

    BSolution(Instance * _inst): inst(_inst) {
        for (int r = 0; r < inst->v; ++r)
            solution.push_back(BVehicle());
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

    void print() {
        for (int r = 0; r < inst->v; ++r) {
            BVehicle & veh = solution[r];
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

    bool addBatch(int k, std::vector<int> & orders, int r) {
        if (solution[r].orderMap.find(k) == solution[r].orderMap.end()) {
            solution[r].sequence.push_back(k);
            solution[r].orderMap[k] = orders;
            return true;
        }
        else
            return false;
    }

    bool insertBatch(int k, int i, std::vector<int> & orders, int r) {
        if (solution[r].orderMap.find(k) == solution[r].orderMap.end()) {
            solution[r].sequence.insert(solution[r].sequence.begin() + i, k);
            solution[r].orderMap[k] = orders;
            return true;
        }
        else
            return false;
    }

    void removeBatch(int k, int r) {
        solution[r].orderMap.erase(k);
        solution[r].sequence.erase(std::remove(solution[r].sequence.begin(), solution[r].sequence.end(), k), solution[r].sequence.end());
    }    

    static bool tryAddToFront(std::vector<BSolution> & front, BSolution & sol) {
        for (int i = 0; i < front.size(); ++i) {
            if (front[i].goalFunction.dominates(sol.goalFunction) || (front[i].goalFunction.crit1 == sol.goalFunction.crit1 && front[i].goalFunction.crit2 == sol.goalFunction.crit2))
                return false;
        }

        std::vector<BSolution> remaining;
        for (int i = 0; i < front.size(); ++i) {
            if (! sol.goalFunction.dominates(front[i].goalFunction))
                remaining.push_back(front[i]);
        }
        front.swap(remaining);
        front.push_back(sol);
        return true;
    }

    static void addFrontToFronts(std::vector<std::vector<GoalFunction>> & fronts, std::vector<BSolution> & front) {
        std::vector<GoalFunction> f;
        for (int i = 0; i < front.size(); ++i)
            f.push_back(front[i].goalFunction);
        fronts.push_back(f);
    }    
};
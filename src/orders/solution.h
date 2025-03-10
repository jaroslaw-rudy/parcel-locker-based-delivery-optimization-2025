#pragma once
#include <map>
#include <queue>
#include <algorithm>
#include "../instance.h"

class OEvent {
    public:
    int timestamp;
    int vehicle;
    int type;   // 0 = travel, 1 = order servicing
    int arg1;   // from location (type 0), order id (type 1)
    int arg2;   // to location (order 0), locker size (type 1)

    OEvent(int ts, int r, int t, int a1, int a2): timestamp(ts), vehicle(r), type(t), arg1(a1), arg2(a2) {}
};

class OEventCmp
{
    public:
  
    bool operator() (const OEvent & lhs, const OEvent & rhs) const {
        return lhs.timestamp > rhs.timestamp;
    }
};

typedef std::priority_queue<OEvent,std::vector<OEvent>,OEventCmp> OEventQueue;

class OVehicle {
    public:
    std::vector<int> sequence;
};

typedef std::vector<OVehicle> OSolutionData;

// orders are distributed among vehicles
class OSolution {
    public:

    Instance * inst;
    OSolutionData solution;
    GoalFunction goalFunction;

    int currentTime;
    OEventQueue eventQueue;
    std::vector<std::vector<int>> freeLockers;
    std::vector<std::vector<int>> vehicleVisited;
    std::vector<int> vehicleLoad;
    std::vector<int> vehicleProgress;
    std::vector<int> vehicleOrder;
    std::vector<int> vehicleLocation;    

    OSolution(Instance * _inst): inst(_inst) {
        for (int r = 0; r < inst->v; ++r)
            solution.push_back(OVehicle());
    }

    bool initialize() {
        currentTime = inst->B;
        eventQueue = OEventQueue();
        goalFunction.crit1 = 0;
        goalFunction.crit2 = 0;
        freeLockers.clear();
        vehicleVisited.clear();
        vehicleLoad.clear();
        vehicleProgress.clear();
        vehicleOrder.clear();
        vehicleLocation.clear();    

        freeLockers;
        for (int k = 0; k <= inst->m; ++k) {
            freeLockers.push_back(std::vector<int>());
            if (k > 0) {
                for (int i = 0; i < 3; ++i)
                    freeLockers[k].push_back(inst->lockers[k][i]);
            }
        }

        for (int r = 0; r < inst->v; ++r) {
            vehicleVisited.push_back(std::vector<int>());
            vehicleLoad.push_back(0);
            for (int i : solution[r].sequence) {
                if (inst->deliveries[i])
                    vehicleLoad[r] += inst->weights[i];
            }
            if (vehicleLoad[r] > inst->C)
                return false;
            vehicleProgress.push_back(0);
            vehicleLocation.push_back(-1);
            if (solution[r].sequence.size() > 0) {
                vehicleOrder.push_back(solution[r].sequence[vehicleProgress[r]]);
                eventQueue.push(OEvent(currentTime, r, 0, 0, inst->locations[vehicleOrder[r]]));
            }
            else
                vehicleOrder.push_back(-1);
        }

        return true;               
    }

    bool agentDecision(int time, int r) {
        int i = vehicleOrder[r];
        if (inst->deliveries[i]) {
            for (int j = inst->sizes[i]; j < 3; ++j) {
                if (freeLockers[vehicleLocation[r]][j] > 0) {
                    eventQueue.push(OEvent(time, r, 1, i, j));
                    return true;
                }
            }

            vehicleVisited[r].push_back(vehicleLocation[r]);
            int closestDist = 0;
            int closestLocation = -1;
            for (int k = 1; k <= inst->m; ++k) {
                if (std::find(vehicleVisited[r].begin(), vehicleVisited[r].end(), k) == vehicleVisited[r].end()) {
                    if (closestLocation == -1 || inst->dist[vehicleLocation[r]][k] < closestDist) {
                        closestDist = inst->dist[vehicleLocation[r]][k];
                        closestLocation = k;
                    }
                }
            }
            if (closestLocation == -1)
                return false;
            eventQueue.push(OEvent(time, r, 0, vehicleLocation[r], closestLocation));
            return true;
        }
        else {
            eventQueue.push(OEvent(time, r, 1, i, inst->sizes[i]));
            return true;
        }
    }

    bool processEvent(OEvent & event) {
        currentTime = event.timestamp;
        int r = event.vehicle;
        int t = event.type;

        if (t == 0) {
            int from = event.arg1;
            int to = event.arg2;
            goalFunction.crit1 += inst->dist[from][to];
            if (to == 0)
                return true;
            else {
                vehicleLocation[r] = to;
                int time;
                if (from == 0)
                    time = currentTime + inst->getTravelTime(from, to, currentTime) + inst->P;
                else
                    time = currentTime + inst->getTravelTime(from, to, currentTime) + inst->P * 2;
                if (! agentDecision(time, r))
                    return false;
            }
            return true;
        }
        else {
            int i = event.arg1;
            int j = event.arg2;

            if (inst->deliveries[i]) {
                vehicleLoad[r] -= inst->weights[i];
                freeLockers[vehicleLocation[r]][j]--;
                if (currentTime + inst->S > goalFunction.crit2)
                    goalFunction.crit2 = currentTime + inst->S;
            }
            else {
                vehicleLoad[r] += inst->weights[i];
                if (vehicleLoad[r] > inst->C)
                    return false;
                freeLockers[vehicleLocation[r]][j]++;
            }

            vehicleProgress[r]++;
            vehicleVisited[r].clear();                 

            if (vehicleProgress[r] == solution[r].sequence.size())
                eventQueue.push(OEvent(currentTime + inst->S, r, 0, vehicleLocation[r], 0));
            else {
                vehicleOrder[r] = solution[r].sequence[vehicleProgress[r]];
                int oldLocation = vehicleLocation[r];
                int newLocation = inst->locations[vehicleOrder[r]];
                if (newLocation != oldLocation)
                    eventQueue.push(OEvent(currentTime + inst->S, r, 0, oldLocation, newLocation));
                else if (! agentDecision(currentTime + inst->S, r))
                    return false;
            }            
        }
        return true;
    }

    bool evaluate() {
        if (! initialize())
            return false;
        while (! eventQueue.empty()) {
            OEvent event = eventQueue.top();
            eventQueue.pop();
            if (! processEvent(event))
                return false;
        }
        return true;
    }

    void print() {
        for (int r = 0; r < inst->v; ++r) {
            OVehicle & veh = solution[r];
            printf("%d: ", r);
            for (int i = 0; i < veh.sequence.size(); ++i)
                    printf("%d ", i);
            printf("\n");
        }
    }

    void addOrder(int i, int r) {
        solution[r].sequence.push_back(i);
    }

    void insertOrder(int i, int pos, int r) {
        solution[r].sequence.insert(solution[r].sequence.begin() + pos, i);
    }

    void removeOrder(int i, int r) {
        solution[r].sequence.erase(std::remove(solution[r].sequence.begin(), solution[r].sequence.end(), i), solution[r].sequence.end());
    }

    void removeOrderByPos(int pos, int r) {
        solution[r].sequence.erase(solution[r].sequence.begin() + pos);
    }        

    static void tryAddToFront(std::vector<OSolution> & front, OSolution & sol) {
        for (int i = 0; i < front.size(); ++i) {
            if (front[i].goalFunction.dominates(sol.goalFunction) || (front[i].goalFunction.crit1 == sol.goalFunction.crit1 && front[i].goalFunction.crit2 == sol.goalFunction.crit2))
                return;
        }

        std::vector<OSolution> remaining;
        for (int i = 0; i < front.size(); ++i) {
            if (! sol.goalFunction.dominates(front[i].goalFunction))
                remaining.push_back(front[i]);
        }
        front.swap(remaining);
        front.push_back(sol);
    }

    static void addFrontToFronts(std::vector<std::vector<GoalFunction>> & fronts, std::vector<OSolution> & front) {
        std::vector<GoalFunction> f;
        for (int i = 0; i < front.size(); ++i)
            f.push_back(front[i].goalFunction);
        fronts.push_back(f);
    } 
};

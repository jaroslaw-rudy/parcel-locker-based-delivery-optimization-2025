#pragma once
#include "../utils.h"
#include "solution.h"

class BGreedyAlgorithm {
    public:

    void run(Instance * inst, std::vector<BSolution> & front) {
        front.clear();

        BSolution solution(inst);

        std::map<int,std::vector<int>> preBatches;
        std::map<int,int> loads;

        for (int i = 0; i < inst->n; ++i) {
            preBatches[inst->locations[i]].push_back(i);
            loads[inst->locations[i]] += inst->weights[i];
        }

        std::vector<std::vector<int>> batches;

        for (auto el : preBatches) {
            if (loads[el.first] <= inst->C) {
                batches.push_back(el.second);
            }
            else {
                int needed = loads[el.first] / inst->C + 1;
                int perVehicle = loads[el.first] / needed;
                int load = 0;
                std::vector<int> batch;
                for (int i : el.second) {
                    if (load + inst->weights[i] <= perVehicle) {
                        batch.push_back(i);
                        load += inst->weights[i];
                    }
                    else {
                        batches.push_back(batch);
                        batch.clear();
                        batch.push_back(i);
                        load = inst->weights[i];
                    }
                }
                if (load > 0)
                    batches.push_back(batch);
            }
        }

        for (int i = 0; i < batches.size(); ++i) {
            int k = inst->locations[batches[i][0]];
            if (i < batches.size() - 1) {
                std::vector<GoalFunction> candidates;
                std::vector<int> indices;
                for (int r = 0; r < inst->v; ++r) {
                    bool succ = solution.addBatch(k, batches[i], r);

                    if (succ && solution.evaluate()) {
                        candidates.push_back(solution.goalFunction);
                        indices.push_back(r);
                    }

                    solution.removeBatch(k, r);
                }

                if (candidates.size() == 0)
                    return;
                int chosenIndex = GoalFunction::topsis(candidates);
                solution.addBatch(k, batches[i], indices[chosenIndex]);
            }
            else {
                for (int r = 0; r < inst->v; ++r) {
                    bool succ = solution.addBatch(k, batches[i], r);
                    if (succ && solution.evaluate()) {
                        BSolution::tryAddToFront(front, solution);
                    }
                    solution.removeBatch(k, r);
                }
            }
        }
    }
};
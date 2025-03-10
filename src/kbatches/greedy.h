#pragma once
#include "../utils.h"
#include "solution.h"

class KBGreedyAlgorithm {
    public:

    void run(Instance * inst, int batchSize, std::vector<KBSolution> & front) {
        front.clear();

        std::map<int,std::vector<int>> preBatches;

        for (int i = 0; i < inst->n; ++i)
            preBatches[inst->locations[i]].push_back(i);

        std::vector<std::vector<int>> batches;
        std::vector<int> batch;

        int load = 0;
        for (auto preBatch : preBatches) {
            if (batch.size() > 0) {
                batches.push_back(batch);
                batch.clear();
                load = 0;
            }
            for (int i : preBatch.second) {
                if (batch.size() >= batchSize || load + inst->weights[i] > inst->C) {
                    batches.push_back(batch);
                    batch.clear();
                    load = 0;
                }
                batch.push_back(i);
                load += inst->weights[i];
            }
        }
        if (batch.size() > 0)
            batches.push_back(batch);

        KBSolution solution(inst);

        for (int i = 0; i < batches.size(); ++i) {
            if (i < batches.size() - 1) {
                std::vector<GoalFunction> candidates;
                std::vector<int> indices;
                for (int r = 0; r < inst->v; ++r) {
                    solution.addBatch(batches[i], r);
                    if (solution.evaluate()) {
                        candidates.push_back(solution.goalFunction);
                        indices.push_back(r);
                    }
                    solution.removeBatch(solution.solution[r].sequence.size() - 1, r);
                }

                int chosenIndex = GoalFunction::topsis(candidates);
                solution.addBatch(batches[i], indices[chosenIndex]);
            }
            else {
                for (int r = 0; r < inst->v; ++r) {
                    solution.addBatch(batches[i], r);
                    if (solution.evaluate()) {
                        KBSolution::tryAddToFront(front, solution);
                    }
                    solution.removeBatch(solution.solution[r].sequence.size() - 1, r);
                }
            }
        }
    }
};
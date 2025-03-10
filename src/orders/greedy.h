#pragma once
#include "../utils.h"
#include "solution.h"

class OGreedyAlgorithm {
    public:

    void run(Instance * inst, std::vector<OSolution> & front) {
        front.clear();

        OSolution solution(inst);

        for (int i = 0; i < inst->n; ++i) {
            if (i < inst->n - 1) {
                std::vector<GoalFunction> candidates;
                std::vector<int> indices;
                for (int r = 0; r < inst->v; ++r) {
                    solution.addOrder(i, r);
                    if (solution.evaluate()) {
                        candidates.push_back(solution.goalFunction);
                        indices.push_back(r);
                    }
                    solution.removeOrder(i, r);
                }

                int chosenIndex = GoalFunction::topsis(candidates);
                solution.addOrder(i, indices[chosenIndex]);
            }
            else {
                for (int r = 0; r < inst->v; ++r) {
                    solution.addOrder(i, r);
                    if (solution.evaluate()) {
                        OSolution::tryAddToFront(front, solution);
                    }
                    solution.removeOrder(i, r);
                }
            }
        }
    }
};
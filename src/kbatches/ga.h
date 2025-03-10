#pragma once
#include "../utils.h"
#include "greedy.h"

struct KBCmp {
    std::vector<double> siw;

    KBCmp(std::vector<double> & _siw): siw(_siw) {}

    bool operator() (int i, int j) {
        return (siw[i] > siw[j]);
    }
};

class KBGAAlgorithm {
    public:

    Instance * inst;
    std::mt19937 mtg;
    std::vector<KBSolution> & front;
    double timeLimit;
    int populationSize;
    std::vector<KBSolution> population;
    std::vector<KBSolution> children;
    double mutationChance;
    double elitPercentage;
    int noOfBatches = 0;
    int batchSize;

    KBGAAlgorithm(Instance * _inst, long _seed, std::vector<KBSolution> & _front, double _timeLimit, int _populationSize, double _mutationChance, double _elitPercentage, int _batchSize):
        inst(_inst),
        mtg(_seed),
        front(_front),
        timeLimit(_timeLimit),
        populationSize(_populationSize),
        mutationChance(_mutationChance),
        elitPercentage(_elitPercentage),
        batchSize(_batchSize)
    {}

    int run() {
        struct timespec timep;
	    clock_gettime(CLOCK_REALTIME, & timep);

        front.clear();

        createInitialPopulation();

        for (int r = 0; r < inst->v; ++r)
            noOfBatches += population[0].solution[r].sequence.size();

        int iter = 0;
        while (timer(timep) < timeLimit) {
            createChildren();
            mutateChildren();
            createNewPopulation();
            iter++;
        }
        return iter;
    }

    void createNewPopulation() {
        std::vector<KBSolution> newPopulation;
        std::vector<GoalFunction> set;

        int childrenPairsNeeded = (populationSize * (1.0 - elitPercentage) / 2.0 + 0.5);
        int eliteSize = populationSize - childrenPairsNeeded * 2;

        for (int i = 0; i < populationSize; ++i)
            set.push_back(population[i].goalFunction);
        
        std::vector<double> siw;
        GoalFunction::topsis2(set, siw);
        std::vector<int> indices;
        for (int i = 0; i < populationSize; ++i)
            indices.push_back(i);
        KBCmp cmp(siw);
        std::sort(indices.begin(), indices.end(), cmp);

        for (int i = 0; i < eliteSize; ++i)
            newPopulation.push_back(population[indices[i]]);
        for (int i = 0; i < children.size(); ++i)
            newPopulation.push_back(children[i]);
        population.swap(newPopulation);
    }

    void createChildren() {
        children.clear();
        int childrenPairsNeeded = (populationSize * (1.0 - elitPercentage) / 2.0 + 0.5);
        while (children.size() < 2 * childrenPairsNeeded) {
            int p1 = tournament();
            int p2 = tournament();
            if (p1 != p2) {
                createChild(p1, p2);
                createChild(p2, p1);
            }
        }
    }

    void createChild(int p1, int p2) {
        std::uniform_int_distribution<int> uid(0, noOfBatches - 1);
        std::uniform_int_distribution<int> uid2(0, noOfBatches - 2);
        int c1 = uid(mtg);
        int c2 = uid2(mtg);
        if (c2 >= c1)
            c2++;
        else
            std::swap(c1, c2);
        std::map<int,int> used;
        KBSolution child(inst);
        int pos = 0;

        KBSolution & parent1 = population[p1];
        KBSolution & parent2 = population[p2];

        for (int r = 0; r < inst->v; ++r) {
            for (int i = 0; i < parent1.solution[r].sequence.size(); ++i) {
                if (pos >= c1 && pos <= c2) {
                    child.addBatch(parent1.solution[r].sequence[i], r);
                    used[parent1.solution[r].sequence[i][0]] = 1;
                }
                pos++;
            }
        }

        for (int r = 0; r < inst->v; ++r) {
            auto veh = parent2.solution[r];
            for (int i = 0; i < veh.sequence.size(); ++i) {
                if (used.count(veh.sequence[i][0]) == 0) {
                    for (int s = 0; s < inst->v; ++s) {
                        int t = (r + s) % inst->v;
                        child.addBatch(veh.sequence[i], t);
                        if (child.evaluate())
                            break;
                        else
                            child.removeBatch(child.solution[t].sequence.size() - 1, t);
                    }
                    used[veh.sequence[i][0]] = 1;
                }
            }
        }

        if (used.size() == noOfBatches)
            children.push_back(child);
        else
            children.push_back(parent1);
    }

    int tournament() {
        std::vector<GoalFunction> set;
        std::vector<int> indices;
        int tournamentSize = sqrt(populationSize) + 0.5;
        std::uniform_int_distribution<int> uid(0, populationSize - 1);
        for (int i = 0; i < tournamentSize; ++i)  {
            int p = uid(mtg);
            set.push_back(population[p].goalFunction);
            indices.push_back(p);
        }
        int idx = GoalFunction::topsis(set);
        return indices[idx];
    }

    void mutateChildren() {
        std::vector<KBSolution> mutated;
        for (int p = 0; p < children.size(); ++p) {
            KBSolution copy = children[p];
            applyRandomInsert(copy);
            if (copy.evaluate()) {
                mutated.push_back(copy);
                KBSolution::tryAddToFront(front, copy);
            }
            else
                mutated.push_back(children[p]);
        }
        children.swap(mutated);
    }

    void createInitialPopulation() {
        KBGreedyAlgorithm greedy;
        greedy.run(inst, batchSize, front);

        std::uniform_int_distribution<int> uid(0, front.size() - 1);

        std::vector<KBSolution> pop;

        int specimenSize = 0;
        for (int r = 0; r < inst->v; ++r)
            specimenSize += front[0].solution[r].sequence.size();

        for (int p = 0; p < populationSize; ++p) {
            int g = uid(mtg);
            population.push_back(front[g]);
            pop.push_back(front[g]);
            for (int i = 0; i < specimenSize / 10; ++i)
                applyRandomInsert(pop[p]);
        }

        for (int p = 0; p < populationSize; ++p) {
            if (pop[p].evaluate()) {
                population[p] = pop[p];
                KBSolution::tryAddToFront(front, population[p]);
            }
        }
    }

    void applyRandomInsert(KBSolution & specimen) {
        std::uniform_int_distribution<int> uiv(0, inst->v - 1);
        int r = uiv(mtg);
        int s = uiv(mtg);
        int p;
        int p2;
        if (specimen.solution[r].sequence.size() == 0)
            return;

        if (r != s) {
            std::uniform_int_distribution<int> uip(0, specimen.solution[r].sequence.size() - 1);
            std::uniform_int_distribution<int> uip2(0, specimen.solution[s].sequence.size());
            p = uip(mtg);
            p2 = uip2(mtg);
        }
        else {
            std::uniform_int_distribution<int> uip(0, specimen.solution[r].sequence.size() - 1);
            std::uniform_int_distribution<int> uip2(0, specimen.solution[r].sequence.size() - 1);
            p = uip(mtg);
            p2 = uip2(mtg);
        }
        std::vector<int> batch = specimen.solution[r].sequence[p];
        specimen.removeBatch(p, r);
        specimen.insertBatch(p2, batch, s);
    }
};

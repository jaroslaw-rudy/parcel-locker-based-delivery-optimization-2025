#pragma once
#include <algorithm>

class GoalFunction {

    public:

    int crit1;
    int crit2;

    GoalFunction(int c1 = 0, int c2 = 0): crit1(c1), crit2(c2) {}

    bool dominates(GoalFunction & other) {
        return (crit1 <= other.crit1 && crit2 <= other.crit2) && (crit1 < other.crit1 || crit2 < other.crit2);
    }

    static bool hviSortCmp(GoalFunction & l, GoalFunction & r) { return (l.crit1 < r.crit1); }

    static double hvi(std::vector<GoalFunction> & front, double z1, double z2) {
        double volume = 0.0;
        std::sort(front.begin(), front.end(), hviSortCmp);
        for (int i = 0; i < front.size() - 1; ++i)
            volume += (front[i + 1].crit1 - front[i].crit1) * (z2 - front[i].crit2);
        volume += (z1 - front[front.size() - 1].crit1) * (z2 - front[front.size() - 1].crit2);
        return volume;
    }

    static void hvis(std::vector<std::vector<GoalFunction>> & fronts, std::vector<double> & volumes, double multiplier = 1.2) {
        volumes.clear();
        double z1 = -1, z2 = -1;
        for (int i = 0; i < fronts.size(); ++i)
            for (int j = 0; j < fronts[i].size(); ++j) {
                if (fronts[i][j].crit1 > z1) z1 = fronts[i][j].crit1;
                if (fronts[i][j].crit2 > z2) z2 = fronts[i][j].crit2;
            }
        z1 *= multiplier;
        z2 *= multiplier;
        if (z2 == 0) {
            std::vector<double> bests;
            for (int i = 0; i < fronts.size(); ++i) {
                double best = -1.0;
                for (int j = 0; j < fronts[i].size(); ++j) {
                    if (fronts[i][j].crit1 < best || best < 0.0)
                        best = fronts[i][j].crit1;
                }
                bests.push_back(best);
            }

            double worst = 0.0;
            for (int i = 0; i < bests.size(); ++i) {
                if (bests[i] > worst)
                    worst = bests[i];
            }

            for (int i = 0; i < bests.size(); ++i)
                volumes.push_back(worst / bests[i]);
        }
        else {
            for (int i = 0; i < fronts.size(); ++i)
                volumes.push_back(hvi(fronts[i], z1, z2));
        }
    }

    static int topsis(std::vector<GoalFunction> & solutions) {
        int m = solutions.size();
        int n = 2;
        double root1 = 0.0, root2 = 0.0;

        double r[m][n];
        double Aw1 = -1, Ab1 = std::numeric_limits<double>::max();
        double Aw2 = -1, Ab2 = std::numeric_limits<double>::max();

        for (int k = 0; k < m; k++)
        {
            root1 += ((double) (solutions[k].crit1)) * ((double) (solutions[k].crit1));
            root2 += ((double) (solutions[k].crit2)) * ((double) (solutions[k].crit2));
        }
        root1 = sqrt(root1);
        root2 = sqrt(root2);

        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++)
                r[i][j] = (j == 0 ? (root1 == 0 ? solutions[i].crit1 : solutions[i].crit1 / root1) : (root2 == 0 ? solutions[i].crit2 : solutions[i].crit2 / root2)) * 0.5;

            if (r[i][0] > Aw1) Aw1 = r[i][0];
            if (r[i][1] > Aw2) Aw2 = r[i][1];

            if (r[i][0] < Ab1) Ab1 = r[i][0];
            if (r[i][1] < Ab2) Ab2 = r[i][1];
        }

        std::vector<double> diw;
        std::vector<double> dib;
        for (int i = 0; i < m; i++)
        {
            diw.push_back(sqrt((r[i][0] - Aw1) * (r[i][0] - Aw1) + (r[i][1] - Aw2) * (r[i][1] - Aw2)));
            dib.push_back(sqrt((r[i][0] - Ab1) * (r[i][0] - Ab1) + (r[i][1] - Ab2) * (r[i][1] - Ab2)));
        }

        double maxSiw = -1.0;
        double siw;
        int idx;
        for (int i = 0; i < m; i++) {
            siw = (diw[i] + dib[i]) == 0 ? diw[i] : diw[i] / (diw[i] + dib[i]);
            if (siw > maxSiw || maxSiw < 0) {
                maxSiw = siw;
                idx = i;
            }
        }
        return idx;
    }

    static void topsis2(std::vector<GoalFunction> & solutions, std::vector<double> & siw) {
        int m = solutions.size();
        int n = 2;
        double root1 = 0.0, root2 = 0.0;

        double r[m][n];
        double Aw1 = -1, Ab1 = std::numeric_limits<double>::max();
        double Aw2 = -1, Ab2 = std::numeric_limits<double>::max();

        for (int k = 0; k < m; k++)
        {
            root1 += ((double) (solutions[k].crit1)) * ((double) (solutions[k].crit1));
            root2 += ((double) (solutions[k].crit2)) * ((double) (solutions[k].crit2));
        }
        root1 = sqrt(root1);
        root2 = sqrt(root2);

        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++)
                r[i][j] = (j == 0 ? (root1 == 0 ? solutions[i].crit1 : solutions[i].crit1 / root1) : (root2 == 0 ? solutions[i].crit2 : solutions[i].crit2 / root2)) * 0.5;

            if (r[i][0] > Aw1) Aw1 = r[i][0];
            if (r[i][1] > Aw2) Aw2 = r[i][1];

            if (r[i][0] < Ab1) Ab1 = r[i][0];
            if (r[i][1] < Ab2) Ab2 = r[i][1];
        }

        std::vector<double> diw;
        std::vector<double> dib;
        for (int i = 0; i < m; i++)
        {
            diw.push_back(sqrt((r[i][0] - Aw1) * (r[i][0] - Aw1) + (r[i][1] - Aw2) * (r[i][1] - Aw2)));
            dib.push_back(sqrt((r[i][0] - Ab1) * (r[i][0] - Ab1) + (r[i][1] - Ab2) * (r[i][1] - Ab2)));
        }

        siw.clear();
        for (int i = 0; i < m; i++)
            siw.push_back((diw[i] + dib[i]) == 0 ? diw[i] : diw[i] / (diw[i] + dib[i]));
    }
};


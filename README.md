Algrithms and dataset instances for publication "Multi-criteria parcel locker-based vehicle routing with pickup and delivery". Contents:

Dataset directory contains 320 base instances used in the main experiments in the paper.

Matrix directory contains real-life distances for each of the considered cities.

Src directory contains code, inclugind:

* main.cpp -- the code to generate basic 320 instances
* instance.h -- instance generation procedure, instance printing format and some of the assumed data (city populations, number of locations, average in-day travel speed etc.)
* Directories orders, groupedLocations, batches and kbatches contain code:
  * solution.h -- objective function computation (incuding feasibility) through a Discrete Event Simulation,
  * greedy.h -- greedy heuristic implementation,
  * ga.h -- Genetic Algorithm implementation.

#include "instance.h"

int main(int argc, char ** argv) {

    Instance::init();

    int seed = 100;
    for (int city = 0; city < 8; ++city) {
        for (int i = 1; i < 5; ++i) {
            double factor = 0.001 * i;
            for (int it = 0; it < 10; ++it) {
                seed += 100;
                Instance * inst = Instance::createRandom(
                    seed,           // seed
                    city,           // city
                    factor,         // ordersFactor
                    0.25,           // pickupToDeliveryFactor
                    1.0,            // locationFactor
                    0.016,          // vehicleFactor
                    0.1,            // occupancyProbability
                    60,             // parkDepartTime
                    30,             // serviceTime
                    700,            // vehicleCapacity
                    32400           // startingTime
                );

                char str[200];
                sprintf(str, "dataset/%d_%d_%.3lf.txt", seed, city, factor);
                inst->printToFile(str);
            }
        }        
    }

    return 0;
}

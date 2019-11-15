#include "distance_t.hpp"


int main() {
    using namespace raspigcd;
    std::vector<generic_position_t<double, 2>> path_points_with_velocity = {
        {0.0,0.0},
        {10.0,10.0},
        {20.0,11.0},
        {20.0,5.0},
        {5.0,5.0},
        {0.0,0.0},
        {-20.0,20.0},
    };
    double t = 0.0, dt = 0.01;
    follow_path_with_velocity<2>(
    path_points_with_velocity,
    [&](const auto& position){
        std::cout << t << " " << position[0] << " " << position[1] << std::endl;
        t += dt;
    },
    dt,
    0.001
    );
    return 0;
}




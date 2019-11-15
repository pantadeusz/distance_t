#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "distance_t.hpp"
#include <thread>
#include <vector>

using namespace raspigcd;

TEST_CASE("distance_t - check vector operations", "[common][distance_t]")
{
    SECTION("when two points are the same relative to the distance_t, then the angle should be 0")
    {
        distance_t middle = {100, 200, 150, 50};
        distance_t a = {130, 200, 150, 50};
        distance_t b = {130, 200, 150, 50};

        auto ret = middle.angle(a, b);
        REQUIRE(ret == 0);
    }
    SECTION("when two points are the same line relative to the distance_t, then the angle should be M_PI")
    {
        distance_t middle = {100, 200, 150, 50};
        distance_t a = {100 + 10, 200 + 20, 150 + 5, 50 + 1};
        distance_t b = {100 - 10, 200 - 20, 150 - 5, 50 - 1};

        auto ret = middle.angle(a, b);
        REQUIRE(ret == Approx(M_PI));
    }
    SECTION("90 deg test should result in M_PI/2")
    {
        distance_t middle = {0, 0, 0, 0};
        distance_t a = {0, 1, 0, 0};
        distance_t b = {1, 0, 0, 0};

        auto ret = middle.angle(a, b);
        REQUIRE(ret == Approx(M_PI / 2));
    }
    SECTION("90 deg test should result in M_PI/2")
    {
        distance_t middle = {0, 0, 0, 0};
        distance_t a = {0, -1, 0, 0};
        distance_t b = {1, 0, 0, 0};

        auto ret = middle.angle(a, b);
        REQUIRE(ret == Approx(M_PI / 2));
    }
    SECTION("90 deg test should result in M_PI/2 also in reverse order")
    {
        distance_t middle = {0, 0, 0, 0};
        distance_t b = {0, -1, 0, 0};
        distance_t a = {1, 0, 0, 0};

        auto ret = middle.angle(a, b);
        REQUIRE(ret == Approx(M_PI / 2));
    }
    //SECTION("180 deg test should result in M_PI")
    //{
    //    distance_t a = {143.849,-17.018,0,0};
    //    distance_t middle = {144.611,-19.4733,0,0};
    //    distance_t b = {144.526,-23.4526,0,0};
    //
    //    auto ret = middle.angle(a,b);
    //    REQUIRE(ret == Approx(M_PI));
    //}
}


TEST_CASE("distance_t - following path with given positions and velocities", "[common][distance_t][follow_path_with_velocity]")
{
    SECTION("when two points are the same relative to the distance_t, then the angle should be 0")
    {
        std::vector<generic_position_t<double, 2>> path_points_with_velocity = {
            {0.0, 0.0},
            {10.0, 10.0},
            {20.0, 11.0},
            {20.0, 5.0},
            {5.0, 5.0},
            {0.0, 0.0},
            {-20.0, 20.0},
        };
        double t = 0.0, dt = 0.01;
        std::vector < double > distances;
        std::vector < double > velocities;
        std::vector < double > times;
        follow_path_with_velocity<2>(
            path_points_with_velocity,
            [&](const auto& position) {
                distances.push_back(position[0]);
                velocities.push_back(position[1]);
                times.push_back(t);
                t += dt;
            },
            dt,
            0.001);
        REQUIRE(times.size() == 998);
        REQUIRE(distances.at(200) == Approx(9.6862653211));
        REQUIRE(velocities.at(200) == Approx(9.841882605));
    }
}


/*
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

*/

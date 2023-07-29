#include "freefall_dragforce_simulation.h"
#include <gtest/gtest.h>
#include <variant>
#include <iterator>
#include <algorithm>

struct Simulator
{

    [[nodiscard]] FreeFallSim::FreeFallSimPlot operator() (FreeFallSim::FreeFallConstGravitySimlation& const_gravity_simulation)
    {
        return const_gravity_simulation.run_sim();
    }
    [[nodiscard]] FreeFallSim::FreeFallSimPlot operator() (FreeFallSim::FreeFallNewtonGravitySimlation& newton_gravity_simulation)
    {
        return newton_gravity_simulation.run_sim();
    }
};


class FreeFallGravitationalModelTest: public ::testing::Test
{
    protected:
    void SetUp() override
    {
        FreeFallSim::FreeFallObjProfile freefall_sim_obj;
        freefall_sim_obj.fluid_density_air = 1.22;
        freefall_sim_obj.kDragCoefficient = 0.47;
        freefall_sim_obj.mass_of_object = 0.0577;
        freefall_sim_obj.radius_of_object = 0.06661/2;
        FreeFallSim::FreeFallSimulationProfile freefall_sim_vars;
        freefall_sim_vars.velocity = 0.0;
        freefall_sim_vars.position = 400;
        freefall_sim_vars.gravity_acceleration = 9.81;
        freefall_sim_vars.time_step = 0.01;
        freefall_sim_vars.sample_factor = 10; // print every status_time sec
        freefall_sim_vars.finish_time = std::numeric_limits<int>::max();
        FreeFallSim::FreeFallSimPlot freefall_sim_plot_vars;
        
        FreeFallSimModels freefall_sim_const_grav_model
        {std::in_place_type<FreeFallSim::FreeFallConstGravitySimlation>,
        freefall_sim_obj, freefall_sim_vars, freefall_sim_plot_vars};
        
        FreeFallSimModels freefall_sim_newton_grav_model
        {std::in_place_type<FreeFallSim::FreeFallNewtonGravitySimlation>,
        freefall_sim_obj, freefall_sim_vars, freefall_sim_plot_vars};
       
        sim_models.emplace_back(freefall_sim_const_grav_model);
        sim_models.emplace_back(freefall_sim_newton_grav_model);

        FreeFallSim::FreeFallSimulationProfile freefall_sim_vars_with_40m;
        freefall_sim_vars_with_40m.velocity = 0.0;
        freefall_sim_vars_with_40m.position = 40;
        freefall_sim_vars_with_40m.gravity_acceleration = 9.81;
        freefall_sim_vars_with_40m.time_step = 0.01;
        freefall_sim_vars_with_40m.sample_factor = 10; // print every status_time sec
        freefall_sim_vars_with_40m.finish_time = std::numeric_limits<int>::max();
        
        FreeFallSimModels freefall_sim_newton_grav_model_40m
        {std::in_place_type<FreeFallSim::FreeFallNewtonGravitySimlation>,
        freefall_sim_obj, freefall_sim_vars_with_40m, freefall_sim_plot_vars};
        sim_models.emplace_back(freefall_sim_newton_grav_model_40m);

        
        FreeFallSimModels freefall_sim_const_grav_model_40m
        {std::in_place_type<FreeFallSim::FreeFallConstGravitySimlation>,
        freefall_sim_obj, freefall_sim_vars_with_40m, freefall_sim_plot_vars};
        sim_models.emplace_back(freefall_sim_const_grav_model_40m);
    }


    void TearDown() override
    {}
    public:
    using FreeFallSimModels = std::variant<FreeFallSim::FreeFallConstGravitySimlation, FreeFallSim::FreeFallNewtonGravitySimlation>;
    FreeFallSim::FreeFallObjProfile freefall_sim_obj;
    FreeFallSim::FreeFallSimulationProfile freefall_sim_vars;
    FreeFallSim::FreeFallSimPlot freefall_sim_plot_vars;
    std::vector<FreeFallSimModels> sim_models;
};


TEST_F(FreeFallGravitationalModelTest, GivenBallProfileConstGravityModelTerminalVelocityReachedWith400mHeight) 
{
    auto sim_plot_vars_const_grav = std::visit(Simulator{}, sim_models[0]);
    std::vector<double> transform_netforce{};
    std::transform(
        sim_plot_vars_const_grav.netforce_data.begin(), sim_plot_vars_const_grav.netforce_data.end(),
        std::back_inserter(transform_netforce), [](double element){ return FreeFallSim::round_to(element, 0.01); }
    );
    auto netforce_zero_count = std::count(std::begin(transform_netforce), std::end(transform_netforce), 0);
    EXPECT_GT(netforce_zero_count, 5);
}

TEST_F(FreeFallGravitationalModelTest, GivenBallProfileNewtonGravityModelTerminalVelocityReachedWith400mHeight) 
{
    auto sim_plot_vars_newton_grav = std::visit(Simulator{}, sim_models[1]);
    std::vector<double> transform_netforce{};
    std::transform(
        sim_plot_vars_newton_grav.netforce_data.begin(), sim_plot_vars_newton_grav.netforce_data.end(),
        std::back_inserter(transform_netforce), [](double element){ return FreeFallSim::round_to(element, 0.01); }
    );
    auto netforce_zero_count = std::count(std::begin(transform_netforce), std::end(transform_netforce), 0);
    EXPECT_GT(netforce_zero_count, 5);
}

TEST_F(FreeFallGravitationalModelTest, GivenBallProfileNewtonGravityModelTerminalVelocityDoesNotReachWith40mHeight) 
{
    auto sim_plot_vars_newton_grav = std::visit(Simulator{}, sim_models[2]);
    std::vector<double> transform_netforce{};
    std::transform(
        sim_plot_vars_newton_grav.netforce_data.begin(), sim_plot_vars_newton_grav.netforce_data.end(),
        std::back_inserter(transform_netforce), [](double element){ return FreeFallSim::round_to(element, 0.01); }
    );
    auto netforce_zero_count = std::count(std::begin(transform_netforce), std::end(transform_netforce), 0);
    EXPECT_EQ(netforce_zero_count, 0);
}

TEST_F(FreeFallGravitationalModelTest, GivenBallProfileConstGravityModelTerminalVelocityDoesNotReachWith40mHeight) 
{
    auto sim_plot_vars_const_grav = std::visit(Simulator{}, sim_models[3]);
    std::vector<double> transform_netforce{};
    std::transform(
        sim_plot_vars_const_grav.netforce_data.begin(), sim_plot_vars_const_grav.netforce_data.end(),
        std::back_inserter(transform_netforce), [](double element){ return FreeFallSim::round_to(element, 0.01); }
    );
    auto netforce_zero_count = std::count(std::begin(transform_netforce), std::end(transform_netforce), 0);
    EXPECT_EQ(netforce_zero_count, 0);
}


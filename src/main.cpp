#include "freefall_dragforce_simulation.h"
#include <iostream>

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

int main()
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
    freefall_sim_vars.sample_factor = 10; 
    freefall_sim_vars.finish_time = std::numeric_limits<int>::max();
    FreeFallSim::FreeFallSimPlot freefall_sim_plot_vars;

  
    using FreeFallSimModels = std::variant<FreeFallSim::FreeFallConstGravitySimlation, FreeFallSim::FreeFallNewtonGravitySimlation>;
    FreeFallSimModels freefall_sim_const_grav_model{std::in_place_type<FreeFallSim::FreeFallConstGravitySimlation>,freefall_sim_obj, freefall_sim_vars, freefall_sim_plot_vars};
    FreeFallSimModels freefall_sim_newton_grav_model{std::in_place_type<FreeFallSim::FreeFallNewtonGravitySimlation>,freefall_sim_obj, freefall_sim_vars, freefall_sim_plot_vars};
    // An example to parameterize the simulation over a uniform method call
    std::vector<FreeFallSimModels> sim_models;
    sim_models.emplace_back(freefall_sim_const_grav_model);
    sim_models.emplace_back(freefall_sim_newton_grav_model);
    
    // The use of visit variant combination allows for maximum flexibility, scalability while
    // keeping SOLID principles intact.
    auto sim_plot_const_grav = std::visit(Simulator{}, sim_models[0]);
    sim_plot_const_grav.plot_time_vs_position();
    sim_plot_const_grav.plot_time_vs_velocity();
    sim_plot_const_grav.plot_time_vs_force();
    std::cout<< "Launching Newton Gravity Model" << "\n";
    auto sim_plot_newton_grav = std::visit(Simulator{}, sim_models[1]);
    sim_plot_newton_grav.plot_time_vs_position();
    sim_plot_newton_grav.plot_time_vs_velocity();
    sim_plot_newton_grav.plot_time_vs_force();
    return 0;
}
#ifndef FREEFALL_DRAGFROCE_SIMULATION_H
#define FREEFALL_DRAGFROCE_SIMULATION_H
#pragma once
#include <iostream>
#include <variant>
#include <vector>
#include <math.h>
#include <cmath>
#include <matplot/matplot.h>

#define ENABLE_CONSOLE_PRINT 1

namespace FreeFallSim
{

    enum class GravityProfile { ConstantGravity, NewtonGravitationModel };
    
    // Struct representing ball characteristics used to approximate ball object dynamics
    // which is used in drag force and weight force equations. 
    struct FreeFallObjProfile
    {
        double kDragCoefficient;         // (Cd)           
        double mass_of_object;           // (m) kg
        double radius_of_object;         // (r) m
        double fluid_density_air;        // (rho) 
    };

    // Assuming Earth as Planet for default constants
    struct FreeFallSimulationProfile
    {
        double kUniversalGravitationConst{6.673e-11}; // (G)Nm/kg^2 Universal Gravitational Constant
        double kMassOfPlanet{5.98e24};                // (M) kg Mass of Earth as Default
        double kRadiusOfPlanet{6.38e6};               // (R) m Radius of the Earth as default
        double gravity_acceleration{9.8};             // (g) m/s^2, acceleration due to gravity
        double position;                              // (x) m position of the object in free fall Sim it is height 
        double velocity;                              // (v) m/s velocity of object  
        float time_step;                              // (t) simulation update factor  
        int sample_factor;                            //  factor to increase or decrease output on plots and samples collected  
        int finish_time{std::numeric_limits<int>::max()}; // used to finish the simulation if it falls within the calculation bounds
    };

    class FreeFallSimPlot
    {
        // Data struct for matplot to plot the FreeFall 

        public:
        std::vector<double> time_data;
        std::vector<double> velocity_data;
        std::vector<double> netforce_data;
        std::vector<double> position_data;

        void plot_time_vs_velocity()
        {
            matplot::plot(this->time_data, this->velocity_data);
            matplot::title("Time(s) vs Velocity(m/s)");
            matplot::xlabel("Time(s)");
            matplot::ylabel("Velocity(m/s)");
            matplot::show();
        }
        void plot_time_vs_position()
        {
            matplot::plot(this->time_data, this->position_data);
            matplot::title("Time(s) vs Position(m)");
            matplot::xlabel("Time(s)");
            matplot::ylabel("Position(m)");
            matplot::show();
        }

        void plot_time_vs_force()
        {
            matplot::plot(this->time_data, this->netforce_data);
            matplot::title("Time(s) vs NetForce(N)");
            matplot::xlabel("Time(s)");
            matplot::ylabel("NetForce(N)");
            matplot::show();
        }
    };

    //   Fd = Cd*rho*v^2*pi*r^2  
    static auto drag_force = [](const FreeFallObjProfile& sim_obj_profile, const FreeFallSimulationProfile& sim_vars) 
    {
        return sim_obj_profile.kDragCoefficient * sim_obj_profile.fluid_density_air * std::pow(sim_vars.velocity, 2) * M_PI * std::pow(sim_obj_profile.radius_of_object, 2);
    };
    
    // Fw = GMm / (R+x)^2
    static auto newton_gravitational_force = [](const FreeFallObjProfile& sim_obj_profile, const FreeFallSimulationProfile& sim_vars)
    {
        return sim_vars.kUniversalGravitationConst * sim_vars.kMassOfPlanet * sim_obj_profile.mass_of_object / std::pow((sim_vars.kRadiusOfPlanet + sim_vars.position), 2);
    };
    
    // Fw = mg
    constexpr auto const_weight_force = [](const FreeFallObjProfile& sim_obj_profile, const FreeFallSimulationProfile& sim_vars)
    { return sim_obj_profile.mass_of_object *  sim_vars.gravity_acceleration; };

    // Used to round of to a provided specificaiton
    /*
        round_to(10.0078, 0.001) = 10.008
        round_to(10.0078, 0.01) = 10.01
        round_to(10.0078, 0.1) = 10
        round_to(10.0078, 1) = 10
        round_to(10.0078, 2) = 10
        round_to(10.0078, 3) = 9
        round_to(10.0078, 4) = 12
    */
    static auto round_to(double value, double precision = 1.0)
    {
        return std::round(value / precision) * precision;
    }

    //NOTE: Based on given two gravity model classes were created deliberately separate
    // this is to show the strength of modern C++ 17 standard provided features aka visit and variant 


    // Simulator class to hold method for free fall simulation under the fluid drag
    class FreeFallConstGravitySimlation
    {
        public:
        // ctor init FreeFall simulation data structs
        explicit FreeFallConstGravitySimlation(FreeFallObjProfile sim_obj_profile, FreeFallSimulationProfile sim_freefall_vars, 
        FreeFallSimPlot sim_plot_vars): m_sim_obj_profile(std::move(sim_obj_profile)), 
        m_freefall_sim_vars(std::move(sim_freefall_vars)), m_freefall_sim_plot_vars(std::move(sim_plot_vars))
        {}
        
        FreeFallSimPlot run_sim();

        FreeFallConstGravitySimlation(const FreeFallConstGravitySimlation& src) = default;
        FreeFallConstGravitySimlation& operator=(const FreeFallConstGravitySimlation& src) = default;
        FreeFallConstGravitySimlation(FreeFallConstGravitySimlation&& src) = default;
        FreeFallConstGravitySimlation& operator=(FreeFallConstGravitySimlation&& src) = default;
        ~FreeFallConstGravitySimlation() = default;

        private:
        FreeFallObjProfile m_sim_obj_profile;
        FreeFallSimulationProfile m_freefall_sim_vars;
        FreeFallSimPlot m_freefall_sim_plot_vars;

    };

    class FreeFallNewtonGravitySimlation
    {

        public:
        // ctor init FreeFall simulation data structs
        explicit FreeFallNewtonGravitySimlation(FreeFallObjProfile sim_obj_profile, FreeFallSimulationProfile sim_freefall_vars, 
        FreeFallSimPlot sim_plot_vars): m_sim_obj_profile(std::move(sim_obj_profile)), 
        m_freefall_sim_vars(std::move(sim_freefall_vars)), m_freefall_sim_plot_vars(std::move(sim_plot_vars))
        {}
        
        FreeFallSimPlot run_sim();


        FreeFallNewtonGravitySimlation(const FreeFallNewtonGravitySimlation& src) = default;
        FreeFallNewtonGravitySimlation& operator=(const FreeFallNewtonGravitySimlation& src) = default;
        FreeFallNewtonGravitySimlation(FreeFallNewtonGravitySimlation&& src) = default;
        FreeFallNewtonGravitySimlation& operator=(FreeFallNewtonGravitySimlation&& src) = default;
        ~FreeFallNewtonGravitySimlation() = default;

        private:
        FreeFallObjProfile m_sim_obj_profile;
        FreeFallSimulationProfile m_freefall_sim_vars;
        FreeFallSimPlot m_freefall_sim_plot_vars;

    };

}

#endif


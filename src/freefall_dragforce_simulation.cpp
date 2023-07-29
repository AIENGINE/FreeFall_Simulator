#include "freefall_dragforce_simulation.h"

#define ENABLE_CONSOLE_PRINT 1

namespace FreeFallSim
{
        
        FreeFallSimPlot FreeFallConstGravitySimlation::run_sim()
        {
            double drag_force_on_obj;
            double current_height;
            double sim_time;    
            auto& height = m_freefall_sim_vars.position;
            // using ref. is necessary as initial_velocity is required to be reflected in the free function where
            // user initialized structs are passed
            auto& initial_velocity = m_freefall_sim_vars.velocity;  
            auto net_force{0.0};
            auto acceleration{0.0};
            auto new_velocity{0.0};
            auto new_height{0.0};
            current_height = height;
            auto status_interval{0.0};
        
            
            while (current_height >= 0)
            {
                drag_force_on_obj = -drag_force(m_sim_obj_profile, m_freefall_sim_vars);
                
                net_force = drag_force_on_obj + const_weight_force(m_sim_obj_profile, m_freefall_sim_vars);
                acceleration = net_force / m_sim_obj_profile.mass_of_object;
                new_velocity = initial_velocity + acceleration * m_freefall_sim_vars.time_step;
                new_height = current_height - (initial_velocity * m_freefall_sim_vars.time_step) - (0.5 * acceleration * std::pow(m_freefall_sim_vars.time_step, 2));
                current_height = new_height;
                initial_velocity = new_velocity;
                
                auto current_cmp_height = round_to(current_height, 1);
                auto samples_output = (int)(current_cmp_height ) % m_freefall_sim_vars.sample_factor;
                if (samples_output == 0 )
                {
                    sim_time = round_to((height - current_height) / initial_velocity, 0.001);
                    if (sim_time > m_freefall_sim_vars.finish_time)
                        break;
                    #if ENABLE_CONSOLE_PRINT    
                    std::cout<< "Time: "<< sim_time<< " ";
                    std::cout<< "Height: "<< current_height<< " ";
                    std::cout<< "NetForce: "<< round_to(net_force, 0.0001) << " ";
                    std::cout<< "Velocity: "<< round_to(initial_velocity, 0.001)<< " \n";
                    #endif //ENABLE_CONSOLE_PRINT
                    m_freefall_sim_plot_vars.time_data.emplace_back(sim_time);
                    m_freefall_sim_plot_vars.position_data.emplace_back(current_height);
                    m_freefall_sim_plot_vars.velocity_data.emplace_back(-initial_velocity);
                    m_freefall_sim_plot_vars.netforce_data.emplace_back(net_force);             
                }

            }
            return std::move(m_freefall_sim_plot_vars);
        }


        FreeFallSimPlot FreeFallNewtonGravitySimlation::run_sim()
        {
            double drag_force_on_obj;
            double current_height;
            double sim_time;    
            auto& height = m_freefall_sim_vars.position;
            // using ref. is necessary as initial_velocity is required to be reflected in the free function where
            // user initialized structs are passed
            auto& initial_velocity = m_freefall_sim_vars.velocity;  
            auto net_force{0.0};
            auto acceleration{0.0};
            auto new_velocity{0.0};
            auto new_height{0.0};
            current_height = height;
            auto status_interval{0.0};
        
            
            while (current_height >= 0)
            {
                drag_force_on_obj = -drag_force(m_sim_obj_profile, m_freefall_sim_vars);
                
                net_force = drag_force_on_obj + newton_gravitational_force(m_sim_obj_profile, m_freefall_sim_vars);
                acceleration = net_force / m_sim_obj_profile.mass_of_object;
                new_velocity = initial_velocity + acceleration * m_freefall_sim_vars.time_step;
                new_height = current_height - (initial_velocity * m_freefall_sim_vars.time_step) - (0.5 * acceleration * std::pow(m_freefall_sim_vars.time_step, 2));
                current_height = new_height;
                initial_velocity = new_velocity;
                
                auto current_cmp_height = round_to(current_height, 1);
                auto samples_output = (int)(current_cmp_height ) % m_freefall_sim_vars.sample_factor;
                if (samples_output == 0 )
                {
                    sim_time = round_to((height - current_height) / initial_velocity, 0.001);
                    if (sim_time > m_freefall_sim_vars.finish_time)
                        break;
                    #if ENABLE_CONSOLE_PRINT    
                    std::cout<< "Time: "<< sim_time<< " ";
                    std::cout<< "Height: "<< current_height<< " ";
                    std::cout<< "NetForce: "<< round_to(net_force, 0.0001) << " ";
                    std::cout<< "Velocity: "<< round_to(initial_velocity, 0.001)<< " \n";
                    #endif //ENABLE_CONSOLE_PRINT
                    m_freefall_sim_plot_vars.time_data.emplace_back(sim_time);
                    m_freefall_sim_plot_vars.position_data.emplace_back(current_height);
                    m_freefall_sim_plot_vars.velocity_data.emplace_back(-initial_velocity);
                    m_freefall_sim_plot_vars.netforce_data.emplace_back(net_force);             
                }

            }
            return std::move(m_freefall_sim_plot_vars);
        }
}



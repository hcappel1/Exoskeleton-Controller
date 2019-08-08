#ifndef STIFFNESS_ESTIMATE_HPP
#define STIFFNESS_ESTIMATE_HPP

#include <apptronik_system/system.hpp>
#include <math.h>
#include <chrono>
#include <random>

#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <fstream>
#include <string.h>

/**  A clean and empty example Nodelet Controller for you to copy and customize. */
namespace stiffness_estimate
{
  using namespace apptronik_system;

  /**  An empty Nodelet controller that does nothing. */
  class StiffnessEstimate: public nodelet::Nodelet
  {
  public:
    boost::shared_ptr<SystemLoop> m_sys;
    boost::shared_ptr<boost::thread> m_system_thread;


    // States
    unsigned int m_fault_status;
    unsigned int m_prev_fault_status;
    double m_actuator_force;
    double m_joint_torque;
    double m_cuff_torque;
    double m_actuator_position;
    double m_motor_current;
    double m_joint_position;
    double m_joint_velocity;

    // Commands
    double m_cmd;
    
    // Waveform Params
    double m_sin_freq_rps;
    double m_sin_amplitude;
    double m_sin_duration;
    double m_sin_offset;
    double m_sin_power;
    int    m_sin_mode;
    double m_sin_cmd;

    double m_cri_freq_rps;

    // Gravity Compensation
    double m_load_mass;
    double m_distance;
    double m_exo_mass;
    double m_gravity_cmd;
    unsigned m_seed = std::chrono::system_clock::now().time_since_epoch().count();

    // Damping Compensation
    double m_vir_damp;
    double m_damping_cmd;
    double m_x_damp;
    int    m_ignore_damp;
    double m_lag_zero_damp;
    double m_lag_pole_damp;

    // Guassian Generator
    double m_x_rand;
    int    m_ignore_rand;
    double m_lag_zero_rand;
    double m_lag_pole_rand;

    // Amplification
    double m_x_cuff;
    int    m_ignore_cuff;
    double m_amplification_cmd;
    double m_amplification_factor;
    double m_lag_zero_cuff;
    double m_lag_pole_cuff;

    std::vector<double> offset_list;
    int index;
    int step;
    double last_time;
    double rest_start_time;
    int port_number;
    double Offset;

    char input_buffer[256];
    LibSerial::SerialStream serial_port ;

    StiffnessEstimate();
    ~StiffnessEstimate();
    void onInit();
    void systemThread();
    void loop(const double& time, const dBitset& fault_bitmap );
    bool exponentialsinSignal(const double& elapsed_time, double& signal, double& power, double& gravity_signal, int &index, double &Offset);
    bool portinit();
    void dataread();
  };
}


#endif // STIFFNESS_ESTIMATE_HPP

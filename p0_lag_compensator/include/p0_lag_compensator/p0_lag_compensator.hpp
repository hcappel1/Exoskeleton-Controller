#ifndef P0_LAG_COMPENSATOR_HPP
#define P0_LAG_COMPENSATOR_HPP

#include <apptronik_system/system.hpp>

/**  A clean and empty example Nodelet Controller for you to copy and customize. */
namespace p0_lag_compensator
{
  using namespace apptronik_system;

  /**  An empty Nodelet controller that does nothing. */
  class LagAmplificationController: public nodelet::Nodelet
  {
  public:
    boost::shared_ptr<SystemLoop> m_sys;
    boost::shared_ptr<boost::thread> m_system_thread;

    // States
    unsigned int m_fault_status;
    unsigned int m_prev_fault_status;
    double m_actuator_force;
    // double m_actuator_torque;
    double m_joint_torque;
    double m_cuff_torque;
    double m_damp_torque;
    double m_actuator_position;
    double m_motor_current;
    double m_joint__position;
    double m_motor_pos;
    double m_motor_vel;
    double m_joint_velocity;

    // integrator states
    double m_x_1; // integrating error
    double m_x_2; // integrating error
    double m_x_3; // integrating error
    double m_x_4; // integrating error
    double m_x_5; // integrating error
    double m_x_c; // integrating error
    double m_xd_c;
    double m_xdd_c;
    double m_x_d; // integrating error
    double m_xd_d;
    double m_xdd_d;
    double m_error;

    // Commands
    double m_cmd_joint_torque;
    double m_cmd;
    double m_chirp;

    // Waveform Params
    double m_chirp_low_freq_rps;
    double m_chirp_high_freq_rps;
    double m_chirp_amplitude;
    double m_chirp_duration;
    double m_chirp_offset;
    int    m_chirp_mode;
    
    // Control parameters
    double m_lag1_zero_rad_s;
    double m_lag1_pole_rad_s;
    double m_lag2_zero_rad_s;
    double m_lag2_pole_rad_s;
    double m_lag3_zero_rad_s;
    double m_lag3_pole_rad_s;
    double m_lag4_zero_rad_s;
    double m_lag4_pole_rad_s;
    double m_lag5_zero_rad_s;
    double m_lag5_pole_rad_s;
    double m_cuff_filter_wn_rad_s;
    double m_cuff_filter_zt_rad_s;
    double m_damp_filter_wn_rad_s;
    double m_damp_filter_zt_rad_s;
    double m_meta_compensator_gain;
    double m_amplification_factor;
    double m_compensator_gain;
    double m_damping;
    int    m_amplification_mode;
    bool   m_ignore_cuff;
    bool   m_ignore_damp;
    double m_added_mass;
    double m_arm_length;
    bool   m_en_grav_comp;
    bool   m_en_load_attn;


    LagAmplificationController();
    ~LagAmplificationController();
    void onInit();
    void systemThread();
    void loop(const double& time, const dBitset& fault_bitmap );
    bool exponentialChirpSignal(const double& elapsed_time, double& signal);
  };
}


#endif // P0_LAG_COMPENSATOR_HPP

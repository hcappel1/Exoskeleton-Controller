#include <stiffness_estimate/stiffness_estimate.hpp>

namespace stiffness_estimate
{
  using namespace apptronik_system;

  // For Nodelets, use onInit() method instead of constructor
  StiffnessEstimate::StiffnessEstimate() {}


  StiffnessEstimate::~StiffnessEstimate()
  {
    m_sys->stop();
    m_system_thread->join();
    ros::Duration(0.1).sleep(); //make sure everyone's done using our pointers
  }

  // onInit() function should not block. It should initialize and then return.
  void StiffnessEstimate::onInit()
  {
    m_system_thread.reset(new boost::thread(boost::bind(&StiffnessEstimate::systemThread, this)));
  }

  //set up ctrl interface here
  void StiffnessEstimate::systemThread()
  {
    std::string slave_name = std::string("Axon_proto_v1");
    ros::NodeHandle nh = getPrivateNodeHandle();

    // User calls SystemLoop Constructor:
    m_sys.reset(new SystemLoop(boost::bind(&StiffnessEstimate::loop, this, _1, _2), nh, {slave_name}));

    nh.param("chirp_low_freq_rps" , m_chirp_low_freq_rps  ,   1.);
    nh.param("chirp_high_freq_rps", m_chirp_high_freq_rps , 100.);
    nh.param("chirp_amplitude"    , m_chirp_amplitude     , 100.);
    nh.param("chirp_offset"       , m_chirp_offset        , 200.);
    nh.param("chirp_duration"     , m_chirp_duration      ,  60.);
    nh.param("chirp_mode"         , m_chirp_mode          ,    4);
    nh.param("load_mass"          , m_load_mass           ,   0.);
    nh.param("exo_mass"           , m_exo_mass            ,   0.);
    nh.param("distance"           , m_distance            ,   0.);
    nh.param("ignore_cuff"        , m_ignore_cuff         ,    1);
    nh.param("amp_factor"         , m_amplification_factor,   1.);
    nh.param("lag_zero_cuff"      , m_lag_zero_cuff       ,6000.);
    nh.param("lag_pole_cuff"      , m_lag_pole_cuff       , 100.);
    nh.param("ignore_damp"        , m_ignore_damp         ,    1);
    nh.param("vir_damp"           , m_vir_damp            ,   0.);
    nh.param("lag_zero_damp"      , m_lag_zero_damp       ,6000.);
    nh.param("lag_pole_damp"      , m_lag_pole_damp       , 100.);
    nh.param("ignore_rand"        , m_ignore_rand         ,    1);    
    nh.param("lag_zero_rand"      , m_lag_zero_rand       ,6000.);
    nh.param("lag_pole_rand"      , m_lag_pole_rand       , 100.);
    nh.param("critical_freq_rps"  , m_cri_freq_rps        ,  32.);
   
    NODELET_INFO_STREAM("#################################");
    NODELET_INFO_STREAM("###  SysID Experiment Params  ###");
    NODELET_INFO_STREAM("#################################");
    NODELET_INFO_STREAM("chirp_low_freq_rps:   " << m_chirp_low_freq_rps  );
    NODELET_INFO_STREAM("chirp_high_freq_rps:  " << m_chirp_high_freq_rps );
    NODELET_INFO_STREAM("chirp_amplitude:      " << m_chirp_amplitude     );
    NODELET_INFO_STREAM("chirp_offset:         " << m_chirp_offset        );
    NODELET_INFO_STREAM("chirp_duration:       " << m_chirp_duration      );
    NODELET_INFO_STREAM("chirp_mode:           " << m_chirp_mode          );
    NODELET_INFO_STREAM("load_mass:            " << m_load_mass           );
    NODELET_INFO_STREAM("exo_mass:             " << m_exo_mass            );
    NODELET_INFO_STREAM("distance:             " << m_distance            );
    NODELET_INFO_STREAM("ignore_cuff:          " << m_ignore_cuff         );
    NODELET_INFO_STREAM("amp_factor:           " << m_amplification_factor);
    NODELET_INFO_STREAM("lag_zero_cuff:        " << m_lag_zero_cuff       );
    NODELET_INFO_STREAM("lag_pole_cuff:        " << m_lag_pole_cuff       );
    NODELET_INFO_STREAM("ignore_damp:          " << m_ignore_damp         );
    NODELET_INFO_STREAM("vir_damp:             " << m_vir_damp            );
    NODELET_INFO_STREAM("lag_zero_damp:        " << m_lag_zero_damp       );
    NODELET_INFO_STREAM("lag_pole_damp:        " << m_lag_pole_damp       );
    NODELET_INFO_STREAM("ignore_rand:          " << m_ignore_rand         );
    NODELET_INFO_STREAM("lag_zero_rand:        " << m_lag_zero_rand       );
    NODELET_INFO_STREAM("lag_pole_rand:        " << m_lag_pole_rand       );
    NODELET_INFO_STREAM("critical_freq_rps:    " << m_cri_freq_rps        );
    NODELET_INFO_STREAM("#################################");

    // User sets run mode for each slave:
    // m_sys->setRunMode(ACTUATOR_FORCE);

    // User registers a state ptr for each MISO topic with desired state info
    m_sys->registerStatePtr( &m_fault_status , "diag__faults__x" , slave_name);
    m_sys->registerStatePtr( &m_actuator_force , "actuator__force__N" , slave_name);
    m_sys->registerStatePtr( &m_joint_torque , "js__joint__effort__Nm" , slave_name);
    m_sys->registerStatePtr( &m_cuff_torque , "cuff__ati__torque__Nm" , slave_name);
    m_sys->registerStatePtr( &m_joint_position , "js__joint__position__rad" , slave_name);
    m_sys->registerStatePtr( &m_motor_current , "motor__current__A" , slave_name);
    m_sys->registerStatePtr( &m_actuator_position , "actuator__position__m" , slave_name);
    m_sys->registerStatePtr( &m_joint_velocity , "js__joint__velocity__radps" , slave_name);
    
    // m_sys->registerStatePtr( &m_cuff_torque , "cmd__joint__effort__mirror__Nm" );


    // User registers a command ptr for each MOSI topic corresponding to the desired mode
    if (m_chirp_mode == 4)
      {      
        m_sys->setRunMode(static_cast<AxonMode>(4));
        m_sys->registerCommandPtr( &m_cmd, "cmd__actuator__effort__n");
    }
    else if (m_chirp_mode == 6)
      {      
        m_sys->setRunMode(static_cast<AxonMode>(6), slave_name);
        m_sys->registerCommandPtr( &m_cmd, "cmd__joint__effort__Nm" , slave_name);
    }

    // User should initialize commands to a reasonable value
    m_cmd = 0;
    m_prev_fault_status = 0;

    // m_seed =sm_chirp_offset, m_chirp_amplitude);

    // construct a trivial random generator engine from a time-based seed:
    // unsigned m_seed = std::chrono::system_clock::now().time_since_epoch().count();
    // std::default_random_engine m_generator (m_seed);
    // std::normal_distribution<double> m_distribution (m_chirp_offset, m_chirp_amplitude);

    //power_list = {0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7};
    // power_list = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0};
    //index      = 0;
    //last_time  = 0;
    //rest_start_time = 0;
    // Must call start to start loop
    m_sys->start();
  }


  //define control behavior here
  void StiffnessEstimate::loop(const double& time, const dBitset& fault_bitmap)
  {
    m_seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine m_generator (m_seed);
    std::normal_distribution<double> m_distribution (m_chirp_offset, m_chirp_amplitude);
    m_cmd = 0.;
    m_gravity_cmd = (m_load_mass * m_distance + m_exo_mass * m_distance / 2.) * 9.81 * sin(m_joint_position);
    // m_damping_cmd = m_vir_damp * m_joint_velocity;
    m_chirp_power = 0.;

    static double last_print_time = 0.0;
    static double last_run_time   = 0.0;

    double dt = time - last_run_time;
    last_run_time = time;

    double error_cuff = - m_cuff_torque * m_amplification_factor + m_cuff_torque;
    double dx_cuff = m_lag_pole_cuff * (error_cuff - m_x_cuff);
    double lag_result_cuff = dx_cuff / m_lag_zero_cuff + m_x_cuff;
    m_x_cuff += dt * dx_cuff;

    double error_damp = m_vir_damp * m_joint_velocity;
    double dx_damp = m_lag_pole_damp * (error_damp - m_x_damp);
    double lag_result_damp = dx_damp / m_lag_zero_damp + m_x_damp;
    m_x_damp += dt * dx_damp;

    double error_rand = m_distribution(m_generator);
    double dx_rand = m_lag_pole_rand * (error_rand - m_x_rand);
    double lag_result_rand = dx_rand / m_lag_zero_rand + m_x_rand;
    m_x_rand += dt * dx_rand;

    if (m_ignore_cuff == 0)
    {
      m_amplification_cmd = lag_result_cuff;
    }
    else
    {
      m_amplification_cmd = 0.;
    }

    if (m_ignore_damp == 0)
    {
      m_damping_cmd = lag_result_damp;
    }
    else
    {
      m_damping_cmd = 0.;
    }

    if (m_ignore_rand == 0)
    {
      m_chirp_cmd = lag_result_rand;
      if (m_chirp_cmd < m_chirp_offset - 3. * m_chirp_amplitude)
      {
        m_chirp_cmd = m_chirp_offset - 3. * m_chirp_amplitude;
      }
      else if (m_chirp_cmd > m_chirp_offset + 3. * m_chirp_amplitude)
      {
        m_chirp_cmd = m_chirp_offset + 3. * m_chirp_amplitude;
      }
    }
    else
    {
      m_chirp_cmd = 0.;
    }

    if(fault_bitmap.any())
    {
      static double last_print_time = 0.0;
      if (time - last_print_time > 1.0 || (m_prev_fault_status == 0))
      {
        NODELET_INFO_STREAM("System Faulted... (Fault code: " << m_fault_status << ")" );
        last_print_time = time;
      }
    }
    else
    {
      static double waveform_start_time = 0.0;
      static bool test_done = false;

      if(m_prev_fault_status)
      {
        NODELET_INFO_STREAM("Starting SysID Experiment...");
        waveform_start_time = time;
      }

      if(!test_done && time-waveform_start_time < m_chirp_duration+5.) {
        NODELET_INFO_STREAM("m_cmd: " << m_chirp_cmd);
        if (m_ignore_rand == 1)
        {
          exponentialChirpSignal(time-waveform_start_time, m_chirp_cmd, m_chirp_power);
        }        
        LoggingManager::logData("gravity_cmd", m_gravity_cmd);
        LoggingManager::logData("amplification_cmd", m_amplification_cmd);
        LoggingManager::logData("damping_cmd", m_damping_cmd);
        LoggingManager::logData("chirp_cmd", m_chirp_cmd);
        LoggingManager::logData("chirp_power", m_chirp_power);
      }
      else{
        NODELET_INFO_STREAM("Experiment Finished!!!");
        test_done = true;
        ros::shutdown();
      }

    }

    m_cmd = m_gravity_cmd + m_amplification_cmd + m_chirp_cmd + m_damping_cmd;
    m_prev_fault_status = m_fault_status;
  }

  bool StiffnessEstimate::exponentialChirpSignal(const double& elapsed_time, double& signal, double& power)
  {
    static double prev_sample_time = 0;
    static double prev_effective_angle = 0;

    double effective_angle, effective_switching_freq_rps;
    double range = m_chirp_high_freq_rps - m_chirp_low_freq_rps;
    double period = elapsed_time - prev_sample_time;

    double k = pow(m_chirp_high_freq_rps / m_chirp_low_freq_rps, 1 / m_chirp_duration);
    double currentfreq = 0;

    prev_sample_time = elapsed_time-5.;

    // if (elapsed_time - start_time <= 5)
    //     { 
    //       signal = (m_chirp_offset / 2.0) * cos((1.0 / 10.0) * 2.0 * 3.1415925 * (elapsed_time - rest_start_time)) + (m_chirp_offset / 2.0);
    //     }

    if(elapsed_time >= 5.0)
    {
      effective_angle = m_chirp_low_freq_rps * (pow(k, elapsed_time-5) - 1) / log(k);
      currentfreq = m_chirp_low_freq_rps * pow(k, elapsed_time-5);
      NODELET_INFO_STREAM("chirp_freq_rps:  " << log10(currentfreq) << "; " << "amplification: " << - m_amplification_cmd / m_cuff_torque);


      if (currentfreq <= m_cri_freq_rps)
        {
          signal = m_chirp_amplitude * sin(effective_angle) + m_chirp_offset; 
          power  = 1;
        }

      else if (currentfreq <= 64.)
        {
          signal = pow((currentfreq /m_cri_freq_rps), 2.) * m_chirp_amplitude * sin(effective_angle) + m_chirp_offset;
          power  = 1;
        }

      else
        {
          signal = 0.;
          power  = 0;
        }
    
      // signal = 0.1 * m_chirp_amplitude * sin(effective_angle) + m_chirp_offset;

      if(period <= 0.0)
      {
        effective_switching_freq_rps = 0.0;
      }

      else
      {
        effective_switching_freq_rps = (effective_angle - prev_effective_angle) / period;
      }

      prev_effective_angle = effective_angle;

      if(effective_switching_freq_rps > m_chirp_high_freq_rps)
      {
        return true;
      }
    }

    else
    {
      signal = m_chirp_offset;
      effective_switching_freq_rps = 0.0;
      power=0.;
    }

    return false;
  }
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(stiffness_estimate::StiffnessEstimate, nodelet::Nodelet)

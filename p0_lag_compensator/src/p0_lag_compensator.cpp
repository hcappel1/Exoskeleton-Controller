#include <p0_lag_compensator/p0_lag_compensator.hpp>



namespace p0_lag_compensator
{
  using namespace apptronik_system;

  // For Nodelets, use onInit() method instead of constructor
  LagAmplificationController::LagAmplificationController() {}


  LagAmplificationController::~LagAmplificationController()
  {
    m_sys->stop();
    m_system_thread->join();
    ros::Duration(0.1).sleep(); //make sure everyone's done using our pointers
  }

  // onInit() function should not block. It should initialize and then return.
  void LagAmplificationController::onInit()
  {
    m_system_thread.reset(new boost::thread(boost::bind(&LagAmplificationController::systemThread, this)));
  }

  //set up ctrl interface here
  void LagAmplificationController::systemThread()
  {
    std::string sn_elbow = std::string("Axon_proto_v1");
    ros::NodeHandle nh = getPrivateNodeHandle();

    // User calls SystemLoop Constructor:
    m_sys.reset(new SystemLoop(boost::bind(&LagAmplificationController::loop, this, _1, _2), nh, {sn_elbow}, true));


    // User sets run mode for each slave:
    // m_sys->setRunMode(JOINT_TORQUE);
/*    m_sys->setRunMode(ACTUATOR_FORCE);
*/  m_sys->setRunMode(static_cast<AxonMode>(6),sn_elbow);

    // User registers a state ptr for each MISO topic with desired state info
    // m_sys->registerStatePtr( &m_fault_status , "diag__faults__x" );
    m_sys->registerStatePtr( &m_fault_status , "diag__faults__x" ,sn_elbow);
    m_sys->registerStatePtr( &m_actuator_force , "actuator__force__N" ,sn_elbow);
    m_sys->registerStatePtr( &m_joint_torque , "js__joint__effort__Nm" ,sn_elbow);
    m_sys->registerStatePtr( &m_cuff_torque , "cuff__ati__torque__Nm" ,sn_elbow);
    m_sys->registerStatePtr( &m_joint__position , "js__joint__position__rad" ,sn_elbow);
    m_sys->registerStatePtr( &m_motor_current , "motor__current__A",sn_elbow);
    m_sys->registerStatePtr( &m_actuator_position , "actuator__position__m",sn_elbow );
    m_sys->registerStatePtr( &m_joint_velocity , "js__joint__velocity__radps" , sn_elbow);

    // User registers a command ptr for each MOSI topic corresponding to the desired mode
    m_sys->registerCommandPtr( &m_cmd_joint_torque, "cmd__joint__effort__Nm" ,sn_elbow);
/*    m_sys->registerCommandPtr( &m_cmd_joint_torque, "cmd__actuator__effort__n" );
*/


    // User should initialize commands to a reasonable value
    m_cmd_joint_torque = 0;
    m_prev_fault_status = 0;


    // User specifies any additional messages to subscribe to (inputs) or
    //   to publish (outputs).  SystemLoop will set up the callbacks / publisher
    //   such that if the message source or destination is within the nodelet
    //   manager, communication will happen via ros' INTRAPROCESS TRANSPORT mechanism
    // The boost pointer provided will be populated with any incoming messages
    //   for inputs and the will be accessed to retrieve outgoing messages

    // m_input_float_msg = boost::shared_ptr<apptronik_msgs::Float32Stamped>(new apptronik_msgs::Float32Stamped());




    // Specify any MISO topics you would like to autolog. (The default behavior
    //    is to autolog all states and commands.  This can be changed by passing
    //    false to the SystemLoop constructor.)

    // LoggingManager::autoLogMISO(slave_name, "diag__faults__x");




    // You can call addServerConnection to instruct ConcurrentServiceClient to
    //   maintain a persistent connection to a service.  This can be helpful
    //   (shorten the delay of future service calls to this service)
    //   you plan to call a service repeatedly.

    // ConcurrentServiceClient::addServerConnection<apptronik_srvs::Float32>("/"+slave_name+"/Control__Cuff__Torque__KP/set");

    nh.param("lag1_zero_rad_s"      , m_lag1_zero_rad_s      ,   10.);
    nh.param("lag1_pole_rad_s"      , m_lag1_pole_rad_s      ,    1.);
    nh.param("lag2_zero_rad_s"      , m_lag2_zero_rad_s      ,   10.);
    nh.param("lag2_pole_rad_s"      , m_lag2_pole_rad_s      ,   10.);
    nh.param("lag3_zero_rad_s"      , m_lag3_zero_rad_s      ,   10.);
    nh.param("lag3_pole_rad_s"      , m_lag3_pole_rad_s      ,   10.);
    nh.param("lag4_zero_rad_s"      , m_lag4_zero_rad_s      ,   10.);
    nh.param("lag4_pole_rad_s"      , m_lag4_pole_rad_s      ,   10.);
    nh.param("lag5_zero_rad_s"      , m_lag5_zero_rad_s      ,   10.);
    nh.param("lag5_pole_rad_s"      , m_lag5_pole_rad_s      ,   10.);
    nh.param("ignore_cuff"          , m_ignore_cuff          , false);
    nh.param("meta_compensator_gain", m_meta_compensator_gain,    .5);
    nh.param("amplification_factor" , m_amplification_factor ,    1.);
    nh.param("amplification_mode"   , m_amplification_mode   ,     0);
    nh.param("cuff_filter_wn_rad_s" , m_cuff_filter_wn_rad_s ,  100.);
    nh.param("cuff_filter_zt_rad_s" , m_cuff_filter_zt_rad_s ,    1.);
    nh.param("ignore_damp"          , m_ignore_damp          , false);
    nh.param("damping"              , m_damping              ,    0.);
    nh.param("damp_filter_wn_rad_s" , m_damp_filter_wn_rad_s ,  100.);
    nh.param("damp_filter_zt_rad_s" , m_damp_filter_zt_rad_s ,    1.);
    nh.param("chirp_low_freq_rps"   , m_chirp_low_freq_rps   ,    1.);
    nh.param("chirp_high_freq_rps"  , m_chirp_high_freq_rps  ,  100.);
    nh.param("chirp_amplitude"      , m_chirp_amplitude      ,  100.);
    nh.param("chirp_offset"         , m_chirp_offset         ,  200.);
    nh.param("chirp_duration"       , m_chirp_duration       ,   60.);
    nh.param("chirp_mode"           , m_chirp_mode           ,     0);

    NODELET_INFO_STREAM("#####################################"            );
    NODELET_INFO_STREAM("###  Welcome to Lag Compensation  ###"            );
    NODELET_INFO_STREAM("#####################################"            );
    NODELET_INFO_STREAM("lag1_zero_rad_s      :" << m_lag1_zero_rad_s      );
    NODELET_INFO_STREAM("lag1_pole_rad_s      :" << m_lag1_pole_rad_s      );
    NODELET_INFO_STREAM("lag2_zero_rad_s      :" << m_lag2_zero_rad_s      );
    NODELET_INFO_STREAM("lag2_pole_rad_s      :" << m_lag2_pole_rad_s      );
    NODELET_INFO_STREAM("lag3_zero_rad_s      :" << m_lag3_zero_rad_s      );
    NODELET_INFO_STREAM("lag3_pole_rad_s      :" << m_lag3_pole_rad_s      );
    NODELET_INFO_STREAM("lag4_zero_rad_s      :" << m_lag4_zero_rad_s      );
    NODELET_INFO_STREAM("lag4_pole_rad_s      :" << m_lag4_pole_rad_s      );
    NODELET_INFO_STREAM("lag5_zero_rad_s      :" << m_lag5_zero_rad_s      );
    NODELET_INFO_STREAM("lag5_pole_rad_s      :" << m_lag5_pole_rad_s      );
    NODELET_INFO_STREAM("ignore_cuff          :" << m_ignore_cuff          );
    NODELET_INFO_STREAM("meta_compensator_gain:" << m_meta_compensator_gain);
    NODELET_INFO_STREAM("amplification_factor :" << m_amplification_factor );
    NODELET_INFO_STREAM("amplification_mode   :" << m_amplification_mode   );
    NODELET_INFO_STREAM("cuff_filter_wn_rad_s :" << m_cuff_filter_wn_rad_s );
    NODELET_INFO_STREAM("cuff_filter_zt_rad_s :" << m_cuff_filter_zt_rad_s );
    NODELET_INFO_STREAM("ignore_damp          :" << m_ignore_damp          );
    NODELET_INFO_STREAM("damping              :" << m_damping              );
    NODELET_INFO_STREAM("damp_filter_wn_rad_s :" << m_damp_filter_wn_rad_s );
    NODELET_INFO_STREAM("damp_filter_zt_rad_s :" << m_damp_filter_zt_rad_s );
    NODELET_INFO_STREAM("chirp_low_freq_rps   :" << m_chirp_low_freq_rps   );
    NODELET_INFO_STREAM("chirp_high_freq_rps  :" << m_chirp_high_freq_rps  );
    NODELET_INFO_STREAM("chirp_amplitude      :" << m_chirp_amplitude      );
    NODELET_INFO_STREAM("chirp_offset         :" << m_chirp_offset         );
    NODELET_INFO_STREAM("chirp_duration       :" << m_chirp_duration       );
    NODELET_INFO_STREAM("chirp_mode           :" << m_chirp_mode           );
    NODELET_INFO_STREAM("#################################"                );

    // Must call start to start loop
    m_sys->start();
  }


  //define control behavior here
  void LagAmplificationController::loop(const double& time, const dBitset& fault_bitmap)
  {
    static double last_print_time = 0.0;
    static double last_run_time   = 0.0;
    static double kp_1_lpf        = 0.0;
    static double kp_2_lpf        = 0.0;

    double dt = time - last_run_time;
    last_run_time = time;

    m_cmd_joint_torque = 0;

    // double cuff_torque = m_cuff_torque;
    m_damp_torque = m_damping * m_joint_velocity;

    m_xdd_c = m_cuff_filter_wn_rad_s * m_cuff_filter_wn_rad_s * (m_cuff_torque - m_x_c - (2. * m_cuff_filter_zt_rad_s / m_cuff_filter_wn_rad_s) * m_xd_c);
    m_xd_c += dt * m_xdd_c;
    m_x_c  += 0.5 * dt * dt * m_xdd_c + dt * m_xd_c;
    double cuff_torque = m_x_c;

    m_xdd_d = m_damp_filter_wn_rad_s * m_damp_filter_wn_rad_s * (m_damp_torque - m_x_d - (2. * m_damp_filter_zt_rad_s / m_damp_filter_wn_rad_s) * m_xd_d);
    m_xd_d += dt * m_xdd_d;
    m_x_d  += 0.5 * dt * dt * m_xdd_d + dt * m_xd_d;
    double damp_torque = m_x_d;

    if (m_ignore_cuff) {cuff_torque = 0.0;}
    // m_compensator_gain = m_meta_compensator_gain / m_amplification_factor;
    if (m_ignore_damp) {damp_torque = 0.0;}

    if (m_amplification_mode == 1)
    {
      m_error =  - (- cuff_torque) * m_amplification_factor + (- cuff_torque);
    }
    else if (m_amplification_mode == 2)
    {
      m_error =  - (- cuff_torque) * m_amplification_factor + (- cuff_torque) + m_joint_torque;
    }
    else
    {
      m_error = cuff_torque;
    }

    double dx1 = m_lag1_pole_rad_s * (    m_error - m_x_1);
    double lag1_result = dx1 / m_lag1_zero_rad_s + m_x_1;
    m_x_1 += dt * dx1;

    double dx2 = m_lag2_pole_rad_s * (lag1_result - m_x_2);
    double lag2_result = dx2 / m_lag2_zero_rad_s + m_x_2;
    m_x_2 += dt * dx2;

    double dx3 = m_lag3_pole_rad_s * (lag2_result - m_x_3);
    double lag3_result = dx3 / m_lag3_zero_rad_s + m_x_3;
    m_x_3 += dt * dx3;

    double dx4 = m_lag4_pole_rad_s * (lag3_result - m_x_4);
    double lag4_result = dx4 / m_lag4_zero_rad_s + m_x_4;
    m_x_4 += dt * dx4;

    double dx5 = m_lag5_pole_rad_s * (lag3_result - m_x_5);
    double lag5_result = dx5 / m_lag5_zero_rad_s + m_x_5;
    m_x_5 += dt * dx5;

    m_cmd = m_meta_compensator_gain * (0. - (lag5_result));
    
    double effective_kp_1 = m_joint_torque / cuff_torque;
    double effective_kp_2 = m_cmd          / cuff_torque;

    kp_1_lpf += dt * 100.0 * (effective_kp_1 - kp_1_lpf);
    kp_2_lpf += dt * 100.0 * (effective_kp_2 - kp_2_lpf);


    if(fault_bitmap.any())
    {
      if (time - last_print_time > 1.0 || (m_prev_fault_status == 0))
      {
        NODELET_INFO_STREAM("System Faulted... (Fault code: " << m_fault_status << ")" );
        NODELET_INFO_STREAM("fault: " << m_fault_status << ", " << "k1: " << kp_1_lpf << ", " << "k2: " << kp_2_lpf << ", " << "u: " << m_motor_current << ", " << "td: " << m_cmd << ", " << "ts: " << m_joint_torque << ", " << "tc: " << m_cuff_torque<<  ")" );
        last_print_time = time;
      }
    }
    else
    {
      // chirp signal
      static double waveform_start_time = 0.0;
      static bool test_done = false;

      if(m_prev_fault_status)
      {
        NODELET_INFO_STREAM("Starting SysID Experiment...");
        waveform_start_time = time;
      }

      if(m_chirp_mode && !test_done && time-waveform_start_time < m_chirp_duration) {
        exponentialChirpSignal( time-waveform_start_time, m_chirp);
      }
      else{
        NODELET_INFO_STREAM("Experiment Finished!!!");
        test_done = true;
        // ros::shutdown();
      }


      // controller
      m_cmd_joint_torque = m_cmd + m_chirp + damp_torque;
      LoggingManager::logData("js__joint__effort__Nm", m_joint_torque);
      LoggingManager::logData("cuff_torque", cuff_torque);

      if (time - last_print_time > 1.0 || (m_prev_fault_status == 0))
      {
        // NODELET_INFO_STREAM("Torque, force, ratio is... "  << cuff_torque << ", " << m_actuator_force<< ", " << m_cuff_torque/m_actuator_force << ")" );
        NODELET_INFO_STREAM("fault: " << m_fault_status << ", " << "k1: " << kp_1_lpf << ", " << "k2: " << kp_2_lpf << ", " << "u: " << m_motor_current << ", " << "td: " << m_cmd << ", " << "ts: " << m_joint_torque << ", " << "tc: " << m_cuff_torque<<  ")" );
        last_print_time = time;
      }
    }
    m_prev_fault_status = m_fault_status;
  }

  bool LagAmplificationController::exponentialChirpSignal(const double& elapsed_time, double& signal)
  {
    static double prev_sample_time = 0;
    static double prev_effective_angle = 0;

    double effective_angle, effective_switching_freq_rps;
    double range = m_chirp_high_freq_rps - m_chirp_low_freq_rps;
    double period = elapsed_time - prev_sample_time;

    double k = pow(m_chirp_high_freq_rps / m_chirp_low_freq_rps, 1 / m_chirp_duration);

    prev_sample_time = elapsed_time;

    if(elapsed_time > 0.0)
    {
      //exponential chirp
      effective_angle = m_chirp_low_freq_rps * (pow(k, elapsed_time) - 1) / log(k);

      signal = m_chirp_amplitude * sin(effective_angle) + m_chirp_offset;

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
    }

    return false;
  }

}



#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(p0_lag_compensator::LagAmplificationController, nodelet::Nodelet)
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
    m_sys->registerStatePtr( &m_motor_pos , "motor__position__Rad" ,sn_elbow);
    m_sys->registerStatePtr( &m_motor_vel , "motor__velocity__Radps" ,sn_elbow);
    


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

    nh.param("lag_zero_rad_s", m_lag_zero_rad_s,   10.);
    nh.param("lag_pole_rad_s", m_lag_pole_rad_s, 1.);
    nh.param("lead_zero_rad_s", m_lead_zero_rad_s,   10.);
    nh.param("lead_pole_rad_s", m_lead_pole_rad_s, 10.);
    nh.param("meta_compensator_gain", m_meta_compensator_gain, .5);
    nh.param("amplification_factor", m_amplification_factor, 1.);
    nh.param("amplification_mode", m_amplification_mode, 0);
    nh.param("ignore_cuff", m_ignore_cuff, false);
    nh.param("added_mass", m_added_mass, 0.0);
    nh.param("arm_length", m_arm_length, 0.5334);
    nh.param("enable_grav_comp", m_en_grav_comp, true);
    nh.param("enable_load_attn", m_en_load_attn, true);



    NODELET_INFO_STREAM("#####################################");
    NODELET_INFO_STREAM("###  Welcome to Lag Compensation  ###");
    NODELET_INFO_STREAM("#####################################");
    NODELET_INFO_STREAM("lag_zero_rad_s: " << m_lag_zero_rad_s);
    NODELET_INFO_STREAM("lag_pole_rad_s: " << m_lag_pole_rad_s);
    NODELET_INFO_STREAM("lead_zero_rad_s: " << m_lead_zero_rad_s);
    NODELET_INFO_STREAM("lead_pole_rad_s: " << m_lead_pole_rad_s);
    NODELET_INFO_STREAM("meta_compensator_gain: " << m_meta_compensator_gain);
    NODELET_INFO_STREAM("amplification_factor: " << m_amplification_factor);
    NODELET_INFO_STREAM("amplification_mode: " << m_amplification_mode);
    NODELET_INFO_STREAM("added_mass: " << m_added_mass);
    NODELET_INFO_STREAM("arm_length: " << m_arm_length);
    NODELET_INFO_STREAM("enable_grav_comp: " << m_en_grav_comp);
    NODELET_INFO_STREAM("enable_load_attn: " << m_en_load_attn);
    NODELET_INFO_STREAM("#################################");

    // Must call start to start loop
    m_sys->start();
  }


  //define control behavior here
  void LagAmplificationController::loop(const double& time, const dBitset& fault_bitmap)
  {
    m_cmd_joint_torque = 0;

    static double grav_comp = 0;
    static double cuff_torque = 0;
    static double joint_torque = 0;
    joint_torque = (!m_en_grav_comp) ? m_joint_torque : (m_joint_torque - grav_comp);
    cuff_torque = m_cuff_torque;
    if (m_ignore_cuff) {cuff_torque = 0.0;}

    static double last_print_time = 0.0;
    static double last_run_time   = 0.0;
    static double kp_1_lpf        = 0.0;
    static double kp_2_lpf        = 0.0;

    double dt = time - last_run_time;
    last_run_time = time;

    m_compensator_gain = m_meta_compensator_gain / m_amplification_factor;

    // double error = .03 * m_actuator_force - cuff_torque * m_amplification_factor;
    // double error =  ( m_joint_torque - cuff_torque * m_amplification_factor );

    constexpr double grav_const = 9.81; // m/s^2
    constexpr double arm_mass = 0.6; // kg
    
    grav_comp = grav_const*(arm_mass+m_added_mass)*m_arm_length*sin(m_joint__position);
    


    double error =  - (- cuff_torque) * m_amplification_factor + (- cuff_torque) + joint_torque;
    double dx1 = m_lag_pole_rad_s * (error - m_x_1);
    double lag_result = dx1 / m_lag_zero_rad_s + m_x_1;
    m_x_1 += dt * dx1;

    double dx2 = m_lead_pole_rad_s * (lag_result - m_x_2);
    double lead_lag_result = dx2 / m_lead_zero_rad_s + m_x_2;
    m_x_2 += dt * dx2;



    m_cmd = 0;
    if(m_en_load_attn)
    {
      if (m_amplification_mode == 1)
      {
        m_cmd = m_compensator_gain * (0. - (lead_lag_result + (- cuff_torque) + joint_torque)) + (m_amplification_factor - 1.) * (- m_cuff_torque);
      }
      else if (m_amplification_mode == 2)
      {
        m_cmd = m_compensator_gain * (0. - (lead_lag_result + (- cuff_torque) + joint_torque)) +   joint_torque;
      }
      else
      {
        m_cmd = m_compensator_gain * (0. - (lead_lag_result));
      }
    }


    if(m_en_grav_comp)
    {
      m_cmd += grav_comp;
    }
    
    double effective_kp_1 = joint_torque / cuff_torque;
    double effective_kp_2 = m_cmd          / cuff_torque;

    kp_1_lpf += dt * 100.0 * (effective_kp_1 - kp_1_lpf);
    kp_2_lpf += dt * 100.0 * (effective_kp_2 - kp_2_lpf);


    if(fault_bitmap.any())
    {
      if (time - last_print_time > 1.0 || (m_prev_fault_status == 0))
      {
        NODELET_INFO_STREAM("System Faulted... (Fault code: " << m_fault_status << ")" );
        // NODELET_INFO_STREAM("fault: " << m_fault_status << ", " << "k1: " << kp_1_lpf << ", " << "k2: " << kp_2_lpf << ", " << "u: " << m_motor_current << ", " << "td: " << m_cmd << ", " << "ts: " << m_joint_torque << ", " << "tc: " << m_cuff_torque<<  ")" );
        last_print_time = time;
      }
    }
    else
    {
      // Do control:
      // m_actuator_force *.03 ~= cuff_torque
          // m_actuator_force - cuff_torque
      


/*      m_cmd_joint_torque = - compensator_gain * lag_result / .03; */
      // m_cmd_joint_torque = compensator_gain * lead_lag_result;
      m_cmd_joint_torque = m_cmd;
      LoggingManager::logData("js__joint__effort__Nm", m_joint_torque);
      LoggingManager::logData("cuff_torque", cuff_torque);
/*
      if (time - last_print_time > 1.0 || (m_prev_fault_status == 0))
      {
        // NODELET_INFO_STREAM("Torque, force, ratio is... "  << cuff_torque << ", " << m_actuator_force<< ", " << m_cuff_torque/m_actuator_force << ")" );
        NODELET_INFO_STREAM("fault: " << m_fault_status << ", " << "k1: " << kp_1_lpf << ", " << "k2: " << kp_2_lpf << ", " << "u: " << m_motor_current << ", " << "td: " << m_cmd << ", " << "ts: " << m_joint_torque << ", " << "tc: " << m_cuff_torque<<  ")" );
        last_print_time = time;
      }*/
    }
    m_prev_fault_status = m_fault_status;
  }
}



#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(p0_lag_compensator::LagAmplificationController, nodelet::Nodelet)
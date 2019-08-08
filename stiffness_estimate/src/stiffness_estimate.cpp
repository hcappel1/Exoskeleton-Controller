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
    NODELET_INFO_STREAM("#################################");

    // User sets run mode for each slave:
    // m_sys->setRunMode(ACTUATOR_FORCE);

    // User registers a state ptr for each MISO topic with desired state info
    m_sys->registerStatePtr( &m_fault_status , "diag__faults__x" , slave_name);
    m_sys->registerStatePtr( &m_actuator_force , "actuator__force__N" , slave_name);
    m_sys->registerStatePtr( &m_joint_torque , "js__joint__effort__Nm" , slave_name);
    m_sys->registerStatePtr( &m_cuff_torque , "cuff__ati__torque__Nm" , slave_name);
    m_sys->registerStatePtr( &m_joint__position , "js__joint__position__rad" , slave_name);
    m_sys->registerStatePtr( &m_motor_current , "motor__current__A" , slave_name);
    m_sys->registerStatePtr( &m_actuator_position , "actuator__position__m" , slave_name);
    
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



    power_list = {0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7};
    // power_list = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0};
    index      = 0;
    last_time  = 0;
    rest_start_time = 0;
    // Must call start to start loop
    m_sys->start();
  }


  //define control behavior here
  void StiffnessEstimate::loop(const double& time, const dBitset& fault_bitmap)
  {
    m_cmd = 0.;
    m_gravity_cmd = (m_load_mass * m_distance + m_exo_mass * m_distance / 2.) * 9.81 * sin(m_joint__position);
    m_chirp_cmd   = 0.;
    m_chirp_power = 0.;

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

      if(!test_done && time-waveform_start_time < m_chirp_duration) {
        exponentialChirpSignal(time-waveform_start_time, m_chirp_cmd, m_chirp_power);
        LoggingManager::logData("gravity_cmd", m_gravity_cmd);
        LoggingManager::logData("chirp_power", m_chirp_power);
      }
      else{
        NODELET_INFO_STREAM("Experiment Finished!!!");
        test_done = true;
        ros::shutdown();
      }

    }

    m_cmd = m_gravity_cmd + m_chirp_cmd;
    m_prev_fault_status = m_fault_status;
  }

  bool StiffnessEstimate::exponentialChirpSignal(const double& elapsed_time, double& signal, double& power)
  {
    static double prev_sample_time = 0;
    static double prev_effective_angle = 0;

    double effective_angle, effective_switching_freq_rps, effective_freq_rps;
    double range = m_chirp_high_freq_rps - m_chirp_low_freq_rps;
    double period = elapsed_time - prev_sample_time;

/*    double k = pow(m_chirp_high_freq_rps / m_chirp_low_freq_rps, 1 / m_chirp_duration);
*/
    prev_sample_time = elapsed_time;

    if(elapsed_time >= 0.0)
    {
      power = power_list[index];
      effective_freq_rps = pow(10.0, power);
      effective_angle = effective_freq_rps * (elapsed_time - last_time);

      if (index>=0 && index<=4)
      {signal = 0.16*m_chirp_amplitude * sin(effective_angle) + m_chirp_offset;} 

      else if (index>=5 && index <=10)
      {signal = 0.16*m_chirp_amplitude * sin(effective_angle) + m_chirp_offset;} 

      else if (index==11)
      {signal = 0.08*sqrt(10.0)*m_chirp_amplitude * sin(effective_angle) + m_chirp_offset;} 

      else if (index==12)
      {signal = 0.4*m_chirp_amplitude * sin(effective_angle) + m_chirp_offset;} 

      else if (index==13)
      {signal = 0.2*sqrt(10.0)*m_chirp_amplitude * sin(effective_angle) + m_chirp_offset;} 

      else if (index==14)
      {signal = m_chirp_amplitude * sin(effective_angle) + m_chirp_offset;} 
      

      if (index<=14 && elapsed_time-last_time > (2.0 * 3.14 / effective_freq_rps) * ceil(10.0 / (2.0 * 3.14 / effective_freq_rps)))
      {
        last_time = elapsed_time;
        rest_start_time = elapsed_time;
        index+=1;
        // NODELET_INFO_STREAM("chirp freq power : " << power);
        NODELET_INFO_STREAM("subtracting bias");
      }

      if (elapsed_time-rest_start_time <= 20)
      {
        last_time = elapsed_time;
        // if (elapsed_time - rest_start_time == 0  && index != 0)
        // {
        //   NODELET_INFO_STREAM("subtracting bias");
        // }
        if (int((elapsed_time - rest_start_time) * 1000.0) == 5000)
        {
          NODELET_INFO_STREAM("resting");
        }
        if (int((elapsed_time - rest_start_time) * 1000.0) == 15000)
        {
          NODELET_INFO_STREAM("adding bias");
        }
        if (int((elapsed_time - rest_start_time) * 1000.0) == 19998)
        {
          NODELET_INFO_STREAM("chirp freq power : " << power);
        }
        if (elapsed_time - rest_start_time <= 5 && index != 0)
        { 
          signal = (m_chirp_offset / 2.0) * cos((1.0 / 10.0) * 2.0 * 3.1415925 * (elapsed_time - rest_start_time)) + (m_chirp_offset / 2.0);
        }
        else if (elapsed_time - rest_start_time <= 15)
        {
          signal = 0.0;
        } 
        else
        {
          signal = (m_chirp_offset / 2.0) * cos((1.0 / 10.0) * 2.0 * 3.1415925 * (elapsed_time - rest_start_time - 10.0)) + (m_chirp_offset / 2.0);
        }
        // signal    = (m_chirp_offset / 2.0) * cos((1.0 / 30.0) * 2.0 * 3.1415925 * (elapsed_time - rest_start_time)) + (m_chirp_offset / 2.0);
        effective_switching_freq_rps = 0.0;  
        power = 10.0;
      }

      if (index >= 15)
      {
        signal = m_chirp_offset;
        effective_switching_freq_rps = 0.0;
        if (elapsed_time - rest_start_time <= 5)
          { 
            signal = (m_chirp_offset / 2.0) * cos((1.0 / 10.0) * 2.0 * 3.1415925 * (elapsed_time - rest_start_time)) + (m_chirp_offset / 2.0);
          }
          else
          {
            signal = 0.0;
          }         
      }

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
PLUGINLIB_EXPORT_CLASS(stiffness_estimate::StiffnessEstimate, nodelet::Nodelet)

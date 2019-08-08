#include <stiffness_estimate/stiffness_estimate.hpp>
using namespace LibSerial ;
using namespace std;
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

    nh.param("sin_freq_rps"     , m_sin_freq_rps      ,   1.);
    nh.param("sin_amplitude"    , m_sin_amplitude     , 100.);
    nh.param("sin_offset"       , m_sin_offset        , 200.);
    nh.param("sin_duration"     , m_sin_duration      ,  60.);
    nh.param("sin_mode"         , m_sin_mode          ,    4);
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
    nh.param("serial_port"        , port_number           ,    0);
   
    NODELET_INFO_STREAM("#################################");
    NODELET_INFO_STREAM("###  SysID Experiment Params  ###");
    NODELET_INFO_STREAM("#################################");
    NODELET_INFO_STREAM("sin_freq_rps:       " << m_sin_freq_rps      );
    NODELET_INFO_STREAM("sin_amplitude:      " << m_sin_amplitude     );
    NODELET_INFO_STREAM("sin_offset:         " << m_sin_offset        );
    NODELET_INFO_STREAM("sin_duration:       " << m_sin_duration      );
    NODELET_INFO_STREAM("sin_mode:           " << m_sin_mode          );
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
    NODELET_INFO_STREAM("serial_port:          " << port_number           );
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
    if (m_sin_mode == 4)
      {      
        m_sys->setRunMode(static_cast<AxonMode>(4));
        m_sys->registerCommandPtr( &m_cmd, "cmd__actuator__effort__n");
    }
    else if (m_sin_mode == 6)
      {      
        m_sys->setRunMode(static_cast<AxonMode>(6), slave_name);
        m_sys->registerCommandPtr( &m_cmd, "cmd__joint__effort__Nm" , slave_name);
    }

    // User should initialize commands to a reasonable value
    m_cmd = 0;
    m_prev_fault_status = 0;

    // offset_list = {0.0, -0.5, -1.0, -1.5,-2.0, 0.0, -0.5, -1.0, -1.5,-2.0,-2.5,-3.0,-3.5,-4.0,-4.5,-5.0,-5.5,-6.0,-6.5,-7.0,-7.5,-8.0,-8.5,-9.0,-9.5,0};
    // offset_list = {0.0, -0.5, -1.0, -1.5,-2.0,-2.5,-3.0,-3.5,-4.0,-4.5,-5.0,-5.5,-6.0,-6.5,-7.0,-7.5,-8.0,-8.5,-9.0,-9.5,-10.0,-10.5,-11.0,-11.5,-12.0,0};
    offset_list = {0.0, -0.0, -0.0, -0.0,-0.0,-2.,-2.0,-2.,-2.0,-2.,-4.0,-4.,-4.0,-4.,-4.0,-6.,-6.0,-6.,-6.0,-6.,-8.0,-8.,-8.0,-8.,-8.0,0};
    index = 0;
    m_sin_power = 0.;
    step = 1;
    Offset = 0.;

    StiffnessEstimate::portinit();

    // Must call start to start loop
    m_sys->start();
  }


  //define control behavior here
  void StiffnessEstimate::loop(const double& time, const dBitset& fault_bitmap)
  {
    m_seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine m_generator (m_seed);
    std::normal_distribution<double> m_distribution (m_sin_offset, m_sin_amplitude);
    m_cmd = 0.;
    m_gravity_cmd = (m_load_mass * m_distance + m_exo_mass * m_distance / 2.) * 9.81 * sin(m_joint_position);
    m_sin_cmd   = 0.;
    m_sin_power = 0.;

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
      m_sin_cmd = lag_result_rand;
      if (m_sin_cmd < m_sin_offset - 3. * m_sin_amplitude)
      {
        m_sin_cmd = m_sin_offset - 3. * m_sin_amplitude;
      }
      else if (m_sin_cmd > m_sin_offset + 3. * m_sin_amplitude)
      {
        m_sin_cmd = m_sin_offset + 3. * m_sin_amplitude;
      }
    }
    else
    {
      m_sin_cmd = 0.;
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

      if(!test_done && time-waveform_start_time < m_sin_duration+5.) {
        // NODELET_INFO_STREAM("m_cmd: " << m_sin_cmd);
        if (m_ignore_rand == 1)
        {
          exponentialsinSignal(time-waveform_start_time, m_sin_cmd, m_sin_power, m_gravity_cmd, index, Offset);
          dataread();
        }        
        LoggingManager::logData("gravity_cmd", m_gravity_cmd);
        LoggingManager::logData("amplification_cmd", m_amplification_cmd);
        LoggingManager::logData("damping_cmd", m_damping_cmd);
        LoggingManager::logData("sin_cmd", m_sin_cmd);
        LoggingManager::logData("sin_power", m_sin_power);
        LoggingManager::logData("EMGdata", input_buffer);
        LoggingManager::logData("Offset", Offset);
      }
      else{
        NODELET_INFO_STREAM("Experiment Finished!!!");
        test_done = true;
        ros::shutdown();
      }

    }

    // m_cmd = m_gravity_cmd + m_amplification_cmd + m_sin_cmd + m_damping_cmd;
    m_cmd = m_gravity_cmd + m_sin_cmd;
    // m_cmd = m_gravity_cmd;
    m_prev_fault_status = m_fault_status;
  }

  bool StiffnessEstimate::exponentialsinSignal(const double& elapsed_time, double& signal, double& power, double& gravity_signal, int &index, double &Offset)
  {
    double effective_angle,effective_time;
    // NODELET_INFO_STREAM(elapsed_time);
    if (elapsed_time<10)
    {
      power = 0;
      if (int(elapsed_time*100)==500)
      {
      NODELET_INFO_STREAM("Reference record starts");
      power = 0;
      return true;
      }
      return true;
    }

    if (elapsed_time>=10 && elapsed_time <20)
    {
      power = 2;
      return true;
    }

    if (int(elapsed_time*100)==2000)
    {
      NODELET_INFO_STREAM("Reference record ends");
      power = 0;
      return true;
    }

    if (elapsed_time>=20 && elapsed_time <22)
    {
      power = 0;
      return true;
    }
    
  
    if (index<=25)
    {
      
    // NODELET_INFO_STREAM(ref);
      if (elapsed_time-last_time > 45. && step ==6)
      {
        // if (elapsed_time-last_time <33)
        // {
        //   // NODELET_INFO_STREAM("Stop");
        //   signal = offset_list[index];
        //   power = 0.0;
        //   return true;
        // }
        if (int((elapsed_time-last_time)*100) == 4500)
        {
          NODELET_INFO_STREAM("Stop");
          NODELET_INFO_STREAM(offset_list[index-1]);
          power = 0.0;
        }
        last_time = elapsed_time;
        signal = offset_list[index-1]*cos(0.628*(elapsed_time-last_time))/2+offset_list[index-1]/2;
        power = 0;
        step = 1;
      }

      if (int((elapsed_time-last_time)*100) == 2500)
      {
        NODELET_INFO_STREAM("Start");
        power = 0;
      }

      if (int((elapsed_time-last_time)*100) == 3000)
      {
        NODELET_INFO_STREAM("StartReal");
        power = 0;
      }

      if (elapsed_time-last_time <= 5)
      {
        // NODELET_INFO_STREAM("Resting");
        // gravity_signal = 0;
        signal = offset_list[index-1]*cos(0.628*(elapsed_time-last_time))/2+offset_list[index-1]/2;
        power = 0;
      }
      else if (elapsed_time-last_time <= 25)
      {
        // NODELET_INFO_STREAM("Resting");
        gravity_signal = 0;
        signal = 0;
        power = 0;
      }
      else if (elapsed_time-last_time <=30)
      {
        signal = offset_list[index]*cos(0.628*(elapsed_time-last_time-20))/2+offset_list[index]/2;
        // signal = 0;
        power = 0;
      }
      else if (elapsed_time-last_time <= 45.1)
      {
        Offset = offset_list[index];
        effective_time = elapsed_time-last_time-30;
        NODELET_INFO_STREAM(effective_time);
        effective_angle = m_sin_freq_rps * effective_time;
        signal = m_sin_amplitude * sin(effective_angle) + offset_list[index]; 
        power  = 1;
        if ( floor(effective_time) == 3*step)
        {
            index++;
            step++;
            NODELET_INFO_STREAM("Offset:"<< offset_list[index]);
            NODELET_INFO_STREAM("Step:"<< step);
        }
      }

    }
    else
    {
      gravity_signal = 0;
      signal = 0;
      power = 0;
    }

    return true;
  }


   bool StiffnessEstimate::portinit(){
      if(port_number == 0)
     {
      serial_port.Open( "/dev/ttyACM0" ) ;
     }
     else if (port_number == 1)
     {
      serial_port.Open( "/dev/ttyACM1" ) ;
     }
     else
     {
      NODELET_INFO_STREAM("wrong port number");
      exit(1) ;
     }

     if ( ! serial_port.good() )
     {
         std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                   << "Error: Could not open serial port."
                   << std::endl ;
         exit(1) ;
         
     }
     //
     // Set the baud rate of the serial port.
     //
     serial_port.SetBaudRate( SerialStreamBuf::BAUD_57600 ) ;
     if ( ! serial_port.good() )
     {
         std::cerr << "Error: Could not set the baud rate." <<  std::endl ;
         exit(1) ;
     }
     //
     // Set the number of data bits.
     //
     serial_port.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
     if ( ! serial_port.good() )
     {
         std::cerr << "Error: Could not set the character size." <<  std::endl ;
         exit(1) ;
     }
     //
     // Disable parity.
     //
     serial_port.SetParity( SerialStreamBuf::PARITY_NONE ) ;
     if ( ! serial_port.good() )
     {
         std::cerr << "Error: Could not disable the parity." <<  std::endl ;
         exit(1) ;
     }
     //
     // Set the number of stop bits.
     //
     serial_port.SetNumOfStopBits( 1 ) ;
     if ( ! serial_port.good() )
     {
         std::cerr << "Error: Could not set the number of stop bits." << std::endl ;
         exit(1) ;
     }
     //
     // Turn off hardware flow control.
     //
     serial_port.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE ) ;
     if ( ! serial_port.good() )
     {
         std::cerr << "Error: Could not use hardware flow control." << std::endl ;
         exit(1) ;
     }
     while( serial_port.rdbuf()->in_avail() == 0 )
     {
         usleep(100) ;
     }
     return true;
   }


    void StiffnessEstimate::dataread(){

     // int k=0;
     // char next_byte = '0';
     // char temp_buffer[256] ;
     // serial_port.get(next_byte);
     // while (next_byte != '\n'){      
     //    temp_buffer[k]=next_byte;
     //    k++;
     //    serial_port.get(next_byte); // HERE I RECEIVE THE FIRST ANSWER
     // }
     // strcpy(input_buffer,temp_buffer);
     int k=0;
     char next_byte = '0';
     serial_port.get(next_byte);
     while (next_byte != '\n'){      
        input_buffer[k]=next_byte;
        k++;
        serial_port.get(next_byte); // HERE I RECEIVE THE FIRST ANSWER
     }
     input_buffer[k-1]='\0';
     // std::cerr << input_buffe
   }

}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(stiffness_estimate::StiffnessEstimate, nodelet::Nodelet)

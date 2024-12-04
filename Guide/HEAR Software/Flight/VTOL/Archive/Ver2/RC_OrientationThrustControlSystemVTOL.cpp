
#include "HEAR_systems/Flight_systems/RC_OrientationThrustControlSystemVTOL.hpp"

#undef NO_STATE_EST

namespace HEAR{

RC_OrientationThrustControlSystemVTOL::RC_OrientationThrustControlSystemVTOL(const std::string& sys_name) : System(sys_name){
}

RC_OrientationThrustControlSystemVTOL::~RC_OrientationThrustControlSystemVTOL(){

}

void RC_OrientationThrustControlSystemVTOL::reset(){};

std::string RC_OrientationThrustControlSystemVTOL::getTypeDescription(){
    return "RC_OrientationThrustControlSystemVTOL";
}

void RC_OrientationThrustControlSystemVTOL::initBlocksLayout(){
    std::string abs_url= this->getAbsoluteURL();


    auto itfc_fact_ros= (InterfaceFactory<ROS>*) this->getInterfaceFactory("ros_interface");
    auto itfc_fact_config= dynamic_cast<InterfaceFactory<UAVConfigurations>*>(this->getInterfaceFactory("config_interface"));
    auto itfc_fact_mavlink= (InterfaceFactory<MAVLink>*) this->getInterfaceFactory("mavlink_interface");
    UAVConfigController* config_ctrl=dynamic_cast<UAVConfigController*>(itfc_fact_config->getController());

    PX4_rc_to_orientation_thrust=new PX4RCtoOrientationThrust("PX4RCtoOrientationThrust");
    PX4_sim_state_est_sys = new SimulatedROSStateEstimator("PX4SimStateEst_sys");
    PX4_state_est_sys = new PX4StateEstimatorRefactor("PX4StateEst_sys");
    orientation_controller= new OrientationController("ctrl_sys/ori");
    orientation_error_calculator= new OrientationErrorCalculator("OrientationErrorCalculator_sys"); //TODO: remove if it only contains Sum3 block 
    orientation_rate_error_calculator= new OrientationErrorCalculator("OrientationRateErrorCalculator_sys"); //TODO: remove if it only contains Sum3 block 
    actu_sys = new ActuatVTOLPX4MAVLinkSystem("Actu_sys");
    vtol_ff = new RCtoOriFeedForward("vtol_ff");
    plane_ff = new RCtoOriFeedForward("plane_ff");

    PX4_rc_to_orientation_thrust->addInterfaceFactory(itfc_fact_ros,"ros_interface");
    PX4_rc_to_orientation_thrust->addInterfaceFactory(itfc_fact_config,"config_interface");
    PX4_rc_to_orientation_thrust->addInterfaceFactory(itfc_fact_mavlink,"mavlink_interface");

    PX4_state_est_sys->addInterfaceFactory(itfc_fact_ros,"ros_interface");
    PX4_state_est_sys->addInterfaceFactory(itfc_fact_config,"config_interface");
    PX4_state_est_sys->addInterfaceFactory(itfc_fact_mavlink,"mavlink_interface");

    PX4_sim_state_est_sys->addInterfaceFactory(itfc_fact_ros,"ros_interface");
    PX4_sim_state_est_sys->addInterfaceFactory(itfc_fact_config,"config_interface");
    PX4_sim_state_est_sys->addInterfaceFactory(itfc_fact_mavlink,"mavlink_interface");
    
    orientation_controller->addInterfaceFactory(itfc_fact_ros,"ros_interface");
    orientation_controller->addInterfaceFactory(itfc_fact_config,"config_interface");
    orientation_controller->addInterfaceFactory(itfc_fact_mavlink,"mavlink_interface");

    orientation_error_calculator->addInterfaceFactory(itfc_fact_ros,"ros_interface");
    orientation_error_calculator->addInterfaceFactory(itfc_fact_config,"config_interface");
    orientation_error_calculator->addInterfaceFactory(itfc_fact_mavlink,"mavlink_interface");

    orientation_rate_error_calculator->addInterfaceFactory(itfc_fact_ros,"ros_interface");
    orientation_rate_error_calculator->addInterfaceFactory(itfc_fact_config,"config_interface");
    orientation_rate_error_calculator->addInterfaceFactory(itfc_fact_mavlink,"mavlink_interface");


    actu_sys->addInterfaceFactory(itfc_fact_ros,"ros_interface");
    actu_sys->addInterfaceFactory(itfc_fact_config,"config_interface");
    actu_sys->addInterfaceFactory(itfc_fact_mavlink,"mavlink_interface");

    vtol_ff->addInterfaceFactory(itfc_fact_ros,"ros_interface");
    vtol_ff->addInterfaceFactory(itfc_fact_config,"config_interface");

    plane_ff->addInterfaceFactory(itfc_fact_ros,"ros_interface");
    plane_ff->addInterfaceFactory(itfc_fact_config,"config_interface");

    auto dt=this->getPeriod_s();
    PX4_rc_to_orientation_thrust->setPeriodForCalculations(dt);
    PX4_state_est_sys->setPeriodForCalculations(dt);
    PX4_sim_state_est_sys->setPeriodForCalculations(dt);
    orientation_controller->setPeriodForCalculations(dt);
    orientation_error_calculator->setPeriodForCalculations(dt);
    orientation_rate_error_calculator->setPeriodForCalculations(dt);
    actu_sys->setPeriodForCalculations(dt);
    vtol_ff->setPeriodForCalculations(dt);
    plane_ff->setPeriodForCalculations(dt);

    // PX4_rc_to_orientation_thrust->printSyncSystemGraph();

    this->addBlock(PX4_rc_to_orientation_thrust,"PX4RCtoOrientationThrust");
    this->addBlock(PX4_state_est_sys, "PX4_state_est_sys");
    this->addBlock(PX4_sim_state_est_sys, "PX4_sim_state_est_sys");
    this->addBlock(orientation_controller, "ctrl_sys/ori");
    this->addBlock(orientation_error_calculator, "orientation_error_calculator");
    this->addBlock(orientation_rate_error_calculator, "orientation_rate_error_calculator");
    this->addBlock(actu_sys,"actu_sys");
    this->addBlock(vtol_ff,"vtol_ff");
    this->addBlock(plane_ff,"plane_ff");

    // ******* Primitive Blocks ******* 
    auto add_ff_with_fb= new Sum3();
    add_ff_with_fb->setOperation(Sum3::ADD);
    this->addBlock(add_ff_with_fb,"add_ff_with_fb");

    auto check_vtol_mode= new CompareGreaterThan(0.0);
    this->addBlock(check_vtol_mode,"check_vtol_mode");
    
    //FB saturation
    auto fb_output_limit=new Saturation3();
    auto fb_output_limit_upper_val=config_ctrl->getValueFromFile<Vector3D<float>>(config_ctrl->getSystemSettingsFilePath("VTOL"),"FB_OUTPUT_LIMIT_UPPER");
    auto fb_output_limit_lower_val=config_ctrl->getValueFromFile<Vector3D<float>>(config_ctrl->getSystemSettingsFilePath("VTOL"),"FB_OUTPUT_LIMIT_LOWER");

    fb_output_limit->setClipValueMaxFirst(fb_output_limit_upper_val.x);
    fb_output_limit->setClipValueMinFirst(fb_output_limit_lower_val.x);
    fb_output_limit->setClipValueMaxSecond(fb_output_limit_upper_val.y);
    fb_output_limit->setClipValueMinSecond(fb_output_limit_lower_val.y);
    fb_output_limit->setClipValueMaxThird(fb_output_limit_upper_val.z);
    fb_output_limit->setClipValueMinThird(fb_output_limit_lower_val.z);

    this->addBlock(fb_output_limit,"fb_output_limit");

    //FF VTOL saturation
    auto ff_vtol_output_limit=new Saturation3();
    auto ff_vtol_output_limit_upper_val=config_ctrl->getValueFromFile<Vector3D<float>>(config_ctrl->getSystemSettingsFilePath("VTOL"),"FF_VTOL_OUTPUT_LIMIT_UPPER");
    auto ff_vtol_output_limit_lower_val=config_ctrl->getValueFromFile<Vector3D<float>>(config_ctrl->getSystemSettingsFilePath("VTOL"),"FF_VTOL_OUTPUT_LIMIT_LOWER");

    ff_vtol_output_limit->setClipValueMaxFirst(ff_vtol_output_limit_upper_val.x);
    ff_vtol_output_limit->setClipValueMinFirst(ff_vtol_output_limit_lower_val.x);
    ff_vtol_output_limit->setClipValueMaxSecond(ff_vtol_output_limit_upper_val.y);
    ff_vtol_output_limit->setClipValueMinSecond(ff_vtol_output_limit_lower_val.y);
    ff_vtol_output_limit->setClipValueMaxThird(ff_vtol_output_limit_upper_val.z);
    ff_vtol_output_limit->setClipValueMinThird(ff_vtol_output_limit_lower_val.z);
    
    this->addBlock(ff_vtol_output_limit,"ff_vtol_output_limit");

    //FF PLANE saturation
    auto ff_plane_output_limit=new Saturation3();
    auto ff_plane_output_limit_upper_val=config_ctrl->getValueFromFile<Vector3D<float>>(config_ctrl->getSystemSettingsFilePath("VTOL"),"FF_PLANE_OUTPUT_LIMIT_UPPER");
    auto ff_plane_output_limit_lower_val=config_ctrl->getValueFromFile<Vector3D<float>>(config_ctrl->getSystemSettingsFilePath("VTOL"),"FF_PLANE_OUTPUT_LIMIT_LOWER");

    ff_plane_output_limit->setClipValueMaxFirst(ff_plane_output_limit_upper_val.x);
    ff_plane_output_limit->setClipValueMinFirst(ff_plane_output_limit_lower_val.x);
    ff_plane_output_limit->setClipValueMaxSecond(ff_plane_output_limit_upper_val.y);
    ff_plane_output_limit->setClipValueMinSecond(ff_plane_output_limit_lower_val.y);
    ff_plane_output_limit->setClipValueMaxThird(ff_plane_output_limit_upper_val.z);
    ff_plane_output_limit->setClipValueMinThird(ff_plane_output_limit_lower_val.z);
    
    this->addBlock(ff_plane_output_limit,"ff_plane_output_limit");


    auto const_fb_plane_mode = new Constant<Vector3D<float>>();
    Vector3D<float> zero_vec;
    zero_vec.x=0.0;
    zero_vec.y=0.0;
    zero_vec.z=0.0;
    const_fb_plane_mode->setValue(zero_vec);
    this->addBlock(const_fb_plane_mode,"const_fb_plane_mode");

    auto const_zero = new Constant<Vector3D<float>>();
    const_zero->setValue(zero_vec);
    this->addBlock(const_zero,"const_zero");

    // Biases blocks
    auto sum_bias_vtol_ff= new Sum3();
    sum_bias_vtol_ff->setOperation(Sum3::ADD);
    auto const_bias_vtol_ff_val=config_ctrl->getValueFromFile<Vector3D<float>>(config_ctrl->getSystemSettingsFilePath("VTOL"),"BIAS_VTOL_FF");
    auto const_bias_vtol_ff=new Constant<Vector3D<float>>();
    const_bias_vtol_ff->setValue(const_bias_vtol_ff_val);
    this->addBlock(const_bias_vtol_ff,"const_bias_vtol_ff");
    this->addBlock(sum_bias_vtol_ff,"sum_bias_vtol_ff");
    
    auto sum_bias_plane_ff= new Sum3();
    sum_bias_plane_ff->setOperation(Sum3::ADD);
    auto const_bias_plane_ff_val=config_ctrl->getValueFromFile<Vector3D<float>>(config_ctrl->getSystemSettingsFilePath("VTOL"),"BIAS_PLANE_FF");
    auto const_bias_plane_ff=new Constant<Vector3D<float>>();
    const_bias_plane_ff->setValue(const_bias_plane_ff_val);
    this->addBlock(const_bias_plane_ff,"const_bias_plane_ff");
    this->addBlock(sum_bias_plane_ff,"sum_bias_plane_ff");

    auto sum_bias_vtol_fb= new Sum3();
    sum_bias_vtol_fb->setOperation(Sum3::ADD);
    auto const_bias_vtol_fb_val=config_ctrl->getValueFromFile<Vector3D<float>>(config_ctrl->getSystemSettingsFilePath("VTOL"),"BIAS_VTOL_FB");
    auto const_bias_vtol_fb=new Constant<Vector3D<float>>();
    const_bias_vtol_fb->setValue(const_bias_vtol_fb_val);
    this->addBlock(const_bias_vtol_fb,"const_bias_vtol_fb");
    this->addBlock(sum_bias_vtol_fb,"sum_bias_vtol_fb");

    //fwd bias
    auto rc_fwd_bias = new Sum();
    rc_fwd_bias->setOperation(Sum::ADD);
    auto rc_fwd_bias_val=config_ctrl->getValueFromFile<float>(config_ctrl->getSystemSettingsFilePath("VTOL"),"BIAS_FWD");
    auto const_rc_fwd_bias=new Constant<float>();
    const_rc_fwd_bias->setValue(rc_fwd_bias_val);
    this->addBlock(rc_fwd_bias,"rc_fwd_bias");
    this->addBlock(const_rc_fwd_bias,"const_rc_fwd_bias");
    // ********************************

    // ******** Mode Switching *********
    this->connect(PX4_rc_to_orientation_thrust->getOutputPort<float>(PX4RCtoOrientationThrust::OP::PUB_CH_VTOL_MODE),check_vtol_mode->getInputPort<float>(CompareGreaterThan::IP::INPUT));
    this->connectAsync(check_vtol_mode->getAsyncOutputPort<int>(CompareGreaterThan::OP::CONDITION_MET_ASYNC),actu_sys->getAsyncInputPort<int>(ActuatVTOLPX4MAVLinkSystem::IP::VTOL_MODE_ASYNC));

    auto switch_ff_vtol_plane = new InvertedSwitch3();
    auto switch_fb_vtol_plane = new InvertedSwitch3();
    this->addBlock(switch_ff_vtol_plane,"switch_ff_vtol_plane");
    this->addBlock(switch_fb_vtol_plane,"switch_fb_vtol_plane");

    this->connectAsync(check_vtol_mode->getAsyncOutputPort<int>(CompareGreaterThan::OP::CONDITION_MET_ASYNC),switch_ff_vtol_plane->getAsyncInputPort<int>(InvertedSwitch3::IP::TRIG_SW_ASYNC));
    this->connectAsync(check_vtol_mode->getAsyncOutputPort<int>(CompareGreaterThan::OP::CONDITION_MET_ASYNC),switch_fb_vtol_plane->getAsyncInputPort<int>(InvertedSwitch3::IP::TRIG_SW_ASYNC));


    // ******** Primary Sync Connections *********
    auto simulate_state_est_input = config_ctrl->getValueFromFile<bool>(config_ctrl->getRCSystemsSettingsFilePath(), "simulate_state_estimator");
    this->connect(PX4_rc_to_orientation_thrust->getOutputPort<Vector3D<float>>(PX4RCtoOrientationThrust::OP::PUB_ORI_DES),orientation_error_calculator->getInputPort<Vector3D<float>>(OrientationErrorCalculator::IP::ORI_REF));
    this->connect(PX4_rc_to_orientation_thrust->getOutputPort<Vector3D<float>>(PX4RCtoOrientationThrust::OP::PUB_ORI_RATE_DES),orientation_rate_error_calculator->getInputPort<Vector3D<float>>(OrientationErrorCalculator::IP::ORI_REF));
    this->connect(PX4_rc_to_orientation_thrust->getOutputPort<Vector3D<float>>(PX4RCtoOrientationThrust::OP::PUB_ORI_DES),vtol_ff->getInputPort<Vector3D<float>>(RCtoOriFeedForward::IP::ORI_DES));
    this->connect(PX4_rc_to_orientation_thrust->getOutputPort<Vector3D<float>>(PX4RCtoOrientationThrust::OP::PUB_ORI_RATE_DES),vtol_ff->getInputPort<Vector3D<float>>(RCtoOriFeedForward::IP::ORI_RATE_DES));
    this->connect(PX4_rc_to_orientation_thrust->getOutputPort<Vector3D<float>>(PX4RCtoOrientationThrust::OP::PUB_ORI_DES),plane_ff->getInputPort<Vector3D<float>>(RCtoOriFeedForward::IP::ORI_DES));
    this->connect(PX4_rc_to_orientation_thrust->getOutputPort<Vector3D<float>>(PX4RCtoOrientationThrust::OP::PUB_ORI_RATE_DES),plane_ff->getInputPort<Vector3D<float>>(RCtoOriFeedForward::IP::ORI_RATE_DES));
   
    if (simulate_state_est_input){
        this->connect(PX4_sim_state_est_sys->getOutputPort<Vector3D<float>>(SimulatedROSStateEstimator::OP::PUB_EST_ORI),orientation_error_calculator->getInputPort<Vector3D<float>>(OrientationErrorCalculator::IP::ORI));
    }else{
        #ifdef NO_STATE_EST
        this->connect(const_zero->getOutputPort<Vector3D<float>>(Constant<Vector3D<float>>::OP::OUTPUT),orientation_error_calculator->getInputPort<Vector3D<float>>(OrientationErrorCalculator::IP::ORI));
        this->connect(const_zero->getOutputPort<Vector3D<float>>(Constant<Vector3D<float>>::OP::OUTPUT),orientation_rate_error_calculator->getInputPort<Vector3D<float>>(OrientationErrorCalculator::IP::ORI));
        #else
        this->connect(PX4_state_est_sys->getOutputPort<Vector3D<float>>(PX4StateEstimatorRefactor::OP::PUB_EST_ORI),orientation_error_calculator->getInputPort<Vector3D<float>>(OrientationErrorCalculator::IP::ORI));
        this->connect(PX4_state_est_sys->getOutputPort<Vector3D<float>>(PX4StateEstimatorRefactor::OP::PUB_EST_ORI_RATE),orientation_rate_error_calculator->getInputPort<Vector3D<float>>(OrientationErrorCalculator::IP::ORI));
        #endif
    }
    this->connect(orientation_error_calculator->getOutputPort<Vector3D<float>>(OrientationErrorCalculator::OP::ERR_RESULT),orientation_controller->getInputPort<Vector3D<float>>(OrientationController::IP::ORI_ANGLE_ERR));
    this->connect(orientation_rate_error_calculator->getOutputPort<Vector3D<float>>(OrientationErrorCalculator::OP::ERR_RESULT),orientation_controller->getInputPort<Vector3D<float>>(OrientationController::IP::ORI_ANGLE_RATE));

    //sum bias plane_ff
    this->connect(plane_ff->getOutputPort<Vector3D<float>>(RCtoOriFeedForward::OP::ANGLE_U_OUTPUT),sum_bias_plane_ff->getInputPort<Vector3D<float>>(Sum3::IP::OPERAND1));
    this->connect(const_bias_plane_ff->getOutputPort<Vector3D<float>>(Constant<Vector3D<float>>::OP::OUTPUT),sum_bias_plane_ff->getInputPort<Vector3D<float>>(Sum3::IP::OPERAND2));
    this->connect(sum_bias_plane_ff->getOutputPort<Vector3D<float>>(Sum3::OP::OUTPUT),ff_plane_output_limit->getInputPort<Vector3D<float>>());
    this->connect(ff_plane_output_limit->getOutputPort<Vector3D<float>>(),switch_ff_vtol_plane->getInputPort<Vector3D<float>>(InvertedSwitch3::IP::NO));


    //sum bias vtol_ff
    this->connect(vtol_ff->getOutputPort<Vector3D<float>>(RCtoOriFeedForward::OP::ANGLE_U_OUTPUT),sum_bias_vtol_ff->getInputPort<Vector3D<float>>(Sum3::IP::OPERAND1));
    this->connect(const_bias_vtol_ff->getOutputPort<Vector3D<float>>(Constant<Vector3D<float>>::OP::OUTPUT),sum_bias_vtol_ff->getInputPort<Vector3D<float>>(Sum3::IP::OPERAND2));
    this->connect(sum_bias_vtol_ff->getOutputPort<Vector3D<float>>(Sum3::OP::OUTPUT),ff_vtol_output_limit->getInputPort<Vector3D<float>>());
    this->connect(ff_vtol_output_limit->getOutputPort<Vector3D<float>>(),switch_ff_vtol_plane->getInputPort<Vector3D<float>>(InvertedSwitch3::IP::NC));

    this->connect(switch_ff_vtol_plane->getOutputPort<Vector3D<float>>(InvertedSwitch3::OP::COM),add_ff_with_fb->getInputPort<Vector3D<float>>(Sum3::IP::OPERAND1));

    //sum bias vtol_fb
    this->connect(orientation_controller->getOutputPort<Vector3D<float>>(OrientationController::OP::ANGLE_U_OUTPUT),sum_bias_vtol_fb->getInputPort<Vector3D<float>>(Sum3::IP::OPERAND1));
    this->connect(const_bias_vtol_fb->getOutputPort<Vector3D<float>>(Constant<Vector3D<float>>::OP::OUTPUT),sum_bias_vtol_fb->getInputPort<Vector3D<float>>(Sum3::IP::OPERAND2));
    this->connect(sum_bias_vtol_fb->getOutputPort<Vector3D<float>>(Sum3::OP::OUTPUT),fb_output_limit->getInputPort<Vector3D<float>>(Saturation3::IP::INPUT));

    this->connect(fb_output_limit->getOutputPort<Vector3D<float>>(Saturation3::OP::OUTPUT),switch_fb_vtol_plane->getInputPort<Vector3D<float>>(InvertedSwitch3::IP::NC));
    this->connect(const_fb_plane_mode->getOutputPort<Vector3D<float>>(Constant<Vector3D<float>>::OP::OUTPUT),switch_fb_vtol_plane->getInputPort<Vector3D<float>>(InvertedSwitch3::IP::NO));
    this->connect(switch_fb_vtol_plane->getOutputPort<Vector3D<float>>(InvertedSwitch3::OP::COM),add_ff_with_fb->getInputPort<Vector3D<float>>(Sum3::IP::OPERAND2));
    this->connect(add_ff_with_fb->getOutputPort<Vector3D<float>>(),actu_sys->getInputPort<Vector3D<float>>(ActuatVTOLPX4MAVLinkSystem::IP::ANGLE_U_INP));


    // Direct RC to actuation
    this->connect(PX4_rc_to_orientation_thrust->getOutputPort<float>(PX4RCtoOrientationThrust::OP::PUB_THRUST),actu_sys->getInputPort<float>(ActuatVTOLPX4MAVLinkSystem::IP::THRUST_ACT_INP));
    this->connect(PX4_rc_to_orientation_thrust->getOutputPort<float>(PX4RCtoOrientationThrust::OP::PUB_FORWARD),rc_fwd_bias->getInputPort<float>(Sum::IP::OPERAND1));
    this->connect(const_rc_fwd_bias->getOutputPort<float>(),rc_fwd_bias->getInputPort<float>(Sum::IP::OPERAND2));
    this->connect(rc_fwd_bias->getOutputPort<float>(Sum::OP::OUTPUT),actu_sys->getInputPort<float>(ActuatVTOLPX4MAVLinkSystem::IP::FORWARD_ACT_INP));

    // angle_u_ff | used with vtol without doors to decouple elevators from motors
    this->connect(const_zero->getOutputPort<Vector3D<float>>(),actu_sys->getInputPort<Vector3D<float>>(ActuatVTOLPX4MAVLinkSystem::IP::ANGLE_U_FF_INP));

    // ******* PX4 Heartbeat ******

    auto px4_heartbeat_block = new StructurePX4HeartbeatMsg_Block();
    this->addBlock(px4_heartbeat_block, "px4_heartbeat_block");
    auto mavlink_pub_heartbeat = itfc_fact_mavlink->createPublisher<mavlink_heartbeat_t>("mavlink_pub/heartbeat");
    mavlink_pub_heartbeat->setDownSamplingFactor(250);
    this->addBlock(mavlink_pub_heartbeat, "MAVLINK_Pub/heartbeast");
    this->connect(px4_heartbeat_block->getOutputPort<mavlink_heartbeat_t>(StructurePX4HeartbeatMsg_Block::OP::HEARTBEAT_MSG), mavlink_pub_heartbeat->getInputPort<mavlink_heartbeat_t>());

    // ****** ROS Publishers *******
    auto ros_pub_switch_ff_vtol_plane = itfc_fact_ros->createPublisher<Vector3D<float>>(abs_url+"/ff_control_output");
    this->addBlock(ros_pub_switch_ff_vtol_plane, "ros_pub_switch_ff_vtol_plane");
    this->connect(switch_ff_vtol_plane->getOutputPort<Vector3D<float>>(InvertedSwitch3::OP::COM), ros_pub_switch_ff_vtol_plane->getInputPort<Vector3D<float>>());

    auto ros_pub_switch_fb_vtol_plane = itfc_fact_ros->createPublisher<Vector3D<float>>(abs_url+"/fb_control_output");
    this->addBlock(ros_pub_switch_fb_vtol_plane, "ros_pub_switch_fb_vtol_plane");
    this->connect(switch_fb_vtol_plane->getOutputPort<Vector3D<float>>(InvertedSwitch3::OP::COM), ros_pub_switch_fb_vtol_plane->getInputPort<Vector3D<float>>());

    auto ros_pub_rc_fwd_bias = itfc_fact_ros->createPublisher<float>(abs_url+"/fwd_control_to_actuation");
    this->addBlock(ros_pub_rc_fwd_bias, "ros_pub_rc_fwd_bias");
    this->connect(rc_fwd_bias->getOutputPort<float>(Sum::OP::OUTPUT), ros_pub_rc_fwd_bias->getInputPort<float>());


    // ************************


    // ******* Print System Graphs ******
    Log::Warn << "******* PX4RCtoOrientationThrust | Sync *******";
    PX4_rc_to_orientation_thrust->printSyncSystemGraphToFile(config_ctrl->getDotGraphLogDirectory());
    Log::Warn << "******* PX4RCtoOrientationThrust | Async *******";
    PX4_rc_to_orientation_thrust->printAsyncSystemGraphToFile(config_ctrl->getDotGraphLogDirectory());

    Log::Warn << "******* PX4StateEstimatorRefactor | Sync *******";
    PX4_state_est_sys->printSyncSystemGraphToFile(config_ctrl->getDotGraphLogDirectory());
    Log::Warn << "******* PX4StateEstimatorRefactor | Async *******";
    PX4_state_est_sys->printAsyncSystemGraphToFile(config_ctrl->getDotGraphLogDirectory());

    Log::Warn << "OrientationController";
    orientation_controller->printSyncSystemGraphToFile(config_ctrl->getDotGraphLogDirectory());
    
    Log::Warn << "OrientationErrorCalculator";
    orientation_error_calculator->printSyncSystemGraphToFile(config_ctrl->getDotGraphLogDirectory());

    Log::Warn << "******* ActuatVTOLPX4MAVLinkSystem | Sync *******";
    actu_sys->printSyncSystemGraphToFile(config_ctrl->getDotGraphLogDirectory());
    Log::Warn << "******* ActuatVTOLPX4MAVLinkSystem | Async *******";
    actu_sys->printAsyncSystemGraphToFile(config_ctrl->getDotGraphLogDirectory());
    
    Log::Warn << "******* RCtoOriFeedForward | Sync *******";
    vtol_ff->printSyncSystemGraphToFile(config_ctrl->getDotGraphLogDirectory());
    Log::Warn << "******* RCtoOriFeedForward | Async *******";
    vtol_ff->printAsyncSystemGraphToFile(config_ctrl->getDotGraphLogDirectory());

}

void RC_OrientationThrustControlSystemVTOL::preProcessLogic(){

}

void RC_OrientationThrustControlSystemVTOL::postProcessLogic(){

}


}

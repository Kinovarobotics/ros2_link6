#include <KeTypes.h>
#include <Device/KeDevice.h>
#include <Kernel/OS/KeDelay.h>
#include <Logger/KaLogger.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <iomanip>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <cmath>

#include <KaKonverseApiTest.h>
#include <google/protobuf/util/json_util.h>
#include <TransportClientUdp.h>
#include <RouterClient.h>
#include <SessionClientRpc.h>

#include "Hardware/KaHardware.h"
#include "Utils/KaKinManager.h"
#include "Configuration/KaArmConfig.h"
#include "CapabilityManager.h"
#include "common/filesystem/utilities.h"

using namespace Kinova::Api;
using namespace google::protobuf::util;

namespace common_fs = common::filesystem::utilities;

namespace
{
    void processConfigChange(Kinova::Api::Base::ConfigurationChangeNotification configNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", configNotif.DebugString().c_str());
        printf("\n");
    }

    void processUpdatingMode(Kinova::Api::Base::UpdatingModeNotification opModeNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", opModeNotif.DebugString().c_str());
        printf("\n");
    }

    void processRobot(Kinova::Api::Base::RobotEventNotification robotNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", robotNotif.DebugString().c_str());
        printf("\n");
    }

    void processControlMode(Kinova::Api::ControlConfig::ControlModeNotification controlModeNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", controlModeNotif.DebugString().c_str());
        printf("\n");
    }

    void processServoingMode(Kinova::Api::Base::ServoingModeNotification servoingModeNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", servoingModeNotif.DebugString().c_str());
        printf("\n");
    }

    void processProgramConfigChange(Kinova::Api::ProgramConfig::ConfigurationChangeNotification configNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", configNotif.DebugString().c_str());
        printf("\n");
    }

    void processProtectionZoneChange(Kinova::Api::ProtectionZone::ProtectionZoneChangeNotification protectionZoneChangeNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", protectionZoneChangeNotif.DebugString().c_str());
        printf("\n");
    }

    /**
     * @brief Process Arm State notifications
     *
     * @param[in] armStateNotif Arm state notification object
     */
    void processArmState(Kinova::Api::Base::ArmStateNotification armStateNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", armStateNotif.DebugString().c_str());
        printf("\n");
    }

    /**
     * @brief Process Arm Speed Factor notifications
     *
     * @param[in] armSpeedFactorNotif Arm speed factor notification object
     */
    void processArmSpeedFactor(Kinova::Api::ControlConfig::ArmSpeedFactorNotification armSpeedFactorNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", armSpeedFactorNotif.DebugString().c_str());
        printf("\n");
    }

    void processToolSphereChange(Kinova::Api::ProtectionZone::ToolSphereChangeNotification toolSphereChangeNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", toolSphereChangeNotif.DebugString().c_str());
        printf("\n");
    }

    void processHandGuidingModeChange(Kinova::Api::Base::HandGuidingModeNotification handGuidingNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", handGuidingNotif.DebugString().c_str());
        printf("\n");
    }

    void processEnablingDeviceChange(Kinova::Api::Base::EnablingDeviceNotification enablingDeviceNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", enablingDeviceNotif.DebugString().c_str());
        printf("\n");
    }

    /**
     * @brief Process Acknowledge Action event
     *
     * @param acknowledgeActionNotif Acknowledge action notification object
     */
    void processAcknowledgeActionEvent(Kinova::Api::Base::AcknowledgeActionNotification acknowledgeActionNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", acknowledgeActionNotif.DebugString().c_str());
        printf("\n");
    }

    void processOperatingModeChange(Kinova::Api::Base::OperatingModeNotification operatingModeNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", operatingModeNotif.DebugString().c_str());
    }

    void processMotionEvent(Kinova::Api::Base::MotionNotification motionNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", motionNotif.DebugString().c_str());
        printf("\n");
    }

    void processSafetyModeEvent(Kinova::Api::SafetyFunctions::SafetyModeChangeNotification safetyModeNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", safetyModeNotif.DebugString().c_str());
        printf("\n");
    }

    void processProgramRequestEvent(Kinova::Api::Base::ProgramRequestNotification programRequestNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", programRequestNotif.DebugString().c_str());
        printf("\n");
    }

    void processProtectiveStopChangeEvent(Kinova::Api::SafetyFunctions::ProtectiveStopChangeNotification protectiveStopChangeNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", protectiveStopChangeNotif.DebugString().c_str());
        printf("\n");
    }

    /**
     * @brief Process safety parameters checksum change event
     *
     * @param[in] safetyParametersChecksumChangeNotif safety parameters checksum change notification
     */
    void processSafetyParametersChecksumChangeEvent(Kinova::Api::SafetyControlUnitConfig::SafetyParametersChecksumChangeNotification safetyParametersChecksumChangeNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", safetyParametersChecksumChangeNotif.DebugString().c_str());
        printf("\n");
    }

    std::string eventTypeToStr(Kinova::Api::Base::ControllerElementEventType eventType)
    {
        switch(eventType)
        {
            case Kinova::Api::Base::ControllerElementEventType::AXIS_MOVED: return "AXIS";
            case Kinova::Api::Base::ControllerElementEventType::BUTTON_DOWN: return "DOWN";
            case Kinova::Api::Base::ControllerElementEventType::BUTTON_UP: return "UP";
            case Kinova::Api::Base::ControllerElementEventType::BUTTON_CLICK: return "CLICK";
            default: return "";
        }
    }

    void processController(Kinova::Api::Base::ControllerNotification controllerNotif)
    {
        if(controllerNotif.state_case() == Kinova::Api::Base::ControllerNotification::kControllerElement)
        {
            static int lines = 0;
            if((lines++ % 20) == 0)
            {
                printf("Controller   Axis   Button   Event   Value\n");
                printf("------------------------------------------\n");
            }
            printf("%10d   %4d   %6d   %5s   %5.2f\n",
                controllerNotif.controller_element().handle().controller_handle().controller_identifier(),
                controllerNotif.controller_element().handle().axis(),
                controllerNotif.controller_element().handle().button(),
                eventTypeToStr(controllerNotif.controller_element().event_type()).c_str(),
                controllerNotif.controller_element().handle().axis()? controllerNotif.controller_element().axis_value() : 0);
        }
        else
        {
            printf("------------------------------------------\n");
            printf("%s\n", controllerNotif.DebugString().c_str());
            printf("\n");
        }
    }

    void processAction(Kinova::Api::Base::ActionNotification actionNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", actionNotif.DebugString().c_str());
        printf("\n");
    }

    void processControlConfig(Kinova::Api::ControlConfig::ControlConfigurationNotification controlConfigNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", controlConfigNotif.DebugString().c_str());
        printf("\n");
    }

    void processDiagnostic(Kinova::Api::Common::DiagnosticNotification diagnosticNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", diagnosticNotif.DebugString().c_str());
        printf("\n");
    }

    void processSafetyFunctions(Kinova::Api::SafetyFunctions::SafetyFunctionChangeNotification safetyFunctionsNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", safetyFunctionsNotif.DebugString().c_str());
        printf("\n");
    }

    void processSafetyIOChange(Kinova::Api::SafetyIO::SafetyIOChangeNotification safetyIONotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", safetyIONotif.DebugString().c_str());
        printf("\n");
    }

    /**
     * @brief Process Remote Access change event
     *
     * @param[in] RemoteAccessNotif Remote Access change notification
     */
    void processRemoteAccessChangeEvent(Kinova::Api::Base::RemoteAccessChangeNotification RemoteAccessNotif)
    {
        printf("------------------------------------------\n");
        printf("%s\n", RemoteAccessNotif.DebugString().c_str());
        printf("\n");
    }
}

KaKonverseAPITest::KaKonverseAPITest(std::shared_ptr<KaKonverseClient> pKonverseClient)
{
    m_nCurrentMsgId     = 0;
    m_pKonverseClient = pKonverseClient;
}

bool KaKonverseAPITest::DeleteProtectionZone(const std::vector<std::string>& args)
{
    Kinova::Api::ProtectionZone::ProtectionZoneHandle handle;
    handle.set_identifier(std::stoi(args[0]));

    m_pKonverseClient->protectionZone->DeleteProtectionZone(handle);

    return true;
}

bool KaKonverseAPITest::GetAllProtectionZones(const std::vector<std::string>& args)
{
    Kinova::Api::ProtectionZone::ProtectionZoneConfigList protectionZoneConfigList = m_pKonverseClient->protectionZone->ReadAllProtectionZones();
    for (const auto& protectionZone: protectionZoneConfigList.protection_zones())
    {
        auto myProtectionZone = std::make_unique<Fw3Db::ProtectionZone>(protectionZone);
	    myProtectionZone->Print();
    }

    return true;
}

bool KaKonverseAPITest::GetProtectionZone(const std::vector<std::string>& args)
{
    Kinova::Api::ProtectionZone::ProtectionZoneHandle handle;
    handle.set_identifier(std::stoi(args[0]));

    auto protectionZone = m_pKonverseClient->protectionZone->ReadProtectionZone(handle);

    auto myProtectionZone = std::make_unique<Fw3Db::ProtectionZone>(protectionZone);
    myProtectionZone->Print();

    return true;
}

uint32_t KaKonverseAPITest::CreateAction(Kinova::Api::Base::Action &action)
{
    Kinova::Api::Base::ActionHandle handle = m_pKonverseClient->baseCfg->CreateAction(action);

    return handle.identifier();
}

void KaKonverseAPITest::FillTwistAction(Kinova::Api::Base::Action &twistAction)
{
    twistAction.set_name("Twist Action");
    twistAction.set_application_data("{\"Json\" : \"twist action user data\"}");

    twistAction.mutable_send_twist_command()->set_reference_frame(Kinova::Api::Common::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED);
    twistAction.mutable_send_twist_command()->set_duration(100);
    twistAction.mutable_send_twist_command()->mutable_twist()->set_linear_x(1.2);
    twistAction.mutable_send_twist_command()->mutable_twist()->set_linear_y(3.4);
    twistAction.mutable_send_twist_command()->mutable_twist()->set_linear_z(5.6);
    twistAction.mutable_send_twist_command()->mutable_twist()->set_angular_x(7.8);
    twistAction.mutable_send_twist_command()->mutable_twist()->set_angular_y(9.0);
    twistAction.mutable_send_twist_command()->mutable_twist()->set_angular_z(1.2);
}

#if 0 // JACO3-7701 Wrench commands temporarily disabled
void KaKonverseAPITest::FillWrenchAction(Kinova::Api::Base::Action &wrenchAction)
{
    wrenchAction.set_name("Wrench Action");
    wrenchAction.set_application_data("{\"Json\" : \"wrench action user data\"}");

    wrenchAction.mutable_send_wrench_command()->set_mode(Kinova::Api::Base::WrenchMode::WRENCH_NORMAL);
    wrenchAction.mutable_send_wrench_command()->set_duration(200);
    wrenchAction.mutable_send_wrench_command()->mutable_wrench()->set_force_x(1.11);
    wrenchAction.mutable_send_wrench_command()->mutable_wrench()->set_force_y(2.22);
    wrenchAction.mutable_send_wrench_command()->mutable_wrench()->set_force_z(3.33);
    wrenchAction.mutable_send_wrench_command()->mutable_wrench()->set_torque_x(4.44);
    wrenchAction.mutable_send_wrench_command()->mutable_wrench()->set_torque_y(5.55);
    wrenchAction.mutable_send_wrench_command()->mutable_wrench()->set_torque_z(6.66);
}
#endif

void KaKonverseAPITest::FillJointSpeedAction(Kinova::Api::Base::Action &jointSpeeds)
{
    jointSpeeds.set_name("Joint Speeds Action");
    jointSpeeds.set_application_data("{\"Json\" : \"joint speeds user data\"}");

    jointSpeeds.mutable_send_joint_speeds()->set_duration(300);

    for(int jointId = 1; jointId < 8; jointId++)
    {
        Kinova::Api::Base::JointSpeed* jointSpeed = jointSpeeds.mutable_send_joint_speeds()->add_joint_speeds();
        jointSpeed->set_joint_identifier(jointId);
        jointSpeed->set_value(1.23 * jointId);
    }
}

#if 0 // JACO3-7701 Torque commands temporarily disabled
void KaKonverseAPITest::FillJointTorqueAction(Kinova::Api::Base::Action &jointTorques)
{
    jointTorques.set_name("Joint Torques Action");
    jointTorques.set_application_data("{\"Json\" : \"joint torques user data\"}");

    jointTorques.mutable_send_joint_torques()->set_duration(400);

    for(int jointId = 1; jointId < 8; jointId++)
    {
        Kinova::Api::Base::JointTorque* jointTorque = jointTorques.mutable_send_joint_torques()->add_joint_torques();
        jointTorque->set_joint_identifier(jointId);
        jointTorque->set_value(2.34 * jointId);
        jointTorque->set_duration(0);
    }
}
#endif

void KaKonverseAPITest::FillNavigateMappingAction(Kinova::Api::Base::Action &navigationDirectionAction)
{
    navigationDirectionAction.set_name("Navigation Direction");
    navigationDirectionAction.set_application_data("{\"Json\" : \"navigation direction action user data\"}");

    navigationDirectionAction.set_navigate_mappings(Kinova::Api::Base::NavigationDirection::NEXT);
}

void KaKonverseAPITest::FillSwitchMappingAction(Kinova::Api::Base::Action &switchMappingAction)
{
    switchMappingAction.set_name("Switch Mapping");
    switchMappingAction.set_application_data("{\"Json\" : \"switch mapping action user data\"}");

    switchMappingAction.mutable_switch_control_mapping()->set_controller_identifier(1);
    switchMappingAction.mutable_switch_control_mapping()->mutable_map_group_handle()->set_identifier(2);
    switchMappingAction.mutable_switch_control_mapping()->mutable_map_handle()->set_identifier(3);
}

void KaKonverseAPITest::FillToggleHandGuidingAction(Kinova::Api::Base::Action &handGuidingMode)
{
    handGuidingMode.set_name("Hand-Guiding Mode");
    handGuidingMode.set_application_data("{\"Json\" : \"hand-guiding mode action user data\"}");

    handGuidingMode.set_toggle_hand_guiding_mode(Kinova::Api::Base::HandGuidingMode::CARTESIAN);
}

void KaKonverseAPITest::FillApplyQuickStopAction(Kinova::Api::Base::Action &quickStopAction)
{
    quickStopAction.set_name("Apply Quick Stop");
    quickStopAction.set_application_data("{\"Json\" : \"apply quick stop action user data\"}");
    quickStopAction.mutable_apply_quick_stop();
}

void KaKonverseAPITest::FillClearFaultsAction(Kinova::Api::Base::Action &clearFaultsAction)
{
    clearFaultsAction.set_name("Clear Faults");
    clearFaultsAction.set_application_data("{\"Json\" : \"clear faults action user data\"}");
    clearFaultsAction.mutable_clear_faults();
}

void KaKonverseAPITest::FillExecAction(Kinova::Api::Base::Action &execAction, uint32_t actionId)
{
    execAction.set_name("Execute");
    execAction.set_application_data("{\"Json\" : \"exec action user data\"}");

    execAction.mutable_execute_action()->set_identifier(actionId);
}

uint32_t KaKonverseAPITest::CreateTwistAction()
{
    printf("Creating TWIST action\n");
    Kinova::Api::Base::Action twistAction;

    FillTwistAction(twistAction);
    return CreateAction(twistAction);
}

#if 0 // JACO3-7701 Wrench commands temporarily disabled
uint32_t KaKonverseAPITest::CreateWrenchAction()
{
    printf("Creating WRENCH action\n");
    Kinova::Api::Base::Action wrenchAction;

    FillWrenchAction(wrenchAction);
    return CreateAction(wrenchAction);
}
#endif

uint32_t KaKonverseAPITest::CreateJointSpeedAction()
{
    printf("Creating JOINT SPEED action\n");
    Kinova::Api::Base::Action jointSpeeds;

    FillJointSpeedAction(jointSpeeds);
    return CreateAction(jointSpeeds);
}

#if 0 // JACO3-7701 Torque commands temporarily disabled
uint32_t KaKonverseAPITest::CreateJointTorqueAction()
{
    printf("Creating JOINT TORQUES action\n");
    Kinova::Api::Base::Action jointTorques;

    FillJointTorqueAction(jointTorques);
    return CreateAction(jointTorques);
}
#endif

uint32_t KaKonverseAPITest::CreateExecAction(uint32_t actionId)
{
    printf("Creating EXEC action ref to %d\n", actionId);
    Kinova::Api::Base::Action execAction;

    FillExecAction(execAction, actionId);
    return CreateAction(execAction);
}

uint32_t KaKonverseAPITest::CreateNavigateMappingAction()
{
    printf("Creating NAVIGATION DIRECTION action\n");
    Kinova::Api::Base::Action navigationDirectionAction;

    FillNavigateMappingAction(navigationDirectionAction);
    return CreateAction(navigationDirectionAction);
}

uint32_t KaKonverseAPITest::CreateSwitchMappingAction()
{
    printf("Creating SWITCH MAPPING action\n");
    Kinova::Api::Base::Action switchMappingAction;

    FillSwitchMappingAction(switchMappingAction);
    return CreateAction(switchMappingAction);
}

uint32_t KaKonverseAPITest::CreateToggleHandGuidingAction()
{
    printf("Creating HAND_GUIDING MODE action\n");
    Kinova::Api::Base::Action handGuidingMode;

    FillToggleHandGuidingAction(handGuidingMode);
    return CreateAction(handGuidingMode);
}

uint32_t KaKonverseAPITest::CreateApplyQuickStopAction()
{
    printf("Creating APPLY QUICK STOP action\n");
    Kinova::Api::Base::Action quickStopAction;

    FillApplyQuickStopAction(quickStopAction);
    return CreateAction(quickStopAction);
}

uint32_t KaKonverseAPITest::CreateClearFaultsAction()
{
    printf("Creating CLEAR FAULTS action\n");
    Kinova::Api::Base::Action clearFaultsAction;

    FillClearFaultsAction(clearFaultsAction);
    return CreateAction(clearFaultsAction);
}

void KaKonverseAPITest::UpdateAction(uint32_t id)
{
    Kinova::Api::Base::Action actionItemIn;


    m_pKonverseClient->baseCfg->UpdateAction(actionItemIn);
}

void KaKonverseAPITest::GetAction(uint32_t id)
{
    Kinova::Api::Base::ActionHandle handle;

    handle.set_identifier(id);
    Kinova::Api::Base::Action actionItem = m_pKonverseClient->baseCfg->ReadAction(handle);

    Fw3Db::Action dbAction(&actionItem);
    dbAction.Print();
}

void KaKonverseAPITest::DeleteAction(uint32_t id)
{
    Kinova::Api::Base::ActionHandle handle;

    handle.set_identifier(id);
    m_pKonverseClient->baseCfg->DeleteAction(handle);
}

bool KaKonverseAPITest::ReadAllActions(std::vector<std::string> args)
{
    Kinova::Api::Base::RequestedActionType requestedActionType;
    requestedActionType.set_action_type((Kinova::Api::Base::ActionType)std::stoi(args[0]));

    Kinova::Api::Base::ActionList actionList = m_pKonverseClient->baseCfg->ReadAllActions(requestedActionType);
    for(auto action: actionList.action_list())
    {
        auto myAction(ActionFactory(&action));
        if(myAction != nullptr)
        {
            myAction->permission = action.handle().permission();
	        myAction->Print();
        }
    }

    return true;
}

bool KaKonverseAPITest::PlayReachCartesianWaypoint(std::vector<std::string> args)
{

    Kinova::Api::Base::WaypointList waypointList;
    waypointList.set_duration(0);
    auto waypoint = waypointList.add_waypoints();
    auto cartesianWaypoint = waypoint->mutable_cartesian_waypoint();

    cartesianWaypoint->mutable_pose()->set_x(std::stof(args[0]));
    cartesianWaypoint->mutable_pose()->set_y(std::stof(args[1]));
    cartesianWaypoint->mutable_pose()->set_z(std::stof(args[2]));
    cartesianWaypoint->mutable_pose()->set_theta_x(std::stof(args[3]));
    cartesianWaypoint->mutable_pose()->set_theta_y(std::stof(args[4]));
    cartesianWaypoint->mutable_pose()->set_theta_z(std::stof(args[5]));

    m_pKonverseClient->baseCfg->ExecuteWaypointTrajectory(waypointList);

    printf("Waiting %d seconds to exit\n", std::stoi(args[6]));
    sleep(std::stoi(args[6]));

    return true;
}

/**
 * @brief Fill and execute a SetToolConfigurationList command action
 *
 * Use the tool configuration list information from the command line, create an Action object and send it to ExecuteAction.
 *
 * @param[in] args Arguments from the command line.
 * @return true always.
 */
bool KaKonverseAPITest::SetToolConfigurationListAction(std::vector<std::string> args)
{
    Kinova::Api::Base::Action setToolConfigurationListAction;

    setToolConfigurationListAction.mutable_handle()->set_action_type(Kinova::Api::Base::SET_TOOL_CONFIGURATION_LIST);
    setToolConfigurationListAction.mutable_handle()->set_identifier(Fw3Db::nToolConfigurationId);
    setToolConfigurationListAction.set_name("Set Tool Configuration List Action");
    setToolConfigurationListAction.set_application_data("{\"Json\" : \"tool configuration user data\"}");

    auto setToolConfigurationList = setToolConfigurationListAction.mutable_tool_configuration_list();

    uint32_t argsIndex = 0;
    for(uint32_t toolId = 0; toolId < std::stoi(args[0]); toolId++)
    {
        auto toolConfiguration = setToolConfigurationList->add_tool_configurations();
        toolConfiguration->mutable_tool_transform()->set_x(std::stof(args[argsIndex + 1]));
        toolConfiguration->mutable_tool_transform()->set_y(std::stof(args[argsIndex + 2]));
        toolConfiguration->mutable_tool_transform()->set_z(std::stof(args[argsIndex + 3]));
        toolConfiguration->mutable_tool_transform()->set_theta_x(std::stof(args[argsIndex + 4]));
        toolConfiguration->mutable_tool_transform()->set_theta_y(std::stof(args[argsIndex + 5]));
        toolConfiguration->mutable_tool_transform()->set_theta_z(std::stof(args[argsIndex + 6]));
        toolConfiguration->set_tool_mass(std::stof(args[argsIndex + 7]));
        toolConfiguration->mutable_tool_mass_center()->set_x(std::stof(args[argsIndex + 8]));
        toolConfiguration->mutable_tool_mass_center()->set_y(std::stof(args[argsIndex + 9]));
        toolConfiguration->mutable_tool_mass_center()->set_z(std::stof(args[argsIndex + 10]));
        toolConfiguration->mutable_tool_inertia_matrix()->Clear();
        argsIndex += 10;

    }

    m_pKonverseClient->baseCfg->ExecuteAction(setToolConfigurationListAction);

    return true;
}

bool KaKonverseAPITest::SendTwistCommand(std::vector<std::string> args)
{
    Kinova::Api::Base::TwistCommand twistCommand;

    twistCommand.set_reference_frame(Kinova::Api::Common::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED);

    twistCommand.mutable_twist()->set_linear_x(std::stof(args[0]));
    twistCommand.mutable_twist()->set_linear_y(std::stof(args[1]));
    twistCommand.mutable_twist()->set_linear_z(std::stof(args[2]));
    twistCommand.mutable_twist()->set_angular_x(std::stof(args[3]));
    twistCommand.mutable_twist()->set_angular_y(std::stof(args[4]));
    twistCommand.mutable_twist()->set_angular_z(std::stof(args[5]));

    printf("Sending twist command: %s\n", twistCommand.DebugString().c_str());
    m_pKonverseClient->baseCfg->SendTwistCommand(twistCommand);

    printf("Waiting %d seconds to exit\n", std::stoi(args[6]));
    sleep(std::stoi(args[6]));

    return true;
}

bool KaKonverseAPITest::SendJointSpeeds(std::vector<std::string> args)
{
    Kinova::Api::Base::JointSpeeds jointSpeeds;

    for(uint32_t jointId = 0; jointId < std::stoi(args[0]); jointId++)
    {
        Kinova::Api::Base::JointSpeed* jointSpeed = jointSpeeds.add_joint_speeds();
        jointSpeed->set_joint_identifier(jointId);
        jointSpeed->set_value(std::stof(args[jointId + 1]));
    }

    m_pKonverseClient->baseCfg->SendJointSpeedsCommand(jointSpeeds);

    printf("Waiting %d seconds to exit\n", std::stoi(args[std::stoi(args[0]) + 1]));
    sleep(std::stoi(args[std::stoi(args[0]) + 1]));

    return true;
}

bool KaKonverseAPITest::ComputeForwardKinematics(std::vector<std::string> args)
{
    Kinova::Api::Base::JointAngles jointData;
    Kinova::Api::Base::Pose poseData;

    for(uint32_t jointId = 0; jointId < args.size(); jointId++)
    {
        Kinova::Api::Base::JointAngle* jAngle = jointData.add_joint_angles();
        jAngle->set_value(std::stof(args[jointId]));
    }

    poseData = m_pKonverseClient->baseCfg->ComputeForwardKinematics(jointData);

    printf("Forward Kinematics result : The cartesian pose obtained from the joint data :"
    "\n=========INPUT=========\n%s"
    "\n=======================\n"
    "is :\n========OUTPUT=========\n%s"
    "\n=======================\n",
    jointData.DebugString().c_str(),
    poseData.DebugString().c_str());

    return true;
}

bool KaKonverseAPITest::ComputeInverseKinematics(std::vector<std::string> args)
{
    Kinova::Api::Base::IKData IKData;
    Kinova::Api::Base::JointAngles resultData;

    IKData.mutable_cartesian_pose()->set_x(std::stof(args[0]));
    IKData.mutable_cartesian_pose()->set_y(std::stof(args[1]));
    IKData.mutable_cartesian_pose()->set_z(std::stof(args[2]));
    IKData.mutable_cartesian_pose()->set_theta_x(std::stof(args[3]));
    IKData.mutable_cartesian_pose()->set_theta_y(std::stof(args[4]));
    IKData.mutable_cartesian_pose()->set_theta_z(std::stof(args[5]));

    for(uint32_t jointId = 6; jointId < args.size(); jointId++)
    {
        Kinova::Api::Base::JointAngle* jAngle = IKData.mutable_guess()->add_joint_angles();
        jAngle->set_value(std::stof(args[jointId]));
    }

    resultData = m_pKonverseClient->baseCfg->ComputeInverseKinematics(IKData);

    printf("Inverse Kinematics result : The joint angles obtained from the cartesian pose: "
    "\n=========INPUT=========\n%s"
    "\n=======================\n"
    "is :\n========OUTPUT=========\n%s"
    "\n=======================\n",
    IKData.DebugString().c_str(),
    resultData.DebugString().c_str());

    return true;
}

bool KaKonverseAPITest::GetProductConfiguration(std::vector<std::string> args)
{
    Kinova::Api::ProductConfiguration::CompleteProductConfiguration productConfig = m_pKonverseClient->baseCfg->GetProductConfiguration();
    printf("%s\n", productConfig.DebugString().c_str());

    return true;
}

bool KaKonverseAPITest::Stop(std::vector<std::string> args)
{
    m_pKonverseClient->baseCfg->Stop();

    return true;
}

void KaKonverseAPITest::PauseAction()
{
    m_pKonverseClient->baseCfg->PauseAction();
}

void KaKonverseAPITest::ResumeAction()
{
    m_pKonverseClient->baseCfg->ResumeAction();
}

void KaKonverseAPITest::StopAction()
{
    m_pKonverseClient->baseCfg->StopAction();
}

uint32_t KaKonverseAPITest::GetUpdatingMode()
{
    Kinova::Api::Base::UpdatingModeInformation updatingmode = m_pKonverseClient->baseCfg->GetUpdatingMode();

    printf("Updating mode is %d\n", updatingmode.updating_mode());
    return updatingmode.updating_mode();
}

uint32_t KaKonverseAPITest::GetControlMode()
{
    Kinova::Api::ControlConfig::ControlModeInformation controlMode = m_pKonverseClient->controlCfg->GetControlMode();

    printf("Control mode is %d\n", controlMode.control_mode());
    return controlMode.control_mode();
}

uint32_t KaKonverseAPITest::GetServoingMode()
{
    Kinova::Api::Base::ServoingModeInformation servoingMode = m_pKonverseClient->baseCfg->GetServoingMode();

    printf("Servoing mode is %d\n", servoingMode.servoing_mode());
    return servoingMode.servoing_mode();
}

bool KaKonverseAPITest::SetServoingMode(std::vector<std::string> args)
{
    Kinova::Api::Base::ServoingModeInformation servoingMode;
    servoingMode.set_servoing_mode((Kinova::Api::Base::ServoingMode)std::stoi(args[0]));

    m_pKonverseClient->baseCfg->SetServoingMode(servoingMode);

    return true;
}

void KaKonverseAPITest::SetServoingMode(uint32_t mode)
{
    Kinova::Api::Base::ServoingModeInformation servoingMode;
    servoingMode.set_servoing_mode((Kinova::Api::Base::ServoingMode)mode);

    m_pKonverseClient->baseCfg->SetServoingMode(servoingMode);
}

void KaKonverseAPITest::SetCartesianReferenceFrame(uint32_t mode)
{
    printf("\n Set Reference Frame \n");
    Kinova::Api::ControlConfig::CartesianReferenceFrameInfo refFrame;
    refFrame.set_reference_frame(Kinova::Api::Common::CARTESIAN_REFERENCE_FRAME_MIXED);

    m_pKonverseClient->controlCfg->SetCartesianReferenceFrame(refFrame);
}

bool KaKonverseAPITest::SetHandGuidingMode(std::vector<std::string> args)
{
    Kinova::Api::Base::HandGuiding handGuiding;
    handGuiding.set_hand_guiding_mode((Kinova::Api::Base::HandGuidingMode)std::stoi(args[0]));

    m_pKonverseClient->baseCfg->SetHandGuidingMode(handGuiding);

    return true;
}

bool KaKonverseAPITest::GetHandGuidingMode(std::vector<std::string> args)
{
    auto handGuiding = m_pKonverseClient->baseCfg->GetHandGuidingMode();

    printf("%s \n", handGuiding.DebugString().c_str());

    return true;
}

bool KaKonverseAPITest::GetIpv4Configuration(std::vector<std::string> args)
{
    Kinova::Api::Base::NetworkHandle networkHandle;
    networkHandle.set_type((Kinova::Api::Base::NetworkType)std::stoi(args[0]));
    Kinova::Api::Base::IPv4Configuration readIPv4 = m_pKonverseClient->baseCfg->GetIPv4Configuration(networkHandle);

    char ipBuf[16];
    char maskBuf[16];
    char gwBuf[16];

    uint32_t ip     = ntohl(readIPv4.ip_address());
    uint32_t mask   = ntohl(readIPv4.subnet_mask());
    uint32_t gw     = ntohl(readIPv4.default_gateway());

    inet_ntop(AF_INET, &ip, ipBuf, sizeof(ipBuf));
    inet_ntop(AF_INET, &mask, maskBuf, sizeof(maskBuf));
    inet_ntop(AF_INET, &gw, gwBuf, sizeof(gwBuf));

    printf("\n");
    printf("==== IPv4 Configuration intfc %d (%s) ====\n",
            std::stoi(args[0]), Kinova::Api::Base::NetworkType_Name(networkHandle.type()).c_str());
    printf("        IPAddress: 0x%08X - %s\n", ip, ipBuf);
    printf("             Mask: 0x%08x - %s\n", mask, maskBuf);
    printf("       Default Gw: 0x%08x - %s\n", gw, gwBuf);
    printf("             DHCP: %s\n", readIPv4.dhcp_enabled()? "Enabled" : "Disabled");

    return true;
}

bool KaKonverseAPITest::SetIpv4Configuration(std::vector<std::string> args)
{
    Kinova::Api::Base::FullIPv4Configuration ipv4Config;
    uint32_t address;
    uint32_t mask;
    uint32_t gateway;

    inet_pton(AF_INET, args[1].c_str(), &address);
    inet_pton(AF_INET, args[2].c_str(), &mask);
    inet_pton(AF_INET, args[3].c_str(), &gateway);

    ipv4Config.mutable_handle()->set_type((Kinova::Api::Base::NetworkType)std::stoi(args[0]));

    ipv4Config.mutable_ipv4_configuration()->set_ip_address(htonl(address));
    ipv4Config.mutable_ipv4_configuration()->set_subnet_mask(htonl(mask));
    ipv4Config.mutable_ipv4_configuration()->set_default_gateway(htonl(gateway));
    ipv4Config.mutable_ipv4_configuration()->set_dhcp_enabled(args[4].compare("on")? false : true);

    m_pKonverseClient->baseCfg->SetIPv4Configuration(ipv4Config);

    return true;
}

bool KaKonverseAPITest::GetIpv4Information(std::vector<std::string> args)
{
    Kinova::Api::Base::NetworkHandle handle;
    handle.set_type((Kinova::Api::Base::NetworkType)std::stoi(args[0]));
    Kinova::Api::Base::IPv4Information readIPv4 = m_pKonverseClient->baseCfg->GetIPv4Information(handle);

    char ipBuf[16];
    char maskBuf[16];
    char gwBuf[16];

    uint32_t ip     = ntohl(readIPv4.ip_address());
    uint32_t mask   = ntohl(readIPv4.subnet_mask());
    uint32_t gw     = ntohl(readIPv4.default_gateway());

    inet_ntop(AF_INET, &ip, ipBuf, sizeof(ipBuf));
    inet_ntop(AF_INET, &mask, maskBuf, sizeof(maskBuf));
    inet_ntop(AF_INET, &gw, gwBuf, sizeof(gwBuf));

    printf("\n");
    printf("==== IPv4 Information intfc %d ====\n", std::stoi(args[0]));
    printf("        IPAddress: 0x%08X - %s\n", ip, ipBuf);
    printf("             Mask: 0x%08x - %s\n", mask, maskBuf);
    printf("       Default Gw: 0x%08x - %s\n", gw, gwBuf);

    return true;
}

/**
* @brief Allows command to get the Remote Access info list using the testclient tool.
* @param[in] args Every arguments related to the testclient tool command. (unused)
* @return True value
*/
bool KaKonverseAPITest::GetRemoteAccessInfoList(std::vector<std::string> args)
{
    auto list = m_pKonverseClient->baseCfg->GetRemoteAccessInfoList();

    printf("\n");
    for (auto info: list.remote_access_info())
    {
        printf("\n");
        printf("==== Remote Access Parameter #%02i ====\n", info.handle().identifier());
        printf("     Description: %s\n", info.description().c_str());
        printf("     Enabled: %s\n" , info.is_enabled() ? "Yes" : "No");
        printf("     Key Required: %s\n" , info.is_key_required() ? "Yes" : "No");
        printf("\n");
    }
    return true;
}

/**
* @brief Allows command to enable a Remote Access using the testclient tool.
* @param[in] args Every arguments related to the testclient tool command - Id of the Remote Access.
* @return True value
*/
bool KaKonverseAPITest::EnableRemoteAccess(std::vector<std::string> args)
{
    Kinova::Api::Base::EnableRemoteAccessArgs remoteAccessArgs;
    remoteAccessArgs.mutable_handle()->set_identifier(std::stoi(args[0]));
    if (args.size() == 2)
    {
        remoteAccessArgs.set_key(args[1]);
    }
    m_pKonverseClient->baseCfg->EnableRemoteAccess(remoteAccessArgs);
    return true;
}

/**
* @brief Allows command to Disable a Remote Access using the testclient tool.
* @param[in] args Every arguments related to the testclient tool command - Id of the Remote Access.
* @return True value
*/
bool KaKonverseAPITest::DisableRemoteAccess(std::vector<std::string> args)
{
    Kinova::Api::Base::RemoteAccessHandle handle;
    handle.set_identifier(std::stoi(args[0]));
    m_pKonverseClient->baseCfg->DisableRemoteAccess(handle);
    return true;
}

/**
* @brief Unlocks API access with unique key
* @param[in] args No arguments
* @return True value
*/
bool KaKonverseAPITest::UnlockApiAccess(std::vector<std::string> args)
{
    hw::InitBoard(false);

    KaKinManager manager;
    manager.Init();

    auto armCfg{KaArmConfig::GetArmConfig()};
    armCfg->Init(manager.GetEditableKin());

    Kinova::Api::Base::EnableRemoteAccessArgs remoteAccessArgs;
    remoteAccessArgs.mutable_handle()->set_identifier(1);
    remoteAccessArgs.set_key(armCfg->GetHashedUnlockKey());

    m_pKonverseClient->baseCfg->EnableRemoteAccess(remoteAccessArgs);

    printf("Api access is now unlocked.\n");

    return true;
}

bool KaKonverseAPITest::GetGravityVector(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::GravityVector gravityVector = m_pKonverseClient->controlCfg->GetGravityVector();

    printf("%s\n", gravityVector.DebugString().c_str());

    return true;
}

bool KaKonverseAPITest::SetGravityVector(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::GravityVector gravityVector;

    gravityVector.set_x(std::stof(args[0]));
    gravityVector.set_y(std::stof(args[1]));
    gravityVector.set_z(std::stof(args[2]));

    m_pKonverseClient->controlCfg->SetGravityVector(gravityVector);

    return true;
}

bool KaKonverseAPITest::ResetGravityVector(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::GravityVector gravityVector;

    gravityVector = m_pKonverseClient->controlCfg->ResetGravityVector();

    printf("%s\n", gravityVector.DebugString().c_str());

    return true;
}

/**
* @brief Allows command to set force torque sensor value to zero using the testclient tool.
* @param[in] args Every arguments related to the testclient tool command.
* @return True value
*/
bool KaKonverseAPITest::ZeroExternalWrenchFromFTSensor(std::vector<std::string> args)
{
    m_pKonverseClient->controlCfg->ZeroExternalWrenchFromFTSensor();

    return true;
}

/**
* @brief Command to reset force torque sensor offset values to zero using the testclient tool.
* @param[in] args Every arguments related to the testclient tool command.
* @return True value
*/
bool KaKonverseAPITest::ResetExternalWrenchFromFTSensor(std::vector<std::string> args)
{
    m_pKonverseClient->controlCfg->ResetExternalWrenchFromFTSensor();

    return true;
}

bool KaKonverseAPITest::SetTwistLinearSoftLimit(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::TwistLinearSoftLimit twistLinearSoftLimit;

    twistLinearSoftLimit.set_control_mode(static_cast<Kinova::Api::ControlConfig::ControlMode>(std::stoi(args[0])));
    twistLinearSoftLimit.set_twist_linear_soft_limit(std::stof(args[1]));

    m_pKonverseClient->controlCfg->SetTwistLinearSoftLimit(twistLinearSoftLimit);

    return true;
}

bool KaKonverseAPITest::ResetTwistLinearSoftLimit(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::TwistLinearSoftLimit twistLinearSoftLimit;
    Kinova::Api::ControlConfig::ControlModeInformation controlModeInfo;
    controlModeInfo.set_control_mode(static_cast<Kinova::Api::ControlConfig::ControlMode>(std::stoi(args[0])));

    twistLinearSoftLimit = m_pKonverseClient->controlCfg->ResetTwistLinearSoftLimit(controlModeInfo);

    printf("%s\n", twistLinearSoftLimit.DebugString().c_str());

    return true;
}

bool KaKonverseAPITest::SetTwistAngularSoftLimit(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::TwistAngularSoftLimit twistAngularSoftLimit;

    twistAngularSoftLimit.set_control_mode(static_cast<Kinova::Api::ControlConfig::ControlMode>(std::stoi(args[0])));
    twistAngularSoftLimit.set_twist_angular_soft_limit(std::stof(args[1]));

    m_pKonverseClient->controlCfg->SetTwistAngularSoftLimit(twistAngularSoftLimit);

    return true;
}

bool KaKonverseAPITest::ResetTwistAngularSoftLimit(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::TwistAngularSoftLimit twistAngularSoftLimit;
    Kinova::Api::ControlConfig::ControlModeInformation controlModeInfo;
    controlModeInfo.set_control_mode(static_cast<Kinova::Api::ControlConfig::ControlMode>(std::stoi(args[0])));

    twistAngularSoftLimit = m_pKonverseClient->controlCfg->ResetTwistAngularSoftLimit(controlModeInfo);

    printf("%s\n", twistAngularSoftLimit.DebugString().c_str());

    return true;
}

bool KaKonverseAPITest::SetJointPositionSoftLimits(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::JointPositionSoftLimits jointPositionSoftLimits;
    jointPositionSoftLimits.set_control_mode( static_cast<Kinova::Api::ControlConfig::ControlMode>(std::stoi(args[0])));

    uint32_t argsIndex = 0;
    for(uint32_t jointId = 0; jointId < std::stoi(args[1]); jointId++)
    {
        auto limits = jointPositionSoftLimits.add_joint_position_soft_limits();
        limits->set_lower(std::stof(args[argsIndex + 2]));
        limits->set_upper(std::stof(args[argsIndex + 3]));
        argsIndex += 2;
    }

    m_pKonverseClient->controlCfg->SetJointPositionSoftLimits(jointPositionSoftLimits);

    return true;
}

bool KaKonverseAPITest::ResetJointPositionSoftLimits(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::JointPositionSoftLimits jointPositionSoftLimits;
    Kinova::Api::ControlConfig::ControlModeInformation controlModeInfo;
    controlModeInfo.set_control_mode(static_cast<Kinova::Api::ControlConfig::ControlMode>(std::stoi(args[0])));

    jointPositionSoftLimits = m_pKonverseClient->controlCfg->ResetJointPositionSoftLimits(controlModeInfo);

    printf("%s\n", jointPositionSoftLimits.DebugString().c_str());

    return true;
}

bool KaKonverseAPITest::SetJointSpeedSoftLimits(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::JointSpeedSoftLimits jointSpeedSoftLimits;
    jointSpeedSoftLimits.set_control_mode( static_cast<Kinova::Api::ControlConfig::ControlMode>(std::stoi(args[0])));

    for(uint32_t jointId = 0; jointId < std::stoi(args[1]); jointId++)
    {
        jointSpeedSoftLimits.add_joint_speed_soft_limits(std::stof(args[jointId + 2]));
    }

    m_pKonverseClient->controlCfg->SetJointSpeedSoftLimits(jointSpeedSoftLimits);

    return true;
}

bool KaKonverseAPITest::ResetJointSpeedSoftLimits(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::JointSpeedSoftLimits jointSpeedSoftLimits;
    Kinova::Api::ControlConfig::ControlModeInformation controlModeInfo;
    controlModeInfo.set_control_mode(static_cast<Kinova::Api::ControlConfig::ControlMode>(std::stoi(args[0])));

    jointSpeedSoftLimits = m_pKonverseClient->controlCfg->ResetJointSpeedSoftLimits(controlModeInfo);

    printf("%s\n", jointSpeedSoftLimits.DebugString().c_str());

    return true;
}

bool KaKonverseAPITest::GetHardLimits(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::KinematicLimits hardLimits = m_pKonverseClient->controlCfg->GetKinematicHardLimits();

    printf("%s\n", hardLimits.DebugString().c_str());

    return true;
}

bool KaKonverseAPITest::GetSoftLimitsRange(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::KinematicLimits softLimitsRange = m_pKonverseClient->controlCfg->GetSoftLimitsRange();

    printf("%s\n", softLimitsRange.DebugString().c_str());

    return true;
}

bool KaKonverseAPITest::GetSoftLimits(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::ControlModeInformation controlMessage;
    controlMessage.set_control_mode(static_cast<Kinova::Api::ControlConfig::ControlMode>(std::stoi(args[0])));
    Kinova::Api::ControlConfig::KinematicLimits softLimits = m_pKonverseClient->controlCfg->GetKinematicSoftLimits(controlMessage);

    printf("%s\n", softLimits.DebugString().c_str());

    return true;
}

bool KaKonverseAPITest::GetAllSoftLimits(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::KinematicLimitsList kinematicLimitsList = m_pKonverseClient->controlCfg->GetAllKinematicSoftLimits();

    printf("%s\n", kinematicLimitsList.DebugString().c_str());

    return true;
}

bool KaKonverseAPITest::GetDesiredSpeed(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::DesiredSpeeds desiredSpeeds = m_pKonverseClient->controlCfg->GetDesiredSpeeds();

    printf("%s\n", desiredSpeeds.DebugString().c_str());
    return true;
}

bool KaKonverseAPITest::SetLinearDesiredSpeed(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::LinearTwist linearTwist;

    linearTwist.set_linear(std::stof(args[0]));

    m_pKonverseClient->controlCfg->SetDesiredLinearTwist(linearTwist);

    return true;
}

bool KaKonverseAPITest::SetAngularDesiredSpeed(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::AngularTwist angularTwist;

    angularTwist.set_angular(std::stof(args[0]));

    m_pKonverseClient->controlCfg->SetDesiredAngularTwist(angularTwist);

    return true;
}

bool KaKonverseAPITest::SetJointDesiredSpeed(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::JointSpeeds jointSpeeds;

    for(uint32_t actuatorIndex = 0; actuatorIndex < std::stoi(args[0]); actuatorIndex++)
    {
        jointSpeeds.add_joint_speed(std::stof(args[actuatorIndex+1]));
    }

    m_pKonverseClient->controlCfg->SetDesiredJointSpeeds(jointSpeeds);

    return true;
}

bool KaKonverseAPITest::SetJointAccelerationSoftLimits(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::JointAccelerationSoftLimits jointAccelerationSoftLimits;
    jointAccelerationSoftLimits.set_control_mode( static_cast<Kinova::Api::ControlConfig::ControlMode>(std::stoi(args[0])));

    for(uint32_t jointId = 0; jointId < std::stoi(args[1]); jointId++)
    {
        jointAccelerationSoftLimits.add_joint_acceleration_soft_limits(std::stof(args[jointId + 2]));
    }

    m_pKonverseClient->controlCfg->SetJointAccelerationSoftLimits(jointAccelerationSoftLimits);

    return true;
}

bool KaKonverseAPITest::ResetJointAccelerationSoftLimits(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::JointAccelerationSoftLimits jointAccelerationSoftLimits;
    Kinova::Api::ControlConfig::ControlModeInformation controlModeInfo;
    controlModeInfo.set_control_mode(static_cast<Kinova::Api::ControlConfig::ControlMode>(std::stoi(args[0])));

    jointAccelerationSoftLimits = m_pKonverseClient->controlCfg->ResetJointAccelerationSoftLimits(controlModeInfo);

    printf("%s\n", jointAccelerationSoftLimits.DebugString().c_str());

    return true;
}

bool KaKonverseAPITest::GetPayloadInformation(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::PayloadInformation payloadInformation = m_pKonverseClient->controlCfg->GetPayloadInformation();

    printf("%s\n", payloadInformation.DebugString().c_str());

    return true;
}

bool KaKonverseAPITest::ResetPayloadInformation(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::PayloadInformation payloadInformation = m_pKonverseClient->controlCfg->ResetPayloadInformation();

    printf("%s\n", payloadInformation.DebugString().c_str());

    return true;
}

bool KaKonverseAPITest::GetFwBundleVersion(std::vector<std::string> args)
{
    Kinova::Api::Base::FirmwareBundleVersions firmwareBundleVersions = m_pKonverseClient->baseCfg->GetFirmwareBundleVersions();

    printf("%s\n", firmwareBundleVersions.DebugString().c_str());

    return true;
}

bool KaKonverseAPITest::SetPayloadInformation(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::PayloadInformation payloadInformation;

    payloadInformation.mutable_payload_mass_center()->set_x(std::stof(args[0]));
    payloadInformation.mutable_payload_mass_center()->set_y(std::stof(args[1]));
    payloadInformation.mutable_payload_mass_center()->set_z(std::stof(args[2]));
    payloadInformation.set_payload_mass(std::stof(args[3]));
    payloadInformation.mutable_payload_inertia()->set_ixx(std::stof(args[4]));
    payloadInformation.mutable_payload_inertia()->set_ixy(std::stof(args[5]));
    payloadInformation.mutable_payload_inertia()->set_ixz(std::stof(args[6]));
    payloadInformation.mutable_payload_inertia()->set_iyy(std::stof(args[7]));
    payloadInformation.mutable_payload_inertia()->set_iyz(std::stof(args[8]));
    payloadInformation.mutable_payload_inertia()->set_izz(std::stof(args[9]));

    m_pKonverseClient->controlCfg->SetPayloadInformation(payloadInformation);

    return true;
}

bool KaKonverseAPITest::GetToolConfiguration(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::ToolConfiguration toolConfiguration = m_pKonverseClient->controlCfg->GetToolConfiguration();

    printf("%s\n", toolConfiguration.DebugString().c_str());

    return true;
}

bool KaKonverseAPITest::SetToolConfiguration(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::ToolConfiguration toolConfiguration;

    toolConfiguration.mutable_tool_transform()->set_x(std::stof(args[0]));
    toolConfiguration.mutable_tool_transform()->set_y(std::stof(args[1]));
    toolConfiguration.mutable_tool_transform()->set_z(std::stof(args[2]));
    toolConfiguration.mutable_tool_transform()->set_theta_x(std::stof(args[3]));
    toolConfiguration.mutable_tool_transform()->set_theta_y(std::stof(args[4]));
    toolConfiguration.mutable_tool_transform()->set_theta_z(std::stof(args[5]));
    toolConfiguration.set_tool_mass(std::stof(args[6]));
    toolConfiguration.mutable_tool_mass_center()->set_x(std::stof(args[7]));
    toolConfiguration.mutable_tool_mass_center()->set_y(std::stof(args[8]));
    toolConfiguration.mutable_tool_mass_center()->set_z(std::stof(args[9]));

    m_pKonverseClient->controlCfg->SetToolConfiguration(toolConfiguration);

    return true;
}

bool KaKonverseAPITest::SetToolConfigurationList(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::ToolConfigurationList toolConfigurationList;

    uint32_t argsIndex = 0;
    for(uint32_t toolId = 0; toolId < std::stoi(args[0]); toolId++)
    {
        auto toolConfiguration = toolConfigurationList.add_tool_configurations();
        toolConfiguration->mutable_tool_transform()->set_x(std::stof(args[argsIndex + 1]));
        toolConfiguration->mutable_tool_transform()->set_y(std::stof(args[argsIndex + 2]));
        toolConfiguration->mutable_tool_transform()->set_z(std::stof(args[argsIndex + 3]));
        toolConfiguration->mutable_tool_transform()->set_theta_x(std::stof(args[argsIndex + 4]));
        toolConfiguration->mutable_tool_transform()->set_theta_y(std::stof(args[argsIndex + 5]));
        toolConfiguration->mutable_tool_transform()->set_theta_z(std::stof(args[argsIndex + 6]));
        toolConfiguration->set_tool_mass(std::stof(args[argsIndex + 7]));
        toolConfiguration->mutable_tool_mass_center()->set_x(std::stof(args[argsIndex + 8]));
        toolConfiguration->mutable_tool_mass_center()->set_y(std::stof(args[argsIndex + 9]));
        toolConfiguration->mutable_tool_mass_center()->set_z(std::stof(args[argsIndex + 10]));
        argsIndex += 10;
    }

    m_pKonverseClient->controlCfg->SetToolConfigurationList(toolConfigurationList);

    return true;
}

bool KaKonverseAPITest::ResetToolConfiguration(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::ToolConfiguration toolConfiguration = m_pKonverseClient->controlCfg->ResetToolConfiguration();

    printf("%s\n", toolConfiguration.DebugString().c_str());

    return true;
}

bool KaKonverseAPITest::RestoreProductConfig(std::vector<std::string> args)
{
    m_pKonverseClient->baseCfg->RestoreFactoryProductConfiguration();

    return true;
}

uint32_t KaKonverseAPITest::GetArmState(std::vector<std::string> args)
{
    Kinova::Api::Base::ArmStateInformation activeState = m_pKonverseClient->baseCfg->GetArmState();

    printf("Active state is %d\n", activeState.active_state());
    return activeState.active_state();
}

uint32_t KaKonverseAPITest::GetUpdatingMode(std::vector<std::string> args)
{
    Kinova::Api::Base::UpdatingModeInformation updatingmode = m_pKonverseClient->baseCfg->GetUpdatingMode();

    printf("Updating mode is %d\n", updatingmode.updating_mode());
    return updatingmode.updating_mode();
}

uint32_t KaKonverseAPITest::GetCurrentOperatingMode(std::vector<std::string> args)
{
    Kinova::Api::Common::ModeSelection modeSelection = m_pKonverseClient->baseCfg->GetCurrentOperatingMode();

    printf("Current operating mode is %d\n", modeSelection.operating_mode());
    return modeSelection.operating_mode();
}

bool KaKonverseAPITest::SelectOperatingMode(std::vector<std::string> args)
{
    Kinova::Api::Common::ModeSelection mode;

    mode.set_operating_mode(static_cast<Kinova::Api::Common::OperatingModeType>(std::stoi(args[0])));
    m_pKonverseClient->baseCfg->SelectOperatingMode(mode);

    return true;
}

bool KaKonverseAPITest::GetTrajectoryReport(std::vector<std::string> args)
{
    Kinova::Api::Base::TrajectoryErrorReport errorReport = m_pKonverseClient->baseCfg->GetTrajectoryErrorReport();

    for(auto entry: errorReport.trajectory_error_elements())
    {
        printf("%s\n", entry.DebugString().c_str());
    }

    return true;
}

bool KaKonverseAPITest::ActivateRobot(std::vector<std::string> args)
{
    m_pKonverseClient->baseCfg->ActivateRobot();

    return true;
}

bool KaKonverseAPITest::DeactivateRobot(std::vector<std::string> args)
{
    m_pKonverseClient->baseCfg->DeactivateRobot();

    return true;
}

bool KaKonverseAPITest::ClearFaults(std::vector<std::string> args)
{
    m_pKonverseClient->baseCfg->ClearFaults();

    return true;
}

bool KaKonverseAPITest::ExitRecovery(std::vector<std::string> args)
{
    m_pKonverseClient->baseCfg->ExitRecoveryState();

    return true;
}

bool KaKonverseAPITest::ConfirmArmPosition(std::vector<std::string> args)
{
    m_pKonverseClient->baseCfg->ConfirmArmPosition();

    return true;
}

bool KaKonverseAPITest::SetToolSphere(std::vector<std::string> args)
{
    Kinova::Api::ProtectionZone::ToolSphere toolSphere;

    toolSphere.set_radius(std::stof(args[0]));
    toolSphere.mutable_center()->set_x(std::stof(args[1]));
    toolSphere.mutable_center()->set_y(std::stof(args[2]));
    toolSphere.mutable_center()->set_z(std::stof(args[3]));

    m_pKonverseClient->protectionZone->SetToolSphere(toolSphere);

    return true;
}

bool KaKonverseAPITest::GetToolSphere(std::vector<std::string> args)
{
    Kinova::Api::ProtectionZone::ToolSphere toolSphere = m_pKonverseClient->protectionZone->GetToolSphere();

    printf("%s\n", toolSphere.DebugString().c_str());

    return true;
}

const char *LimitTypeProtoToStr(Kinova::Api::DeviceConfig::DiagnosticLimitType limitType)
{
    switch(limitType)
    {
        case Kinova::Api::DeviceConfig::DiagnosticLimitType::DIAGNOSTIC_LIMIT_TYPE_MINIMAL:
            return "MIN";

        case Kinova::Api::DeviceConfig::DiagnosticLimitType::DIAGNOSTIC_LIMIT_TYPE_MAXIMAL:
            return "MAX";

        case Kinova::Api::DeviceConfig::DiagnosticLimitType::DIAGNOSTIC_LIMIT_TYPE_EVENT:
            return "EVT";

        default:
            return "UNK";
    }
}

const char *StatusValueProtoToStr(Kinova::Api::Common::DiagnosticStatusValue status)
{
    switch(status)
    {
        case Kinova::Api::Common::DiagnosticStatusValue::DIAGNOSTIC_STATUS_WARNING:
            return "WARN";

        case Kinova::Api::Common::DiagnosticStatusValue::DIAGNOSTIC_STATUS_ERROR:
            return "ERROR";

        case Kinova::Api::Common::DiagnosticStatusValue::DIAGNOSTIC_STATUS_NORMAL:
            return "NORM";

        default:
            return "UNK";
    }
}

bool KaKonverseAPITest::GetAllDiagnosticInformation(std::vector<std::string> args)
{
    const std::vector<Kinova::Api::Common::DiagnosticBank> banks = {Kinova::Api::Common::DIAGNOSTIC_BANK_A,
                                                                    Kinova::Api::Common::DIAGNOSTIC_BANK_B,
                                                                    Kinova::Api::Common::DIAGNOSTIC_BANK_C,
                                                                    Kinova::Api::Common::DIAGNOSTIC_BANK_D};
    for (const auto& bank: banks)
    {
        Kinova::Api::DeviceConfig::DiagnosticBank diagnosticBank;
        diagnosticBank.set_bank(bank);
        try
        {
            Kinova::Api::DeviceConfig::DiagnosticInformationList informationList = m_pKonverseClient->deviceCfg->GetAllDiagnosticInformation(diagnosticBank, std::stoi(args[0]));

            printf("Id       Chg  Warn  Err  Type  Def warn thrs  Def err thrs  Up limit  Low limit  Status\n");
            printf("---------------------------------------------------------------------------------------\n");
            for(auto diagnosticInfo: informationList.information())
            {
                printf("%8x %3s  %4s  %3s  %4s  %13.2f  %12.2f  %8.2f  %9.2f  %6s\n",
                    diagnosticInfo.handle().identifier(),
                    diagnosticInfo.can_change_diagnostic_state()? "Y" : "N",
                    diagnosticInfo.has_warning_threshold()? "Y" : "N",
                    diagnosticInfo.has_error_threshold()? "Y" : "N",
                    LimitTypeProtoToStr(diagnosticInfo.limit_type()),
                    diagnosticInfo.default_warning_threshold(),
                    diagnosticInfo.default_error_threshold(),
                    diagnosticInfo.upper_hard_limit(),
                    diagnosticInfo.lower_hard_limit(),
                    StatusValueProtoToStr(diagnosticInfo.status()));
            }
            printf("\n");
        }
        catch(...) {}
    }

    return true;
}

bool KaKonverseAPITest::GetAllDiagnosticConfiguration(std::vector<std::string> args)
{
    const std::vector<Kinova::Api::Common::DiagnosticBank> banks = {Kinova::Api::Common::DIAGNOSTIC_BANK_A,
                                                                    Kinova::Api::Common::DIAGNOSTIC_BANK_B,
                                                                    Kinova::Api::Common::DIAGNOSTIC_BANK_C,
                                                                    Kinova::Api::Common::DIAGNOSTIC_BANK_D};
    for (const auto& bank: banks)
    {
        Kinova::Api::DeviceConfig::DiagnosticBank diagnosticBank;
        diagnosticBank.set_bank(bank);
        try
        {
            Kinova::Api::DeviceConfig::DiagnosticConfigurationList configurationList = m_pKonverseClient->deviceCfg->GetAllDiagnosticConfiguration(diagnosticBank, std::stoi(args[0]));

            printf("Id        Warn thrs  Err thrs  State\n");
            printf("------------------------------------\n");
            for(auto diagnosticConfig: configurationList.configuration())
            {
                printf("%8x  %9.2f  %8.2f  %5s\n",
                        diagnosticConfig.handle().identifier(),
                        diagnosticConfig.warning_threshold(),
                        diagnosticConfig.error_threshold(),
                        diagnosticConfig.enable().enable()? "ENA" : "DIS");
            }
            printf("\n");
        }
        catch(...) {}
    }
    return true;
}

void KaKonverseAPITest::GetCartesianPose()
{
    Kinova::Api::Base::Pose pose = m_pKonverseClient->baseCfg->GetMeasuredCartesianPose();

    printf("===== Cartesian Pose:\n");
    printf("         x: %.2f\n", pose.x());
    printf("         y: %.2f\n", pose.y());
    printf("         z: %.2f\n", pose.z());
    printf("   theta x: %.2f\n", pose.theta_x());
    printf("   theta y: %.2f\n", pose.theta_y());
    printf("   theta z: %.2f\n", pose.theta_z());
}

void KaKonverseAPITest::GetJointAngles()
{
    Kinova::Api::Base::JointAngles jointAngles = m_pKonverseClient->baseCfg->GetMeasuredJointAngles();

    printf("===== Joint Angles:\n");

    for(auto angle: jointAngles.joint_angles())
    {
        printf("  joint id: %d, value: %.2f\n", angle.joint_identifier(), angle.value());
    }
}

void KaKonverseAPITest::Reboot()
{
    m_pKonverseClient->baseCfg->Reboot();
}

bool KaKonverseAPITest::GetFeedback()
{
    Kinova::Api::BaseCyclic::Feedback feedback = m_pKonverseClient->baseCyclicRT->RefreshFeedback();

    printf("\tnActiveState:                   %" PRIu32 "\n", feedback.base().active_state());
    printf("\tfArmCurrent_amp:                %.2f\n",        feedback.base().arm_current());
    printf("\tfArmVoltage_v:                  %.2f\n",        feedback.base().arm_voltage());
    printf("\tfTemperatureAmbient_celsius:    %.2f\n",        feedback.base().temperature_ambient());
    printf("\tfTemperatureCPU_celsius:        %.2f\n",        feedback.base().temperature_cpu());
    printf("\tfIMUAccelerationX_ms2:          %.2f\n",        feedback.base().imu_acceleration_x());
    printf("\tfIMUAccelerationY_ms2:          %.2f\n",        feedback.base().imu_acceleration_y());
    printf("\tfIMUAccelerationZ_ms2:          %.2f\n",        feedback.base().imu_acceleration_z());
    printf("\tfIMUAngularVelocityX_ms2:       %.2f\n",        feedback.base().imu_angular_velocity_x());
    printf("\tfIMUAngularVelocityY_ms2:       %.2f\n",        feedback.base().imu_angular_velocity_y());
    printf("\tfIMUAngularVelocityZ_ms2:       %.2f\n",        feedback.base().imu_angular_velocity_z());
    printf("\tfToolTwistLinearX_ms:           %.2f\n",        feedback.base().tool_twist_linear_x());
    printf("\tfToolTwistLinearY_ms:           %.2f\n",        feedback.base().tool_twist_linear_y());
    printf("\tfToolTwistLinearZ_ms:           %.2f\n",        feedback.base().tool_twist_linear_z());
    printf("\tfToolTwistAngularX_ms:          %.2f\n",        feedback.base().tool_twist_angular_x());
    printf("\tfToolTwistAngularY_ms:          %.2f\n",        feedback.base().tool_twist_angular_y());
    printf("\tfToolTwistAngularZ_ms:          %.2f\n",        feedback.base().tool_twist_angular_z());
    printf("\tfToolPoseX_m:                   %.2f\n",        feedback.base().tool_pose_x());
    printf("\tfToolPoseY_m:                   %.2f\n",        feedback.base().tool_pose_y());
    printf("\tfToolPoseZ_m:                   %.2f\n",        feedback.base().tool_pose_z());
    printf("\tfToolPoseThetaX_deg:            %.2f\n",        feedback.base().tool_pose_theta_x());
    printf("\tfToolPoseThetaY_deg:            %.2f\n",        feedback.base().tool_pose_theta_y());
    printf("\tfToolPoseThetaZ_deg:            %.2f\n",        feedback.base().tool_pose_theta_z());
    printf("\tfToolExternalWrenchForceX_n:    %.2f\n",        feedback.base().tool_external_wrench_force_x());
    printf("\tfToolExternalWrenchForceY_n:    %.2f\n",        feedback.base().tool_external_wrench_force_y());
    printf("\tfToolExternalWrenchForceZ_n:    %.2f\n",        feedback.base().tool_external_wrench_force_z());
    printf("\tfToolExternalWrenchTorqueX_nm:  %.2f\n",        feedback.base().tool_external_wrench_torque_x());
    printf("\tfToolExternalWrenchTorqueY_nm:  %.2f\n",        feedback.base().tool_external_wrench_torque_y());
    printf("\tfToolExternalWrenchTorqueZ_nm:  %.2f\n",        feedback.base().tool_external_wrench_torque_z());
    printf("\tnFaultBankA:                    %" PRIu32 "\n", feedback.base().fault_bank_a());
    printf("\tnFaultBankB:                    %" PRIu32 "\n", feedback.base().fault_bank_b());
    printf("\tnWarningBankA:                  %" PRIu32 "\n", feedback.base().warning_bank_a());
    printf("\tnWarningBankB:                  %" PRIu32 "\n", feedback.base().warning_bank_b());
    printf("\tnActivePayload_kg:              %.2f\n",        feedback.base().active_payload());
    printf("\tnStateBank:                     %" PRIu32 "\n", feedback.base().state_bank());

    uint32_t nActuatorIdx = 0;
    for (auto actuator: feedback.actuators())
    {
        printf("\tActuator Index %" PRIu32 "\n", nActuatorIdx);
        printf("\t\tnStatusFlags:              %" PRIu32 "\n", actuator.status_flags());
        printf("\t\tnJitterComm_us:            %" PRIu32 "\n", actuator.jitter_comm());
        printf("\t\tfPosition_deg:             %.2f\n",          actuator.position());
        printf("\t\tfVelocity_degpsec:         %.2f\n",          actuator.velocity());
        printf("\t\tfTorque_nm:                %.2f\n",          actuator.torque());
        printf("\t\tfCurrentMotor_amp:         %.2f\n",          actuator.current_motor());
        printf("\t\tfVoltage_v:                %.2f\n",          actuator.voltage());
        printf("\t\tfTemperatureMotor_celsius: %.2f\n",          actuator.temperature_motor());
        printf("\t\tfTemperatureMCU_celsius:   %.2f\n",          actuator.temperature_core());
        printf("\t\tnFaultBankA:               %" PRIu32 "\n", actuator.fault_bank_a());
        printf("\t\tnFaultBankB:               %" PRIu32 "\n", actuator.fault_bank_b());
        printf("\t\tnWarningBankA:             %" PRIu32 "\n", actuator.warning_bank_a());
        printf("\t\tnWarningBankB:             %" PRIu32 "\n", actuator.warning_bank_b());
        nActuatorIdx++;
    }

    return true;
}

bool KaKonverseAPITest::GetCustomData()
{
    Kinova::Api::BaseCyclic::FieldCustomType customCommand;

    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_JOINT_FEEDBACK_POSITION);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_JOINT_FEEDBACK_SPEED);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_JOINT_FEEDBACK_ACCELERATION);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_JOINT_FEEDBACK_CURRENT);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_JOINT_FEEDBACK_TORQUE);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_JOINT_FEEDBACK_TIME);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_JOINT_FEEDBACK_SEGMENT);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_CARTESIAN_FEEDBACK_POSE);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_CARTESIAN_FEEDBACK_VELOCITY);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_CARTESIAN_FEEDBACK_WRENCH);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_CARTESIAN_FEEDBACK_ELBOWVELOCITY);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_JOINT_COMMAND_POSITION);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_JOINT_COMMAND_SPEED);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_JOINT_COMMAND_ACCELERATION);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_JOINT_COMMAND_CURRENT);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_JOINT_COMMAND_TORQUE);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_JOINT_COMMAND_TIME);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_JOINT_COMMAND_SEGMENT);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_CARTESIAN_COMMAND_POSE);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_CARTESIAN_COMMAND_VELOCITY);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_CARTESIAN_COMMAND_WRENCH);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_INPUT_CARTESIAN_COMMAND_ELBOWVELOCITY);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_OUTPUT_JOINT_COMMAND_POSITION);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_OUTPUT_JOINT_COMMAND_SPEED);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_OUTPUT_JOINT_COMMAND_ACCELERATION);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_OUTPUT_JOINT_COMMAND_CURRENT);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_OUTPUT_JOINT_COMMAND_TORQUE);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_OUTPUT_JOINT_COMMAND_TIME);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_OUTPUT_JOINT_COMMAND_SEGMENT);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_OUTPUT_CARTESIAN_INFORMATION_POSE);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_OUTPUT_CARTESIAN_INFORMATION_VELOCITY);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_OUTPUT_CARTESIAN_INFORMATION_WRENCH);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_OUTPUT_CARTESIAN_INFORMATION_ELBOWVELOCITY);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_OUTPUT_CARTESIAN_INFORMATION_COMMANDED_POSE);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_OUTPUT_CARTESIAN_INFORMATION_COMMANDED_VELOCITY);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_OUTPUT_CARTESIAN_INFORMATION_COMMANDED_WRENCH);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_OUTPUT_CARTESIAN_INFORMATION_COMMANDED_ELBOWVELOCITY);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_OUTPUT_VECTOR1);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_OUTPUT_VECTOR2);
    customCommand.add_command_list(Kinova::Api::BaseCyclic::CustomType::CUSTOM_KONTROL_OUTPUT_STATUS);

    Kinova::Api::BaseCyclic::CustomData customData = m_pKonverseClient->baseCyclicRT->RefreshCustomData(customCommand);

    for (Kinova::Api::BaseCyclic::RepeatedFieldCustomData custom_data : customData.custom_datas())
    {
        if (custom_data.data_size() > 0)
        {
            switch (custom_data.data(0).values_case())
            {
                case Kinova::Api::BaseCyclic::FieldCustomData::ValuesCase::kFloat:
                    for (Kinova::Api::BaseCyclic::FieldCustomData data: custom_data.data())
                    {
                        printf("%11.6f ", data.float_());
                    }
                break;

                case Kinova::Api::BaseCyclic::FieldCustomData::ValuesCase::kBool:
                    for (Kinova::Api::BaseCyclic::FieldCustomData data: custom_data.data())
                    {
                        printf("%i ", data.bool_());
                    }
                break;

                case Kinova::Api::BaseCyclic::FieldCustomData::ValuesCase::kFix32:
                    for (Kinova::Api::BaseCyclic::FieldCustomData data: custom_data.data())
                    {
                        printf("%i ", data.fix32());
                    }
                break;

                case Kinova::Api::BaseCyclic::FieldCustomData::ValuesCase::kInt32:
                    for (Kinova::Api::BaseCyclic::FieldCustomData data: custom_data.data())
                    {
                        printf("%i ", data.int32());
                    }
                break;

                case Kinova::Api::BaseCyclic::FieldCustomData::ValuesCase::kUint32:
                    for (Kinova::Api::BaseCyclic::FieldCustomData data: custom_data.data())
                    {
                        printf("%u ", data.uint32());
                    }
                break;
            }
        }
        printf("\n");
    }

    uint32_t nActuatorIdx = 0;
    for (auto actuator: customData.actuators_custom_data())
    {
        printf("\tActuator Index %" PRIu32 "\n", nActuatorIdx);
        printf("\t\tnCustomData0:              %" PRIu32 "\n", actuator.custom_data_0());
        printf("\t\tnCustomData1:              %" PRIu32 "\n", actuator.custom_data_1());
        printf("\t\tnCustomData2:              %" PRIu32 "\n", actuator.custom_data_2());
        printf("\t\tnCustomData3:              %" PRIu32 "\n", actuator.custom_data_3());
        printf("\t\tnCustomData4:              %" PRIu32 "\n", actuator.custom_data_4());
        printf("\t\tnCustomData5:              %" PRIu32 "\n", actuator.custom_data_5());
        printf("\t\tnCustomData6:              %" PRIu32 "\n", actuator.custom_data_6());
        printf("\t\tnCustomData7:              %" PRIu32 "\n", actuator.custom_data_7());
        printf("\t\tnCustomData8:              %" PRIu32 "\n", actuator.custom_data_8());
        printf("\t\tnCustomData9:              %" PRIu32 "\n", actuator.custom_data_9());
        printf("\t\tnCustomData10:             %" PRIu32 "\n", actuator.custom_data_10());
        printf("\t\tnCustomData11:             %" PRIu32 "\n", actuator.custom_data_11());
        printf("\t\tnCustomData12:             %" PRIu32 "\n", actuator.custom_data_12());
        printf("\t\tnCustomData13:             %" PRIu32 "\n", actuator.custom_data_13());
        printf("\t\tnCustomData14:             %" PRIu32 "\n", actuator.custom_data_14());
        printf("\t\tnCustomData15:             %" PRIu32 "\n", actuator.custom_data_15());
        nActuatorIdx++;
    }

    return true;
}

uint32_t KaKonverseAPITest::SubConfigChangeNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->baseCfg->OnNotificationConfigurationChangeTopic(processConfigChange, notifOptions);
    printf("OnNotificationChangedConfig -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

uint32_t KaKonverseAPITest::SubProgramChangeNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->programCfg->OnNotificationConfigurationChangeTopic(processProgramConfigChange, notifOptions);
    printf("OnNotificationProgramConfig -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

uint32_t KaKonverseAPITest::SubProtectionZoneChangeNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->protectionZone->OnNotificationProtectionZoneChangeTopic(processProtectionZoneChange, notifOptions);
    printf("OnNotificationProtectionZone -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

/**
 * @brief Subscribes to Arm State notification
 *
 * @return uint32_t Notification identifier
 */
uint32_t KaKonverseAPITest::SubArmStateNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->baseCfg->OnNotificationArmStateTopic(processArmState, notifOptions);
    printf("OnNotificationArmStateTopic -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}


/**
 * @brief Subscribes to Arm Speed Factor notification
 *
 * @return uint32_t Notification identifier
 */
uint32_t KaKonverseAPITest::SubArmSpeedFactorNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->controlCfg->OnNotificationArmSpeedFactorTopic(processArmSpeedFactor, notifOptions);
    printf("OnNotificationArmSpeedFactorTopic -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}
uint32_t KaKonverseAPITest::SubToolSphereChangeeNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->protectionZone->OnNotificationToolSphereChangeTopic(processToolSphereChange, notifOptions);
    printf("OnToolSphereChange -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

uint32_t KaKonverseAPITest::SubUpdatingModeNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->baseCfg->OnNotificationUpdatingModeTopic(processUpdatingMode, notifOptions);
    printf("OnNotificationUpdatingMode -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

uint32_t KaKonverseAPITest::SubControlConfigNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->controlCfg->OnNotificationControlConfigurationTopic(processControlConfig, notifOptions);
    printf("OnNotificationControlConfig -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

uint32_t KaKonverseAPITest::SubRobotNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->baseCfg->OnNotificationRobotEventTopic(processRobot, notifOptions);
    printf("OnNotificationRobotEvent -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

uint32_t KaKonverseAPITest::SubControlModeNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->controlCfg->OnNotificationControlModeTopic(processControlMode, notifOptions);
    printf("OnNotificationControlMode -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

uint32_t KaKonverseAPITest::SubServoingModeNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->baseCfg->OnNotificationServoingModeTopic(processServoingMode, notifOptions);
    printf("OnNotificationServoingMode -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

uint32_t KaKonverseAPITest::SubControllerNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->baseCfg->OnNotificationControllerTopic(processController, notifOptions);
    printf("OnNotificationController -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

uint32_t KaKonverseAPITest::SubActionNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->baseCfg->OnNotificationActionTopic(processAction, notifOptions);
    printf("OnNotificationAction -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

void KaKonverseAPITest::UnsubNotif(uint32_t id)
{
    Kinova::Api::Common::NotificationHandle notificationHandle;
    notificationHandle.set_identifier(id);
    m_pKonverseClient->baseCfg->Unsubscribe(notificationHandle);

    printf("Unsubscribed notifId = %d \n", notificationHandle.identifier() );
}

uint32_t KaKonverseAPITest::SubDiagnosticNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->deviceCfg->OnNotificationDiagnosticTopic(processDiagnostic, notifOptions);
    printf("OnDiagnosticConfig -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

uint32_t KaKonverseAPITest::SubSafetyFunctionsChangeNotif()
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::Common::NotificationOptions notifOptions;
        Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->safetyFunctions->OnNotificationSafetyFunctionChangeTopic(processSafetyFunctions, notifOptions);
        printf("OnSafetyFunctionsChange -> notifId = %d \n", notifInfo.identifier() );
        return notifInfo.identifier();
    }
    else
    {
        printf("Safety functions not supported on this product\n");
        return 0;
    }
}

uint32_t KaKonverseAPITest::SubSafetyIOChangeNotif()
{
    if (CapabilityManager::hasSafetyIo())
    {
        Kinova::Api::Common::NotificationOptions notifOptions;
        Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->safetyIO->OnNotificationSafetyIOChangeTopic(processSafetyIOChange, notifOptions);
        printf("OnSafetyIOChange -> notifId = %d \n", notifInfo.identifier() );
        return notifInfo.identifier();
    }
    else
    {
        printf("Safety io not supported on this product\n");
        return 0;
    }
}

uint32_t KaKonverseAPITest::SubHandGuidingModeNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->baseCfg->OnNotificationHandGuidingModeTopic(processHandGuidingModeChange, notifOptions);
    printf("OnHandGuidingMode -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

uint32_t KaKonverseAPITest::SubEnablingDeviceNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->baseCfg->OnNotificationEnablingDeviceTopic(processEnablingDeviceChange, notifOptions);
    printf("OnEnablingDeviceChange -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

uint32_t KaKonverseAPITest::SubOperatingModeNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->baseCfg->OnNotificationOperatingModeTopic(processOperatingModeChange, notifOptions);
    printf("OnOperatingModeChange -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

uint32_t KaKonverseAPITest::SubMotionNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->baseCfg->OnNotificationMotionTopic(processMotionEvent, notifOptions);
    printf("OnMotionNotificationEvent -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

uint32_t KaKonverseAPITest::SubSafetyModeChangeNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->safetyFunctions->OnNotificationSafetyModeChangeTopic(processSafetyModeEvent, notifOptions);
    printf("OnNotificationSafetyModeChangeTopic -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

uint32_t KaKonverseAPITest::SubProgramRequestNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->baseCfg->OnNotificationProgramRequestTopic(processProgramRequestEvent, notifOptions);
    printf("OnNotificationProgramRequestTopic -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

uint32_t KaKonverseAPITest::SubProtectiveStopChangeNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->safetyFunctions->OnNotificationProtectiveStopChangeTopic(processProtectiveStopChangeEvent, notifOptions);
    printf("OnNotificationProtectiveStopChangeTopic -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

/**
 * @brief Subscribes to safety parameters checksun change notification
 *
 * @return uint32_t Notification identifier
 */
uint32_t KaKonverseAPITest::SubSafetyParametersChecksumChangeNotif()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->safetyControlUnitCfg->OnNotificationSafetyParametersChecksumChangeTopic(processSafetyParametersChecksumChangeEvent, notifOptions);
    printf("OnNotificationSafetyParametersChecksumChangeTopic -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

/**
 * @brief Subscribes to Remote Access change notification
 *
 * @return uint32_t Notification identifier
 */
uint32_t KaKonverseAPITest::SubRemoteAccessChangeTopic()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->baseCfg->OnNotificationRemoteAccessChangeTopic(processRemoteAccessChangeEvent, notifOptions);
    printf("OnNotificationRemoteAccessChangeTopic -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

/**
 * @brief Subscribes to Acknowledge action notification
 *
 * @return uint32_t Notification identifier
 */
uint32_t KaKonverseAPITest::SubAcknowledgeActionTopic()
{
    Kinova::Api::Common::NotificationOptions notifOptions;
    Kinova::Api::Common::NotificationHandle notifInfo = m_pKonverseClient->baseCfg->OnNotificationAcknowledgeActionTopic(processAcknowledgeActionEvent, notifOptions);
    printf("OnNotificationAcknowledgeActionTopic -> notifId = %d \n", notifInfo.identifier() );

    return notifInfo.identifier();
}

bool KaKonverseAPITest::IsSafetyFunctionSupported(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::SafetyFunction safetyFunction;
        safetyFunction.set_safety_function_type(static_cast<Kinova::Api::SafetyFunctions::SafetyFunctionType>(std::stoi(args[0])));

        m_pKonverseClient->safetyFunctions->IsSupported(safetyFunction);
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }
    return true;
}

bool KaKonverseAPITest::SetSafetyJointPositionLimits(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::JointPositionLimits jointPositionLimits;
        jointPositionLimits.mutable_info()->set_index(std::stoi(args[0]));
        jointPositionLimits.set_lower_limit(std::stof(args[1]));
        jointPositionLimits.set_upper_limit(std::stof(args[2]));

        m_pKonverseClient->safetyFunctions->SetJointPositionLimits(jointPositionLimits);
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }
    return true;
}

bool KaKonverseAPITest::SetAllSafetyJointPositionLimits(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::JointPositionLimitsList jointPositionLimitsList;
        uint32_t argsIndex = 0;
        for(uint32_t jointId = 0; jointId < std::stoi(args[0]); jointId++)
        {
            auto jointPositionLimits = jointPositionLimitsList.add_joints_position_limits();
            jointPositionLimits->mutable_info()->set_index(jointId);
            jointPositionLimits->set_lower_limit(std::stof(args[argsIndex + 1]));
            jointPositionLimits->set_upper_limit(std::stof(args[argsIndex + 2]));
            argsIndex += 2;
        }

        m_pKonverseClient->safetyFunctions->SetAllJointPositionLimits(jointPositionLimitsList);
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }
    return true;
}

bool KaKonverseAPITest::GetSafetyJointPositionLimits(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::JointPositionLimits jointPositionLimits;

        Kinova::Api::SafetyFunctions::JointPositionInfo jointPositionInfo;
        jointPositionInfo.set_index(std::stoi(args[0]));

        jointPositionLimits = m_pKonverseClient->safetyFunctions->GetJointPositionLimits(jointPositionInfo);
        printf("%s\n", jointPositionLimits.DebugString().c_str());
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }

    return true;
}

bool KaKonverseAPITest::GetAllSafetyJointPositionLimits(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::JointPositionLimitsList jointPositionLimitsList;
        jointPositionLimitsList = m_pKonverseClient->safetyFunctions->GetAllJointPositionLimits();
        printf("%s\n", jointPositionLimitsList.DebugString().c_str());
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }

    return true;
}

bool KaKonverseAPITest::SetSafetyJointSpeedLimit(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::JointSpeedLimit jointSpeedLimit;
        jointSpeedLimit.mutable_info()->set_index(std::stoi(args[1]));
        jointSpeedLimit.mutable_info()->set_mode(static_cast<Kinova::Api::SafetyFunctions::SafetySystemMode>(std::stoi(args[0])));
        jointSpeedLimit.set_limit(std::stof(args[2]));

        m_pKonverseClient->safetyFunctions->SetJointSpeedLimit(jointSpeedLimit);
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }
    return true;
}

bool KaKonverseAPITest::SetAllSafetyJointSpeedLimit(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::JointSpeedLimitList jointSpeedLimitList;
        uint32_t argsIndex = 0;
        for(uint32_t jointId = 0; jointId < std::stoi(args[1]); jointId++)
        {
            auto jointSpeedLimit = jointSpeedLimitList.add_joints_speed_limit();
            jointSpeedLimit->mutable_info()->set_index(jointId);
            jointSpeedLimit->mutable_info()->set_mode(static_cast<Kinova::Api::SafetyFunctions::SafetySystemMode>(std::stoi(args[0])));
            jointSpeedLimit->set_limit(std::stof(args[argsIndex + 2]));
            argsIndex++;
        }

        m_pKonverseClient->safetyFunctions->SetAllJointSpeedLimit(jointSpeedLimitList);
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }
    return true;
}

bool KaKonverseAPITest::GetSafetyJointSpeedLimit(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::JointSpeedLimit jointSpeedLimit;

        Kinova::Api::SafetyFunctions::JointSpeedInfo jointSpeedInfo;
        jointSpeedInfo.set_index(std::stoi(args[1]));
        jointSpeedInfo.set_mode(static_cast<Kinova::Api::SafetyFunctions::SafetySystemMode>(std::stoi(args[0])));

        jointSpeedLimit = m_pKonverseClient->safetyFunctions->GetJointSpeedLimit(jointSpeedInfo);
        printf("%s\n", jointSpeedLimit.DebugString().c_str());
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }

    return true;
}

bool KaKonverseAPITest::GetAllSafetyJointSpeedLimit(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::JointSpeedLimitList jointSpeedLimitList;
        jointSpeedLimitList = m_pKonverseClient->safetyFunctions->GetAllJointSpeedLimit();
        printf("%s\n", jointSpeedLimitList.DebugString().c_str());
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }

    return true;
}

bool KaKonverseAPITest::SetSafetyTcpSpeedLimits(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::TcpSpeedLimits tcpSpeedLimits;
        tcpSpeedLimits.mutable_info()->set_mode(static_cast<Kinova::Api::SafetyFunctions::SafetySystemMode>(std::stoi(args[0])));
        tcpSpeedLimits.set_translation_limit(std::stof(args[1]));
        tcpSpeedLimits.set_orientation_limit(std::stof(args[2]));

        m_pKonverseClient->safetyFunctions->SetTcpSpeedLimits(tcpSpeedLimits);
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }
    return true;
}

bool KaKonverseAPITest::SetAllSafetyTcpSpeedLimits(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::TcpSpeedLimitsList tcpSpeedLimitsList;

        auto tcpSpeedLimits = tcpSpeedLimitsList.add_tcp_speed_limits();
        tcpSpeedLimits->mutable_info()->set_mode(Kinova::Api::SafetyFunctions::SafetySystemMode::SAFETY_SYSTEM_MODE_NORMAL);
        tcpSpeedLimits->set_translation_limit(std::stof(args[0]));
        tcpSpeedLimits->set_orientation_limit(std::stof(args[1]));

        tcpSpeedLimits = tcpSpeedLimitsList.add_tcp_speed_limits();
        tcpSpeedLimits->mutable_info()->set_mode(Kinova::Api::SafetyFunctions::SafetySystemMode::SAFETY_SYSTEM_MODE_REDUCED);
        tcpSpeedLimits->set_translation_limit(std::stof(args[2]));
        tcpSpeedLimits->set_orientation_limit(std::stof(args[3]));


        m_pKonverseClient->safetyFunctions->SetAllTcpSpeedLimits(tcpSpeedLimitsList);
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }
    return true;
}

bool KaKonverseAPITest::GetSafetyTcpSpeedLimits(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::TcpSpeedLimits tcpSpeedLimits;

        Kinova::Api::SafetyFunctions::TcpInfo tcpInfo;
        tcpInfo.set_mode(static_cast<Kinova::Api::SafetyFunctions::SafetySystemMode>(std::stoi(args[0])));

        tcpSpeedLimits = m_pKonverseClient->safetyFunctions->GetTcpSpeedLimits(tcpInfo);
        printf("%s\n", tcpSpeedLimits.DebugString().c_str());
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }

    return true;
}

bool KaKonverseAPITest::GetAllSafetyTcpSpeedLimits(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::TcpSpeedLimitsList tcpSpeedLimitsList;
        tcpSpeedLimitsList = m_pKonverseClient->safetyFunctions->GetAllTcpSpeedLimits();
        printf("%s\n", tcpSpeedLimitsList.DebugString().c_str());
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }

    return true;
}

bool KaKonverseAPITest::SetSafetyElbowSpeedLimit(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::ElbowSpeedLimit elbowSpeedLimit;
        elbowSpeedLimit.mutable_info()->set_mode(static_cast<Kinova::Api::SafetyFunctions::SafetySystemMode>(std::stoi(args[0])));
        elbowSpeedLimit.set_translation_limit(std::stof(args[1]));

        m_pKonverseClient->safetyFunctions->SetElbowSpeedLimit(elbowSpeedLimit);
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }
    return true;
}

bool KaKonverseAPITest::SetAllSafetyElbowSpeedLimit(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::ElbowSpeedLimitList elbowSpeedLimitList;

        auto elbowSpeedLimit = elbowSpeedLimitList.add_elbow_speed_limit();
        elbowSpeedLimit->mutable_info()->set_mode(Kinova::Api::SafetyFunctions::SafetySystemMode::SAFETY_SYSTEM_MODE_NORMAL);
        elbowSpeedLimit->set_translation_limit(std::stof(args[0]));

        elbowSpeedLimit = elbowSpeedLimitList.add_elbow_speed_limit();
        elbowSpeedLimit->mutable_info()->set_mode(Kinova::Api::SafetyFunctions::SafetySystemMode::SAFETY_SYSTEM_MODE_REDUCED);
        elbowSpeedLimit->set_translation_limit(std::stof(args[1]));


        m_pKonverseClient->safetyFunctions->SetAllElbowSpeedLimit(elbowSpeedLimitList);
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }
    return true;
}

bool KaKonverseAPITest::GetSafetyElbowSpeedLimit(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::ElbowSpeedLimit elbowSpeedLimit;

        Kinova::Api::SafetyFunctions::ElbowInfo elbowInfo;
        elbowInfo.set_mode(static_cast<Kinova::Api::SafetyFunctions::SafetySystemMode>(std::stoi(args[0])));

        elbowSpeedLimit = m_pKonverseClient->safetyFunctions->GetElbowSpeedLimit(elbowInfo);
        printf("%s\n", elbowSpeedLimit.DebugString().c_str());
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }

    return true;
}

bool KaKonverseAPITest::GetAllSafetyElbowSpeedLimit(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::ElbowSpeedLimitList elbowSpeedLimitList;
        elbowSpeedLimitList = m_pKonverseClient->safetyFunctions->GetAllElbowSpeedLimit();
        printf("%s\n", elbowSpeedLimitList.DebugString().c_str());
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }

    return true;
}

bool KaKonverseAPITest::GetSafetyFunctionsLimitsRange(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::SafetyFunctionsLimitsRange safetyFunctionsLimitsRange;
        safetyFunctionsLimitsRange = m_pKonverseClient->safetyFunctions->GetSafetyFunctionsLimitsRange();
        printf("%s\n", safetyFunctionsLimitsRange.DebugString().c_str());
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }

    return true;
}

bool KaKonverseAPITest::GetSafetyFunctionsLimits(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::SafetyFunctionsLimits safetyFunctionsLimits;
        safetyFunctionsLimits = m_pKonverseClient->safetyFunctions->GetSafetyFunctionsLimits();
        printf("%s\n", safetyFunctionsLimits.DebugString().c_str());
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }

    return true;
}

/**
* @brief Allows command to get safety function status, related joint and value from SCU.
* @param[in] args Every arguments related to the testclient tool command.
* @return True value
*/
bool KaKonverseAPITest::GetSafetyFunctionsStatus(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::SafetyFunctionsStatus safetyFunctionsStatus;
        safetyFunctionsStatus = m_pKonverseClient->safetyFunctions->GetSafetyFunctionsStatus();

        uint32_t safety_function_status = safetyFunctionsStatus.safety_functions_status();
        printf("safety_function_status: %u (0x%08x)\n", safety_function_status, safety_function_status);

        for (int index = 0; index < safetyFunctionsStatus.safety_function_triggered_details_size(); index++)
        {
            const auto& tmp = safetyFunctionsStatus.safety_function_triggered_details(index);
            switch (tmp.safety_function_details_case())
            {
                case Kinova::Api::SafetyFunctions::TriggeredSafetyFunctionDetails::kJointPositionDetails:
                {
                    printf("joint_index: %u\n", tmp.joint_position_details().joint_id());
                    printf("joint_value: %f\n", tmp.joint_position_details().joint_position());
                    break;
                }
                case Kinova::Api::SafetyFunctions::TriggeredSafetyFunctionDetails::kJointSpeedDetails:
                {
                    printf("joint_index: %u\n", tmp.joint_speed_details().joint_id());
                    printf("joint_value: %f\n", tmp.joint_speed_details().joint_speed());
                    break;
                }
                default:
                {
                    printf("Error: Unable to retrieve safety function triggered details");
                }
            }
        }
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }

    return true;
}

bool KaKonverseAPITest::GetSafetySystemMode(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::SafetySystem safetySystem;
        safetySystem = m_pKonverseClient->safetyFunctions->GetSafetySystemMode();
        printf("%s\n", safetySystem.DebugString().c_str());
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }

    return true;
}

bool KaKonverseAPITest::GetProtectiveStopStatus(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::ProtectiveStopStatus protectiveStopStatus;
        protectiveStopStatus = m_pKonverseClient->safetyFunctions->GetProtectiveStopStatus();
        printf("%s\n", protectiveStopStatus.DebugString().c_str());
    }
    else
    {
        printf("Safety functions not supported on this product\n");
    }

    return true;
}

bool KaKonverseAPITest::SetSafetyIOConfiguration(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyIo())
    {
        Kinova::Api::SafetyIO::SafetyIOConfiguration ioConfig;
        switch (static_cast<Kinova::Api::SafetyIO::SafetyIOType>(std::stoi(args[1])))
        {
            case Kinova::Api::SafetyIO::SAFETY_IO_TYPE_INPUT:
                ioConfig.mutable_input_configuration()->set_channel(std::stoi(args[0]));
                ioConfig.mutable_input_configuration()->set_input_function(
                    static_cast<Kinova::Api::SafetyIO::SafetyInputFunction>(std::stoi(args[2]))
                );
                break;
            case Kinova::Api::SafetyIO::SAFETY_IO_TYPE_OUTPUT:
                ioConfig.mutable_output_configuration()->set_channel(std::stoi(args[0]));
                ioConfig.mutable_output_configuration()->set_output_event(
                    static_cast<Kinova::Api::SafetyIO::SafetyOutputEvent>(std::stoi(args[2]))
                );
                break;
            default:
                break;

        }

        m_pKonverseClient->safetyIO->SetSafetyIOConfiguration(ioConfig);
    }
    else
    {
        printf("Safety io not supported on this product\n");
    }
    return true;
}

bool KaKonverseAPITest::SetAllSafetyIOConfiguration(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyIo())
    {
        Kinova::Api::SafetyIO::SafetyIOConfigurationList ioConfigList;
        uint32_t argsIndex = 0;
        for(uint32_t ioCount = 0; ioCount < std::stoi(args[0]); ioCount++)
        {
            auto ioConfig = ioConfigList.add_safety_io_configs();
            switch (static_cast<Kinova::Api::SafetyIO::SafetyIOType>(std::stoi(args[argsIndex + 2])))
            {
                case Kinova::Api::SafetyIO::SAFETY_IO_TYPE_INPUT:
                    ioConfig->mutable_input_configuration()->set_channel(std::stoi(args[argsIndex + 1]));
                    ioConfig->mutable_input_configuration()->set_input_function(
                        static_cast<Kinova::Api::SafetyIO::SafetyInputFunction>(std::stoi(args[argsIndex + 3]))
                    );
                    break;
                case Kinova::Api::SafetyIO::SAFETY_IO_TYPE_OUTPUT:
                    ioConfig->mutable_output_configuration()->set_channel(std::stoi(args[argsIndex + 1]));
                    ioConfig->mutable_output_configuration()->set_output_event(
                        static_cast<Kinova::Api::SafetyIO::SafetyOutputEvent>(std::stoi(args[argsIndex + 3]))
                    );
                    break;
                default:
                    break;

            }
            argsIndex += 3;
        }

        m_pKonverseClient->safetyIO->SetAllSafetyIOConfiguration(ioConfigList);
    }
    else
    {
        printf("Safety io not supported on this product\n");
    }

    return true;
}

bool KaKonverseAPITest::GetSafetyIOConfiguration(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyIo())
    {
        Kinova::Api::SafetyIO::SafetyIOConfiguration safetyIOConfig;

        Kinova::Api::SafetyIO::SafetyIOInfo safetyIOInfo;
        safetyIOInfo.set_channel(std::stoi(args[0]));
        safetyIOInfo.set_io_type(static_cast<Kinova::Api::SafetyIO::SafetyIOType>(std::stoi(args[1])));

        safetyIOConfig = m_pKonverseClient->safetyIO->GetSafetyIOConfiguration(safetyIOInfo);
        printf("%s\n", safetyIOConfig.DebugString().c_str());
    }
    else
    {
        printf("Safety io not supported on this product\n");
    }

    return true;
}

bool KaKonverseAPITest::GetAllSafetyIOConfiguration(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyIo())
    {
        Kinova::Api::SafetyIO::SafetyIOConfigurationList safetyIOConfigList;

        safetyIOConfigList = m_pKonverseClient->safetyIO->GetAllSafetyIOConfiguration();
        printf("%s\n", safetyIOConfigList.DebugString().c_str());
    }
    else
    {
        printf("Safety io not supported on this product\n");
    }

    return true;
}

bool KaKonverseAPITest::GetSafetyIOStatus(std::vector<std::string> args)
{
    if (CapabilityManager::hasSafetyIo())
    {
        Kinova::Api::SafetyIO::SafetyIOChannelStatus safetyIOStatus;
        safetyIOStatus = m_pKonverseClient->safetyIO->GetSafetyIOStatus();
        printf("%s\n", safetyIOStatus.DebugString().c_str());
    }
    else
    {
        printf("Safety io not supported on this product\n");
    }

    return true;
}

/**
 * @brief Gets the collision detection ranges for the limit's configuration
 * @param[in] args Every arguments related to the testclient tool command (none in this case).
 * @return True
 */
bool KaKonverseAPITest::GetCollisionDetectionRangesList(std::vector<std::string> args)
{
    auto rangesList = m_pKonverseClient->controlCfg->GetCollisionDetectionRangesList();

    for(uint16_t index = 0U; index < rangesList.ranges_size(); ++index)
    {
        printf("Collision detection ranges for the specific Safety System Mode : \n");
        printf("%s \n", rangesList.ranges(index).DebugString().c_str());
    }

    return true;
}

/**
 * @brief Sets the collision detection limits in the system
 * @param[in] args Every arguments related to the testclient tool command (safety system mode 1, tcp limit 1, elbow limit 1, activation status 1, safety system mode 2, tcp limit 2, elbow limit 2, activation status 2).
 * @return True
 */
bool KaKonverseAPITest::SetAllCollisionDetectionLimits(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::CollisionDetectionLimitsList limitsList;
    Kinova::Api::ControlConfig::CollisionDetectionLimits* limits;

    limits = limitsList.add_limits();
    limits->set_mode(static_cast<Kinova::Api::SafetyFunctions::SafetySystemMode>(std::stoi(args[0])));
    limits->set_tcp_limit(static_cast<float>(std::stoi(args[1])));
    limits->set_elbow_limit(static_cast<float>(std::stoi(args[2])));
    limits->set_enabled(static_cast<bool>(std::stoi(args[3])));

    limits = limitsList.add_limits();
    limits->set_mode(static_cast<Kinova::Api::SafetyFunctions::SafetySystemMode>(std::stoi(args[4])));
    limits->set_tcp_limit(static_cast<float>(std::stoi(args[5])));
    limits->set_elbow_limit(static_cast<float>(std::stoi(args[6])));
    limits->set_enabled(static_cast<bool>(std::stoi(args[7])));

    m_pKonverseClient->controlCfg->SetAllCollisionDetectionLimits(limitsList);

    return true;
}

/**
 * @brief Gets the actual collision detection limits of the system
 * @param[in] args Every arguments related to the testclient tool command (none in this case).
 * @return True
 */
bool KaKonverseAPITest::GetAllCollisionDetectionLimits(std::vector<std::string> args)
{
    auto limitsList = m_pKonverseClient->controlCfg->GetAllCollisionDetectionLimits();

    for(uint16_t index = 0U; index < limitsList.limits_size(); ++index)
    {
        printf("Collision Detection limits for the specific Safety System Mode : \n");
        printf("%s \n", limitsList.limits(index).DebugString().c_str());
    }

    return true;
}

/**
 * @brief Gets the energy limitation ranges for the limits configuration
 * @param[in] args Every arguments related to the testclient tool command (none in this case).
 * @return True
 */
bool KaKonverseAPITest::GetEnergyLimitsRangesList(std::vector<std::string> args)
{
    auto rangesList = m_pKonverseClient->controlCfg->GetEnergyLimitsRangesList();

    for(uint16_t index = 0U; index < rangesList.ranges_size(); ++index)
    {
        printf("Energy ranges for the specific Safety System Mode : \n");
        printf("%s \n", rangesList.ranges(index).DebugString().c_str());
    }

    return true;
}

/**
 * @brief Sets the energy limits in the system
 * @param[in] args Every arguments related to the testclient tool command (safety system mode 1, tcp limit 1, elbow limit 1, activation status 1, safety system mode 2, tcp limit 2, elbow limit 2, activation status 2).
 * @return True
 */
bool KaKonverseAPITest::SetAllEnergyLimits(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::EnergyLimitsList limitsList;
    Kinova::Api::ControlConfig::EnergyLimits* limits;

    limits = limitsList.add_limits();
    limits->set_mode(static_cast<Kinova::Api::SafetyFunctions::SafetySystemMode>(std::stoi(args[0])));
    limits->set_tcp_limit(static_cast<float>(std::stoi(args[1])));
    limits->set_elbow_limit(static_cast<float>(std::stoi(args[2])));
    limits->set_enabled(static_cast<bool>(std::stoi(args[3])));

    limits = limitsList.add_limits();
    limits->set_mode(static_cast<Kinova::Api::SafetyFunctions::SafetySystemMode>(std::stoi(args[4])));
    limits->set_tcp_limit(static_cast<float>(std::stoi(args[5])));
    limits->set_elbow_limit(static_cast<float>(std::stoi(args[6])));
    limits->set_enabled(static_cast<bool>(std::stoi(args[7])));

    m_pKonverseClient->controlCfg->SetAllEnergyLimits(limitsList);

    return true;
}

/**
 * @brief Gets the actual energy limits of the system
 * @param[in] args Every arguments related to the testclient tool command (none in this case).
 * @return True
 */
bool KaKonverseAPITest::GetAllEnergyLimits(std::vector<std::string> args)
{
    auto limitsList = m_pKonverseClient->controlCfg->GetAllEnergyLimits();

    for(uint16_t index = 0U; index < limitsList.limits_size(); ++index)
    {
        printf("Energy limits for the specific Safety System Mode : \n");
        printf("%s \n", limitsList.limits(index).DebugString().c_str());
    }

    return true;
}

bool KaKonverseAPITest::SetSafetySystemMode(std::vector<std::string> args)
 {
    if (CapabilityManager::hasSafetyFunctions())
    {
        Kinova::Api::SafetyFunctions::SafetySystem safetySystem;

        safetySystem.set_mode(static_cast<Kinova::Api::SafetyFunctions::SafetySystemMode>(std::stoi(args[0])));
        m_pKonverseClient->safetyFunctions->SetSafetySystemMode(safetySystem);
        sleep(std::stoi(args[1]));
    }
    else
    {
        printf("This function is not supported for this product\n");
    }

    return true;
}

/**
 * @brief Gets the arm speed factor
 * @param[in] args Every arguments related to the testclient tool command.
 * @return True
 */
bool KaKonverseAPITest::GetArmSpeedFactor(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::DesiredArmSpeedFactor armSpeedFactor;

    armSpeedFactor = m_pKonverseClient->controlCfg->GetArmSpeedFactor();

    printf("Arm Speed Factor: %f\n", armSpeedFactor.value());

    return true;
}

/**
 * @brief Sets the arm speed factor
 * @param[in] args Every arguments related to the testclient tool command.
 * @return True
 */
bool KaKonverseAPITest::SetArmSpeedFactor(std::vector<std::string> args)
{
    Kinova::Api::ControlConfig::DesiredArmSpeedFactor armSpeedFactor;
    armSpeedFactor.set_value(std::stof(args[0]));

    m_pKonverseClient->controlCfg->SetArmSpeedFactor(armSpeedFactor);

    return true;
}

/**
 * @brief API test layer for ProgramOptions object update to database
 *
 * @param args string arguments
 * @return always true
 */
bool KaKonverseAPITest::SetProgramOptions(std::vector<std::string> args)
{
    Kinova::Api::ProgramConfig::ProgramOptions options;
    options.set_need_acknowledge_before_run_auto(std::stoi(args[0]));

    m_pKonverseClient->programCfg->SetProgramOptions(options);

    return true;
}

/**
 * @brief API test layer for ProgramOptions object retrieve from database
 *
 * @param args string arguments
 * @return always true
 */
bool KaKonverseAPITest::GetProgramOptions(std::vector<std::string> args)
{
    auto options = m_pKonverseClient->programCfg->GetProgramOptions();

    printf("     Enabled: %s\n", options.need_acknowledge_before_run_auto() ? "Yes" : "No");

    return true;
}

/**
 * @brief API test layer for ProgramConfiguration object retrieve from database
 *
 * @param args string arguments
 * @return always true
 */
bool KaKonverseAPITest::GetProgramConfiguration(std::vector<std::string> args)
{
    Kinova::Api::Common::ProgramHandle inProgramHandle;
    Kinova::Api::ProgramConfig::Program outProgram;

    inProgramHandle.set_identifier(std::stoi(args[0]));
    outProgram = m_pKonverseClient->programCfg->ReadProgram(inProgramHandle);

    std::string output;
    MessageToJsonString(outProgram, &output);

    std::cout << output << std::endl;

    return true;
}

/**
 * @brief API test layer to import arm calibration
 *
 * @param[in] args string arguments
 * @return always true
 */
bool KaKonverseAPITest::ImportArmCalibration(std::vector<std::string> args)
{
    auto [success, readData] = common_fs::ReadRawDataFromFile(args[0]);
    if (success)
    {
        Kinova::Api::Base::FSReadData data;
        data.set_data(std::string(readData.begin(), readData.end()));

        m_pKonverseClient->baseCfg->ImportArmCalibration(data);
    }
    else
    {
        printf("File '%s' couldn't be read.\n", args[0].c_str());
    }

    return true;
}

/**
 * @brief API test layer to restore neutral arm calibration
 *
 * @param[in] args string arguments
 * @return always true
 */
bool KaKonverseAPITest::RestoreNeutralArmCalibration(std::vector<std::string> args)
{
    m_pKonverseClient->baseCfg->RestoreNeutralArmCalibration();

    return true;
}

/**
 * @brief API test layer to get arm calibration status
 *
 * @param[in] args string arguments
 * @return always true
 */
bool KaKonverseAPITest::GetArmCalibrationStatus(std::vector<std::string> args)
{
    auto armCalibrationStatus{m_pKonverseClient->baseCfg->GetArmCalibrationStatus()};
    printf("%s\n", armCalibrationStatus.DebugString().c_str());

    return true;
}

std::shared_ptr<Fw3Db::Action> KaKonverseAPITest::ActionFactory(Kinova::Api::Base::Action *action)
{
    switch(action->action_parameters_case())
    {
        case Kinova::Api::Base::Action::kSendTwistCommand:
            return std::shared_ptr<Fw3Db::ActionTwist>(new Fw3Db::ActionTwist(action));

        case Kinova::Api::Base::Action::kChangeTwist:
            return std::shared_ptr<Fw3Db::ActionChangeTwist>(new Fw3Db::ActionChangeTwist(action));

        case Kinova::Api::Base::Action::kSendJointSpeeds:
            return std::shared_ptr<Fw3Db::ActionJointSpeed>(new Fw3Db::ActionJointSpeed(action));

        case Kinova::Api::Base::Action::kChangeJointSpeeds:
            return std::shared_ptr<Fw3Db::ActionChangeJointSpeed>(new Fw3Db::ActionChangeJointSpeed(action));

        case Kinova::Api::Base::Action::kNavigateMappings:
            return std::shared_ptr<Fw3Db::ActionNavigateMapping>(new Fw3Db::ActionNavigateMapping(action));

        case Kinova::Api::Base::Action::kNavigateJoints:
            return std::shared_ptr<Fw3Db::ActionNavigateJoints>(new Fw3Db::ActionNavigateJoints(action));

        case Kinova::Api::Base::Action::kSwitchControlMapping:
            return std::shared_ptr<Fw3Db::ActionSwitchMapping>(new Fw3Db::ActionSwitchMapping(action));

        case Kinova::Api::Base::Action::kToggleHandGuidingMode:
            return std::shared_ptr<Fw3Db::ActionToggleAdmittance>(new Fw3Db::ActionToggleAdmittance(action));

        case Kinova::Api::Base::Action::kApplyQuickStop:
            return std::shared_ptr<Fw3Db::ActionApplyEmergencyStop>(new Fw3Db::ActionApplyEmergencyStop(action));

        case Kinova::Api::Base::Action::kClearFaults:
            return std::shared_ptr<Fw3Db::ActionClearFaults>(new Fw3Db::ActionClearFaults(action));

        case Kinova::Api::Base::Action::kExecuteAction:
            return std::shared_ptr<Fw3Db::ActionExecute>(new Fw3Db::ActionExecute(action));

        case Kinova::Api::Base::Action::kStopAction:
            return std::shared_ptr<Fw3Db::ActionStop>(new Fw3Db::ActionStop(action));

        default:
            printf("Got UNKNOWN action parameter %d\n", action->action_parameters_case());
            return nullptr;
    }
}


float KaKonverseAPITest::StringToFloat(const std::string& s, float mult)
{
    try
    {
        return std::stof(s) * mult;
    }
    catch(const std::out_of_range)
    {
        throw std::out_of_range(s + " is out of float range");
    }
    catch(const std::invalid_argument)
    {
        throw std::invalid_argument(s + " is an invalid argument");
    }
}

bool KaKonverseAPITest::RunCyclicLoop(std::vector<std::string> args)
{
    // Parse optional parameters
    // Usage: run_cyclic_loop [actuator_index] [clockwise] [velocity_deg_per_sec]
    //   actuator_index:     0-5, which actuator to move (default: 0)
    //   clockwise:          1 = clockwise (default), 0 = counter-clockwise
    //   velocity_deg_per_sec: target velocity in deg/s (default: 3.0)
    const uint32_t servoing_mode = 3;  // Always LOW_LEVEL_SERVOING
    const char* mode_name = "LOW_LEVEL_SERVOING";

    uint32_t moving_actuator = 0;  // Default: first actuator
    if (args.size() > 0)
    {
        try {
            moving_actuator = std::stoi(args[0]);
        }
        catch (...) {
            printf("Error: Invalid actuator_index. Use: run_cyclic_loop [actuator_index] [clockwise]\n");
            return false;
        }
    }

    bool clockwise = true;  // Default: clockwise
    if (args.size() > 1)
    {
        try {
            clockwise = (std::stoi(args[1]) != 0);
        }
        catch (...) {
            printf("Error: Invalid clockwise value. Use: run_cyclic_loop [actuator_index] [clockwise] [velocity_deg_per_sec]\n");
            return false;
        }
    }

    float velocity_deg_per_sec = 3.0f;  // Default: 3 deg/s
    if (args.size() > 2)
    {
        try {
            velocity_deg_per_sec = std::stof(args[2]);
        }
        catch (...) {
            printf("Error: Invalid velocity. Use: run_cyclic_loop [actuator_index] [clockwise] [velocity_deg_per_sec]\n");
            return false;
        }
    }

    const float amplitude_deg = 10.0f;               // Always move 10 degrees
    const float acceleration_deg_per_sec2 = 25.0f;   // Accel/decel at 25 deg/s^2
    const float deceleration_deg_per_sec2 = 25.0f;

    printf("Starting cyclic loop test (trapezoidal velocity profile):\n");
    printf("  Servoing mode: %s (%u)\n", mode_name, servoing_mode);
    printf("  Actuator: %u\n", moving_actuator);
    printf("  Direction: %s\n", clockwise ? "clockwise" : "counter-clockwise");
    printf("  Target velocity: %.2f deg/s\n", velocity_deg_per_sec);
    printf("  Total angle: %.2f deg\n", amplitude_deg);
    printf("  Fixed acceleration: %.2f deg/s^2\n", acceleration_deg_per_sec2);
    printf("  Fixed deceleration: %.2f deg/s^2\n", deceleration_deg_per_sec2);

    // Create UDP transport and router for BaseCyclic
    printf("\nCreating UDP transport on port 10001...\n");
    auto transport = new Kinova::Api::TransportClientUdp();
    transport->connect("127.0.0.1", 10001);

    auto router = new Kinova::Api::RouterClient(transport, [](Kinova::Api::KError err){
        std::cerr << "UDP Router Error: " << err.toString() << std::endl;
    });

    // Create BaseCyclic client on UDP
    auto baseCyclicUdp = new Kinova::Api::BaseCyclic::BaseCyclicClient(router);

    // Create Session client on UDP router and authenticate
    printf("Creating UDP session...\n");
    auto sessionUdp = new Kinova::Api::Session::SessionClient(router);

    auto createSessionInfo = Kinova::Api::Session::CreateSessionInfo();
    createSessionInfo.set_username("admin");
    createSessionInfo.set_password("admin");
    createSessionInfo.set_session_inactivity_timeout(60000);  // 60 seconds for test
    createSessionInfo.set_connection_inactivity_timeout(60000);

    try
    {
        sessionUdp->CreateSession(createSessionInfo);
        printf("UDP session created successfully!\n");
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        printf("Error creating UDP session: %s\n", ex.what());
        delete baseCyclicUdp;
        delete sessionUdp;
        delete router;
        transport->disconnect();
        delete transport;
        return false;
    }

    // Wait for session to establish
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    // Switch to selected servoing mode (via MQTT)
    printf("\nSwitching to %s mode...\n", mode_name);
    SetServoingMode(servoing_mode);

    // Get initial feedback to establish starting positions
    // Send multiple synchronization cycles to ensure stable communication
    printf("Synchronizing position (sending hold commands)...\n");
    Kinova::Api::BaseCyclic::Feedback initial_feedback;

    // Send 30 hold-position cycles to synchronize.
    // IMPORTANT: Send hold commands from cycle 0, not just RefreshFeedback.
    // In LOW_LEVEL mode the actuator needs a BaseCyclic position setpoint immediately
    // after mode transition. Delaying commands (RefreshFeedback-only cycles) leaves
    // the actuator without a setpoint, causing Code 50030 when the first Refresh()
    // command eventually arrives.
    for (int sync_cycle = 0; sync_cycle < 30; sync_cycle++)
    {
        try
        {
            // Get current feedback first, then command that exact position.
            Kinova::Api::BaseCyclic::Feedback current_sync_feedback;
            current_sync_feedback = baseCyclicUdp->RefreshFeedback();

            // Log initial positions on first cycle
            if (sync_cycle == 0)
            {
                printf("Initial positions captured (cycle 0):\n");
                for (int i = 0; i < current_sync_feedback.actuators_size(); i++)
                {
                    printf("  Act %d: %.3f deg\n", i, current_sync_feedback.actuators(i).position());
                }
            }

            // Build and send hold command using the position we JUST received
            Kinova::Api::BaseCyclic::Command hold_cmd;
            hold_cmd.set_frame_id(sync_cycle);
            for (int i = 0; i < current_sync_feedback.actuators_size(); i++)
            {
                auto* actuator = hold_cmd.add_actuators();
                actuator->set_flags(0);
                actuator->set_position(current_sync_feedback.actuators(i).position());
                actuator->set_velocity(0.0f);
            }

            if (sync_cycle == 0)
            {
                printf("First hold commands sent (cycle 0) using fresh feedback positions\n");
            }

            current_sync_feedback = baseCyclicUdp->Refresh(hold_cmd);

            // Update feedback for next cycle
            initial_feedback = current_sync_feedback;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));  // 1ms between sync cycles
        }
        catch (Kinova::Api::KDetailedException& ex)
        {
            printf("Error during synchronization cycle %d: %s\n", sync_cycle, ex.what());
            delete baseCyclicUdp;
            delete sessionUdp;
            delete router;
            transport->disconnect();
            delete transport;
            return false;
        }
    }
    printf("Position synchronized\n");

    // Store initial joint positions and initialize target positions
    std::vector<float> initial_positions;
    std::vector<float> target_positions;  // Track commanded positions independently
    printf("Initial joint positions:\n");
    if (moving_actuator >= (uint32_t)initial_feedback.actuators_size())
    {
        printf("Error: actuator_index %u out of range (robot has %d actuators, valid range: 0-%d)\n",
               moving_actuator, initial_feedback.actuators_size(), initial_feedback.actuators_size() - 1);
        return false;
    }

    for (uint32_t i = 0; i < initial_feedback.actuators_size(); i++)
    {
        float pos = initial_feedback.actuators(i).position();
        initial_positions.push_back(pos);
        target_positions.push_back(pos);  // Start at current position
        if (i == moving_actuator)
        {
            printf("  Joint %u: %.2f deg\n", i, pos);
        }
    }

    // Calculate trapezoidal (or triangular) profile parameters
    const float direction = clockwise ? 1.0f : -1.0f;
    const float total_angle_to_move = fabs(amplitude_deg);
    const float accel = fabs(acceleration_deg_per_sec2);

    // Clamp peak velocity to what is physically reachable within the given amplitude.
    // If the requested velocity is too high, the profile becomes triangular (no constant
    // velocity phase): the robot accelerates to the clamped peak then immediately decelerates.
    const float max_reachable_velocity = sqrtf(accel * total_angle_to_move);
    float target_velocity = fabs(velocity_deg_per_sec);
    if (target_velocity > max_reachable_velocity)
    {
        printf("  Note: Peak velocity clamped %.2f -> %.2f deg/s (triangular profile, amplitude too small)\n",
               target_velocity, max_reachable_velocity);
        target_velocity = max_reachable_velocity;
    }

    // Calculate time durations for each phase
    const float accel_time = target_velocity / accel;
    const float decel_time = accel_time;  // Symmetric profile

    // Distance covered during acceleration and deceleration
    const float accel_distance = 0.5f * accel * accel_time * accel_time;  // d = 0.5*a*t²
    const float decel_distance = accel_distance;
    const float const_velocity_distance = total_angle_to_move - accel_distance - decel_distance;
    const float const_time = (const_velocity_distance > 0.0f) ? (const_velocity_distance / target_velocity) : 0.0f;

    printf("  Accel time: %.2f s (distance: %.2f deg)\n", accel_time, accel_distance);
    printf("  Const velocity time: %.2f s (distance: %.2f deg)\n", const_time, const_velocity_distance);
    printf("  Decel time: %.2f s (distance: %.2f deg)\n\n", decel_time, decel_distance);

    // Timing parameters
    const uint32_t CYCLE_TIME_US = 1000;  // 1ms = 1kHz target
    const float WARMUP_TIME_SEC = 0.5f;  // 500ms warmup (increased for stability after brake release)

    printf("\nStarting cyclic loop at 1kHz (1ms per cycle)...\n");
    printf("Warmup period: %.0f ms\n", WARMUP_TIME_SEC * 1000.0f);
    printf("Will stop automatically 1s after motion completes\n\n");

    uint32_t cycle_count = 0;
    bool running = true;
    auto test_start_time = std::chrono::high_resolution_clock::now();
    float last_print_time = 0.0f;
    float angle_moved = 0.0f;  // Track cumulative angle moved
    uint32_t complete_hold_cycles = 0;  // Cycles spent holding after motion complete

    // Store current feedback for next cycle
    Kinova::Api::BaseCyclic::Feedback current_feedback = initial_feedback;

    while (running)
    {
        auto cycle_start = std::chrono::high_resolution_clock::now();

        // Calculate actual elapsed time
        float elapsed_time_sec = std::chrono::duration_cast<std::chrono::microseconds>(
            cycle_start - test_start_time).count() / 1000000.0f;

        // Prepare command
        Kinova::Api::BaseCyclic::Command command;
        command.set_frame_id(cycle_count);

        // Calculate target velocity and acceleration based on time (trapezoidal profile)
        // Use angle_moved only as stopping condition
        float current_velocity;
        float current_accel;  // Current acceleration for position calculation
        float motion_time = elapsed_time_sec - WARMUP_TIME_SEC;  // Time since motion started

        if (elapsed_time_sec < WARMUP_TIME_SEC)
        {
            // Warmup phase - no motion
            current_velocity = 0.0f;
            current_accel = 0.0f;
        }
        else if (motion_time >= 0.0f && motion_time < 0.001f)
        {
            // Just started motion - print trace
            printf("\n*** MOTION STARTED - Robot should be moving now! ***\n");
            printf("  Time: %.3f s, Angle moved: %.2f deg\n\n", elapsed_time_sec, angle_moved);
            current_velocity = accel * motion_time;
            current_accel = accel;
        }
        else if (angle_moved >= total_angle_to_move)
        {
            // Motion complete - hold briefly then exit
            current_velocity = 0.0f;
            current_accel = 0.0f;
            complete_hold_cycles++;
            if (complete_hold_cycles >= 1000)  // 1 second hold then exit
            {
                running = false;
            }
        }
        else if (motion_time < accel_time)
        {
            // Acceleration phase: v = a*t
            current_velocity = accel * motion_time;
            current_accel = accel;  // Positive acceleration
        }
        else if (motion_time < (accel_time + const_time))
        {
            // Constant velocity phase
            current_velocity = target_velocity;
            current_accel = 0.0f;  // No acceleration
        }
        else if (motion_time < (accel_time + const_time + decel_time))
        {
            // Deceleration phase: v = v_target - a*(t - t_decel_start)
            float decel_elapsed = motion_time - accel_time - const_time;
            current_velocity = target_velocity - accel * decel_elapsed;
            if (current_velocity < 0.0f) current_velocity = 0.0f;
            current_accel = -accel;  // Negative acceleration (deceleration)
        }
        else
        {
            // Time-based motion complete - hold briefly then exit
            current_velocity = 0.0f;
            current_accel = 0.0f;
            complete_hold_cycles++;
            if (complete_hold_cycles >= 1000)  // 1 second hold then exit
            {
                running = false;
            }
        }

        // Note: TCP velocity limiting is now handled by Kontrol's energy-based limiting
        // in LowLevelPassthroughMode. No testclient-side limiting needed.

        // Build commands for all actuators
        // Only the first num_actuators will move, rest hold position
        for (uint32_t i = 0; i < current_feedback.actuators_size(); i++)
        {
            auto* actuator = command.add_actuators();

            // Set flags to 0 (no special flags needed for LOW_LEVEL mode)
            actuator->set_flags(0);

            if (i == moving_actuator)
            {
                // This actuator should move
                // Compute position increment using velocity-only: Δx = v*Δt
                // (Removed acceleration term to prevent tracking errors)
                const float dt = 0.001f;  // 1ms cycle time @ 1kHz
                float position_increment = direction * current_velocity * dt;

                // Update target position for this actuator
                target_positions[i] += position_increment;

                // Set position and velocity commands
                actuator->set_position(target_positions[i]);
                actuator->set_velocity(fabs(current_velocity));

                angle_moved += fabs(position_increment);
            }
            else
            {
                // This actuator should hold position (use tracked target, not feedback)
                actuator->set_position(target_positions[i]);
                actuator->set_velocity(0.0f);
            }
        }

        // Send command and get feedback
        try
        {
            Kinova::Api::BaseCyclic::Feedback new_feedback = baseCyclicUdp->Refresh(command);
            current_feedback = new_feedback; // Store for next cycle

            // Print status every second
            if (elapsed_time_sec - last_print_time >= 1.0f)
            {
                auto cycle_end = std::chrono::high_resolution_clock::now();
                auto cycle_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    cycle_end - cycle_start).count();
                float cycle_rate_hz = cycle_count / elapsed_time_sec;

                // Determine current phase based on time
                const char* phase = "warmup";
                if (elapsed_time_sec >= WARMUP_TIME_SEC)
                {
                    float motion_time_status = elapsed_time_sec - WARMUP_TIME_SEC;
                    if (angle_moved >= total_angle_to_move || motion_time_status >= (accel_time + const_time + decel_time))
                    {
                        phase = "complete";
                    }
                    else if (motion_time_status < accel_time)
                    {
                        phase = "accel";
                    }
                    else if (motion_time_status < (accel_time + const_time))
                    {
                        phase = "const";
                    }
                    else
                    {
                        phase = "decel";
                    }
                }

                printf("Time: %.1fs [%s], Cycles: %u (%.0f Hz), Cycle: %ld us, Moved: %.2f/%.2f deg, J%u: %.2f deg, V%u: %.2f deg/s\n",
                       elapsed_time_sec,
                       phase,
                       cycle_count,
                       cycle_rate_hz,
                       cycle_time_us,
                       angle_moved,
                       total_angle_to_move,
                       moving_actuator,
                       current_feedback.actuators_size() > (int)moving_actuator ? current_feedback.actuators(moving_actuator).position() : 0.0f,
                       moving_actuator,
                       current_feedback.actuators_size() > (int)moving_actuator ? current_feedback.actuators(moving_actuator).velocity() : 0.0f);

                last_print_time = elapsed_time_sec;

                // Check for faults
                if (current_feedback.base().fault_bank_a() != 0 || current_feedback.base().fault_bank_b() != 0)
                {
                    printf("FAULT DETECTED! Bank A: 0x%08X, Bank B: 0x%08X\n",
                           current_feedback.base().fault_bank_a(),
                           current_feedback.base().fault_bank_b());
                    running = false;
                }
            }
        }
        catch (Kinova::Api::KDetailedException& ex)
        {
            printf("Error during cyclic loop (time %.2fs, cycle %u): %s\n",
                   elapsed_time_sec, cycle_count, ex.what());
            running = false;
            break;
        }

        cycle_count++;

        // Busy-wait to maintain 1kHz rate (1ms per cycle)
        auto cycle_end = std::chrono::high_resolution_clock::now();
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
            cycle_end - cycle_start).count();

        // Spin until 1ms has elapsed from cycle start
        while (elapsed_us < CYCLE_TIME_US)
        {
            cycle_end = std::chrono::high_resolution_clock::now();
            elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
                cycle_end - cycle_start).count();
        }
    }

    auto final_time = std::chrono::high_resolution_clock::now();
    float total_time_sec = std::chrono::duration_cast<std::chrono::microseconds>(
        final_time - test_start_time).count() / 1000000.0f;
    float avg_cycle_rate_hz = cycle_count / total_time_sec;
    float avg_cycle_time_us = (total_time_sec * 1000000.0f) / cycle_count;

    printf("\nCyclic loop completed:\n");
    printf("  Total time: %.2f seconds\n", total_time_sec);
    printf("  Total cycles: %u\n", cycle_count);
    printf("  Average rate: %.0f Hz\n", avg_cycle_rate_hz);
    printf("  Average cycle time: %.0f us\n", avg_cycle_time_us);
    // RC1-762: Switch back to SINGLE_LEVEL via the gRPC API to exercise Fix 6 + Fix 8
    // (guard StartCyclic on exit + SetControlMode(kIdle) on exit).
    // The armbase journal should show:
    //   [RC1-762] LOW->SINGLE exit: Skipping StartCyclic() - already in cyclic
    //   [RC1-762] LOW->SINGLE exit: Setting Kontrol to kIdle
    printf("\nSwitching back to SINGLE_LEVEL_SERVOING via API (RC1-762 Fix 6+8 test)...\n");
    SetServoingMode(2);  // 2 = SINGLE_LEVEL_SERVOING
    printf("Back in SINGLE_LEVEL_SERVOING. Robot should now accept jog/trajectory commands.\n");

    // Cleanup UDP session
    printf("\nClosing UDP session...\n");
    try
    {
        sessionUdp->CloseSession();
    }
    catch (...)
    {
        // Ignore errors on cleanup
    }

    delete baseCyclicUdp;
    delete sessionUdp;
    delete router;
    transport->disconnect();
    delete transport;

    printf("UDP connection closed.\n");

    return true;
}
bool KaKonverseAPITest::HoldPosition(std::vector<std::string> args)
{
    // Usage: hold_position [duration_sec]
    float duration_sec = 5.0f;
    if (args.size() > 0)
    {
        try { duration_sec = std::stof(args[0]); }
        catch (...) { printf("Error: Invalid duration. Usage: hold_position [duration_sec]\n"); return false; }
    }

    printf("=== Hold Position Test ===\n");
    printf("Duration: %.1f s\n", duration_sec);
    printf("Goal: send initial position back at 1kHz, robot must NOT move, no faults expected.\n\n");

    // --- UDP setup (same as RunCyclicLoop) ---
    auto transport = new Kinova::Api::TransportClientUdp();
    transport->connect("127.0.0.1", 10001);

    auto router = new Kinova::Api::RouterClient(transport, [](Kinova::Api::KError err){
        std::cerr << "UDP Router Error: " << err.toString() << std::endl;
    });

    auto baseCyclicUdp = new Kinova::Api::BaseCyclic::BaseCyclicClient(router);
    auto sessionUdp    = new Kinova::Api::Session::SessionClient(router);

    auto createSessionInfo = Kinova::Api::Session::CreateSessionInfo();
    createSessionInfo.set_username("admin");
    createSessionInfo.set_password("admin");
    createSessionInfo.set_session_inactivity_timeout(60000);
    createSessionInfo.set_connection_inactivity_timeout(60000);

    auto cleanup = [&]() {
        try { sessionUdp->CloseSession(); } catch (...) {}
        delete baseCyclicUdp;
        delete sessionUdp;
        delete router;
        transport->disconnect();
        delete transport;
    };

    try
    {
        sessionUdp->CreateSession(createSessionInfo);
        printf("UDP session created.\n");
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        printf("Error creating UDP session: %s\n", ex.what());
        cleanup();
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    // Switch to LOW_LEVEL_SERVOING
    printf("Switching to LOW_LEVEL_SERVOING...\n");
    SetServoingMode(3);

    printf("Waiting for brakes to release (5s)...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    // Capture initial positions (5 feedback-only cycles to let things settle)
    printf("Capturing initial positions...\n");
    Kinova::Api::BaseCyclic::Feedback initial_feedback;
    for (int i = 0; i < 5; i++)
    {
        try { initial_feedback = baseCyclicUdp->RefreshFeedback(); }
        catch (Kinova::Api::KDetailedException& ex)
        {
            printf("Error getting initial feedback: %s\n", ex.what());
            cleanup();
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    printf("Initial positions (these will be commanded throughout the test):\n");
    for (int i = 0; i < initial_feedback.actuators_size(); i++)
    {
        printf("  Joint %d: %.3f deg\n", i, initial_feedback.actuators(i).position());
    }
    printf("\nStarting hold loop at 1kHz for %.1f s...\n\n", duration_sec);

    // --- Hold loop ---
    const uint32_t CYCLE_TIME_US = 1000;
    uint32_t cycle_count = 0;
    bool running = true;
    float last_print_time = 0.0f;
    auto test_start = std::chrono::high_resolution_clock::now();
    Kinova::Api::BaseCyclic::Feedback current_feedback = initial_feedback;

    while (running)
    {
        auto cycle_start = std::chrono::high_resolution_clock::now();
        float elapsed_sec = std::chrono::duration_cast<std::chrono::microseconds>(
            cycle_start - test_start).count() / 1000000.0f;

        if (elapsed_sec >= duration_sec)
            break;

        // Build command: always send the captured initial positions
        Kinova::Api::BaseCyclic::Command command;
        command.set_frame_id(cycle_count);
        for (int i = 0; i < initial_feedback.actuators_size(); i++)
        {
            auto* actuator = command.add_actuators();
            actuator->set_flags(0);
            actuator->set_position(initial_feedback.actuators(i).position());
            actuator->set_velocity(0.0f);
        }

        try
        {
            current_feedback = baseCyclicUdp->Refresh(command);

            if (elapsed_sec - last_print_time >= 1.0f)
            {
                auto cycle_end = std::chrono::high_resolution_clock::now();
                auto cycle_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    cycle_end - cycle_start).count();
                float cycle_rate_hz = cycle_count / elapsed_sec;

                printf("Time: %.1fs, Cycles: %u (%.0f Hz), CycleTime: %ld us, J0: cmd=%.3f actual=%.3f deg\n",
                       elapsed_sec, cycle_count, cycle_rate_hz, cycle_time_us,
                       initial_feedback.actuators(0).position(),
                       current_feedback.actuators_size() > 0 ? current_feedback.actuators(0).position() : 0.0f);

                last_print_time = elapsed_sec;

                if (current_feedback.base().fault_bank_a() != 0 || current_feedback.base().fault_bank_b() != 0)
                {
                    printf("FAULT DETECTED! Bank A: 0x%08X, Bank B: 0x%08X\n",
                           current_feedback.base().fault_bank_a(),
                           current_feedback.base().fault_bank_b());
                    running = false;
                }
            }
        }
        catch (Kinova::Api::KDetailedException& ex)
        {
            printf("Error at cycle %u (%.2fs): %s\n", cycle_count, elapsed_sec, ex.what());
            running = false;
            break;
        }

        cycle_count++;

        // Busy-wait to maintain 1kHz
        auto cycle_end = std::chrono::high_resolution_clock::now();
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
            cycle_end - cycle_start).count();
        while (elapsed_us < CYCLE_TIME_US)
        {
            cycle_end = std::chrono::high_resolution_clock::now();
            elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
                cycle_end - cycle_start).count();
        }
    }

    auto final_time = std::chrono::high_resolution_clock::now();
    float total_sec = std::chrono::duration_cast<std::chrono::microseconds>(
        final_time - test_start).count() / 1000000.0f;

    printf("\nHold position test complete:\n");
    printf("  Total time: %.2f s\n", total_sec);
    printf("  Total cycles: %u\n", cycle_count);
    printf("  Average rate: %.0f Hz\n", cycle_count / total_sec);
    if (current_feedback.actuators_size() > 0)
    {
        printf("  Final J0 position: %.3f deg (initial: %.3f deg, delta: %.3f deg)\n",
               current_feedback.actuators(0).position(),
               initial_feedback.actuators(0).position(),
               current_feedback.actuators(0).position() - initial_feedback.actuators(0).position());
    }
    printf("  Faults: %s\n",
           (current_feedback.base().fault_bank_a() == 0 && current_feedback.base().fault_bank_b() == 0)
           ? "NONE" : "YES (check above)");

    cleanup();
    return true;
}

bool KaKonverseAPITest::TestUdpFeedback(std::vector<std::string> args)
{
    printf("\n=== Testing UDP BaseCyclic RefreshFeedback ===\n\n");

    // Create UDP transport and router
    printf("Creating UDP transport on port 10001...\n");
    auto transport = new Kinova::Api::TransportClientUdp();
    transport->connect("127.0.0.1", 10001);

    auto router = new Kinova::Api::RouterClient(transport, [](Kinova::Api::KError err){
        std::cerr << "UDP Router Error: " << err.toString() << std::endl;
    });

    // Create BaseCyclic client
    auto baseCyclicUdp = new Kinova::Api::BaseCyclic::BaseCyclicClient(router);

    // Create Session client on UDP router
    printf("Creating session on UDP router...\n");
    auto sessionUdp = new Kinova::Api::Session::SessionClient(router);

    // Create session with admin credentials
    auto createSessionInfo = Kinova::Api::Session::CreateSessionInfo();
    createSessionInfo.set_username("admin");
    createSessionInfo.set_password("admin");
    createSessionInfo.set_session_inactivity_timeout(20000);
    createSessionInfo.set_connection_inactivity_timeout(10000);

    try
    {
        sessionUdp->CreateSession(createSessionInfo);
        printf("UDP session created successfully!\n\n");
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        printf("Error creating UDP session: %s\n", ex.what());
        delete baseCyclicUdp;
        delete sessionUdp;
        delete router;
        transport->disconnect();
        delete transport;
        return false;
    }

    // Wait a bit for session to establish
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    // Loop calling RefreshFeedback and display actuator positions
    printf("Press Ctrl+C to stop\n\n");
    printf("%-10s ", "Time");
    for (uint32_t i = 0; i < 6; i++)
    {
        printf("Actuator%u ", i);
    }
    printf("\n");
    printf("%-10s ", "(sec)");
    for (uint32_t i = 0; i < 6; i++)
    {
        printf("%-10s ", "(deg)");
    }
    printf("\n");
    printf("----------------------------------------");
    printf("----------------------------------------\n");

    auto start_time = std::chrono::high_resolution_clock::now();
    uint32_t iteration = 0;

    while (iteration < 100)  // Run for 100 iterations
    {
        try
        {
            auto feedback = baseCyclicUdp->RefreshFeedback();

            auto now = std::chrono::high_resolution_clock::now();
            float elapsed_sec = std::chrono::duration_cast<std::chrono::microseconds>(
                now - start_time).count() / 1000000.0f;

            printf("%-10.2f ", elapsed_sec);

            for (uint32_t i = 0; i < std::min(6u, (uint32_t)feedback.actuators_size()); i++)
            {
                printf("%-10.2f ", feedback.actuators(i).position());
            }
            printf("\n");

            iteration++;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        catch (Kinova::Api::KDetailedException& ex)
        {
            printf("\nError getting feedback: %s\n", ex.what());
            break;
        }
    }

    // Cleanup
    printf("\nClosing UDP session...\n");
    try
    {
        sessionUdp->CloseSession();
    }
    catch (...)
    {
        // Ignore errors on cleanup
    }

    delete baseCyclicUdp;
    delete sessionUdp;
    delete router;
    transport->disconnect();
    delete transport;

    printf("Test complete!\n");
    return true;
}

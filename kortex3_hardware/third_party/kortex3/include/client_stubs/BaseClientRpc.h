#ifndef __BASECLIENT_H__
#define __BASECLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "Base.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace Base
{
	enum FunctionUids
	{
			eUidCreateUserProfile = 0x20001,
			eUidUpdateUserProfile = 0x20002,
			eUidReadUserProfile = 0x20003,
			eUidDeleteUserProfile = 0x20004,
			eUidReadAllUserProfiles = 0x20005,
			eUidReadAllUsers = 0x20006,
			eUidChangePassword = 0x20007,
			eUidCreateAction = 0x2002a,
			eUidReadAction = 0x2002b,
			eUidReadAllActions = 0x2002c,
			eUidDeleteAction = 0x2002d,
			eUidUpdateAction = 0x2002e,
			eUidExecuteAction = 0x20030,
			eUidPauseAction = 0x20031,
			eUidStopAction = 0x20032,
			eUidResumeAction = 0x20033,
			eUidGetIPv4Configuration = 0x2003b,
			eUidSetIPv4Configuration = 0x2003c,
			eUidUnsubscribe = 0x20061,
			eUidConfigurationChangeTopic = 0x20062,
			eUidOperatingModeTopic = 0x20065,
			eUidControllerTopic = 0x20069,
			eUidActionTopic = 0x2006a,
			eUidRobotEventTopic = 0x2006b,
			eUidStop = 0x20070,
			eUidGetMeasuredCartesianPose = 0x20073,
			eUidSendTwistJoystickCommand = 0x20078,
			eUidSendTwistCommand = 0x20079,
			eUidGetMeasuredJointAngles = 0x2007e,
			eUidSendJointSpeedsCommand = 0x20084,
			eUidSendSelectedJointSpeedCommand = 0x20085,
			eUidApplyQuickStop = 0x20091,
			eUidClearFaults = 0x20092,
			eUidSetServoingMode = 0x20098,
			eUidGetServoingMode = 0x20099,
			eUidServoingModeTopic = 0x2009a,
			eUidRestoreFactorySettings = 0x200a0,
			eUidReboot = 0x200a2,
			eUidFactoryTopic = 0x200a4,
			eUidGetActuatorCount = 0x200ab,
			eUidGetArmState = 0x200af,
			eUidArmStateTopic = 0x200b0,
			eUidGetIPv4Information = 0x200b1,
			eUidSendJointSpeedsJoystickCommand = 0x200bb,
			eUidSendSelectedJointSpeedJoystickCommand = 0x200bc,
			eUidGetProductConfiguration = 0x200c6,
			eUidRestoreFactoryProductConfiguration = 0x200ce,
			eUidGetTrajectoryErrorReport = 0x200cf,
			eUidGetFirmwareBundleVersions = 0x200e0,
			eUidExecuteWaypointTrajectory = 0x200e2,
			eUidValidateWaypointList = 0x200ec,
			eUidComputeForwardKinematics = 0x200ed,
			eUidComputeInverseKinematics = 0x200ee,
			eUidActivateRobot = 0x200ef,
			eUidUpdatingModeTopic = 0x200f0,
			eUidSetUpdatingMode = 0x200f1,
			eUidGetUpdatingMode = 0x200f2,
			eUidSelectOperatingMode = 0x200f3,
			eUidExitRecoveryState = 0x200f4,
			eUidDeactivateRobot = 0x200f5,
			eUidGetCurrentOperatingMode = 0x200f6,
			eUidReadAllUserRoles = 0x200f7,
			eUidSetHandGuidingMode = 0x200f8,
			eUidGetHandGuidingMode = 0x200f9,
			eUidHandGuidingModeTopic = 0x200fa,
			eUidGetLocalUpdateFileList = 0x200fb,
			eUidEnablingDeviceTopic = 0x200fc,
			eUidConfirmArmPosition = 0x200fd,
			eUidMotionTopic = 0x200fe,
			eUidGetLastRecordedJointAngles = 0x200ff,
			eUidProgramRequestTopic = 0x20100,
			eUidGetStorageList = 0x20101,
			eUidFileSystemList = 0x20102,
			eUidFileWrite = 0x20103,
			eUidFileRead = 0x20104,
			eUidGetRemoteAccessInfoList = 0x20105,
			eUidEnableRemoteAccess = 0x20106,
			eUidDisableRemoteAccess = 0x20107,
			eUidRemoteAccessChangeTopic = 0x20108,
			eUidImportArmCalibration = 0x20109,
			eUidExportArmCalibration = 0x2010a,
			eUidRestoreNeutralArmCalibration = 0x2010b,
			eUidGetArmCalibrationStatus = 0x2010c,
			eUidArmCalibrationStatusChangeTopic = 0x2010d,
			eUidAcknowledgeActionTopic = 0x2010e,
	};
	
	class BaseClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdBase;
	public:
		BaseClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		Kinova::Api::Common::UserProfileHandle CreateUserProfile(const FullUserProfile& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::Common::UserProfileHandle> CreateUserProfile_async(const FullUserProfile& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void CreateUserProfile_callback(const FullUserProfile& message, std::function<void(const Error&, const Kinova::Api::Common::UserProfileHandle&)> callback, uint32_t deviceId = 0);

		void UpdateUserProfile(const UserProfile& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> UpdateUserProfile_async(const UserProfile& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void UpdateUserProfile_callback(const UserProfile& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		UserProfile ReadUserProfile(const Kinova::Api::Common::UserProfileHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<UserProfile> ReadUserProfile_async(const Kinova::Api::Common::UserProfileHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ReadUserProfile_callback(const Kinova::Api::Common::UserProfileHandle& message, std::function<void(const Error&, const UserProfile&)> callback, uint32_t deviceId = 0);

		void DeleteUserProfile(const Kinova::Api::Common::UserProfileHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> DeleteUserProfile_async(const Kinova::Api::Common::UserProfileHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void DeleteUserProfile_callback(const Kinova::Api::Common::UserProfileHandle& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		UserProfileList ReadAllUserProfiles(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<UserProfileList> ReadAllUserProfiles_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ReadAllUserProfiles_callback(std::function<void(const Error&, const UserProfileList&)> callback, uint32_t deviceId = 0);

		UserList ReadAllUsers(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<UserList> ReadAllUsers_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ReadAllUsers_callback(std::function<void(const Error&, const UserList&)> callback, uint32_t deviceId = 0);

		void ChangePassword(const PasswordChange& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> ChangePassword_async(const PasswordChange& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ChangePassword_callback(const PasswordChange& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void ExecuteAction(const Action& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> ExecuteAction_async(const Action& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ExecuteAction_callback(const Action& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void PauseAction(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> PauseAction_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void PauseAction_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void StopAction(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> StopAction_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void StopAction_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void ResumeAction(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> ResumeAction_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ResumeAction_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		IPv4Configuration GetIPv4Configuration(const NetworkHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<IPv4Configuration> GetIPv4Configuration_async(const NetworkHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetIPv4Configuration_callback(const NetworkHandle& message, std::function<void(const Error&, const IPv4Configuration&)> callback, uint32_t deviceId = 0);

		void Unsubscribe(const Kinova::Api::Common::NotificationHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		Kinova::Api::Common::NotificationHandle OnNotificationConfigurationChangeTopic(std::function<void(ConfigurationChangeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		Kinova::Api::Common::NotificationHandle OnNotificationOperatingModeTopic(std::function<void(OperatingModeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		Kinova::Api::Common::NotificationHandle OnNotificationControllerTopic(std::function<void(ControllerNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		Kinova::Api::Common::NotificationHandle OnNotificationActionTopic(std::function<void(ActionNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		Kinova::Api::Common::NotificationHandle OnNotificationRobotEventTopic(std::function<void(RobotEventNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		void Stop(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> Stop_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void Stop_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		Pose GetMeasuredCartesianPose(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Pose> GetMeasuredCartesianPose_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetMeasuredCartesianPose_callback(std::function<void(const Error&, const Pose&)> callback, uint32_t deviceId = 0);

		void SendTwistJoystickCommand(const TwistCommand& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SendTwistJoystickCommand_async(const TwistCommand& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SendTwistJoystickCommand_callback(const TwistCommand& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void SendTwistCommand(const TwistCommand& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SendTwistCommand_async(const TwistCommand& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SendTwistCommand_callback(const TwistCommand& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		JointAngles GetMeasuredJointAngles(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<JointAngles> GetMeasuredJointAngles_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetMeasuredJointAngles_callback(std::function<void(const Error&, const JointAngles&)> callback, uint32_t deviceId = 0);

		void SendJointSpeedsCommand(const JointSpeeds& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SendJointSpeedsCommand_async(const JointSpeeds& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SendJointSpeedsCommand_callback(const JointSpeeds& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void SendSelectedJointSpeedCommand(const JointSpeed& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SendSelectedJointSpeedCommand_async(const JointSpeed& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SendSelectedJointSpeedCommand_callback(const JointSpeed& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void ApplyQuickStop(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> ApplyQuickStop_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ApplyQuickStop_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void ClearFaults(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> ClearFaults_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ClearFaults_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void SetServoingMode(const ServoingModeInformation& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetServoingMode_async(const ServoingModeInformation& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetServoingMode_callback(const ServoingModeInformation& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		ServoingModeInformation GetServoingMode(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ServoingModeInformation> GetServoingMode_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetServoingMode_callback(std::function<void(const Error&, const ServoingModeInformation&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationServoingModeTopic(std::function<void(ServoingModeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		void Reboot(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> Reboot_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void Reboot_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		ActuatorInformation GetActuatorCount(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ActuatorInformation> GetActuatorCount_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetActuatorCount_callback(std::function<void(const Error&, const ActuatorInformation&)> callback, uint32_t deviceId = 0);

		ArmStateInformation GetArmState(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ArmStateInformation> GetArmState_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetArmState_callback(std::function<void(const Error&, const ArmStateInformation&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationArmStateTopic(std::function<void(ArmStateNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		IPv4Information GetIPv4Information(const NetworkHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<IPv4Information> GetIPv4Information_async(const NetworkHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetIPv4Information_callback(const NetworkHandle& message, std::function<void(const Error&, const IPv4Information&)> callback, uint32_t deviceId = 0);

		void SendJointSpeedsJoystickCommand(const JointSpeeds& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SendJointSpeedsJoystickCommand_async(const JointSpeeds& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SendJointSpeedsJoystickCommand_callback(const JointSpeeds& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void SendSelectedJointSpeedJoystickCommand(const JointSpeed& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SendSelectedJointSpeedJoystickCommand_async(const JointSpeed& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SendSelectedJointSpeedJoystickCommand_callback(const JointSpeed& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		Kinova::Api::ProductConfiguration::CompleteProductConfiguration GetProductConfiguration(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::ProductConfiguration::CompleteProductConfiguration> GetProductConfiguration_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetProductConfiguration_callback(std::function<void(const Error&, const Kinova::Api::ProductConfiguration::CompleteProductConfiguration&)> callback, uint32_t deviceId = 0);

		TrajectoryErrorReport GetTrajectoryErrorReport(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<TrajectoryErrorReport> GetTrajectoryErrorReport_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetTrajectoryErrorReport_callback(std::function<void(const Error&, const TrajectoryErrorReport&)> callback, uint32_t deviceId = 0);

		FirmwareBundleVersions GetFirmwareBundleVersions(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<FirmwareBundleVersions> GetFirmwareBundleVersions_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetFirmwareBundleVersions_callback(std::function<void(const Error&, const FirmwareBundleVersions&)> callback, uint32_t deviceId = 0);

		void ExecuteWaypointTrajectory(const WaypointList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> ExecuteWaypointTrajectory_async(const WaypointList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ExecuteWaypointTrajectory_callback(const WaypointList& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		WaypointValidationReport ValidateWaypointList(const WaypointList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<WaypointValidationReport> ValidateWaypointList_async(const WaypointList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ValidateWaypointList_callback(const WaypointList& message, std::function<void(const Error&, const WaypointValidationReport&)> callback, uint32_t deviceId = 0);

		Pose ComputeForwardKinematics(const JointAngles& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Pose> ComputeForwardKinematics_async(const JointAngles& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ComputeForwardKinematics_callback(const JointAngles& message, std::function<void(const Error&, const Pose&)> callback, uint32_t deviceId = 0);

		JointAngles ComputeInverseKinematics(const IKData& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<JointAngles> ComputeInverseKinematics_async(const IKData& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ComputeInverseKinematics_callback(const IKData& message, std::function<void(const Error&, const JointAngles&)> callback, uint32_t deviceId = 0);

		void ActivateRobot(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> ActivateRobot_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ActivateRobot_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationUpdatingModeTopic(std::function<void(UpdatingModeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		DEPRECATED_MSG("This RPC was replaced by UpgradeSWU located in SoftwareUpdate service.") void SetUpdatingMode(const UpdatingModeInformation& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		DEPRECATED_MSG("This RPC was replaced by UpgradeSWU located in SoftwareUpdate service.") std::future<void> SetUpdatingMode_async(const UpdatingModeInformation& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		DEPRECATED_MSG("This RPC was replaced by UpgradeSWU located in SoftwareUpdate service.") void SetUpdatingMode_callback(const UpdatingModeInformation& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		UpdatingModeInformation GetUpdatingMode(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<UpdatingModeInformation> GetUpdatingMode_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetUpdatingMode_callback(std::function<void(const Error&, const UpdatingModeInformation&)> callback, uint32_t deviceId = 0);

		void SelectOperatingMode(const Kinova::Api::Common::ModeSelection& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SelectOperatingMode_async(const Kinova::Api::Common::ModeSelection& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SelectOperatingMode_callback(const Kinova::Api::Common::ModeSelection& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void ExitRecoveryState(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> ExitRecoveryState_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ExitRecoveryState_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void DeactivateRobot(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> DeactivateRobot_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void DeactivateRobot_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::ModeSelection GetCurrentOperatingMode(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::Common::ModeSelection> GetCurrentOperatingMode_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetCurrentOperatingMode_callback(std::function<void(const Error&, const Kinova::Api::Common::ModeSelection&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::UserRoleList ReadAllUserRoles(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::Common::UserRoleList> ReadAllUserRoles_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ReadAllUserRoles_callback(std::function<void(const Error&, const Kinova::Api::Common::UserRoleList&)> callback, uint32_t deviceId = 0);

		void SetHandGuidingMode(const HandGuiding& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetHandGuidingMode_async(const HandGuiding& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetHandGuidingMode_callback(const HandGuiding& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		HandGuiding GetHandGuidingMode(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<HandGuiding> GetHandGuidingMode_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetHandGuidingMode_callback(std::function<void(const Error&, const HandGuiding&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationHandGuidingModeTopic(std::function<void(HandGuidingModeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		Kinova::Api::Common::NotificationHandle OnNotificationEnablingDeviceTopic(std::function<void(EnablingDeviceNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		Kinova::Api::Common::NotificationHandle OnNotificationMotionTopic(std::function<void(MotionNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		JointAngles GetLastRecordedJointAngles(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<JointAngles> GetLastRecordedJointAngles_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetLastRecordedJointAngles_callback(std::function<void(const Error&, const JointAngles&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationProgramRequestTopic(std::function<void(ProgramRequestNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		DEPRECATED_MSG("This RPC was moved to Storage service.") Kinova::Api::Common::StorageMountPointList GetStorageList(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		DEPRECATED_MSG("This RPC was moved to Storage service.") std::future<Kinova::Api::Common::StorageMountPointList> GetStorageList_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		DEPRECATED_MSG("This RPC was moved to Storage service.") void GetStorageList_callback(std::function<void(const Error&, const Kinova::Api::Common::StorageMountPointList&)> callback, uint32_t deviceId = 0);

		DEPRECATED_MSG("This RPC was moved to Storage service and renamed to GetFileSystemList.") Kinova::Api::Common::FSItemList FileSystemList(const FSListArgs& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		DEPRECATED_MSG("This RPC was moved to Storage service and renamed to GetFileSystemList.") std::future<Kinova::Api::Common::FSItemList> FileSystemList_async(const FSListArgs& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		DEPRECATED_MSG("This RPC was moved to Storage service and renamed to GetFileSystemList.") void FileSystemList_callback(const FSListArgs& message, std::function<void(const Error&, const Kinova::Api::Common::FSItemList&)> callback, uint32_t deviceId = 0);

		DEPRECATED_MSG("This RPC was moved to Storage service.") void FileWrite(const FSWriteArgs& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		DEPRECATED_MSG("This RPC was moved to Storage service.") std::future<void> FileWrite_async(const FSWriteArgs& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		DEPRECATED_MSG("This RPC was moved to Storage service.") void FileWrite_callback(const FSWriteArgs& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		DEPRECATED_MSG("This RPC was moved to Storage service.") FSReadData FileRead(const FSReadArgs& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		DEPRECATED_MSG("This RPC was moved to Storage service.") std::future<FSReadData> FileRead_async(const FSReadArgs& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		DEPRECATED_MSG("This RPC was moved to Storage service.") void FileRead_callback(const FSReadArgs& message, std::function<void(const Error&, const FSReadData&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationAcknowledgeActionTopic(std::function<void(AcknowledgeActionNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});


	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif
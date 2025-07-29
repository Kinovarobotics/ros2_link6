#ifndef __CONTROLCONFIGCLIENT_H__
#define __CONTROLCONFIGCLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "ControlConfig.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace ControlConfig
{
	enum FunctionUids
	{
			eUidSetGravityVector = 0x100001,
			eUidGetGravityVector = 0x100002,
			eUidSetPayloadInformation = 0x100003,
			eUidGetPayloadInformation = 0x100004,
			eUidSetToolConfiguration = 0x100005,
			eUidGetToolConfiguration = 0x100006,
			eUidControlConfigurationTopic = 0x100007,
			eUidUnsubscribe = 0x100008,
			eUidSetCartesianReferenceFrame = 0x100009,
			eUidGetCartesianReferenceFrame = 0x10000a,
			eUidSetLockedCartesianAxes = 0x10000b,
			eUidGetLockedCartesianAxes = 0x10000c,
			eUidGetControlMode = 0x10000d,
			eUidSetJointSpeedSoftLimits = 0x10000e,
			eUidSetTwistLinearSoftLimit = 0x10000f,
			eUidSetTwistAngularSoftLimit = 0x100010,
			eUidSetJointAccelerationSoftLimits = 0x100011,
			eUidGetKinematicHardLimits = 0x100012,
			eUidGetKinematicSoftLimits = 0x100013,
			eUidGetAllKinematicSoftLimits = 0x100014,
			eUidSetDesiredLinearTwist = 0x100015,
			eUidSetDesiredAngularTwist = 0x100016,
			eUidSetDesiredJointSpeeds = 0x100017,
			eUidGetDesiredSpeeds = 0x100018,
			eUidResetGravityVector = 0x100019,
			eUidResetPayloadInformation = 0x10001a,
			eUidResetToolConfiguration = 0x10001b,
			eUidResetJointSpeedSoftLimits = 0x10001c,
			eUidResetTwistLinearSoftLimit = 0x10001d,
			eUidResetTwistAngularSoftLimit = 0x10001e,
			eUidResetJointAccelerationSoftLimits = 0x10001f,
			eUidControlModeTopic = 0x100020,
			eUidSetJointPositionSoftLimits = 0x100021,
			eUidResetJointPositionSoftLimits = 0x100022,
			eUidGetSoftLimitsRange = 0x100023,
			eUidSetToolConfigurationList = 0x100024,
			eUidZeroExternalWrenchFromFTSensor = 0x100027,
			eUidResetExternalWrenchFromFTSensor = 0x100028,
			eUidSetTcpTranslationSpeedSaturation = 0x100029,
			eUidGetTcpTranslationSpeedSaturation = 0x10002a,
			eUidGetCollisionDetectionRangesList = 0x10002b,
			eUidSetAllCollisionDetectionLimits = 0x10002c,
			eUidGetAllCollisionDetectionLimits = 0x10002d,
			eUidGetEnergyLimitsRangesList = 0x10002e,
			eUidSetAllEnergyLimits = 0x10002f,
			eUidGetAllEnergyLimits = 0x100030,
			eUidArmSpeedFactorTopic = 0x100031,
			eUidSetArmSpeedFactor = 0x100032,
			eUidGetArmSpeedFactor = 0x100033,
	};
	
	class ControlConfigClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdControlConfig;
	public:
		ControlConfigClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		void SetGravityVector(const GravityVector& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetGravityVector_async(const GravityVector& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetGravityVector_callback(const GravityVector& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		GravityVector GetGravityVector(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<GravityVector> GetGravityVector_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetGravityVector_callback(std::function<void(const Error&, const GravityVector&)> callback, uint32_t deviceId = 0);

		void SetPayloadInformation(const PayloadInformation& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetPayloadInformation_async(const PayloadInformation& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetPayloadInformation_callback(const PayloadInformation& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		PayloadInformation GetPayloadInformation(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<PayloadInformation> GetPayloadInformation_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetPayloadInformation_callback(std::function<void(const Error&, const PayloadInformation&)> callback, uint32_t deviceId = 0);

		ToolConfiguration GetToolConfiguration(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ToolConfiguration> GetToolConfiguration_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetToolConfiguration_callback(std::function<void(const Error&, const ToolConfiguration&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationControlConfigurationTopic(std::function<void(ControlConfigurationNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		void Unsubscribe(const Kinova::Api::Common::NotificationHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetCartesianReferenceFrame(const CartesianReferenceFrameInfo& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetCartesianReferenceFrame_async(const CartesianReferenceFrameInfo& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetCartesianReferenceFrame_callback(const CartesianReferenceFrameInfo& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		CartesianReferenceFrameInfo GetCartesianReferenceFrame(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<CartesianReferenceFrameInfo> GetCartesianReferenceFrame_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetCartesianReferenceFrame_callback(std::function<void(const Error&, const CartesianReferenceFrameInfo&)> callback, uint32_t deviceId = 0);

		void SetLockedCartesianAxes(const AxisLockConfig& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetLockedCartesianAxes_async(const AxisLockConfig& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetLockedCartesianAxes_callback(const AxisLockConfig& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		AxisLockConfig GetLockedCartesianAxes(const ControlModeInformation& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<AxisLockConfig> GetLockedCartesianAxes_async(const ControlModeInformation& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetLockedCartesianAxes_callback(const ControlModeInformation& message, std::function<void(const Error&, const AxisLockConfig&)> callback, uint32_t deviceId = 0);

		ControlModeInformation GetControlMode(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ControlModeInformation> GetControlMode_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetControlMode_callback(std::function<void(const Error&, const ControlModeInformation&)> callback, uint32_t deviceId = 0);

		KinematicLimits GetKinematicHardLimits(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<KinematicLimits> GetKinematicHardLimits_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetKinematicHardLimits_callback(std::function<void(const Error&, const KinematicLimits&)> callback, uint32_t deviceId = 0);

		void SetDesiredLinearTwist(const LinearTwist& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetDesiredLinearTwist_async(const LinearTwist& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetDesiredLinearTwist_callback(const LinearTwist& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void SetDesiredAngularTwist(const AngularTwist& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetDesiredAngularTwist_async(const AngularTwist& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetDesiredAngularTwist_callback(const AngularTwist& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void SetDesiredJointSpeeds(const JointSpeeds& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetDesiredJointSpeeds_async(const JointSpeeds& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetDesiredJointSpeeds_callback(const JointSpeeds& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		DesiredSpeeds GetDesiredSpeeds(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<DesiredSpeeds> GetDesiredSpeeds_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetDesiredSpeeds_callback(std::function<void(const Error&, const DesiredSpeeds&)> callback, uint32_t deviceId = 0);

		GravityVector ResetGravityVector(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<GravityVector> ResetGravityVector_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ResetGravityVector_callback(std::function<void(const Error&, const GravityVector&)> callback, uint32_t deviceId = 0);

		PayloadInformation ResetPayloadInformation(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<PayloadInformation> ResetPayloadInformation_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ResetPayloadInformation_callback(std::function<void(const Error&, const PayloadInformation&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationControlModeTopic(std::function<void(ControlModeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		void ZeroExternalWrenchFromFTSensor(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> ZeroExternalWrenchFromFTSensor_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ZeroExternalWrenchFromFTSensor_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void ResetExternalWrenchFromFTSensor(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> ResetExternalWrenchFromFTSensor_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ResetExternalWrenchFromFTSensor_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		DEPRECATED_MSG("Replaced by GetArmSpeedFactor") LinearTwist GetTcpTranslationSpeedSaturation(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		DEPRECATED_MSG("Replaced by GetArmSpeedFactor") std::future<LinearTwist> GetTcpTranslationSpeedSaturation_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		DEPRECATED_MSG("Replaced by GetArmSpeedFactor") void GetTcpTranslationSpeedSaturation_callback(std::function<void(const Error&, const LinearTwist&)> callback, uint32_t deviceId = 0);

		CollisionDetectionRangesList GetCollisionDetectionRangesList(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<CollisionDetectionRangesList> GetCollisionDetectionRangesList_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetCollisionDetectionRangesList_callback(std::function<void(const Error&, const CollisionDetectionRangesList&)> callback, uint32_t deviceId = 0);

		void SetAllCollisionDetectionLimits(const CollisionDetectionLimitsList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetAllCollisionDetectionLimits_async(const CollisionDetectionLimitsList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetAllCollisionDetectionLimits_callback(const CollisionDetectionLimitsList& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		CollisionDetectionLimitsList GetAllCollisionDetectionLimits(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<CollisionDetectionLimitsList> GetAllCollisionDetectionLimits_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetAllCollisionDetectionLimits_callback(std::function<void(const Error&, const CollisionDetectionLimitsList&)> callback, uint32_t deviceId = 0);

		EnergyLimitsRangesList GetEnergyLimitsRangesList(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<EnergyLimitsRangesList> GetEnergyLimitsRangesList_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetEnergyLimitsRangesList_callback(std::function<void(const Error&, const EnergyLimitsRangesList&)> callback, uint32_t deviceId = 0);

		void SetAllEnergyLimits(const EnergyLimitsList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetAllEnergyLimits_async(const EnergyLimitsList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetAllEnergyLimits_callback(const EnergyLimitsList& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		EnergyLimitsList GetAllEnergyLimits(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<EnergyLimitsList> GetAllEnergyLimits_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetAllEnergyLimits_callback(std::function<void(const Error&, const EnergyLimitsList&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationArmSpeedFactorTopic(std::function<void(ArmSpeedFactorNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		void SetArmSpeedFactor(const DesiredArmSpeedFactor& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetArmSpeedFactor_async(const DesiredArmSpeedFactor& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetArmSpeedFactor_callback(const DesiredArmSpeedFactor& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		DesiredArmSpeedFactor GetArmSpeedFactor(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<DesiredArmSpeedFactor> GetArmSpeedFactor_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetArmSpeedFactor_callback(std::function<void(const Error&, const DesiredArmSpeedFactor&)> callback, uint32_t deviceId = 0);


	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif
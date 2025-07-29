#ifndef __SAFETYFUNCTIONSCLIENT_H__
#define __SAFETYFUNCTIONSCLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "SafetyFunctions.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace SafetyFunctions
{
	enum FunctionUids
	{
			eUidIsSupported = 0x290001,
			eUidSetJointPositionLimits = 0x290002,
			eUidSetAllJointPositionLimits = 0x290003,
			eUidGetJointPositionLimits = 0x290004,
			eUidGetAllJointPositionLimits = 0x290005,
			eUidSetJointSpeedLimit = 0x290006,
			eUidSetAllJointSpeedLimit = 0x290007,
			eUidGetJointSpeedLimit = 0x290008,
			eUidGetAllJointSpeedLimit = 0x290009,
			eUidUnsubscribe = 0x29000a,
			eUidSafetyFunctionChangeTopic = 0x29000b,
			eUidGetSafetyFunctionsStatus = 0x29000c,
			eUidSetTcpSpeedLimits = 0x29000d,
			eUidSetAllTcpSpeedLimits = 0x29000e,
			eUidGetTcpSpeedLimits = 0x29000f,
			eUidGetAllTcpSpeedLimits = 0x290010,
			eUidSetTcpForceLimit = 0x290011,
			eUidSetAllTcpForceLimit = 0x290012,
			eUidGetTcpForceLimit = 0x290013,
			eUidGetAllTcpForceLimit = 0x290014,
			eUidSetTcpOrientationLimits = 0x290015,
			eUidSetAllTcpOrientationLimits = 0x290016,
			eUidGetTcpOrientationLimits = 0x290017,
			eUidGetAllTcpOrientationLimits = 0x290018,
			eUidSetElbowSpeedLimit = 0x290019,
			eUidSetAllElbowSpeedLimit = 0x29001a,
			eUidGetElbowSpeedLimit = 0x29001b,
			eUidGetAllElbowSpeedLimit = 0x29001c,
			eUidSetElbowForceLimit = 0x29001d,
			eUidSetAllElbowForceLimit = 0x29001e,
			eUidGetElbowForceLimit = 0x29001f,
			eUidGetAllElbowForceLimit = 0x290020,
			eUidGetSafetyFunctionsLimitsRange = 0x290021,
			eUidGetSafetyFunctionsLimits = 0x290022,
			eUidGetSafetySystemMode = 0x290023,
			eUidSetSafetySystemMode = 0x290024,
			eUidSafetyModeChangeTopic = 0x290025,
			eUidGetProtectiveStopStatus = 0x290026,
			eUidProtectiveStopChangeTopic = 0x290027,
	};
	
	class SafetyFunctionsClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdSafetyFunctions;
	public:
		SafetyFunctionsClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		void SetJointPositionLimits(const JointPositionLimits& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetJointPositionLimits_async(const JointPositionLimits& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetJointPositionLimits_callback(const JointPositionLimits& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void SetAllJointPositionLimits(const JointPositionLimitsList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetAllJointPositionLimits_async(const JointPositionLimitsList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetAllJointPositionLimits_callback(const JointPositionLimitsList& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		JointPositionLimits GetJointPositionLimits(const JointPositionInfo& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<JointPositionLimits> GetJointPositionLimits_async(const JointPositionInfo& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetJointPositionLimits_callback(const JointPositionInfo& message, std::function<void(const Error&, const JointPositionLimits&)> callback, uint32_t deviceId = 0);

		JointPositionLimitsList GetAllJointPositionLimits(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<JointPositionLimitsList> GetAllJointPositionLimits_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetAllJointPositionLimits_callback(std::function<void(const Error&, const JointPositionLimitsList&)> callback, uint32_t deviceId = 0);

		void SetJointSpeedLimit(const JointSpeedLimit& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetJointSpeedLimit_async(const JointSpeedLimit& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetJointSpeedLimit_callback(const JointSpeedLimit& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void SetAllJointSpeedLimit(const JointSpeedLimitList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetAllJointSpeedLimit_async(const JointSpeedLimitList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetAllJointSpeedLimit_callback(const JointSpeedLimitList& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		JointSpeedLimit GetJointSpeedLimit(const JointSpeedInfo& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<JointSpeedLimit> GetJointSpeedLimit_async(const JointSpeedInfo& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetJointSpeedLimit_callback(const JointSpeedInfo& message, std::function<void(const Error&, const JointSpeedLimit&)> callback, uint32_t deviceId = 0);

		JointSpeedLimitList GetAllJointSpeedLimit(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<JointSpeedLimitList> GetAllJointSpeedLimit_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetAllJointSpeedLimit_callback(std::function<void(const Error&, const JointSpeedLimitList&)> callback, uint32_t deviceId = 0);

		void Unsubscribe(const Kinova::Api::Common::NotificationHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		Kinova::Api::Common::NotificationHandle OnNotificationSafetyFunctionChangeTopic(std::function<void(SafetyFunctionChangeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		SafetyFunctionsStatus GetSafetyFunctionsStatus(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<SafetyFunctionsStatus> GetSafetyFunctionsStatus_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetSafetyFunctionsStatus_callback(std::function<void(const Error&, const SafetyFunctionsStatus&)> callback, uint32_t deviceId = 0);

		void SetTcpSpeedLimits(const TcpSpeedLimits& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetTcpSpeedLimits_async(const TcpSpeedLimits& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetTcpSpeedLimits_callback(const TcpSpeedLimits& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void SetAllTcpSpeedLimits(const TcpSpeedLimitsList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetAllTcpSpeedLimits_async(const TcpSpeedLimitsList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetAllTcpSpeedLimits_callback(const TcpSpeedLimitsList& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		TcpSpeedLimits GetTcpSpeedLimits(const TcpInfo& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<TcpSpeedLimits> GetTcpSpeedLimits_async(const TcpInfo& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetTcpSpeedLimits_callback(const TcpInfo& message, std::function<void(const Error&, const TcpSpeedLimits&)> callback, uint32_t deviceId = 0);

		TcpSpeedLimitsList GetAllTcpSpeedLimits(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<TcpSpeedLimitsList> GetAllTcpSpeedLimits_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetAllTcpSpeedLimits_callback(std::function<void(const Error&, const TcpSpeedLimitsList&)> callback, uint32_t deviceId = 0);

		void SetElbowSpeedLimit(const ElbowSpeedLimit& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetElbowSpeedLimit_async(const ElbowSpeedLimit& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetElbowSpeedLimit_callback(const ElbowSpeedLimit& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void SetAllElbowSpeedLimit(const ElbowSpeedLimitList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetAllElbowSpeedLimit_async(const ElbowSpeedLimitList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetAllElbowSpeedLimit_callback(const ElbowSpeedLimitList& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		ElbowSpeedLimit GetElbowSpeedLimit(const ElbowInfo& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ElbowSpeedLimit> GetElbowSpeedLimit_async(const ElbowInfo& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetElbowSpeedLimit_callback(const ElbowInfo& message, std::function<void(const Error&, const ElbowSpeedLimit&)> callback, uint32_t deviceId = 0);

		ElbowSpeedLimitList GetAllElbowSpeedLimit(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ElbowSpeedLimitList> GetAllElbowSpeedLimit_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetAllElbowSpeedLimit_callback(std::function<void(const Error&, const ElbowSpeedLimitList&)> callback, uint32_t deviceId = 0);

		SafetyFunctionsLimitsRange GetSafetyFunctionsLimitsRange(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<SafetyFunctionsLimitsRange> GetSafetyFunctionsLimitsRange_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetSafetyFunctionsLimitsRange_callback(std::function<void(const Error&, const SafetyFunctionsLimitsRange&)> callback, uint32_t deviceId = 0);

		SafetyFunctionsLimits GetSafetyFunctionsLimits(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<SafetyFunctionsLimits> GetSafetyFunctionsLimits_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetSafetyFunctionsLimits_callback(std::function<void(const Error&, const SafetyFunctionsLimits&)> callback, uint32_t deviceId = 0);

		SafetySystem GetSafetySystemMode(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<SafetySystem> GetSafetySystemMode_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetSafetySystemMode_callback(std::function<void(const Error&, const SafetySystem&)> callback, uint32_t deviceId = 0);

		void SetSafetySystemMode(const SafetySystem& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetSafetySystemMode_async(const SafetySystem& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetSafetySystemMode_callback(const SafetySystem& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationSafetyModeChangeTopic(std::function<void(SafetyModeChangeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		ProtectiveStopStatus GetProtectiveStopStatus(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ProtectiveStopStatus> GetProtectiveStopStatus_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetProtectiveStopStatus_callback(std::function<void(const Error&, const ProtectiveStopStatus&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationProtectiveStopChangeTopic(std::function<void(ProtectiveStopChangeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});


	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif
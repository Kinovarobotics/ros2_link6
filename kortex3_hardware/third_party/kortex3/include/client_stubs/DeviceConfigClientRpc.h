#ifndef __DEVICECONFIGCLIENT_H__
#define __DEVICECONFIGCLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "DeviceConfig.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace DeviceConfig
{
	enum FunctionUids
	{
			eUidGetRunMode = 0x90001,
			eUidSetRunMode = 0x90002,
			eUidGetDeviceType = 0x90003,
			eUidGetFirmwareVersion = 0x90004,
			eUidGetBootloaderVersion = 0x90005,
			eUidGetModelNumber = 0x90006,
			eUidGetPartNumber = 0x90007,
			eUidGetSerialNumber = 0x90008,
			eUidGetMACAddress = 0x90009,
			eUidGetIPv4Settings = 0x9000a,
			eUidSetIPv4Settings = 0x9000b,
			eUidGetPartNumberRevision = 0x9000c,
			eUidRebootRequest = 0x9000e,
			eUidSetDiagnosticEnable = 0x9000f,
			eUidSetDiagnosticErrorThreshold = 0x90010,
			eUidSetDiagnosticWarningThreshold = 0x90011,
			eUidSetDiagnosticConfiguration = 0x90012,
			eUidGetDiagnosticConfiguration = 0x90013,
			eUidGetDiagnosticInformation = 0x90014,
			eUidGetDiagnosticEnable = 0x90015,
			eUidGetDiagnosticStatus = 0x90016,
			eUidClearAllDiagnosticStatus = 0x90017,
			eUidClearDiagnosticStatus = 0x90018,
			eUidGetAllDiagnosticConfiguration = 0x90019,
			eUidGetAllDiagnosticInformation = 0x9001a,
			eUidResetDiagnosticDefaults = 0x9001b,
			eUidDiagnosticTopic = 0x9001c,
			eUidExecuteCalibration = 0x90022,
			eUidGetCalibrationResult = 0x90023,
			eUidStopCalibration = 0x90024,
	};
	
	class DeviceConfigClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdDeviceConfig;
	public:
		DeviceConfigClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		DeviceType GetDeviceType(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<DeviceType> GetDeviceType_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetDeviceType_callback(std::function<void(const Error&, const DeviceType&)> callback, uint32_t deviceId = 0);

		FirmwareVersion GetFirmwareVersion(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<FirmwareVersion> GetFirmwareVersion_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetFirmwareVersion_callback(std::function<void(const Error&, const FirmwareVersion&)> callback, uint32_t deviceId = 0);

		BootloaderVersion GetBootloaderVersion(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<BootloaderVersion> GetBootloaderVersion_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetBootloaderVersion_callback(std::function<void(const Error&, const BootloaderVersion&)> callback, uint32_t deviceId = 0);

		ModelNumber GetModelNumber(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ModelNumber> GetModelNumber_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetModelNumber_callback(std::function<void(const Error&, const ModelNumber&)> callback, uint32_t deviceId = 0);

		PartNumber GetPartNumber(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<PartNumber> GetPartNumber_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetPartNumber_callback(std::function<void(const Error&, const PartNumber&)> callback, uint32_t deviceId = 0);

		SerialNumber GetSerialNumber(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<SerialNumber> GetSerialNumber_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetSerialNumber_callback(std::function<void(const Error&, const SerialNumber&)> callback, uint32_t deviceId = 0);

		MACAddress GetMACAddress(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<MACAddress> GetMACAddress_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetMACAddress_callback(std::function<void(const Error&, const MACAddress&)> callback, uint32_t deviceId = 0);

		PartNumberRevision GetPartNumberRevision(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<PartNumberRevision> GetPartNumberRevision_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetPartNumberRevision_callback(std::function<void(const Error&, const PartNumberRevision&)> callback, uint32_t deviceId = 0);


	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif
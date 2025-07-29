#ifndef __WRISTCONFIGC61CLIENT_H__
#define __WRISTCONFIGC61CLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "WristConfigC61.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace WristConfig
{
	enum FunctionUids
	{
			eUidSetDigitalIOConfiguration = 0x2c0001,
			eUidGetDigitalIOConfiguration = 0x2c0002,
			eUidSetDigitalOutputHighState = 0x2c0003,
			eUidSetDigitalOutputLowState = 0x2c0004,
			eUidGetDigitalIOInfo = 0x2c0005,
			eUidGetAllDigitalIOInfo = 0x2c0006,
			eUidSetAnalogIOConfiguration = 0x2c0007,
			eUidGetAnalogIOConfiguration = 0x2c0008,
			eUidSetAnalogValue = 0x2c0009,
			eUidGetAnalogIOInfo = 0x2c000a,
			eUidGetAllAnalogIOInfo = 0x2c000b,
			eUidClearFaults = 0x2c000c,
			eUidGetRS485Configuration = 0x2c000d,
			eUidSetRS485Configuration = 0x2c000e,
			eUidGetPowerSupply24VState = 0x2c000f,
			eUidSetPowerSupply24VState = 0x2c0010,
			eUidGetArmSerialNumber = 0x2c0011,
	};
	
	class WristConfigC61Client : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdWristConfigC61;
	public:
		WristConfigC61Client(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		void SetDigitalIOConfiguration(const Kinova::Api::IndustrialIO::DigitalIOConfiguration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetDigitalIOConfiguration_async(const Kinova::Api::IndustrialIO::DigitalIOConfiguration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetDigitalIOConfiguration_callback(const Kinova::Api::IndustrialIO::DigitalIOConfiguration& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		Kinova::Api::IndustrialIO::DigitalIOConfiguration GetDigitalIOConfiguration(const Kinova::Api::IndustrialIO::DigitalChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::IndustrialIO::DigitalIOConfiguration> GetDigitalIOConfiguration_async(const Kinova::Api::IndustrialIO::DigitalChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetDigitalIOConfiguration_callback(const Kinova::Api::IndustrialIO::DigitalChannelIdentifier& message, std::function<void(const Error&, const Kinova::Api::IndustrialIO::DigitalIOConfiguration&)> callback, uint32_t deviceId = 0);

		void SetDigitalOutputHighState(const Kinova::Api::IndustrialIO::DigitalChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetDigitalOutputHighState_async(const Kinova::Api::IndustrialIO::DigitalChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetDigitalOutputHighState_callback(const Kinova::Api::IndustrialIO::DigitalChannelIdentifier& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void SetDigitalOutputLowState(const Kinova::Api::IndustrialIO::DigitalChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetDigitalOutputLowState_async(const Kinova::Api::IndustrialIO::DigitalChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetDigitalOutputLowState_callback(const Kinova::Api::IndustrialIO::DigitalChannelIdentifier& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		Kinova::Api::IndustrialIO::DigitalIOInfo GetDigitalIOInfo(const Kinova::Api::IndustrialIO::DigitalChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::IndustrialIO::DigitalIOInfo> GetDigitalIOInfo_async(const Kinova::Api::IndustrialIO::DigitalChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetDigitalIOInfo_callback(const Kinova::Api::IndustrialIO::DigitalChannelIdentifier& message, std::function<void(const Error&, const Kinova::Api::IndustrialIO::DigitalIOInfo&)> callback, uint32_t deviceId = 0);

		Kinova::Api::IndustrialIO::DigitalIOInfoList GetAllDigitalIOInfo(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::IndustrialIO::DigitalIOInfoList> GetAllDigitalIOInfo_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetAllDigitalIOInfo_callback(std::function<void(const Error&, const Kinova::Api::IndustrialIO::DigitalIOInfoList&)> callback, uint32_t deviceId = 0);

		void SetAnalogIOConfiguration(const Kinova::Api::IndustrialIO::AnalogIOConfiguration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetAnalogIOConfiguration_async(const Kinova::Api::IndustrialIO::AnalogIOConfiguration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetAnalogIOConfiguration_callback(const Kinova::Api::IndustrialIO::AnalogIOConfiguration& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		Kinova::Api::IndustrialIO::AnalogIOConfiguration GetAnalogIOConfiguration(const Kinova::Api::IndustrialIO::AnalogIOChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::IndustrialIO::AnalogIOConfiguration> GetAnalogIOConfiguration_async(const Kinova::Api::IndustrialIO::AnalogIOChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetAnalogIOConfiguration_callback(const Kinova::Api::IndustrialIO::AnalogIOChannelIdentifier& message, std::function<void(const Error&, const Kinova::Api::IndustrialIO::AnalogIOConfiguration&)> callback, uint32_t deviceId = 0);

		void SetAnalogValue(const Kinova::Api::IndustrialIO::AnalogOutput& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetAnalogValue_async(const Kinova::Api::IndustrialIO::AnalogOutput& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetAnalogValue_callback(const Kinova::Api::IndustrialIO::AnalogOutput& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		Kinova::Api::IndustrialIO::AnalogIOInfo GetAnalogIOInfo(const Kinova::Api::IndustrialIO::AnalogIOChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::IndustrialIO::AnalogIOInfo> GetAnalogIOInfo_async(const Kinova::Api::IndustrialIO::AnalogIOChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetAnalogIOInfo_callback(const Kinova::Api::IndustrialIO::AnalogIOChannelIdentifier& message, std::function<void(const Error&, const Kinova::Api::IndustrialIO::AnalogIOInfo&)> callback, uint32_t deviceId = 0);

		Kinova::Api::IndustrialIO::AnalogIOInfoList GetAllAnalogIOInfo(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::IndustrialIO::AnalogIOInfoList> GetAllAnalogIOInfo_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetAllAnalogIOInfo_callback(std::function<void(const Error&, const Kinova::Api::IndustrialIO::AnalogIOInfoList&)> callback, uint32_t deviceId = 0);

		void ClearFaults(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> ClearFaults_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ClearFaults_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		RS485Configuration GetRS485Configuration(const RS485DeviceIdentification& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<RS485Configuration> GetRS485Configuration_async(const RS485DeviceIdentification& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetRS485Configuration_callback(const RS485DeviceIdentification& message, std::function<void(const Error&, const RS485Configuration&)> callback, uint32_t deviceId = 0);

		void SetRS485Configuration(const RS485Configuration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetRS485Configuration_async(const RS485Configuration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetRS485Configuration_callback(const RS485Configuration& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		PowerSupply24VState GetPowerSupply24VState(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<PowerSupply24VState> GetPowerSupply24VState_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetPowerSupply24VState_callback(std::function<void(const Error&, const PowerSupply24VState&)> callback, uint32_t deviceId = 0);

		void SetPowerSupply24VState(const PowerSupply24VState& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetPowerSupply24VState_async(const PowerSupply24VState& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetPowerSupply24VState_callback(const PowerSupply24VState& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		Kinova::Api::DeviceConfig::SerialNumber GetArmSerialNumber(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::DeviceConfig::SerialNumber> GetArmSerialNumber_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetArmSerialNumber_callback(std::function<void(const Error&, const Kinova::Api::DeviceConfig::SerialNumber&)> callback, uint32_t deviceId = 0);


	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif
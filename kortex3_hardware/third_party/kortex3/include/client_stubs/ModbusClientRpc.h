#ifndef __MODBUSCLIENT_H__
#define __MODBUSCLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "Modbus.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace Modbus
{
	enum FunctionUids
	{
			eUidReadCoils = 0x200001,
			eUidReadDiscreteInputs = 0x200002,
			eUidReadHoldingRegisters = 0x200003,
			eUidReadInputRegisters = 0x200004,
			eUidWriteSingleCoil = 0x200005,
			eUidWriteSingleHoldRegister = 0x200006,
			eUidWriteMultipleCoils = 0x200007,
			eUidWriteMultipleHoldRegisters = 0x200008,
			eUidInitConnection = 0x200009,
			eUidCloseConnection = 0x20000b,
	};
	
	class ModbusClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdModbus;
	public:
		ModbusClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		ReadCoilResponse ReadCoils(const ReadRequest& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ReadCoilResponse> ReadCoils_async(const ReadRequest& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ReadCoils_callback(const ReadRequest& message, std::function<void(const Error&, const ReadCoilResponse&)> callback, uint32_t deviceId = 0);

		ReadCoilResponse ReadDiscreteInputs(const ReadRequest& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ReadCoilResponse> ReadDiscreteInputs_async(const ReadRequest& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ReadDiscreteInputs_callback(const ReadRequest& message, std::function<void(const Error&, const ReadCoilResponse&)> callback, uint32_t deviceId = 0);

		ReadRegisterResponse ReadHoldingRegisters(const ReadRequest& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ReadRegisterResponse> ReadHoldingRegisters_async(const ReadRequest& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ReadHoldingRegisters_callback(const ReadRequest& message, std::function<void(const Error&, const ReadRegisterResponse&)> callback, uint32_t deviceId = 0);

		ReadRegisterResponse ReadInputRegisters(const ReadRequest& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ReadRegisterResponse> ReadInputRegisters_async(const ReadRequest& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ReadInputRegisters_callback(const ReadRequest& message, std::function<void(const Error&, const ReadRegisterResponse&)> callback, uint32_t deviceId = 0);

		void WriteSingleCoil(const SingleCoilWriteRequest& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> WriteSingleCoil_async(const SingleCoilWriteRequest& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void WriteSingleCoil_callback(const SingleCoilWriteRequest& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void WriteSingleHoldRegister(const SingleRegisterWriteRequest& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> WriteSingleHoldRegister_async(const SingleRegisterWriteRequest& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void WriteSingleHoldRegister_callback(const SingleRegisterWriteRequest& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void WriteMultipleCoils(const MultipleCoilsWriteRequest& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> WriteMultipleCoils_async(const MultipleCoilsWriteRequest& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void WriteMultipleCoils_callback(const MultipleCoilsWriteRequest& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void WriteMultipleHoldRegisters(const MultipleRegistersWriteRequest& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> WriteMultipleHoldRegisters_async(const MultipleRegistersWriteRequest& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void WriteMultipleHoldRegisters_callback(const MultipleRegistersWriteRequest& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		ClientHandle InitConnection(const ConnectionParameters& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ClientHandle> InitConnection_async(const ConnectionParameters& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void InitConnection_callback(const ConnectionParameters& message, std::function<void(const Error&, const ClientHandle&)> callback, uint32_t deviceId = 0);

		void CloseConnection(const ClientHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> CloseConnection_async(const ClientHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void CloseConnection_callback(const ClientHandle& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);


	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif
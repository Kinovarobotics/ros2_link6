#ifndef __PROGRAMRUNNERCLIENT_H__
#define __PROGRAMRUNNERCLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "ProgramRunner.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace ProgramRunner
{
	enum FunctionUids
	{
			eUidCreateProgram = 0x230001,
			eUidDeleteProgram = 0x230002,
			eUidReadAllPrograms = 0x230004,
			eUidReadProgram = 0x230005,
			eUidUpdateProgram = 0x230006,
			eUidExportProgram = 0x230007,
			eUidImportProgram = 0x230008,
			eUidGetStatus = 0x230009,
			eUidPause = 0x23000a,
			eUidResume = 0x23000b,
			eUidStart = 0x23000c,
			eUidStop = 0x23000d,
			eUidStateChangeTopic = 0x23000e,
			eUidStatusChangeTopic = 0x23000f,
			eUidConfigurationChangeTopic = 0x230010,
			eUidExecutionEventTopic = 0x230011,
			eUidUnsubscribe = 0x230012,
			eUidStartActions = 0x230013,
			eUidValidateProgram = 0x230014,
			eUidMigrateProgram = 0x230015,
			eUidMigrateAllPrograms = 0x230016,
			eUidGetLastBreakingChangeVersions = 0x230017,
	};
	
	class ProgramRunnerClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdProgramRunner;
	public:
		ProgramRunnerClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		Kinova::Api::Common::ProgramHandle CreateProgram(const Kinova::Api::ProgramConfig::Program& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::Common::ProgramHandle> CreateProgram_async(const Kinova::Api::ProgramConfig::Program& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void CreateProgram_callback(const Kinova::Api::ProgramConfig::Program& message, std::function<void(const Error&, const Kinova::Api::Common::ProgramHandle&)> callback, uint32_t deviceId = 0);

		void DeleteProgram(const Kinova::Api::Common::ProgramHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> DeleteProgram_async(const Kinova::Api::Common::ProgramHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void DeleteProgram_callback(const Kinova::Api::Common::ProgramHandle& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		Kinova::Api::ProgramConfig::ProgramList ReadAllPrograms(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::ProgramConfig::ProgramList> ReadAllPrograms_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ReadAllPrograms_callback(std::function<void(const Error&, const Kinova::Api::ProgramConfig::ProgramList&)> callback, uint32_t deviceId = 0);

		Kinova::Api::ProgramConfig::Program ReadProgram(const Kinova::Api::Common::ProgramHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::ProgramConfig::Program> ReadProgram_async(const Kinova::Api::Common::ProgramHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ReadProgram_callback(const Kinova::Api::Common::ProgramHandle& message, std::function<void(const Error&, const Kinova::Api::ProgramConfig::Program&)> callback, uint32_t deviceId = 0);

		Kinova::Api::ProgramConfig::Program UpdateProgram(const Kinova::Api::ProgramConfig::Program& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::ProgramConfig::Program> UpdateProgram_async(const Kinova::Api::ProgramConfig::Program& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void UpdateProgram_callback(const Kinova::Api::ProgramConfig::Program& message, std::function<void(const Error&, const Kinova::Api::ProgramConfig::Program&)> callback, uint32_t deviceId = 0);

		Kinova::Api::ProgramConfig::ProgramJSON ExportProgram(const Kinova::Api::Common::ProgramHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::ProgramConfig::ProgramJSON> ExportProgram_async(const Kinova::Api::Common::ProgramHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ExportProgram_callback(const Kinova::Api::Common::ProgramHandle& message, std::function<void(const Error&, const Kinova::Api::ProgramConfig::ProgramJSON&)> callback, uint32_t deviceId = 0);

		ImportProgramResult ImportProgram(const Kinova::Api::ProgramConfig::ProgramJSON& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ImportProgramResult> ImportProgram_async(const Kinova::Api::ProgramConfig::ProgramJSON& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ImportProgram_callback(const Kinova::Api::ProgramConfig::ProgramJSON& message, std::function<void(const Error&, const ImportProgramResult&)> callback, uint32_t deviceId = 0);

		StatusInformation GetStatus(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<StatusInformation> GetStatus_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetStatus_callback(std::function<void(const Error&, const StatusInformation&)> callback, uint32_t deviceId = 0);

		void Pause(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> Pause_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void Pause_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void Resume(const RunnableHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> Resume_async(const RunnableHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void Resume_callback(const RunnableHandle& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void Start(const ProgramStartConfiguration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> Start_async(const ProgramStartConfiguration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void Start_callback(const ProgramStartConfiguration& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void Stop(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> Stop_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void Stop_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationStateChangeTopic(std::function<void(StateChangeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		Kinova::Api::Common::NotificationHandle OnNotificationStatusChangeTopic(std::function<void(StatusChangeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		Kinova::Api::Common::NotificationHandle OnNotificationConfigurationChangeTopic(std::function<void(Kinova::Api::ProgramConfig::ConfigurationChangeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		Kinova::Api::Common::NotificationHandle OnNotificationExecutionEventTopic(std::function<void(ExecutionEventNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		void Unsubscribe(const Kinova::Api::Common::NotificationHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void StartActions(const ActionsStartConfiguration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> StartActions_async(const ActionsStartConfiguration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void StartActions_callback(const ActionsStartConfiguration& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void ValidateProgram(const ProgramValidationConfiguration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> ValidateProgram_async(const ProgramValidationConfiguration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ValidateProgram_callback(const ProgramValidationConfiguration& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void MigrateProgram(const Kinova::Api::Common::ProgramHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> MigrateProgram_async(const Kinova::Api::Common::ProgramHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void MigrateProgram_callback(const Kinova::Api::Common::ProgramHandle& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void MigrateAllPrograms(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> MigrateAllPrograms_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void MigrateAllPrograms_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);


	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif
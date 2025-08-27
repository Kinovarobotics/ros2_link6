#ifndef _PLUGIN_UTILITY_H_
#define _PLUGIN_UTILITY_H_

#include <kortex_plugin_sdk/utils/ErrorHandler.h>
#include <Common.pb.h>
#include <ToolPlugin.pb.h>
#include <VariableManagerClientRpc.h>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <vector>

namespace Kinova
{
namespace Api
{
namespace Utilities
{

static constexpr auto kortex_variables_field_name = "__variables__";

nlohmann::ordered_json ParseSchemaFromFile(const std::filesystem::path& path);

/**
 * @brief Validate a JSON document against a JSON Schema. This overload throws when the validation fails.
 * 
 * @param json_value JSON document to validate.
 * @param json_schema JSON Schema to validate against.
 */
void ValidateJsonData(const nlohmann::json& json_value, const nlohmann::ordered_json& json_schema);

/**
 * @brief Validate a JSON document against a JSON Schema. This overload does not throw and will fill the CustomErrorHandler.
 * 
 * @param json_value JSON document to validate.
 * @param json_schema JSON Schema to validate against.
 * @param err Custom error handler for the validation error. Contains the details of the validation failure.
 * @return true If the validation passed.
 * @return false If the validation failed.
 */
bool ValidateJsonData(const nlohmann::json& json_value, const nlohmann::ordered_json& json_schema, CustomErrorHandler& err);

Common::Timestamp GetTimestamp();

std::vector<std::string> GetFilesFromFolder(const std::filesystem::path& path);

/**
 * @brief Detect a Kortex Runner variable in a JSON document.
 * 
 * @param j JSON document to check for a variable in
 * @return true If a variable is present
 * @return false If no variable is present
 */
bool ContainsKortexRunnerVariable(const nlohmann::json& j);

/**
 * @brief Get the Kortex Runner Variable from a JSON Field, without the "${}"
 * 
 * @param j JSON Field in which to search
 * @return std::string The variable
 */
std::string GetKortexRunnerVariableFromJSONField(const nlohmann::json& j);

/**
 * @brief Resolves variables received from Web interface in JSON
 * 
 * @param j JSON document in which properties may contain variables as values. E.g.: "any_property": "${any_variable_name}"
 * @param variable_manager_client Variable manager client which will call the Interpolate RPC
 * @param handle Program handle to resolve with
 * @throw Will throw if a variable cannot be interpolated.
 * @return nlohmann::json The JSON document with interpolated values. The variables field contains the JSON diff with original variables.
 */
nlohmann::json VariableManagerResolver(const nlohmann::json& j, std::shared_ptr<VariableManager::VariableManagerClient>& variable_manager_client, const Kinova::Api::Common::ProgramHandle& handle);

/**
 * @brief Returns variables back into the JSON document for Web interface in JSON
 * 
 * @param input JSON document to return variables to. The JSON diff in the variables field is used to return variables in the document.
 * @return nlohmann::json Modified JSON document with original variables.
 */
nlohmann::json VariableManagerReassembler(const nlohmann::json& input);

/**
 * @brief Add a variable to a JSON document
 * 
 * @param input JSON document to add variable to. The JSON document must have the variables field populated with a JSON diff,
 *              as returned by VariableManagerResolver and as all inputs of UI and Global Triggers.
 * @param pointer Pointer to the field where the variable is to be added.
 * @param variable Full variable expression to add, e.g. "${my_variable}" or "${my_array[${my_index}]}".
 * @return nlohmann::json Modified JSON document with removed variable in the variables field.
 */
nlohmann::json AddVariable(const nlohmann::json& input, const nlohmann::json::json_pointer& pointer, const std::string& variable);

/**
 * @brief Removes a variable from a JSON document
 * 
 * @param input JSON document to remove variable from. The JSON document must have the variables field populated with a JSON diff,
 *              as returned by VariableManagerResolver and as all inputs of UI and Global Triggers.
 * @param pointer Pointer to the field where the variable must be removed.
 * @return nlohmann::json Modified JSON document with removed variable in the variables field.
 */
nlohmann::json RemoveVariable(const nlohmann::json& input, const nlohmann::json::json_pointer& pointer);

/**
 * @brief Removes a Pose variable from a JSON document
 * 
 * @param input JSON document to remove Pose variable from. The JSON document must have the variables field populated with a JSON diff,
 *              as returned by VariableManagerResolver and as all inputs of UI and Global Triggers.
 * @param pointer Pointer to the pose object where the variable must be removed.
 * @return nlohmann::json Modified JSON document with removed variable in the variables field.
 * @exception std::invalid_argument is thrown if the JSON object pointed to is not a Pose.
 */
nlohmann::json RemovePoseVariable(const nlohmann::json& input, const nlohmann::json::json_pointer& pointer);

/**
 * @brief Removes a Pose's Position components variables from a JSON document (x, y, z)
 * 
 * @param input JSON document to remove Position variable from. The JSON document must have the variables field populated with a JSON diff,
 *              as returned by VariableManagerResolver and as all inputs of UI and Global Triggers.
 * @param pointer Pointer to the position object where the variable must be removed.
 * @return nlohmann::json Modified JSON document with removed variable in the variables field.
 * @exception std::invalid_argument is thrown if the JSON object pointed to is not a Position.
 */
nlohmann::json RemovePositionVariable(const nlohmann::json& input, const nlohmann::json::json_pointer& pointer);

/**
 * @brief Create unique name from Action's name, ActionHandle and ProgramHandle
 * 
 * @param action Action from which to extract the information
 * @return std::string Unique name
 */
std::string GetUniqueActionName(const Kinova::Api::Plugin::Action& action);

/**
 * @brief Get the Program Handle From Action Unique Name
 * 
 * @param action_unique_name Action unique name
 * @throw invalid_argument If no program identifier is found within the name, or it is malformed.
 * @return Common::ProgramHandle Program handle
 */
Common::ProgramHandle GetProgramHandleFromActionUniqueName(const std::string& action_unique_name);

} // namespace Utilities
} // namespace Api
} // namespace Kinova

#endif //_PLUGIN_UTILITY_H_
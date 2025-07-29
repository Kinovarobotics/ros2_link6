#ifndef CORE_BENCHMARKER_H
#define CORE_BENCHMARKER_H

#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <atomic>
#include <vector>
#include <queue>
#include <chrono>
#include <memory>
#include <list>

namespace Kinova::Api::Util::Benchmarking
{

/**
 * @brief Enumerator used to identify the benchmarking element type.
 * 
 */
enum class BenchmarkElementType
{
    // The entry used when Start() is called.
    Start,
    // The entry used when Step() is called.
    Step,
    // The entry used when Stop() is called.
    Stop,
    // The entry used when LogTrace() is called.
    LogTrace
};

/**
 * @brief Convert the enum BenchmarkElementType to a string.
 * 
 * @param type The value to convert.
 * @return std::string The string representing the enum.
 */
std::string BenchmarkElementTypeToString(BenchmarkElementType type);

/**
 * @brief Wrap the data string for the csv file and escapes invalid characters.
 * It basically wraps the string with double quotes (") character 
 * and escapes any double quote (") character the original string contains by adding another 
 * double quote character (" -> "").
 * 
 * @param data The data string to wrap.
 * @return std::string The wrapped data string.
 */
std::string WrapDataForCsv(const std::string& data);

/**
 * @brief Data structure to hold a single benchmark timepoint
 * 
 */
struct BenchmarkElement
{
    std::chrono::time_point<std::chrono::high_resolution_clock> timepoint;
    std::string message;
    BenchmarkElementType type;
};

/**
 * @brief Data structure to hold a full benchmark 
 * 
 */
struct Benchmark
{
    /**
     * @brief Name of the Benchmark element
     * 
     */
    std::string name;

    /**
     * @brief Identifier of the benchmarker who caught the measure
     * 
     */
    std::string benchmarker_id;

    /**
     * @brief All timepoints in the Benchmark.
     * 
     */
    std::vector<BenchmarkElement> elements;

    /**
     * @brief Set to true if this Benchmark was stopped
     * 
     */
    bool done = false;

    /**
     * @brief Computes durations from all the timepoints, in milliseconds.
     * @note  It has the same size as the number of elements and its first element is always 0.0.
     * 
     * @return std::vector<double> 
     */
    std::vector<double> ComputeDurations() const;

    /**
     * @brief Operator << to output to streams.
     * 
     * @param os Output stream
     * @param b  Benchmark to output
     * @return std::ostream& 
     */
    friend std::ostream& operator<<(std::ostream& os, const Benchmark& b);
};

/**
 * @brief Utility class used for benchmarking.
 * 
 */ 
class CoreBenchmarker
{
    private:
        std::atomic<bool> m_active;
        std::string m_identifier;

        std::unordered_map<std::string, Benchmark> m_ongoing_sections;
        mutable std::recursive_mutex m_ongoing_sections_mutex;
        std::queue<Benchmark> m_completed_sections;
        mutable std::recursive_mutex m_completed_sections_mutex;

    public:
        /**
         * @brief Construct a new Core Benchmarker object.
         * 
         * @param identifier The unique identifier that will be used in the benchmarking traces.
         * @param activate Indicates if we want to activate the benchmarking or not.
         */
        CoreBenchmarker(const std::string& identifier, bool activate = false);

        /**
         * @brief Custom copy constructor for the Core Benchmarker object.
         * 
         */
        CoreBenchmarker(const CoreBenchmarker&);

        /**
         * @brief Destroy the Core Benchmarker object.
         * 
         */
        virtual ~CoreBenchmarker();

        /**
         * @brief Get the Identifier of the CoreBenchmarker object.
         * 
         * @return const std::string Returns the identifier.
         */
        const std::string GetId() const { return m_identifier;};

        /**
         * @brief Get the Ongoing Benchmarks.
         * 
         * @return std::unordered_map<std::string, Benchmark> The map of ongoing benchmarks per section name
         */
        std::unordered_map<std::string, Benchmark> GetOngoingBenchmarks() const;

        /**
         * @brief Get the Benchmarks that are completed.
         * 
         * @return std::vector<Benchmark>  All the completed benchmarks of this benchmarker.
         */
        std::vector<Benchmark> GetCompletedBenchmarks() const;

        /**
         * @brief Get the Benchmarker's activation status 
         * 
         * @return true If active 
         * @return false If inactive
         */
        bool IsActive() const { return m_active.load();}
    
        /**
         * @brief Activates/deactivates the benchmarking.
         * 
         * @param activate If true, will activate the benchmarking, otherwise it deactivates it.
         */
        void Activate(bool activate);

        /**
         * @brief Starts the benchmarking for a unique section name.
         * 
         * @param proposed_section_name The proposed section name to benchmark, this must be unique.
         * @param message Optional message to add to this benchmark element.
         * @return const std::string Returns the unique section name that was used to start the benchmarking.
         * @note You should use the returned identifier in the future calls to Stop() and Step().
         */
        const std::string Start(const std::string& proposed_section_name, const std::string& message = "");

        /**
         * @brief Stops the benchmarking.
         * 
         * @param section_name The section name to stop benchmarking.
         * @param message Optional message to add to this benchmark element.
         */
        void Stop(const std::string& section_name, const std::string& message = "");

        /**
         * @brief Adds a step in the benchmarker for the section.
         * 
         * @param section_name The section name to add a step to the benchmarking.
         * @param message Optional message to add to this benchmark element.
         */
        void Step(const std::string& section_name, const std::string& message = "");

        /**
         * @brief Logs a trace with a specific message.
         * 
         * @param message The message to log.
         */
        void LogTrace(const std::string& message);

        /**
         * @brief This will transfer all of this benchmarker's benchmark elements to a global map
         * @pre   This will do nothing if the Benchmarker is active.
         * 
         */
        void TransferAllBenchmarksToGlobalMap();

    private:

        std::string EmplaceWithRetries(const std::string& first_section_name);

    // --------------------------------------------------
    // Static (global) benchmarkers functions and members
    
    private:

        static std::unordered_set<std::string> m_benchmarker_ids;
        static std::unordered_map<std::string, std::vector<Benchmark>> m_all_benchmarks_by_benchmarker;
        static std::recursive_mutex m_benchmarkers_mutex;
        static std::list<CoreBenchmarker*> m_benchmarkers;

    public:

        /**
         * @brief Prints to stdout the registered Benchmarkers' identifiers.
         * 
         */
        static void PrintExistingBenchmarkers();

        /**
         * @brief Clears the registered Benchmarkers' list.
         * @warning Don't use this outside of unit tests unless you absolutely now what you're doing.
         * 
         */
        static void ClearAllBenchmarkersList();

        /**
         * @brief Get all the Benchmarks by Benchmarker identifier
         * 
         * @return std::unordered_map<std::string, std::vector<Benchmark>>
         */
        static std::unordered_map<std::string, std::vector<Benchmark>> GetBenchmarksGlobalMap();

        /**
         * @brief Outputs the contents of the global benchmarks map to a CSV file using `sep` as a separator
         * 
         * @param filepath          Path where to write the file. If it exists, it will be truncated.
         * @param sep               Separator to use for the CSV.
         * @param append_timestamp  If true, will append the timestamp to the csv filename (before the extension).
         * @return std::string      Returns the new file path used (will be different than the input filepath if append_timestamp was true)
         * @post  The file has been created if it didn't exist, and has been overwritten if it existed.
         */
        static std::string OutputGlobalMapToCSVFile(const std::string& filepath, char sep = ';', bool append_timestamp = true);

        /**
         * @brief Outputs the contents of all the benchmarkers to a CSV file using `sep` as a separator
         * 
         * @param filepath          Path where to write the file. If it exists, it will be truncated.
         * @param sep               Separator to use for the CSV.
         * @param append_timestamp  If true, will append the timestamp to the csv filename (before the extension).
         * @return std::string      Returns the new file path used (will be different than the input filepath if append_timestamp was true)
         * @post  The file has been created if it didn't exist, and has been overwritten if it existed.
         *          All benchmarkers will have there benchmarking data cleared in the execution of this function.
         * @note  This will pause the benchmarking of all benchmarkers the time we write to the file 
         *          and clear all benchmarkers from there benchmarking data.
         */
        static std::string OutputAllBenchmarkingToCSVFile(const std::string& filepath, char sep = ';', bool append_timestamp = true);

        /**
         * @brief Gets the transformed output file path with or without an appended timestamp.
         * 
         * @param original_file_path    The input file path.
         * @param append_timestamp      If true, we will append the timestamp to the file name.
         * @return std::string Returns the transformed output file path.
         */
        static std::string GetOutputFilePath(const std::string& original_file_path, bool append_timestamp);

};

} // End of namespace Kinova::Api::Util::Benchmarking
#endif
#ifndef _KORTEXAPI_KORTEX_PROFILER_H
#define _KORTEXAPI_KORTEX_PROFILER_H

#include <chrono>
#include <map>
#include <vector>
#include <iostream>
#include <mutex>
#include <thread>

namespace
{
    constexpr auto KORTEX_PROFILER_VERBOSE_PREFIX = "[KORTEX PROFILER] --> ";
}

namespace Kinova::Api::Util::Profiler
{


/**
 * Profile container that tracks execution time for a tagged section of code
 */
class KortexProfile
{
    public:

    /**
     * Default constructor
     */
    KortexProfile() = default;

    /**
     * Constructor with tag name - starts the profiling timer
     *
     * @param[in] tagName Identifier for this profiling section
     */
    KortexProfile(std::string tagName): m_tagName{tagName}
    {
        m_timepoints.push_back(std::chrono::high_resolution_clock::now());
    }

    /**
     * Get the duration between the last two recorded timepoints
     *
     * @return Duration in milliseconds between the last two timepoints
     * @exception std::runtime_error if there are fewer than 2 timepoints recorded
     */
    std::chrono::duration<double, std::milli> getLastDuration()
    {
        // Last - one before last is the last duration
        if (m_timepoints.size() <= 1)
        {
            throw std::runtime_error("KortexProfile::getLastDuration : Cannot compute duration because there are not enough timepoints.");
        }
        return m_timepoints.at(m_timepoints.size() - 1) - m_timepoints.at(m_timepoints.size() - 2);;
    }

    /**
     * Stop the profiling and record the final duration
     *
     * @post Adds the current timepoint and calculates the duration from the previous timepoint
     */
    void Stop()
    {
        m_timepoints.push_back(std::chrono::high_resolution_clock::now());
        m_durations.push_back(getLastDuration().count());
    }

    /**
     * Record an intermediate profiling step
     *
     * @post Adds the current timepoint and calculates the duration from the previous timepoint
     * @note Similar to Stop() but allows continuing to add more steps
     */
    void Step()
    {
        m_timepoints.push_back(std::chrono::high_resolution_clock::now());
        m_durations.push_back(getLastDuration().count());
    }

    /**
     * Clear all recorded profiling data
     *
     * @post All timepoints and durations are removed
     */
    void ClearStepData()
    {
        m_timepoints.clear();
        m_durations.clear();
    }

    /**
     * Get the vector of all recorded durations
     *
     * @return Vector containing all durations in milliseconds
     */
    std::vector<double> GetDurations() const
    {
        return m_durations;
    }

    /**
     * Get the vector of all recorded timepoints
     *
     * @return Vector containing all high-resolution timepoints
     */
    std::vector<std::chrono::time_point<std::chrono::high_resolution_clock>> GetTimepoints() const
    {
        return m_timepoints;
    }

    /**
     * Get the tag name for this profile
     *
     * @return The identifier string for this profiling section
     */
    std::string GetTagName() const
    {
        return m_tagName;
    }

    /**
     * Stream output operator for displaying profile results
     *
     * @param[in,out] os Output stream to write to
     * @param[in] profile The KortexProfile to display
     * @return Reference to the output stream
     */
    friend std::ostream& operator<<(std::ostream& os, const KortexProfile& profile)
    {
        os << KORTEX_PROFILER_VERBOSE_PREFIX << "Result for section " << profile.m_tagName << std::endl;
        for(auto result = profile.m_durations.begin(); result != profile.m_durations.end(); ++result)
        {
            os << "\t- " << *result << " ms" << std::endl;
        }
        return os;
    }

    private:
    std::vector<double> m_durations;
    std::vector<std::chrono::time_point<std::chrono::high_resolution_clock>> m_timepoints;
    std::string m_tagName;
};

// This map holds all the profiles started with the static function profilerStart().
static std::map<std::string, KortexProfile> allProfiles;

// Mutex used to protect the allProfiles map
static std::mutex allProfilesMutex;

// This sets the verbosity level if an issue occurs.
static uint32_t m_verbosityLevel = 0;

/**
 * Static profiler utility for tracking execution time of code sections
 *
 * This class provides a global profiling system that allows starting, stopping,
 * and tracking multiple named profiling sections across the application.
 */
class KortexProfiler
{
    public:

    /**
     * Start profiling a named section
     *
     * @param[in] section_name Unique identifier for the profiling section
     * @post A new KortexProfile is created and started for this section name
     * @note If a profile with this name already exists, a warning is issued (if verbosity > 0)
     */
    static void profilerStart(const std::string& section_name)
    {
        std::lock_guard<std::mutex> guard(allProfilesMutex);
        auto result = allProfiles.emplace(section_name, KortexProfile{section_name});

        if(!result.second)
        {
            if(m_verbosityLevel > 0)
            {
                std::cerr << KORTEX_PROFILER_VERBOSE_PREFIX << "Trying to Start a profile on an already existing section name ("
                                                            << section_name << ")." << std::endl;
            }
            return;
        }
    }

    /**
     * Stop profiling a named section and optionally display results
     *
     * @param[in] section_name Identifier of the profiling section to stop
     * @param[in] display_result If true, prints the profiling results to stdout
     * @return Copy of the KortexProfile containing all recorded timing data
     * @post The profile is removed from the active profiles map
     * @note If the profile doesn't exist, returns an empty KortexProfile
     */
    static KortexProfile profilerStop(const std::string& section_name, bool display_result = true)
    {
        std::lock_guard<std::mutex> guard(allProfilesMutex);
        if ( allProfiles.find(section_name) == allProfiles.end() )
        {
            if(m_verbosityLevel > 0)
            {
                std::cerr << KORTEX_PROFILER_VERBOSE_PREFIX << "Trying to Stop a profile that does not exist ("
                                                            << section_name << ")." << std::endl;
            }
            return KortexProfile{};
        }

        allProfiles[section_name].Stop();

        if(display_result)
        {
            std::cout << allProfiles[section_name];
        }
        auto ret = allProfiles[section_name];
        allProfiles.erase(section_name);
        return ret;
    }

    /**
     * Record an intermediate profiling step for a named section
     *
     * @param[in] section_name Identifier of the profiling section
     * @post If the profile exists, adds a step timepoint; if not, creates a new profile
     * @note This allows measuring multiple sub-steps within a single profiling section
     */
    static void profilerStep(const std::string& section_name)
    {
        std::lock_guard<std::mutex> guard(allProfilesMutex);
        if ( allProfiles.find(section_name) == allProfiles.end() )
        {
            allProfiles.emplace(section_name, KortexProfile{section_name});
        }
        else
        {
            allProfiles[section_name].Step();
        }
    }

    /**
     * Set the verbosity level for profiler warnings and messages
     *
     * @param[in] level Verbosity level (0 = silent, higher values = more verbose)
     */
    static void setVerbosityLevel(const uint32_t& level)
    {
        m_verbosityLevel = level;
    }

    /**
     * Clear all recorded data for a specific profiling section
     *
     * @param[in] section_name Identifier of the profiling section to clear
     * @post All timepoints and durations for this section are removed
     * @note The profile remains in the map but with empty data
     */
    static void clearStepProfileRecord(const std::string& section_name)
    {
        std::lock_guard<std::mutex> guard(allProfilesMutex);
        allProfiles[section_name].ClearStepData();
    }

    /**
     * Clear all profiling records for all sections
     *
     * @post All profiles are removed from the map
     */
    static void clearAllProfileRecord()
    {
        std::lock_guard<std::mutex> guard(allProfilesMutex);
        allProfiles.clear();
    }

    /**
     * Display the profiling results for a specific section
     *
     * @param[in] section_name Identifier of the profiling section to display
     * @post Results are printed to stdout
     */
    static void displayStepProfileRecord(const std::string& section_name)
    {
        std::lock_guard<std::mutex> guard(allProfilesMutex);
        std::cout << allProfiles[section_name];
    }
};

} // End of namespace Kinova::Api::Util::Profiler

#endif /* _KORTEXAPI_KORTEX_PROFILER_H */

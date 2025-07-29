#ifndef SCOPED_BENCHMARKER_H
#define SCOPED_BENCHMARKER_H

#include "CoreBenchmarker.h"
#include <memory>
#include <iostream>

namespace Kinova::Api::Util::Benchmarking
{

/**
 * @brief Utility class used for scoped benchmarking.
 * It will start the benchmarking upon construction and end it when the object is destroyed.
 * 
 */
class ScopedBenchmarker
{
    private:
        // The main benchmarker object used to benchmark.
        std::shared_ptr<CoreBenchmarker> m_benchmarker;

        // The section name associated to this scoped benchmark.
        std::string m_section_name;

        // To keep track if Stop() was called so we do not call a second time in the destructor.
        std::atomic<bool> m_was_stopped;

    public:
        /**
         * @brief Construct a new Scoped Benchmarker object
         * 
         * @param benchmarker The benchmarker object to use for benchmarking.
         * @param section_name The unique section name to use for benchmarking.
         * @param message The optional message to add in the benchmarking.
         */
        ScopedBenchmarker(std::shared_ptr<CoreBenchmarker> benchmarker, const std::string& section_name, const std::string& message = "")
        : m_benchmarker{benchmarker}
        , m_was_stopped{false}
        {
            if (m_benchmarker)
            {
                m_section_name = m_benchmarker->Start(section_name, message);
            }
            else
            {
                std::cerr << "Error: ScopedBenchmarker(): m_benchmarker is null." << std::endl;
            }
        }

        /**
         * @brief Destroy the Scoped Benchmarker object. It will stop the benchmarking for this object.
         * 
         */
        virtual ~ScopedBenchmarker()
        {
            Stop(m_section_name);
        }

        /**
         * @brief Adds a step to the benchmarking for this object.
         * @param message The optional message to add in the benchmarking.
         * 
         */
        void Step(const std::string& message = "")
        {
            if (!m_was_stopped && m_benchmarker)
            {
                m_benchmarker->Step(m_section_name, message);
            }
        }
        
        /**
         * @brief Stops the benchmarking for this object.
         * @param message The optional message to add in the benchmarking.
         * 
         */
        void Stop(const std::string& message = "")
        {
            if (!m_was_stopped && m_benchmarker)
            {
                m_benchmarker->Stop(m_section_name, message);
            }
        }

};

} // End of namespace Kinova::Api::Util::Benchmarking
#endif

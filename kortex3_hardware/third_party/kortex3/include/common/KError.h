#ifndef KINOVAERROR_H
#define KINOVAERROR_H

#include <string>
#include <sstream>

#include "Frame.pb.h"

#include "HeaderInfo.h"

namespace Kinova
{
namespace Api
{
    /**
     * Error wrapper class for Kinova API errors
     *
     * KError encapsulates error information from Kortex API calls, including
     * error codes, sub-codes, descriptions, and optional frame header information.
     * This class is used throughout the API to represent error conditions in a
     * structured way.
     *
     * @note This class is copyable and can be used in error callbacks
     */
    class KError
    {
    public:
        /**
         * Constructor with error codes and description
         *
         * @param[in] errorCode Primary error code indicating error category
         * @param[in] errorSubCode Secondary error code for more specific error information
         * @param[in] errorDescription Human-readable error description
         */
        KError(Kinova::Api::ErrorCodes errorCode, Kinova::Api::SubErrorCodes errorSubCode, std::string errorDescription);

        /**
         * Constructor with header information, error codes and description
         *
         * @param[in] header Frame header information from the message that caused the error
         * @param[in] errorCode Primary error code indicating error category
         * @param[in] errorSubCode Secondary error code for more specific error information
         * @param[in] errorDescription Human-readable error description
         */
        KError(const HeaderInfo& header, Kinova::Api::ErrorCodes errorCode, Kinova::Api::SubErrorCodes errorSubCode, std::string errorDescription);

        /**
         * Constructor from an Error protobuf message
         *
         * @param[in] error Protobuf Error message
         */
        KError(const Error& error);

        /**
         * Constructor from header and Error protobuf message
         *
         * @param[in] header Frame header information
         * @param[in] error Protobuf Error message
         */
        KError(const HeaderInfo& header, const Error& error);

        /**
         * Static factory method to create an Error protobuf message
         *
         * @param[in] errorCode Primary error code
         * @param[in] errorSubCode Secondary error code
         * @param[in] errorDescription Human-readable error description
         * @return Populated Error protobuf message
         */
        static Error fillError(Kinova::Api::ErrorCodes errorCode, Kinova::Api::SubErrorCodes errorSubCode, std::string errorDescription);

        /**
         * Get a string representation of the error
         *
         * @return Formatted string containing error code, sub-code and description
         */
        std::string toString() const;

        /**
         * Check if header information is available
         *
         * @return True if header information was provided, false otherwise
         */
        bool            isThereHeaderInfo();

        /**
         * Get the header information
         *
         * @return Header information associated with this error
         * @pre isThereHeaderInfo() must return true
         */
        HeaderInfo      getHeader();

        /**
         * Get the underlying Error protobuf message
         *
         * @return The Error protobuf message
         */
        Error           getError() const;

        // Default copy assignment operator
        KError& operator =(const KError& other) = default;

    private:
        bool            m_isThereHeaderInfo;
        HeaderInfo      m_header;
        Error           m_error;
    };

} // namespace Api
} // namespace Kinova

#endif

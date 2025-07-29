#ifndef KINOVASERVEREXCEPTION_H
#define KINOVASERVEREXCEPTION_H

#include <string>
#include <sstream>

#include "KBasicException.h"

#include <Frame.pb.h>
#include "KError.h"

#include "HeaderInfo.h"

namespace Kinova
{
namespace Api
{
    /**
     * Exception class with detailed Kortex API error information
     *
     * KDetailedException extends KBasicException to include structured error
     * information (error codes, sub-codes) from the Kortex API. This exception
     * is thrown by API methods when server-side or protocol errors occur.
     *
     * @note Use getErrorInfo() to access the detailed error information
     */
    class KDetailedException : public KBasicException
    {
    public:
        /**
         * Constructor from KError
         *
         * @param[in] error KError object containing detailed error information
         */
        KDetailedException(const KError& error);

        /**
         * Copy constructor
         *
         * @param[in] other Another KDetailedException to copy from
         */
        KDetailedException(const KDetailedException &other);

        /**
         * Get the exception message
         *
         * @return C-string containing the error message
         */
        virtual const char* what() const throw() override;

        /**
         * Get a formatted string representation of the exception
         *
         * @return String with detailed error information including codes
         */
        virtual std::string toString() override;

        /**
         * Get the detailed error information
         *
         * @return KError object containing error codes and description
         */
        KError      getErrorInfo() const { return m_error; }

    private:
        void init(const HeaderInfo& header, const Error& error);
    
        KError       m_error;
        std::string  m_errorStr;
    };

} // namespace Api
} // namespace Kinova

#endif

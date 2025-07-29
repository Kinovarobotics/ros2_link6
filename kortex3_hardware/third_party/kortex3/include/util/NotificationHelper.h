#ifndef NOTIFICATION_HELPER_H
#define NOTIFICATION_HELPER_H

#include <Common.pb.h>

#include <mutex>
#include <condition_variable>
#include <deque>
#include <vector>
#include <iterator>
#include <chrono>
#include <thread>

template <class T>
class NotificationHelper
{

public:

    NotificationHelper(bool verbose = false) : m_verbose(verbose) {}
    ~NotificationHelper() {}

    void Handler(const T& notif)
    {
        if (m_verbose)
        {
            std::cout << "Received notification with content : " << std::endl << notif.DebugString() << std::endl;
        }
        std::lock_guard<std::mutex> lk{m_mutex};
        m_notifs.push_back(notif);
    }
    
    std::vector<T> GetNotifications() const {return m_notifs;}
    size_t GetSize() const {return m_notifs.size();}

    void ClearNotifications() 
    {
        std::lock_guard<std::mutex> lk{m_mutex};
        m_notifs.clear();
    }

    void PrintNotifications() const
    {
        int i = 0;
        for (const auto n : m_notifs)
        {
            std::cout << "Notification " << i << " : " << std::endl << n.DebugString() << std::endl;
        }
    }

    Kinova::Api::Common::NotificationHandle GetHandle() const {return m_handle;}
    void SetHandle(const Kinova::Api::Common::NotificationHandle& handle) {m_handle = handle;}

    std::vector<T> WaitForMultipleNotifications(unsigned int size, uint32_t timeout_value = 10000)
    {
        if (size == 0)
        {
            throw std::invalid_argument("Size has to be > 0 in WaitForMultipleNotifications!");
        }

        _Wait(size, timeout_value);
        std::vector<T> ret;
        std::lock_guard<std::mutex> lk{m_mutex};
        std::copy(m_notifs.begin(), m_notifs.begin() + size, std::back_inserter(ret));
        m_notifs.erase(m_notifs.begin(), m_notifs.begin() + size);
        return ret;
    }

    T WaitForOneNotification(uint32_t timeout_value = 10000)
    {
        _Wait(1, timeout_value);
        const auto ret = m_notifs.front();
        std::lock_guard<std::mutex> lk{m_mutex};
        m_notifs.pop_front();
        return ret;
    }
    

private:

    void _Wait(unsigned int size, uint32_t timeout_value)
    {
        auto start_time = std::chrono::system_clock::now();
        bool timeout_reached = false;
        bool result = false;
        while (!timeout_reached && !result) 
        {
            std::unique_lock<std::mutex> lk{m_mutex};
            m_cv.wait_for(lk, std::chrono::milliseconds{LOOP_TIME},
                [this, size, start_time, timeout_value, &timeout_reached, &result]()
                {
                    auto time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time);
                    timeout_reached = time_elapsed.count() > timeout_value;
                    result = m_notifs.size() >= size;
                    return true; // the wait is in a while anyway so return true
                }
            );
        }

        // If the wait times out
        if(timeout_reached)
        {
            throw std::runtime_error("WaitForNotification timeout");
        }
    }

    std::deque<T> m_notifs;
    std::mutex m_mutex;
    std::condition_variable m_cv;
    Kinova::Api::Common::NotificationHandle m_handle;
    static constexpr std::chrono::milliseconds LOOP_TIME = std::chrono::milliseconds{20};
    bool m_verbose;
};

#endif //  NOTIFICATION_HELPER_H
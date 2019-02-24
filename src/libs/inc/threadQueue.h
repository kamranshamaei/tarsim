/**
 *
 * @file: configParser.h
 *
 * @Created on: April 20, 2018
 * @Author: Kamran Shamaei
 *
 *
 * @brief -
 * <Requirement Doc Reference>
 * <Design Doc Reference>
 *
 * @copyright Copyright [2017-2018] Kamran Shamaei .
 * All Rights Reserved.
 *
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 */

#ifndef THREAD_QUEUE_H
#define THREAD_QUEUE_H

#include <condition_variable>
#include <deque>
#include <mutex>
#include <queue>
namespace tarsim {
template<typename Type> class ThreadQueue {
public:
    ThreadQueue(): m_mutex(), m_queue() {}
    ~ThreadQueue() = default;

    void push(const Type& item)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_queue.push_back(item);
    }

    Type pop()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        Type output(m_queue.front());
        m_queue.pop_front();
        return output;
    }

    Type front() const
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        Type output(m_queue.front());
        return output;
    }

    Type back() const
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        Type output(m_queue.back());
        return output;
    }

    bool isEmpty() const
    {
        return m_queue.empty();
    }

    void clear()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_queue.clear();
    }

    unsigned int size()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        return m_queue.size();
    }

    Type deplete()
    {
        Type output;
        std::unique_lock<std::mutex> lock(m_mutex);
        while (!m_queue.empty()) {
            output = m_queue.front();
            m_queue.pop_front();

        }
        return output;
    }


protected:
    mutable std::mutex m_mutex;
    std::deque<Type> m_queue;
};
} // end of namespace tarsim
#endif /* THREAD_QUEUE_H */

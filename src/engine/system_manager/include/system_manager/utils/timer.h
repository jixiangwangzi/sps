#ifndef _CTIMER_H_
#define _CTIMER_H_
#include <thread>                                                             
#include <chrono>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include <boost/thread.hpp>  
#include <boost/date_time/posix_time/posix_time.hpp>  

#define DEBUG
//using namespace boost;
typedef std::function<void (const boost::system::error_code&)> timer_callback;
//typedef std::function<void (const boost::system::error_code&)> task_callback;
typedef std::function<void ()> task_callback;

class CTimer{
public:
    CTimer(){}
    CTimer(int timeout, task_callback callback)
        :timeout_(timeout), tick_(timeout), task_callback_(callback), stopped_(true){
        wait_thread_ = std::make_shared<std::thread>([this]() {
#if 0
            boost::asio::io_service io;
            timer = new boost::asio::deadline_timer(io, boost::posix_time::seconds(timeout_));
            while (!stopped) {
                timer_callback callback = [&](const boost::system::error_code& err) 
                {
                    if(!stopped){
#ifdef DEBUG
                        ROS_INFO("%s, Time is due, run task here", __func__);
#endif
                        task_callback_(err); 
                        timer->expires_at(timer->expires_at() + boost::posix_time::seconds(timeout_));
                        timer->async_wait(callback);
                    }
                };

                timer->async_wait(callback);
                io.run();
            }
#else
            std::unique_lock <std::mutex> lock(mutex_);  
            while(stopped_) {
                condition_.wait(lock);
            }
            // create a time point pointing to 10 second in future
            std::chrono::system_clock::time_point timepoint =
            std::chrono::system_clock::now() + std::chrono::seconds(1);
            std::this_thread::sleep_until(timepoint);
            tick_--;
            if(tick_<=0){
                ROS_INFO("%s, time is due, go to task_callback", __func__);
                task_callback_(); 
            }
#endif
        });
#ifdef DEBUG
        ROS_INFO("CTimer::start exit");
#endif

    }
    virtual ~CTimer(){
        if (wait_thread_ && wait_thread_->joinable()) {
            wait_thread_->join();
        }
    }
private:
    std::shared_ptr<std::thread> wait_thread_;
#if 0
    boost::asio::deadline_timer *timer;
#endif
    task_callback task_callback_;
    int timeout_;
    int tick_;
    bool stopped_;
    std::mutex mutex_;
    std::condition_variable condition_;
public:
    void start(){
#ifdef DEBUG
        ROS_INFO("CTimer::start enter");
#endif
        std::unique_lock <std::mutex> lock(mutex_);
        condition_.notify_all();
        tick_ = timeout_;
        stopped_ = false;
    }

    void resume(){
        // ROS_INFO("CTimer::resume enter");
        std::unique_lock <std::mutex> lock(mutex_);
        tick_= timeout_;
    }

    void stop(){
        ROS_INFO("CTimer::stop enter");
        std::unique_lock <std::mutex> lock(mutex_);
        stopped_ = true;
#if 0
        timer->cancel();
        delete timer;
#endif
        ROS_INFO("CTimer::stop exit");
    }
};
#endif //end of define

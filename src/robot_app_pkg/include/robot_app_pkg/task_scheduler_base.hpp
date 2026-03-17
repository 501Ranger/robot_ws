#ifndef __TASK_SCHEDULER_BASE_HPP
#define __TASK_SCHEDULER_BASE_HPP

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// 任务生命周期状态
enum class TaskState { PENDING,
                       RUNNING,
                       COMPLETED,
                       FAILED };
class Task {
private:
    int task_id_;
    std::string task_name_;
    std::atomic<TaskState> state_;
    std::function<void()> execute_func_;

public:
    // 任务构造：包含唯一 id、名称与可执行回调
    Task(int id, const std::string& name, const std::function<void()>& func)
        : task_id_(id),
          task_name_(name),
          state_(TaskState::PENDING),
          execute_func_(func) {}
    // 执行任务并更新状态，记录日志
    void run() {
        state_.store(TaskState::RUNNING);
        RCLCPP_INFO(rclcpp::get_logger("task_scheduler"),
                    "开始执行任务 : %s, id : %d", task_name_.c_str(), task_id_);
        try {
            if (!execute_func_) throw std::runtime_error("Empty execute_func_");
            execute_func_();
            state_.store(TaskState::COMPLETED);
            RCLCPP_INFO(rclcpp::get_logger("task_scheduler"),
                        "任务 : %s, id : %d 执行完成", task_name_.c_str(), task_id_);
        } catch (const std::exception& e) {
            state_.store(TaskState::FAILED);
            RCLCPP_INFO(rclcpp::get_logger("task_scheduler"),
                        "任务 : %s, id : %d 执行异常：%s", task_name_.c_str(), task_id_, e.what());
        }
    }
    // 线程安全读取当前状态
    TaskState get_state() const {
        return state_.load();
    }
    // 任务 id
    int get_id() const {
        return task_id_;
    }
    // 任务名称
    std::string get_name() const {
        return task_name_;
    }
};

class ThreadSafeQueue {
private:
    std::mutex que_mtx_;
    std::queue<std::shared_ptr<Task>> task_queue_;
    std::condition_variable task_cv_;
    std::atomic<bool> is_running_;

public:
    // 队列默认处于运行状态
    ThreadSafeQueue() : is_running_{true} {}
    // 入队并唤醒等待线程
    void push(std::shared_ptr<Task> new_task) {
        {
            std::lock_guard<std::mutex> lock(this->que_mtx_);
            task_queue_.push(std::move(new_task));
        }
        task_cv_.notify_one();
    }
    // 出队：若队列关闭且为空则返回 nullptr
    std::shared_ptr<Task> pop() {
        std::unique_lock<std::mutex> lock(this->que_mtx_);
        task_cv_.wait(lock, [this]() { return !is_running_ || !task_queue_.empty(); });
        if (task_queue_.empty()) return nullptr;
        auto t = task_queue_.front();
        task_queue_.pop();
        return t;
    }
    // 关闭队列并唤醒所有等待线程
    void abort() {
        {
            std::lock_guard<std::mutex> lock(this->que_mtx_);
            is_running_ = false;
        }
        task_cv_.notify_all();
    }
    // 判断队列是否仍可接收/处理任务
    bool is_active() {
        return is_running_.load();
    }
};

class TaskScheduler {
private:
    std::vector<std::thread> thread_vec_;
    ThreadSafeQueue task_safe_queue_;

public:
    // 启动 n 个工作线程，持续从队列获取任务执行
    void start(int n) {
        for (int i = 0; i < n; i++) {
            thread_vec_.emplace_back([this]() {
                while (rclcpp::ok()) {
                    auto task_ptr = task_safe_queue_.pop();
                    if (!task_ptr) break;
                    task_ptr->run();
                }
            });
        }
    }
    // 停止调度器：关闭队列并等待线程退出
    void stop() {
        task_safe_queue_.abort();
        for (auto& t : thread_vec_)
            if (t.joinable()) t.join();
        RCLCPP_INFO(rclcpp::get_logger("task_scheduler"),
                    "所有线程已关闭");
    }
    // 提交任务到队列（调度器关闭后直接忽略）
    void submit_task(std::shared_ptr<Task> new_task) {
        if (!task_safe_queue_.is_active()) return;
        RCLCPP_INFO(rclcpp::get_logger("task_scheduler"),
                    "任务 : %s 已提交", new_task->get_name().c_str());
        task_safe_queue_.push(new_task);
    }
    // 析构时确保停止调度器，避免后台线程悬挂
    ~TaskScheduler() {
        if (!task_safe_queue_.is_active()) return;
        stop();
    }
};
#endif  // !__TASK_SCHEDULER_BASE_HPP

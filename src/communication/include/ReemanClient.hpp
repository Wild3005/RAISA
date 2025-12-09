#pragma once

#include <string>
#include <optional>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include "nlohmann_json.hpp"
#include <cpr/cpr.h>
#include <rclcpp/rclcpp.hpp>

using json = nlohmann::json;

// ============================================================
// ENDPOINT DEFINITIONS
// ============================================================
// GET
#define EP_GET_POSE "/reeman/pose"
#define EP_GET_MODE "/reeman/get_mode"
#define EP_GET_POWER "/reeman/base_encode"
#define EP_GET_LASER "/reeman/laser"
#define EP_GET_SPEED "/reeman/speed"
#define EP_GET_NAV_STATUS "/reeman/nav_status"
#define EP_GET_CALIB_POINTS "/reeman/position"
#define EP_GET_ROUTES "/reeman/navi_routes"
#define EP_GET_MAP_LIST "/reeman/history_map"
#define EP_GET_CURRENT_MAP "/reeman/current_map"
#define EP_GET_IMU "/reeman/imu"
#define EP_GET_GLOBAL_PLAN "/reeman/global_plan"
#define EP_GET_SPECIAL_POLYGON "/reeman/special_polygon"

// POST
#define EP_POST_SPEED "/cmd/speed"
#define EP_POST_NAV "/cmd/nav"
#define EP_POST_NAV_NAME "/cmd/nav_name"
#define EP_POST_CANCEL_GOAL "/cmd/cancel_goal"
#define EP_POST_RELOC_ABSOLUTE "/cmd/reloc_absolute"
#define EP_POST_CHARGE "/cmd/charge"
#define EP_POST_MOVE "/cmd/move"
#define EP_POST_TURN "/cmd/turn"
#define EP_POST_SET_MODE "/cmd/set_mode"
#define EP_POST_SAVE_MAP "/cmd/save_map"
#define EP_POST_MAX_SPEED "/cmd/max_speed"
#define EP_POST_APPLY_MAP "/cmd/apply_map"

#define MAX_TIMEOUT_MS 1000

// ============================================================
// ReemanClient WITH QUEUED HTTP EXECUTION
// ============================================================

class ReemanClient
{
public:
    explicit ReemanClient(const std::string &host)
    {
        baseUrl_ = "http://" + host;
        worker_running_ = true;
        worker_thread_ = std::thread(&ReemanClient::workerLoop, this);
    }

    ~ReemanClient()
    {
        worker_running_ = false;
        queue_cv_.notify_all();
        if (worker_thread_.joinable())
            worker_thread_.join();
    }

private:
    // ============================================================
    // QUEUE INFRASTRUCTURE
    // ============================================================

    struct HttpTask
    {
        bool is_post;
        std::string path;
        json body;
        std::promise<std::optional<json>> promise;
    };

    std::string baseUrl_;

    std::queue<HttpTask> task_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread worker_thread_;
    std::atomic<bool> worker_running_{false};

    // ============================================================
    // REAL HTTP EXECUTION (ONLY RUN IN WORKER THREAD)
    // ============================================================

    std::optional<json> doHttpGet(const std::string &path)
    {
        auto res = cpr::Get(cpr::Url{baseUrl_ + path}, cpr::Timeout{MAX_TIMEOUT_MS});

        if (res.error)
        {
            RCLCPP_WARN(rclcpp::get_logger("ReemanClient"),
                        "GET %s error: %s", path.c_str(), res.error.message.c_str());
            return std::nullopt;
        }

        if (res.status_code < 200 || res.status_code >= 300)
        {
            RCLCPP_WARN(rclcpp::get_logger("ReemanClient"),
                        "GET %s HTTP %ld", path.c_str(), res.status_code);
            return std::nullopt;
        }

        try
        {
            return json::parse(res.text);
        }
        catch (...)
        {
            return std::nullopt;
        }
    }

    std::optional<json> doHttpPost(const std::string &path, const json &body)
    {
        auto res = cpr::Post(
            cpr::Url{baseUrl_ + path},
            cpr::Body{body.dump()},
            cpr::Header{{"Content-Type", "application/json"}},
            cpr::Timeout{MAX_TIMEOUT_MS});

        if (res.error)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ReemanClient"),
                         "POST %s error: %s", path.c_str(), res.error.message.c_str());
            return std::nullopt;
        }

        if (res.status_code < 200 || res.status_code >= 300)
        {
            RCLCPP_WARN(rclcpp::get_logger("ReemanClient"),
                        "POST %s HTTP %ld", path.c_str(), res.status_code);
            return std::nullopt;
        }

        try
        {
            return json::parse(res.text);
        }
        catch (...)
        {
            return std::nullopt;
        }
    }

    // ============================================================
    // WORKER LOOP (SERIAL EXECUTION)
    // ============================================================

    void workerLoop()
    {
        while (worker_running_)
        {
            HttpTask task;

            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [&]()
                               { return !task_queue_.empty() || !worker_running_; });

                if (!worker_running_)
                    return;

                task = std::move(task_queue_.front());
                task_queue_.pop();
            }

            std::optional<json> result;

            if (task.is_post)
                result = doHttpPost(task.path, task.body);
            else
                result = doHttpGet(task.path);

            task.promise.set_value(result);

            // HARD THROTTLE ANTI OVERLOAD (IMPORTANT)
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    }

    // ============================================================
    // QUEUED API
    // ============================================================

    std::optional<json> queueGet(const std::string &path)
    {
        HttpTask task;
        task.is_post = false;
        task.path = path;

        auto future = task.promise.get_future();
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            task_queue_.push(std::move(task));
        }
        queue_cv_.notify_one();
        return future.get();
    }

    std::optional<json> queuePost(const std::string &path, const json &body)
    {
        HttpTask task;
        task.is_post = true;
        task.path = path;
        task.body = body;

        auto future = task.promise.get_future();
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            task_queue_.push(std::move(task));
        }
        queue_cv_.notify_one();
        return future.get();
    }

public:
    // ============================================================
    // SPEED & MOTION
    // ============================================================
    bool sendSlowSpeed(float vx, float vth)
    {
        queuePost(EP_POST_SPEED, {{"vx", vx}, {"vth", vth}, {"slow", 1}}).has_value();
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        return queuePost(EP_POST_SPEED, {{"vx", 0.0}, {"vth", 0.0}, {"slow", 1}}).has_value();
    }

    bool sendSpeed(float vx, float vth)
    {
        return queuePost(EP_POST_SPEED, {{"vx", vx}, {"vth", vth}}).has_value();
    }

    bool moveDistance(float d, int dir, float speed)
    {
        return queuePost(EP_POST_MOVE, {{"distance", d}, {"direction", dir}, {"speed", speed}}).has_value();
    }

    bool turnAngle(float a, int dir, float speed)
    {
        return queuePost(EP_POST_TURN, {{"angle", a}, {"direction", dir}, {"speed", speed}}).has_value();
    }

    // ============================================================
    // NAVIGATION
    // ============================================================

    bool sendNav(float x, float y, float th)
    {
        auto r = queuePost(EP_POST_NAV, {{"x", x}, {"y", y}, {"theta", th}});
        return r && r->value("status", "fail") == "success";
    }

    bool sendNavByName(const std::string &name)
    {
        auto r = queuePost(EP_POST_NAV_NAME, {{"point", name}});
        return r && r->value("status", "fail") == "success";
    }

    bool cancelNav()
    {
        return queuePost(EP_POST_CANCEL_GOAL, json::object()).has_value();
    }

    bool relocateAbsolute(float x, float y, float th)
    {
        return queuePost(EP_POST_RELOC_ABSOLUTE, {{"x", x}, {"y", y}, {"theta", th}}).has_value();
    }

    bool goToChargePoint(const std::string &p = "Charging pile")
    {
        return queuePost(EP_POST_CHARGE, {{"type", 0}, {"point", p}}).has_value();
    }

    std::optional<json> getNavStatus() { return queueGet(EP_GET_NAV_STATUS); }

    // ============================================================
    // STATE & SENSOR
    // ============================================================

    std::optional<json> getPose() { return queueGet(EP_GET_POSE); }
    std::optional<json> getPower() { return queueGet(EP_GET_POWER); }
    std::optional<json> getLaser() { return queueGet(EP_GET_LASER); }
    std::optional<json> getSpeedState() { return queueGet(EP_GET_SPEED); }
    std::optional<json> getIMU() { return queueGet(EP_GET_IMU); }

    std::optional<int> getMode()
    {
        auto r = queueGet(EP_GET_MODE);
        if (!r || !r->contains("mode"))
            return std::nullopt;
        return (*r)["mode"].get<int>();
    }

    // ============================================================
    // MAPPING
    // ============================================================

    bool setMode(int mode)
    {
        return queuePost(EP_POST_SET_MODE, {{"mode", mode}}).has_value();
    }

    bool saveMap()
    {
        return queuePost(EP_POST_SAVE_MAP, json::object()).has_value();
    }

    bool applyMap(const std::string &name)
    {
        return queuePost(EP_POST_APPLY_MAP, {{"name", name}}).has_value();
    }

    std::optional<json> getMapList() { return queueGet(EP_GET_MAP_LIST); }

    std::optional<std::string> getCurrentMapName()
    {
        auto r = queueGet(EP_GET_CURRENT_MAP);
        if (!r || !r->contains("name"))
            return std::nullopt;
        return (*r)["name"].get<std::string>();
    }

    // ============================================================
    // ROUTES & POINTS
    // ============================================================

    std::optional<json> getCalibrationPoints() { return queueGet(EP_GET_CALIB_POINTS); }
    std::optional<json> getRoutes() { return queueGet(EP_GET_ROUTES); }
};

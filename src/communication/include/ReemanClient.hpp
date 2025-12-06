#pragma once

#include <string>
#include <optional>
#include "nlohmann_json.hpp"
#include <cpr/cpr.h>
#include <rclcpp/rclcpp.hpp>

using json = nlohmann::json;

// ============================================================
// ENDPOINT DEFINITIONS
// ============================================================

// GET endpoints
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

// POST endpoints
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

// ============================================================
// ReemanClient (Header-only)
// ============================================================

class ReemanClient
{
public:
    explicit ReemanClient(const std::string &host)
    {
        baseUrl_ = "http://" + host;
    }

private:
    std::string baseUrl_;

    // ============================================================
    // Internal HTTP GET helper
    // ============================================================
    std::optional<json> httpGet(const std::string &path)
    {
        auto res = cpr::Get(
            cpr::Url{baseUrl_ + path},
            cpr::Timeout{3000});

        if (res.error)
        {
            RCLCPP_WARN(rclcpp::get_logger("ReemanClient"),
                        "GET %s error: %s",
                        path.c_str(),
                        res.error.message.c_str());
            return std::nullopt;
        }

        if (res.status_code < 200 || res.status_code >= 300)
        {
            RCLCPP_WARN(rclcpp::get_logger("ReemanClient"),
                        "GET %s HTTP %ld",
                        path.c_str(),
                        res.status_code);
            return std::nullopt;
        }

        try
        {
            return json::parse(res.text);
        }
        catch (...)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ReemanClient"),
                         "Invalid JSON GET %s",
                         path.c_str());
            return std::nullopt;
        }
    }

    // ============================================================
    // Internal HTTP POST helper
    // ============================================================
    std::optional<json> httpPost(const std::string &path, const json &body)
    {
        auto res = cpr::Post(
            cpr::Url{baseUrl_ + path},
            cpr::Body{body.dump()},
            cpr::Header{{"Content-Type", "application/json"}},
            cpr::Timeout{3000});

        if (res.error)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ReemanClient"),
                         "POST %s error: %s",
                         path.c_str(),
                         res.error.message.c_str());
            return std::nullopt;
        }

        if (res.status_code < 200 || res.status_code >= 300)
        {
            RCLCPP_WARN(rclcpp::get_logger("ReemanClient"),
                        "POST %s HTTP %ld",
                        path.c_str(),
                        res.status_code);
            return std::nullopt;
        }

        try
        {
            return json::parse(res.text);
        }
        catch (...)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ReemanClient"),
                         "Invalid JSON POST %s",
                         path.c_str());
            return std::nullopt;
        }
    }

public:
    // ============================================================
    // SPEED & MOTION
    // ============================================================

    bool sendSpeed(float vx, float vth)
    {
        json body = {{"vx", vx}, {"vth", vth}};
        return httpPost(EP_POST_SPEED, body).has_value();
    }

    bool moveDistance(float distance_cm, int direction, float speed_mps)
    {
        json body = {
            {"distance", distance_cm},
            {"direction", direction},
            {"speed", speed_mps}};
        return httpPost(EP_POST_MOVE, body).has_value();
    }

    bool turnAngle(float angle_deg, int direction, float speed_rad)
    {
        json body = {
            {"angle", angle_deg},
            {"direction", direction},
            {"speed", speed_rad}};
        return httpPost(EP_POST_TURN, body).has_value();
    }

    // ============================================================
    // NAVIGATION
    // ============================================================

    bool sendNav(float x, float y, float theta_rad)
    {
        json body = {{"x", x}, {"y", y}, {"theta", theta_rad}};
        auto res = httpPost(EP_POST_NAV, body);
        return res && res->value("status", "fail") == "success";
    }

    bool sendNavByName(const std::string &name)
    {
        json body = {{"point", name}};
        auto res = httpPost(EP_POST_NAV_NAME, body);
        return res && res->value("status", "fail") == "success";
    }

    bool cancelNav()
    {
        return httpPost(EP_POST_CANCEL_GOAL, json::object()).has_value();
    }

    bool relocateAbsolute(float x, float y, float theta_rad)
    {
        json body = {{"x", x}, {"y", y}, {"theta", theta_rad}};
        return httpPost(EP_POST_RELOC_ABSOLUTE, body).has_value();
    }

    bool goToChargePoint(const std::string &point = "Charging pile")
    {
        json body = {{"type", 0}, {"point", point}};
        return httpPost(EP_POST_CHARGE, body).has_value();
    }

    std::optional<json> getNavStatus()
    {
        return httpGet(EP_GET_NAV_STATUS);
    }

    // ============================================================
    // STATE & SENSOR API
    // ============================================================

    std::optional<json> getPose()
    {
        return httpGet(EP_GET_POSE);
    }

    std::optional<int> getMode()
    {
        auto res = httpGet(EP_GET_MODE);
        if (!res || !res->contains("mode"))
            return std::nullopt;
        return (*res)["mode"].get<int>();
    }

    std::optional<json> getPower()
    {
        return httpGet(EP_GET_POWER);
    }

    std::optional<json> getLaser()
    {
        return httpGet(EP_GET_LASER);
    }

    std::optional<json> getSpeedState()
    {
        return httpGet(EP_GET_SPEED);
    }

    std::optional<json> getIMU()
    {
        return httpGet(EP_GET_IMU);
    }

    std::optional<json> getGlobalPlan()
    {
        return httpGet(EP_GET_GLOBAL_PLAN);
    }

    std::optional<json> getSpecialPolygon()
    {
        return httpGet(EP_GET_SPECIAL_POLYGON);
    }

    // ============================================================
    // MAPPING
    // ============================================================

    bool setMode(int mode)
    {
        json body = {{"mode", mode}};
        return httpPost(EP_POST_SET_MODE, body).has_value();
    }

    bool saveMap()
    {
        return httpPost(EP_POST_SAVE_MAP, json::object()).has_value();
    }

    bool applyMap(const std::string &name)
    {
        json body = {{"name", name}};
        return httpPost(EP_POST_APPLY_MAP, body).has_value();
    }

    std::optional<json> getMapList()
    {
        return httpGet(EP_GET_MAP_LIST);
    }

    std::optional<std::string> getCurrentMapName()
    {
        auto res = httpGet(EP_GET_CURRENT_MAP);
        if (!res || !res->contains("name"))
            return std::nullopt;
        return (*res)["name"].get<std::string>();
    }

    // ============================================================
    // WAYPOINTS & ROUTES
    // ============================================================

    std::optional<json> getCalibrationPoints()
    {
        return httpGet(EP_GET_CALIB_POINTS);
    }

    std::optional<json> getRoutes()
    {
        return httpGet(EP_GET_ROUTES);
    }
};

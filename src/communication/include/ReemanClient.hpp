#pragma once

#include <string>
#include <optional>
#include <vector>
#include "nlohmann_json.hpp"
#include <cpr/cpr.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

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
#define EP_POST_VIRTUAL_WALL "/cmd/restrict_layer"

// Virtual wall segment structure
struct VirtualWallSegment {
    float x1, y1, x2, y2;
};

// ============================================================
// ReemanClient
// ============================================================

class ReemanClient {
private:
    std::string base_url_;
    int timeout_ms_ = 5000; // 5 detik timeout

    // ============================================================
    // Internal HTTP GET helper (menggunakan CPR)
    // ============================================================
    std::optional<json> httpGet(const std::string &path) {
        std::string url = base_url_ + path;
        
        auto res = cpr::Get(
            cpr::Url{url},
            cpr::Timeout{timeout_ms_},
            cpr::ConnectTimeout{3000});

        if (res.error) {
            std::cerr << "[ReemanClient] GET " << path << " error: " 
                      << res.error.message << std::endl;
            return std::nullopt;
        }

        if (res.status_code < 200 || res.status_code >= 300) {
            std::cerr << "[ReemanClient] HTTP " << res.status_code 
                      << " for " << path << std::endl;
            return std::nullopt;
        }

        try {
            return json::parse(res.text);
        } catch (const json::exception& e) {
            std::cerr << "[ReemanClient] JSON parse error: " << e.what() << std::endl;
            return std::nullopt;
        }
    }

    // ============================================================
    // Internal HTTP POST helper (menggunakan CPR)
    // ============================================================
    std::optional<json> httpPost(const std::string &path, const json &data) {
        std::string url = base_url_ + path;
        std::string payload = data.dump();

        auto res = cpr::Post(
            cpr::Url{url},
            cpr::Body{payload},
            cpr::Header{{"Content-Type", "application/json"}},
            cpr::Timeout{timeout_ms_},
            cpr::ConnectTimeout{3000});

        if (res.error) {
            std::cerr << "[ReemanClient] POST " << path << " error: " 
                      << res.error.message << std::endl;
            return std::nullopt;
        }

        if (res.status_code < 200 || res.status_code >= 300) {
            std::cerr << "[ReemanClient] HTTP " << res.status_code 
                      << " for POST " << path << std::endl;
            return std::nullopt;
        }

        try {
            return json::parse(res.text);
        } catch (const json::exception& e) {
            std::cerr << "[ReemanClient] JSON parse error POST: " << e.what() << std::endl;
            return std::nullopt;
        }
    }

    bool httpPostSimple(const std::string &path, const json &data) {
        auto res = httpPost(path, data);
        return res.has_value();
    }

public:
    explicit ReemanClient(const std::string &host) {
        base_url_ = "http://" + host;
        std::cout << "[ReemanClient] Initialized with base_url: " << base_url_ << std::endl;
        
        // Test connection
        std::cout << "[ReemanClient] Testing connection to " << base_url_ << std::endl;
        auto test = httpGet("/");
        if (test) {
            std::cout << "[ReemanClient] Connection OK!" << std::endl;
        } else {
            std::cerr << "[ReemanClient] WARNING: Initial connection test failed" << std::endl;
        }
    }

    ~ReemanClient() = default;

    // ============================================================
    // SPEED & MOTION
    // ============================================================

    bool sendSpeed(float vx, float vth) {
        json body = {{"vx", vx}, {"vth", vth}};
        return httpPostSimple(EP_POST_SPEED, body);
    }

    bool moveDistance(float distance_cm, int direction, float speed_mps) {
        json body = {
            {"distance", distance_cm},
            {"direction", direction},
            {"speed", speed_mps}};
        return httpPostSimple(EP_POST_MOVE, body);
    }

    bool turnAngle(float angle_deg, int direction, float speed_rad) {
        json body = {
            {"angle", angle_deg},
            {"direction", direction},
            {"speed", speed_rad}};
        return httpPostSimple(EP_POST_TURN, body);
    }

    // ============================================================
    // NAVIGATION
    // ============================================================

    bool sendNav(float x, float y, float theta_rad) {
        json body = {{"x", x}, {"y", y}, {"theta", theta_rad}};
        auto res = httpPost(EP_POST_NAV, body);
        return res && res->value("status", "fail") == "success";
    }

    bool sendNavByName(const std::string &name) {
        json body = {{"point", name}};
        auto res = httpPost(EP_POST_NAV_NAME, body);
        return res && res->value("status", "fail") == "success";
    }

    bool cancelNav() {
        return httpPostSimple(EP_POST_CANCEL_GOAL, json::object());
    }

    bool relocateAbsolute(float x, float y, float theta_rad) {
        json body = {{"x", x}, {"y", y}, {"theta", theta_rad}};
        return httpPostSimple(EP_POST_RELOC_ABSOLUTE, body);
    }

    bool goToChargePoint(const std::string &point = "Charging pile") {
        json body = {{"type", 0}, {"point", point}};
        return httpPostSimple(EP_POST_CHARGE, body);
    }

    // ============================================================
    // VIRTUAL WALL
    // ============================================================

    bool postVirtualWall(const std::vector<VirtualWallSegment>& segments) {
        json waypoints = json::array();
        for (const auto& seg : segments) {
            json pose = {
                {"point1", {{"x", seg.x1}, {"y", seg.y1}}},
                {"point2", {{"x", seg.x2}, {"y", seg.y2}}}
            };
            waypoints.push_back({{"pose", pose}});
        }
        json payload = {{"waypoints", waypoints}};
        return httpPostSimple(EP_POST_VIRTUAL_WALL, payload);
    }

    // ============================================================
    // STATE & SENSOR API
    // ============================================================

    std::optional<json> getPose() {
        return httpGet(EP_GET_POSE);
    }

    std::optional<int> getMode() {
        auto res = httpGet(EP_GET_MODE);
        if (!res || !res->contains("mode"))
            return std::nullopt;
        return (*res)["mode"].get<int>();
    }

    std::optional<json> getPower() {
        return httpGet(EP_GET_POWER);
    }

    std::optional<json> getLaser() {
        return httpGet(EP_GET_LASER);
    }

    std::optional<json> getSpeedState() {
        return httpGet(EP_GET_SPEED);
    }

    std::optional<json> getNavStatus() {
        return httpGet(EP_GET_NAV_STATUS);
    }

    std::optional<json> getIMU() {
        return httpGet(EP_GET_IMU);
    }

    std::optional<json> getGlobalPlan() {
        return httpGet(EP_GET_GLOBAL_PLAN);
    }

    std::optional<json> getSpecialPolygon() {
        return httpGet(EP_GET_SPECIAL_POLYGON);
    }

    // ============================================================
    // MAPPING
    // ============================================================

    bool setMode(int mode) {
        json body = {{"mode", mode}};
        return httpPostSimple(EP_POST_SET_MODE, body);
    }

    bool saveMap() {
        return httpPostSimple(EP_POST_SAVE_MAP, json::object());
    }

    bool applyMap(const std::string &name) {
        json body = {{"name", name}};
        return httpPostSimple(EP_POST_APPLY_MAP, body);
    }

    std::optional<json> getMapList() {
        return httpGet(EP_GET_MAP_LIST);
    }

    std::optional<std::string> getCurrentMapName() {
        auto res = httpGet(EP_GET_CURRENT_MAP);
        if (!res || !res->contains("name"))
            return std::nullopt;
        return (*res)["name"].get<std::string>();
    }

    // ============================================================
    // WAYPOINTS & ROUTES
    // ============================================================

    std::optional<json> getCalibrationPoints() {
        return httpGet(EP_GET_CALIB_POINTS);
    }

    std::optional<json> getRoutes() {
        return httpGet(EP_GET_ROUTES);
    }
};

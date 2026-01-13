#pragma once
#include <fstream>
#include <string>
#include <chrono>

class CsvLogger
{
public:
    CsvLogger(const std::string &path)
    {
        file_.open(path, std::ios::out | std::ios::app);
        if (file_.tellp() == 0)
        {
            file_ <<
                "timestamp,"
                "robot_x,robot_y,robot_theta,"
                "robot_mode,robot_fsm_mode,"
                "human_x,human_y,human_theta,"
                "human_linear,human_angular,"
                "human_mode\n";
        }
    }

    ~CsvLogger()
    {
        if (file_.is_open())
            file_.close();
    }

    template<typename... Args>
    void log(Args... args)
    {
        file_ << timestamp() << ",";
        write(args...);
        file_ << "\n";
    }

    inline std::string make_time_based_csv_name(const std::string &prefix,const std::string &dir = "/home/raisa/raisa_logs/")
    {
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);

        std::tm tm{};
        localtime_r(&t, &tm);

        std::ostringstream oss;
        oss << dir << "/"
            << prefix << "_"
            << std::put_time(&tm, "%Y%m%d_%H%M%S")
            << ".csv";

        return oss.str();
    }


private:
    std::ofstream file_;

    static long long timestamp()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
            .count();
    }

    template<typename T>
    void write(T value)
    {
        file_ << value;
    }

    template<typename T, typename... Args>
    void write(T value, Args... args)
    {
        file_ << value << ",";
        write(args...);
    }
};

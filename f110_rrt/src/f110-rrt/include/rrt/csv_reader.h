#ifndef SRC_CSV_READER_H
#define SRC_CSV_READER_H

#include <fstream>
#include <utility>
#include <vector>
#include <string>
#include <array>        // 추가
#include <stdexcept>    // 추가
#include <boost/algorithm/string.hpp>

namespace rrt
{

/// A Class to read csv data files
class CSVReader
{
    std::string fileName;
    std::string delimeter;

public:
    explicit CSVReader(std::string filename, std::string delm = ",")
        : fileName(std::move(filename)), delimeter(std::move(delm))
    {}

    /// Function to fetch data from a CSV File
    std::vector<std::array<double, 2>> getData()
    {
        std::ifstream file(fileName);
        if (!file)
            throw std::runtime_error("Invalid Path for csv file: " + fileName);

        std::vector<std::array<double, 2>> dataList;
        std::string line;
        size_t ln = 0;

        while (std::getline(file, line))
        {
            ++ln;

            // 윈도우 CR 제거
            if (!line.empty() && line.back() == '\r') line.pop_back();

            // 공백/주석(#) 라인 스킵
            auto first = line.find_first_not_of(" \t");
            if (first == std::string::npos) continue;
            if (line[first] == '#') continue;

            // 콤마/세미콜론 모두 허용
            std::vector<std::string> vec;
            boost::algorithm::split(vec, line, boost::is_any_of(",;"));

            if (vec.size() < 2) continue;

            // UTF-8 BOM 제거
            if (vec[0].size() >= 3 &&
                static_cast<unsigned char>(vec[0][0]) == 0xEF &&
                static_cast<unsigned char>(vec[0][1]) == 0xBB &&
                static_cast<unsigned char>(vec[0][2]) == 0xBF)
            {
                vec[0].erase(0, 3);
            }

            try {
                double x = std::stod(vec[0]);
                double y = std::stod(vec[1]);
                dataList.push_back({x, y});
            } catch (const std::exception& e) {
                // 숫자 변환 실패 라인은 무시
                ROS_WARN("Invalid data at line %zu: '%s'. Error: %s", ln, line.c_str(), e.what());
                continue;
            }
        }

        if (dataList.empty())
            throw std::runtime_error("CSV parsed 0 waypoints: " + fileName);

        return dataList;
    }
};

} // namespace rrt

#endif // SRC_CSV_READER_H


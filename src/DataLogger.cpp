#include "rbd/DataLogger.hpp"
#include <iostream>

namespace rbd {

DataLogger::DataLogger(const std::string& filepath) {
    file.open(filepath);
    if (!file.is_open()) {
        std::cerr << "ERROR: Failed to open log file: " << filepath << std::endl;
    }
}

DataLogger::~DataLogger() {
    if (file.is_open()) {
        file.close();
    }
}

void DataLogger::writeHeader(const std::vector<std::string>& columnNames) {
    if (!file.is_open()) return;
    
    file << "time"; // Time is always the first column
    for (const auto& name : columnNames) {
        file << "," << name;
    }
    file << "\n";
}

void DataLogger::logRow(double time, const std::vector<double>& values) {
    if (!file.is_open()) return;
    
    file << time;
    for (double val : values) {
        file << "," << val;
    }
    file << "\n";
}

} // namespace rbd
#pragma once
#include <string>
#include <vector>
#include <fstream>

namespace rbd {

class DataLogger {
private:
    std::ofstream file;

public:
    // Opens the file at the given path
    DataLogger(const std::string& filepath);
    
    // Safely closes the file
    ~DataLogger();
    
    // Writes the comma-separated column headers
    void writeHeader(const std::vector<std::string>& columnNames);
    
    // Writes a single row of comma-separated numeric data
    void logRow(double time, const std::vector<double>& values);
};

} // namespace rbd
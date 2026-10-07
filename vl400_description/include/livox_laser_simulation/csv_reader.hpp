#ifndef LIVOX_LASER_SIMULATION_CSV_READER_HPP
#define LIVOX_LASER_SIMULATION_CSV_READER_HPP

#include <exception>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace livox_laser_simulation
{

class CsvReader
{
public:
  static bool ReadCsvFile(const std::string &_file_name,
                          std::vector<std::vector<double>> &_data)
  {
    std::ifstream stream(_file_name);
    if (!stream)
      return false;

    std::string line;
    std::getline(stream, line);
    while (std::getline(stream, line))
    {
      if (line.empty())
        continue;

      std::stringstream line_stream(line);
      std::vector<double> row;
      std::string value;
      try
      {
        while (std::getline(line_stream, value, ','))
          row.push_back(std::stod(value));
      }
      catch (const std::exception &)
      {
        continue;
      }
      if (!row.empty())
        _data.push_back(std::move(row));
    }
    return true;
  }
};

}  // namespace livox_laser_simulation

#endif  // LIVOX_LASER_SIMULATION_CSV_READER_HPP

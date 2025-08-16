// python_bindings.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <unordered_set>
#include <vector>
#include <map>

namespace py = pybind11;

// Include the structures from cl_cbs.cpp directly or redefine them
struct Location {
  Location(double x, double y) : x(x), y(y) {}
  double x;
  double y;

  bool operator<(const Location& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }
};

// Define a simplified State class that matches cl_cbs.cpp
struct State {
  State(double x, double y, double yaw, int time = 0)
      : time(time), x(x), y(y), yaw(yaw) {}
  State() = default;

  bool operator==(const State& s) const {
    return std::tie(time, x, y, yaw) == std::tie(s.time, s.x, s.y, s.yaw);
  }

  // Getter methods for Python bindings
  double getX() const { return x; }
  double getY() const { return y; }
  double getTheta() const { return yaw; }
  int getTime() const { return time; }

  int time;
  double x;
  double y;
  double yaw;
};

// Simple result structure
struct SimplePlanResult {
    bool success;
    double cost;
    double runtime;
    std::map<int, std::vector<std::vector<double>>> paths;
    std::string error;
};

// Function to call the actual CL-CBS executable and parse results
SimplePlanResult solve_clcbs_via_executable(
    const std::vector<int> &map_size,
    const std::vector<std::vector<double>> &obstacles,
    const std::vector<std::vector<double>> &starts,
    const std::vector<std::vector<double>> &goals
) {
    SimplePlanResult result;
    result.success = false;
    result.cost = 0.0;
    result.runtime = 0.0;

    try {
        // Create temporary YAML input file
        std::string temp_input = "/tmp/cl_cbs_input.yaml";
        std::string temp_output = "/tmp/cl_cbs_output.yaml";
        
        std::ofstream yaml_out(temp_input);
        
        // Write YAML input file
        yaml_out << "map:" << std::endl;
        yaml_out << "  dimensions: [" << map_size[0] << ", " << map_size[1] << "]" << std::endl;
        yaml_out << "  obstacles:" << std::endl;
        for (const auto& obs : obstacles) {
            if (obs.size() >= 2) {
                yaml_out << "    - [" << obs[0] << ", " << obs[1] << "]" << std::endl;
            }
        }
        
        yaml_out << "agents:" << std::endl;
        for (size_t i = 0; i < starts.size() && i < goals.size(); ++i) {
            if (starts[i].size() >= 3 && goals[i].size() >= 3) {
                yaml_out << "  - start: [" << starts[i][0] << ", " << starts[i][1] 
                         << ", " << starts[i][2] << "]" << std::endl;
                yaml_out << "    goal: [" << goals[i][0] << ", " << goals[i][1] 
                         << ", " << goals[i][2] << "]" << std::endl;
            }
        }
        yaml_out.close();
        
        // Execute CL-CBS
        std::string command = "./CL-CBS -i " + temp_input + " -o " + temp_output;
        int ret = system(command.c_str());
        
        if (ret != 0) {
            result.error = "CL-CBS executable failed";
            return result;
        }
        
        // Parse output YAML
        try {
            YAML::Node output_config = YAML::LoadFile(temp_output);
            
            if (output_config["statistics"]) {
                auto stats = output_config["statistics"];
                result.cost = stats["cost"].as<double>();
                result.runtime = stats["runtime"].as<double>();
            }
            
            if (output_config["schedule"]) {
                auto schedule = output_config["schedule"];
                for (auto agent_it = schedule.begin(); agent_it != schedule.end(); ++agent_it) {
                    std::string agent_name = agent_it->first.as<std::string>();
                    int agent_id = std::stoi(agent_name.substr(5)); // Remove "agent" prefix
                    
                    std::vector<std::vector<double>> path;
                    for (const auto& state : agent_it->second) {
                        std::vector<double> point = {
                            state["x"].as<double>(),
                            state["y"].as<double>(),
                            state["yaw"].as<double>(),
                            state["t"].as<double>()
                        };
                        path.push_back(point);
                    }
                    result.paths[agent_id] = path;
                }
                result.success = true;
            }
            
        } catch (const std::exception& e) {
            result.error = std::string("Failed to parse output: ") + e.what();
        }
        
        // Clean up temp files
        remove(temp_input.c_str());
        remove(temp_output.c_str());
        
    } catch (const std::exception& e) {
        result.error = std::string("Exception: ") + e.what();
    }
    
    return result;
}

/**
 * Python-friendly wrapper that returns a dictionary
 */
py::dict solve_clcbs(
    const std::vector<int> &map_size,
    const std::vector<std::vector<double>> &obstacles,
    const std::vector<std::vector<double>> &starts,
    const std::vector<std::vector<double>> &goals
) {
    auto result = solve_clcbs_via_executable(map_size, obstacles, starts, goals);
    
    py::dict py_result;
    py_result["success"] = result.success;
    py_result["cost"] = result.cost;
    py_result["runtime"] = result.runtime;
    py_result["error"] = result.error;
    
    py::dict agent_paths;
    for (const auto& [agent_id, path] : result.paths) {
        py::list trajectory;
        for (const auto& point : path) {
            py::list py_point;
            for (double val : point) {
                py_point.append(val);
            }
            trajectory.append(py_point);
        }
        agent_paths[py::int_(agent_id)] = trajectory;
    }
    py_result["paths"] = agent_paths;
    
    return py_result;
}

/**
 * Direct YAML file solver
 */
py::dict solve_clcbs_from_yaml(const std::string& input_file, const std::string& output_file = "") {
    std::string actual_output = output_file.empty() ? "/tmp/cl_cbs_output.yaml" : output_file;
    
    std::string command = "./CL-CBS -i " + input_file + " -o " + actual_output;
    int ret = system(command.c_str());
    
    py::dict py_result;
    if (ret != 0) {
        py_result["success"] = false;
        py_result["error"] = "CL-CBS executable failed";
        py_result["paths"] = py::dict();
        return py_result;
    }
    
    try {
        YAML::Node output_config = YAML::LoadFile(actual_output);
        
        py_result["success"] = true;
        py_result["error"] = "";
        
        if (output_config["statistics"]) {
            auto stats = output_config["statistics"];
            py_result["cost"] = stats["cost"].as<double>();
            py_result["runtime"] = stats["runtime"].as<double>();
        }
        
        py::dict agent_paths;
        if (output_config["schedule"]) {
            auto schedule = output_config["schedule"];
            for (auto agent_it = schedule.begin(); agent_it != schedule.end(); ++agent_it) {
                std::string agent_name = agent_it->first.as<std::string>();
                int agent_id = std::stoi(agent_name.substr(5)); // Remove "agent" prefix
                
                py::list trajectory;
                for (const auto& state : agent_it->second) {
                    py::list point;
                    point.append(state["x"].as<double>());
                    point.append(state["y"].as<double>());
                    point.append(state["yaw"].as<double>());
                    point.append(state["t"].as<double>());
                    trajectory.append(point);
                }
                agent_paths[py::int_(agent_id)] = trajectory;
            }
        }
        py_result["paths"] = agent_paths;
        
    } catch (const std::exception& e) {
        py_result["success"] = false;
        py_result["error"] = std::string("Failed to parse output: ") + e.what();
        py_result["paths"] = py::dict();
    }
    
    return py_result;
}

PYBIND11_MODULE(cl_cbs, m) {
    m.doc() = "CL-CBS: Multi-Agent Path Finding for Car-Like Robots";
    
    m.def("solve", &solve_clcbs,
          "Solve CL-CBS given map size, obstacles, and start/goal poses",
          py::arg("map_size"), py::arg("obstacles"), 
          py::arg("starts"), py::arg("goals"));
    
    m.def("solve_from_yaml", &solve_clcbs_from_yaml,
          "Solve CL-CBS from YAML file",
          py::arg("input_file"), py::arg("output_file") = "");
    
    py::class_<State>(m, "State")
        .def(py::init<>())
        .def(py::init<double, double, double, int>())
        .def("getX", &State::getX)
        .def("getY", &State::getY) 
        .def("getTheta", &State::getTheta)
        .def("getTime", &State::getTime)
        .def_readwrite("x", &State::x)
        .def_readwrite("y", &State::y)
        .def_readwrite("yaw", &State::yaw)
        .def_readwrite("time", &State::time);
}
#ifndef INSTANCE_HPP
#define INSTANCE_HPP

#include <vector>

// Forward declarations
struct Pose {
    double x, y, theta;
    
    Pose() : x(0), y(0), theta(0) {}
    Pose(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
};

struct Obstacle {
    double x, y, width, height;
    
    Obstacle() : x(0), y(0), width(0), height(0) {}
    Obstacle(double x_, double y_, double w_, double h_) 
        : x(x_), y(y_), width(w_), height(h_) {}
};

struct Agent {
    int id;
    Pose start;
    Pose goal;
    
    Agent() : id(-1) {}
    Agent(int id_, const Pose& start_, const Pose& goal_) 
        : id(id_), start(start_), goal(goal_) {}
};

struct Instance {
    int map_width;
    int map_height;
    std::vector<Obstacle> obstacles;
    std::vector<Agent> agents;
    
    Instance() : map_width(0), map_height(0) {}
    
    void addAgent(const Pose& start, const Pose& goal) {
        agents.emplace_back(static_cast<int>(agents.size()), start, goal);
    }
    
    void addObstacle(double x, double y, double width, double height) {
        obstacles.emplace_back(x, y, width, height);
    }
    
    size_t getNumAgents() const {
        return agents.size();
    }
    
    size_t getNumObstacles() const {
        return obstacles.size();
    }
};

// Result structures
struct PathPoint {
    double x, y, theta;
    double time;
    int agent_id;
    
    PathPoint() : x(0), y(0), theta(0), time(0), agent_id(-1) {}
    PathPoint(double x_, double y_, double theta_, double t_ = 0, int id_ = -1)
        : x(x_), y(y_), theta(theta_), time(t_), agent_id(id_) {}
};

struct AgentPath {
    int agent_id;
    std::vector<PathPoint> path;
    
    AgentPath() : agent_id(-1) {}
    AgentPath(int id) : agent_id(id) {}
    
    void addPoint(double x, double y, double theta, double time = 0) {
        path.emplace_back(x, y, theta, time, agent_id);
    }
    
    size_t size() const {
        return path.size();
    }
    
    bool empty() const {
        return path.empty();
    }
    
    // Iterator support for range-based loops
    auto begin() { return path.begin(); }
    auto end() { return path.end(); }
    auto begin() const { return path.begin(); }
    auto end() const { return path.end(); }
};

struct PlanResult {
    std::vector<AgentPath> paths;
    bool success;
    double computation_time;
    int expanded_nodes;
    
    PlanResult() : success(false), computation_time(0.0), expanded_nodes(0) {}
    
    void addAgentPath(int agent_id) {
        paths.emplace_back(agent_id);
    }
    
    AgentPath* getAgentPath(int agent_id) {
        for (auto& path : paths) {
            if (path.agent_id == agent_id) {
                return &path;
            }
        }
        return nullptr;
    }
    
    size_t getNumAgents() const {
        return paths.size();
    }
};

#endif // INSTANCE_HPP
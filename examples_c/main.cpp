#include <string>
#include "parsing.hpp"
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
void RunExternal(EdgeDrone drone, bool logging);

bool logging = true;

int main(int argc, char* argv[]) {
  std::string config_path = "./1.toml";
  if (argc == 2) {
    if (std::string(argv[1]) == "--no-logging") {
      logging = false;
    } else {
      config_path = std::string(argv[1]);
    }
  }
  EdgeDrone drone;
  try {
    auto cfg = load_edge_drone_config(config_path);

    std::cout << "UUID: " << static_cast<int>(cfg.drone.uuid) << '\n';
    std::cout << "Position: " << cfg.drone.position[0] << ", "
              << cfg.drone.position[1] << ", " << cfg.drone.position[2] << '\n';
    drone = cfg.drone;
  } catch (const std::exception& e) {
    std::cerr << "Config error: " << e.what() << '\n';
    return 1;
  }
  RunExternal(drone, logging);
}

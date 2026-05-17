#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <unistd.h>
#include "Node.hpp"

void WriteToUuid(uint8_t uuid, uint8_t* payload, size_t len) {
  std::string file = "infile" + std::to_string(uuid) + ".txt";
  FILE* f = fopen(file.c_str(), "a");
  if (f == NULL) {
    std::cerr << "Failed to open file\n";
  }
  write(fileno(f), payload, len);
}

int ReadWithTimeout(uint8_t* pos, int fi, size_t bufsize) {
  auto bytesize = read(fi, pos, bufsize);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  return bytesize;
}

void RunExternal(EdgeDrone drone, bool logging) {
  drone.start = std::chrono::steady_clock::now();
  const std::string file = "infile" + std::to_string(drone.uuid) + ".txt";
  std::ofstream create(file);
  FILE* f = fopen(file.c_str(), "r");
  if (f == NULL) {
    std::cerr << "Failed to open file\n";
  }

  // Start at end of existing file
  while (true) {
    std::vector<uint8_t> buffer;
    buffer.resize(1024);
    auto bytes = ReadWithTimeout(buffer.data(), fileno(f), 1024);
    // process some meta stuff for next request
    // for now all meta information gets set on startup so its all synced
    drone.position = drone.position + drone.velocity;
    if (bytes > 0) {
      if (buffer[0] == commandtype::ALIVE) {
        if (logging) {
          std::cout << "RECEIVING ALIVE_PING ";
        }
        AliveBroadcast ab;
        ab.BuildFromVector(buffer);
        drone.ProcessAliveBroadcast(ab, logging);
      }
      if (buffer[0] == commandtype::NEIGHBOUR_REQUEST_ALIVE) {
        if (logging) {
          std::cout << "RECEIVING DRONE_MISSING ";
        }
        NeigbourRequest nr;
        nr.BuildFromVector(buffer);
        if (logging) {
          std::cout << "SENDING DRONE_LASTSEEN ";
        }
        auto payload = drone.ProcessNeighbourRequest(nr, logging);
        for (auto& neighbour : drone.neighbours) {
          WriteToUuid(neighbour, payload.data(), payload.size());
        }
      }
      if (buffer[0] == commandtype::NEIGHBOUR_RESPONSE_ALIVE) {
        if (logging) {
          std::cout << "RECEIVING DRONE_LASTSEEN ";
        }
        NeighbourResponse nr;
        nr.BuildFromVector(buffer);
        drone.ProcessNeighbourResponse(nr, logging);
      }
    } else {
      std::vector<uint8_t> toremove;
      for (auto it = drone.neighbours_missing_blocked.begin();
           it != drone.neighbours_missing_blocked.end(); it++) {
        if (it->second < std::chrono::steady_clock::now()) {
          toremove.emplace_back(it->first);
        }
      }
      for (auto& k : toremove) {
        drone.neighbours_missing_blocked.erase(k);
      }
      // sending and invalidation
      if (drone.last_ping_sent + std::chrono::seconds(1) <
          std::chrono::steady_clock::now()) {
        if (logging) std::cout << "SENDING ALIVE_PING ";
        auto payload = drone.BuildAlivePing(logging);
        for (auto& neighbour : drone.neighbours) {
          WriteToUuid(neighbour, payload.data(), payload.size());
        }
        drone.last_ping_sent = std::chrono::steady_clock::now();
      }

      for (auto& kv : drone.neighbours_timeout) {
        if (kv.second + std::chrono::seconds(5) <
                std::chrono::steady_clock::now() &&
            drone.neighbours_missing_sent.find(kv.first) ==
                drone.neighbours_missing_sent.end() &&
            drone.neighbours_missing_blocked.find(kv.first) ==
                drone.neighbours_missing_blocked.end()) {
          if (logging) std::cout << "SENDING DRONE_MISSING ";
          auto payload = drone.BuildDroneMissing(kv.first, logging);
          for (auto& neighbour : drone.neighbours) {
            WriteToUuid(neighbour, payload.data(), payload.size());
          }
          drone.neighbours_missing_sent.insert_or_assign(
              kv.first, std::chrono::steady_clock::now());
          drone.neighbours_missing_blocked.insert_or_assign(
              kv.first,
              std::chrono::steady_clock::now() + std::chrono::seconds(5));
        }
      }

      for (auto& kv : drone.neighbours_missing_sent) {
        if (kv.second + std::chrono::seconds(15) <
            std::chrono::steady_clock::now()) {
          std::cout << "GROUP connection fully lost, sleeping"
                    << drone.getTimeSinceStart() << std::endl;
          logging = false;
        }
      }
    }
  }
}

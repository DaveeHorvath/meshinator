#include <string>
#include <sys/epoll.h>
#include "Node.hpp"
#include "wfb_rs.h"
#include <toml++/toml.hpp>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
void RunExternalSimulated(EdgeDrone drone);
void RunExternal(EdgeDrone drone);
#pragma region config

struct EdgeDroneConfig {
  EdgeDrone drone{};
  std::string iface_storage;
};
static uint8_t read_u8(const toml::table& tbl, const char* key) {
  auto value = tbl[key].value<int64_t>();
  if (!value)
    throw std::runtime_error(std::string("Missing or invalid uint8_t field: ") +
                             key);

  return static_cast<uint8_t>(*value);
}

static std::vector<uint8_t> read_u8_neighbours(const toml::table& tbl,
                                               const char* key) {
  std::vector<uint8_t> result;

  const auto* arr = tbl[key].as_array();

  if (!arr)
    throw std::runtime_error(std::string("missing or invalid array: ") + key);

  result.reserve(arr->size());

  for (const auto& node : *arr) {
    auto value = node.value<int64_t>();

    if (!value)
      throw std::runtime_error(std::string("non-integer value in: ") + key);

    if (*value < 0 || *value > 255)
      throw std::runtime_error(std::string("uint8_t out of range in: ") + key);

    result.push_back(static_cast<uint8_t>(*value));
  }

  return result;
}

static uint32_t read_u32(const toml::table& tbl, const char* key) {
  auto value = tbl[key].value<int64_t>();
  if (!value)
    throw std::runtime_error(
        std::string("Missing or invalid uint32_t field: ") + key);

  return static_cast<uint32_t>(*value);
}

static float read_float(const toml::table& tbl, const char* key) {
  auto value = tbl[key].value<double>();
  if (!value)
    throw std::runtime_error(std::string("Missing or invalid float field: ") +
                             key);

  return static_cast<float>(*value);
}

EdgeDroneConfig load_edge_drone_config(const std::string& path) {
  auto config = toml::parse_file(path);

  auto* edge = config["edge_drone"].as_table();
  if (!edge) throw std::runtime_error("Missing [edge_drone] section");

  EdgeDroneConfig result;

  result.drone.group_uuid = read_u8(*edge, "group_uuid");
  result.drone.uuid = read_u8(*edge, "uuid");

  result.iface_storage = edge->get("iface")->value_or<std::string>("");

  result.drone.iface = result.iface_storage.c_str();

  result.drone.rx_ignore_self_injected =
      read_u8(*edge, "rx_ignore_self_injected");

  result.drone.rx_ring_size = read_u32(*edge, "rx_ring_size");

  result.drone.stream_id = read_u32(*edge, "stream_id");

  result.drone.bandwidth = read_u8(*edge, "bandwidth");

  auto uuids = read_u8_neighbours(*edge, "group");
  for (uint8_t uuid : uuids) {
    result.drone.neighbours_timeout.insert_or_assign(
        uuid, std::chrono::steady_clock::now());
  }

  result.drone.internal_uuid = read_u8(*edge, "internal_uuid");

  result.drone.internal_stream_id = read_u32(*edge, "internal_stream_id");

  auto* pos = edge->get("position")->as_table();
  if (!pos) throw std::runtime_error("Missing [edge_drone.position] section");

  result.drone.position[0] = read_float(*pos, "x");
  result.drone.position[1] = read_float(*pos, "y");
  result.drone.position[2] = read_float(*pos, "z");

  return result;
}
#pragma endregion
bool simulated = true;
bool logging = true;
int main(int argc, char* argv[]) {
  std::string config_path = "./1.toml";
  if (argc == 3) {
    if (std::string(argv[1]) == "--logging") {
      logging = true;
    } else {
      config_path = std::string(argv[1]);
    }
    config_path = std::string(argv[2]);
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
  if (simulated)
    RunExternalSimulated(drone);
  else
    RunExternal(drone);
}

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

void RunExternalSimulated(EdgeDrone drone) {
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
        WriteToUuid(0, payload.data(), payload.size());
      }

      if (buffer[0] == commandtype::NEIGHBOUR_RESPONSE_ALIVE) {
        if (logging) {
          std::cout << "RECEIVING DRONE_LASTSEEN ";
        }
        NeighbourResponse nr;
        nr.BuildFromVector(buffer);
        drone.ProcessNeighbourResponse(nr, logging);
      }
    }
    // sending and invalidation
    if (drone.last_ping_sent + std::chrono::seconds(1) <
        std::chrono::steady_clock::now()) {
      if (logging) std::cout << "SENDING ALIVE_PING ";
      auto payload = drone.BuildAlivePing(logging);
      WriteToUuid(drone.uuid + 1 % 2, payload.data(), payload.size());
      drone.last_ping_sent = std::chrono::steady_clock::now();
    }

    for (auto& kv : drone.neighbours_timeout) {
      if (kv.second + std::chrono::seconds(5) <
              std::chrono::steady_clock::now() &&
          drone.neighbours_missing_sent.find(kv.first) ==
              drone.neighbours_missing_sent.end()) {
        if (logging) std::cout << "SENDING DRONE_MISSING ";
        auto payload = drone.BuildDroneMissing(kv.first, logging);
        WriteToUuid(0, payload.data(), payload.size());
        drone.neighbours_missing_sent.insert_or_assign(
            kv.first, std::chrono::steady_clock::now());
        break;
      }
    }
    for (auto& kv : drone.neighbours_missing_sent) {
      if (kv.second + std::chrono::seconds(15) <
          std::chrono::steady_clock::now()) {
        std::cout << "GROUP connection fully lost, exiting simulation"
                  << std::endl;
        std::exit(0);
      }
    }
  }
}

void RunExternal(EdgeDrone drone) {
  uint32_t abi = wfb_rs_abi_version();
  size_t max_payload = wfb_rs_max_payload();
  // printf("wfb_rs ABI v%u, max payload=%zu\n", abi, max_payload);
  wfb_rx_handle* rx = NULL;
  wfb_rx_config rx_cfg = {
      .iface = drone.iface,
      .stream_id = drone.stream_id,
      .ignore_self_injected = drone.rx_ignore_self_injected,
      .ring_size = drone.rx_ring_size,
  };
  wfb_tx_handle* tx = NULL;
  wfb_tx_config tx_cfg = {
      .iface = drone.iface,
      .stream_id = drone.stream_id,
      .frame_type = 0x08,
      .mcs_index = 1,
      .bandwidth = drone.bandwidth,
  };
  uint32_t recv_socket = wfb_rx_open(&rx_cfg, &rx);
  uint32_t group_socket = wfb_tx_open(&tx_cfg, &tx);
  while (true) {
    // receive
    std::vector<uint8_t> buffer;
    wfb_rx_meta meta{};
    size_t metasize = sizeof(wfb_rx_meta);
    buffer.reserve(max_payload);
    int32_t bytes =
        wfb_rx_recv(rx, buffer.data(), max_payload, 50, &metasize, &meta);
    // process some meta stuff for next request
    // for now all meta information gets set on startup so its all synced

    if (bytes <= 0) continue;
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
      wfb_tx_send(tx, payload.data(), payload.size(), 1);
    }

    if (buffer[0] == commandtype::NEIGHBOUR_RESPONSE_ALIVE) {
      if (logging) {
        std::cout << "RECEIVING DRONE_LASTSEEN ";
      }
      NeighbourResponse nr;
      nr.BuildFromVector(buffer);
      drone.ProcessNeighbourResponse(nr, logging);
    }
    // sending and invalidation
    if (drone.last_ping_sent + std::chrono::seconds(1) <
        std::chrono::steady_clock::now()) {
      if (logging) std::cout << "SENDING ALIVE_PING ";
      auto payload = drone.BuildAlivePing(logging);
      wfb_tx_send(tx, payload.data(), payload.size(), 1);
      drone.last_ping_sent = std::chrono::steady_clock::now();
    }

    for (auto& kv : drone.neighbours_timeout) {
      if (kv.second + std::chrono::seconds(5) <
          std::chrono::steady_clock::now()) {
        if (logging) std::cout << "SENDING DRONE_MISSING ";
        auto payload = drone.BuildDroneMissing(kv.first, logging);
        wfb_tx_send(tx, payload.data(), payload.size(), 1);
        break;
      }
    }
  }
}
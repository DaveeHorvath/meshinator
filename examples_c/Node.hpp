#ifndef NODE_HPP
#define NODE_HPP

#include <vector>
#include "Vector3.hpp"
#include <cstdint>
#include <unordered_map>
#include <chrono>
#include <cstring>
std::vector<uint8_t> serialize_time_to_vector(
    std::chrono::steady_clock::time_point tp) {
  using namespace std::chrono;

  auto ticks = tp.time_since_epoch().count();

  std::vector<uint8_t> data(sizeof(ticks));
  std::memcpy(data.data(), &ticks, sizeof(ticks));

  return data;
}

std::chrono::steady_clock::time_point deserialize(
    const std::vector<uint8_t>& data) {
  using namespace std::chrono;

  steady_clock::rep ticks{};

  if (data.size() != sizeof(ticks)) throw std::runtime_error("invalid size");

  std::memcpy(&ticks, data.data(), sizeof(ticks));

  return steady_clock::time_point{steady_clock::duration{ticks}};
}

// 4 bit
enum commandtype : uint8_t {
  ALIVE = 1,
  NEIGHBOUR_REQUEST_ALIVE = (1 << 1),
  NEIGHBOUR_RESPONSE_ALIVE = (1 << 2),
  //   OUT_OF_GROUP_RTS = (1 << 3),
  //   OUT_OF_GROUP_CTS = (1 << 4)
};

struct General {
  uint8_t sender_uuid;
  uint8_t target_uuid;
  Vector3 sender_position;
  std::chrono::time_point<std::chrono::steady_clock> time;
  commandtype type;
  std::vector<uint8_t> toMessage() {
    std::vector<uint8_t> msg;
    msg.emplace_back(type);
    msg.emplace_back(sender_uuid);
    msg.emplace_back(target_uuid);
    uint8_t* pos_pointer = reinterpret_cast<uint8_t*>(sender_position.data);
    for (int i = 0; i < 3 * sizeof(float); i++) {
      msg.push_back(pos_pointer[i]);
    }
    std::vector<uint8_t> time_vector = serialize_time_to_vector(time);
    for (uint8_t v : time_vector) {
      msg.emplace_back(v);
    }
    return msg;
  }
};

// broadcast without target
struct AliveBroadcast {
  General general;
  void BuildFromVector(std::vector<uint8_t> data) {
    general.type = commandtype(data[0]);
    general.sender_uuid = data[1];
    general.target_uuid = data[2];
    std::vector<uint8_t> pos;
    for (int i = 0; i < 3 * sizeof(float); i++) {
      pos.push_back(data[i + 3]);
    }
    float* vec = reinterpret_cast<float*>(pos.data());
    general.sender_position = Vector3(vec[0], vec[1], vec[2]);
    std::vector<uint8_t> time;
    for (int i = 0;
         i < sizeof(std::chrono::time_point<std::chrono::steady_clock>); i++) {
      time.push_back(data[i + 3 + 3 * sizeof(float)]);
    }
    general.time = deserialize(time);
  }
  std::vector<uint8_t> BuildMessage() {
    general.type = commandtype::ALIVE;
    return general.toMessage();
  }
};

// who are your neighbours
struct NeighbourResponse {
  General general;
  uint8_t missing_uuid;
  std::chrono::time_point<std::chrono::steady_clock> thirdLastSeen;
  std::vector<uint8_t> BuildMessage() {
    general.type = commandtype::NEIGHBOUR_RESPONSE_ALIVE;
    std::vector<uint8_t> base = general.toMessage();
    std::vector<uint8_t> time = serialize_time_to_vector(thirdLastSeen);
    base.insert(base.end(), time.begin(), time.end());
    return base;
  }
  void BuildFromVector(std::vector<uint8_t> data) {
    general.type = commandtype(data[0]);
    general.sender_uuid = data[1];
    general.target_uuid = data[2];
    std::vector<uint8_t> pos;
    for (int i = 0; i < 3 * sizeof(float); i++) {
      pos.push_back(data[i + 3]);
    }
    float* vec = reinterpret_cast<float*>(pos.data());
    general.sender_position = Vector3(vec[0], vec[1], vec[2]);
    std::vector<uint8_t> time;
    for (int i = 0;
         i < sizeof(std::chrono::time_point<std::chrono::steady_clock>); i++) {
      time.push_back(data[i + 3 + 3 * sizeof(float)]);
    }
    general.time = deserialize(time);
    std::vector<uint8_t> other_time;
    for (int i = 0;
         i < sizeof(std::chrono::time_point<std::chrono::steady_clock>); i++) {
      other_time.push_back(
          data[i + 3 + 3 * sizeof(float) +
               sizeof(std::chrono::time_point<std::chrono::steady_clock>)]);
    }
    general.time = deserialize(time);
    thirdLastSeen = deserialize(other_time);
    missing_uuid =
        data[3 + 3 * sizeof(float) +
             2 * sizeof(std::chrono::time_point<std::chrono::steady_clock>)];
  }
};

struct NeigbourRequest {
  General general;
  uint8_t missing_uuid;
  void BuildFromVector(std::vector<uint8_t> data) {
    general.type = commandtype(data[0]);
    general.sender_uuid = data[1];
    general.target_uuid = data[2];
    std::vector<uint8_t> pos;
    for (int i = 0; i < 3 * sizeof(float); i++) {
      pos.push_back(data[i + 3]);
    }
    float* vec = reinterpret_cast<float*>(pos.data());
    general.sender_position = Vector3(vec[0], vec[1], vec[2]);
    std::vector<uint8_t> time;
    for (int i = 0;
         i < sizeof(std::chrono::time_point<std::chrono::steady_clock>); i++) {
      time.push_back(data[i + 3 + 3 * sizeof(float)]);
    }
    general.time = deserialize(time);
  }
  std::vector<uint8_t> BuildMessage() {
    general.type = commandtype::NEIGHBOUR_REQUEST_ALIVE;
    auto tmp = general.toMessage();
    tmp.push_back(missing_uuid);
    return tmp;
  }
};

struct EdgeDrone {
  std::chrono::time_point<std::chrono::steady_clock> start;
  uint64_t getTimeSinceStart() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               (std::chrono::steady_clock::now() - start))
        .count();
  }
  std::vector<uint8_t> neighbours;
  std::chrono::time_point<std::chrono::steady_clock> last_ping_sent;
  // drone info
  uint8_t group_uuid;
  Vector3 position;
  Vector3 velocity{1 * 0.005, 1 * 0.005, 0};
  uint8_t uuid;
  std::unordered_map<uint8_t,
                     std::chrono::time_point<std::chrono::steady_clock>>
      neighbours_timeout;
  std::unordered_map<uint8_t,
                     std::chrono::time_point<std::chrono::steady_clock>>
      neighbours_missing_sent;
  std::unordered_map<uint8_t,
                     std::chrono::time_point<std::chrono::steady_clock>>
      neighbours_missing_blocked;
  std::unordered_map<uint8_t, Vector3> neighbours_lastping;
  uint8_t internal_uuid;
  std::chrono::time_point<std::chrono::steady_clock> internal_last_msg;
  // antenna info
  const char* iface;
  uint8_t rx_ignore_self_injected;
  uint32_t rx_ring_size;
  uint32_t stream_id;
  uint8_t bandwidth;
  uint32_t internal_stream_id;

  std::vector<uint8_t> BuildAlivePing(bool logging) {
    if (logging)
      std::cout << std::to_string(uuid) << " " << position[0] << ","
                << position[1] << "," << position[2] << " "
                << getTimeSinceStart() << std::endl;
    AliveBroadcast ab;
    General general;
    general.sender_position = position;
    general.sender_uuid = uuid;
    general.target_uuid = 0x0;
    general.time = std::chrono::steady_clock::now();
    ab.general = general;
    return ab.BuildMessage();
  }

  std::vector<uint8_t> BuildDroneMissing(uint8_t missing_uuid, bool logging) {
    if (logging)
      std::cout << std::to_string(uuid) << " " << std::to_string(missing_uuid)
                << " " << std::to_string(group_uuid) << " "
                << getTimeSinceStart() << std::endl;
    NeigbourRequest nr;
    General general;
    general.sender_position = position;
    general.sender_uuid = uuid;
    general.target_uuid = 0x0;
    general.time = std::chrono::steady_clock::now();
    nr.general = general;
    nr.missing_uuid = missing_uuid;
    return nr.BuildMessage();
  }

  void ProcessAliveBroadcast(AliveBroadcast ab, bool logging) {
    if (logging)
      std::cout << std::to_string(ab.general.sender_uuid) << " "
                << ab.general.sender_position[0] << ","
                << ab.general.sender_position[1] << ","
                << ab.general.sender_position[2] << " " << getTimeSinceStart()
                << std::endl;
    neighbours_timeout.insert_or_assign(ab.general.sender_uuid,
                                        ab.general.time);
    neighbours_lastping.insert_or_assign(ab.general.sender_uuid,
                                         ab.general.sender_position);
    auto found = neighbours_missing_sent.find(ab.general.sender_uuid);
    if (found != neighbours_missing_sent.end())
      neighbours_missing_sent.erase(ab.general.sender_uuid);
  };

  std::vector<uint8_t> ProcessNeighbourRequest(NeigbourRequest nr,
                                               bool logging) {
    if (logging)
      std::cout << std::to_string(nr.general.sender_uuid) << " "
                << std::to_string(nr.missing_uuid) << " "
                << std::to_string(group_uuid) << " " << getTimeSinceStart()
                << std::endl;
    neighbours_timeout.insert_or_assign(nr.general.sender_uuid,
                                        nr.general.time);
    neighbours_lastping.insert_or_assign(nr.general.sender_uuid,
                                         nr.general.sender_position);
    NeighbourResponse response;
    response.missing_uuid = nr.missing_uuid;
    response.thirdLastSeen = neighbours_timeout.at(nr.missing_uuid);
    General general;
    general.sender_position = position;
    general.sender_uuid = uuid;
    general.target_uuid = nr.general.sender_uuid;
    general.time = std::chrono::steady_clock::now();
    response.general = general;
    return response.BuildMessage();
  }

  void ProcessNeighbourResponse(NeighbourResponse nr, bool logging) {
    if (logging)
      std::cout << std::to_string(nr.general.sender_uuid) << " "
                << std::to_string(nr.missing_uuid) << " "
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                       nr.thirdLastSeen - start)
                       .count()
                << " " << getTimeSinceStart() << std::endl;
    neighbours_timeout.insert_or_assign(nr.general.sender_uuid,
                                        nr.general.time);
    neighbours_lastping.insert_or_assign(nr.general.sender_uuid,
                                         nr.general.sender_position);
    if (std::chrono::steady_clock::now() <
        nr.thirdLastSeen + std::chrono::seconds(10)) {
      auto current = neighbours_timeout.find(nr.missing_uuid);
      if (current == neighbours_timeout.end())
        neighbours_timeout.insert_or_assign(nr.missing_uuid, nr.thirdLastSeen);
      else
        neighbours_timeout.insert_or_assign(
            nr.missing_uuid, std::max(nr.thirdLastSeen, current->second));
    } else if (logging)
      std::cout << "RECEIVE LOST_DRONE " << std::to_string(uuid) << " "
                << std::to_string(nr.missing_uuid) << " " << getTimeSinceStart()
                << std::endl;
  }
};

#endif
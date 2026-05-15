#ifndef NODE_HPP
#define NODE_HPP

#include <vector>
#include "Vector3.hpp"
#include <cstdint>
#include <unordered_map>
#include <chrono>

// 4 bit
enum commandtype : uint8_t {
  ALIVE = 0u,
  GETPOS = 1u,
  NEIGHBOUR_REQ = 2u,
  NEIGHBOUR_RSP = 3u,
  ALIVE = 8u,  // position
  OTHER = 9u
};

// 256 + time
struct General {
  uint8_t ttl;                      // 8 bits
  uint8_t sender_uuid;              // 8 bits
  uint8_t target_uuid;              // 8 bits
  uint8_t likely_global_direction;  // 8 bits
  Vector3 sender_position;          // 3 * 32bits
  std::chrono::time_point<std::chrono::steady_clock> time;
  commandtype type;  // 4 bits
  uint8_t checksum;  // 4 bits
  std::vector<uint8_t> toMessage() {
    std::vector<uint8_t> msg{20};
    msg.emplace_back(ttl);
    msg.emplace_back(sender_uuid);
    msg.emplace_back(target_uuid);
    msg.emplace_back(likely_global_direction);
    // position
    msg.emplace_back(type << 4 | 1 /* checksum */);
  }
};

// broadcast without target
struct PositionPacket {
  General general;
};

// who are your neighbours
struct NeighbourResponse {
  General general;
  std::vector<uint8_t> uuids;        // n * 8
  std::vector<uint8_t> other_close;  // m * 8
  std::vector<Vector3> positions;    // n * 32
};

class Node {
 public:
  // information from ins/gps/flight controller
  Vector3 position;
  uint8_t uuid;  // 8 bits
  std::unordered_map<uint8_t, Vector3> tier1{};
  float currentDistance = 0;

  // alive responses
  bool isWaitingAlive;
  std::chrono::time_point<std::chrono::steady_clock> endWaitingAliveTime;
  std::vector<PositionPacket> alive_response;

  void sendAlivePacket() {
    std::vector<uint8_t> b = BuildAlivePacket();
    // set radioheader
    // send packet
    (void)b;
    // set state
    isWaitingAlive = true;
    endWaitingAliveTime =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(10);
  }

  // later increasing size based on neighbours
  std::vector<uint8_t> BuildAlivePacket() {
    General g{};
    g.type = commandtype::ALIVE;
    g.ttl = 1;                      // for now
    g.likely_global_direction = 0;  // all
    g.sender_position = position;
    g.sender_uuid = uuid;
    g.target_uuid = 0;  // any
    g.time = std::chrono::steady_clock::now();
    return g.toMessage();
  }

  uint8_t GetDirection(uint8_t target) {
    // 4 directions
    Vector3 target_pos = tier1.find(target)->second;
    if (target_pos.data[0] > position.data[0]) {
      if (target_pos.data[1] > position.data[1])
        return 1;
      else
        return 2;
    } else {
      if (target_pos.data[1] > position.data[1])
        return 3;
      else
        return 4;
    }
  }

  std::vector<uint8_t> BuildNeighbourRequest(uint8_t target) {
    General g{};
    g.type = commandtype::NEIGHBOUR_REQ;
    g.ttl = 1;
    g.likely_global_direction = GetDirection(target);
    g.sender_position = position;
    g.sender_uuid = uuid;
    g.target_uuid = target;
    g.time = std::chrono::steady_clock::now();
    return g.toMessage();
  }

  std::vector<uint8_t> BuildNeighbourResponse(uint8_t from) {
    General g{};
    g.type = commandtype::NEIGHBOUR_RSP;
    g.ttl = 2;
    g.likely_global_direction = GetDirection(from);
    g.sender_position = position;
    g.sender_uuid = uuid;
    g.target_uuid = from;
    g.time = std::chrono::steady_clock::now();
    NeighbourResponse res;
    res.general = g;
    std::vector<uint8_t> base = g.toMessage();
    for (auto k : tier1) {
      base.push_back(k.first);
      uint8_t* pos = reinterpret_cast<uint8_t*>(&k.second);
      for (int i = 0; i < 4; i++) {
        base.push_back(pos[i]);
      }
    }
    return base;
  }

  void Process() {
    if (endWaitingAliveTime > std::chrono::steady_clock::now()) {
      isWaitingAlive = false;
      if (alive_response.size() < 16) {
        currentDistance += 10;
        sendAlivePacket();
      }
    }
    OnReceivingAckPosition();
  }

  void OnReceivingAckPosition(uint8_t uuid_, Vector3 pos) {
    if (!isWaitingAlive) return;
    tier1.insert_or_assign(uuid_, pos);
  }
  void OnReceivingPositioning(uint8_t uuid_, Vector3 pos) {
    tier1.insert_or_assign(uuid_, pos);
  }

  void OnReceivingNeighbourReq(uint8_t uuid_) {
    auto msg = BuildNeighbourResponse(uuid_);
    // send message at current distance
  }

  // later
  void OnCacheTimeout() {}
};

#endif
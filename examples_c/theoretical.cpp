
#include <string>
#include <cstdint>
#include <vector>
#include "Node.hpp"
#include <iostream>
#include "wfb_rs.h"

// this file should serve as example why getting the physical device to work
// is kinda required, lots of work, but didnt make the product better

void RunExternal(EdgeDrone drone, bool logging) {
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
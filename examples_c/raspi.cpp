#include <arpa/inet.h>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

#include "Node.hpp"

constexpr uint16_t MULTICAST_PORT = 5000;
constexpr const char* MULTICAST_GROUP = "239.255.0.1";
constexpr size_t MAX_PAYLOAD = 1024;

int CreateMulticastSocket(bool receiver) {
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    perror("socket");
    return -1;
  }

  int reuse = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  if (receiver) {
    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(MULTICAST_PORT);
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, reinterpret_cast<sockaddr*>(&local_addr),
             sizeof(local_addr)) < 0) {
      perror("bind");
      close(sock);
      return -1;
    }

    ip_mreq group{};
    group.imr_multiaddr.s_addr = inet_addr(MULTICAST_GROUP);
    group.imr_interface.s_addr = htonl(INADDR_ANY);

    if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &group, sizeof(group)) <
        0) {
      perror("IP_ADD_MEMBERSHIP");
      close(sock);
      return -1;
    }

    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 50000;

    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  }

  return sock;
}

void SendMulticast(int sock, const std::vector<uint8_t>& payload) {
  sockaddr_in group_addr{};
  group_addr.sin_family = AF_INET;
  group_addr.sin_port = htons(MULTICAST_PORT);
  group_addr.sin_addr.s_addr = inet_addr(MULTICAST_GROUP);

  sendto(sock, payload.data(), payload.size(), 0,
         reinterpret_cast<sockaddr*>(&group_addr), sizeof(group_addr));
}

void RunExternal(EdgeDrone drone, bool logging) {
  int rx_socket = CreateMulticastSocket(true);
  int tx_socket = CreateMulticastSocket(false);

  if (rx_socket < 0 || tx_socket < 0) {
    std::cerr << "Failed to create multicast sockets\n";
    return;
  }

  while (true) {
    std::vector<uint8_t> buffer(MAX_PAYLOAD);

    ssize_t bytes =
        recvfrom(rx_socket, buffer.data(), buffer.size(), 0, nullptr, nullptr);

    if (bytes > 0) {
      buffer.resize(bytes);

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

        SendMulticast(tx_socket, payload);
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

    if (drone.last_ping_sent + std::chrono::seconds(1) <
        std::chrono::steady_clock::now()) {
      if (logging) {
        std::cout << "SENDING ALIVE_PING ";
      }

      auto payload = drone.BuildAlivePing(logging);

      SendMulticast(tx_socket, payload);

      drone.last_ping_sent = std::chrono::steady_clock::now();
    }

    for (auto& kv : drone.neighbours_timeout) {
      if (kv.second + std::chrono::seconds(5) <
          std::chrono::steady_clock::now()) {
        if (logging) {
          std::cout << "SENDING DRONE_MISSING ";
        }

        auto payload = drone.BuildDroneMissing(kv.first, logging);

        SendMulticast(tx_socket, payload);

        break;
      }
    }
  }

  close(rx_socket);
  close(tx_socket);
}
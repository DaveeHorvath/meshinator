
#include "parsing.hpp"

#include <chrono>
#include <stdexcept>

uint8_t read_u8(const toml::table& tbl, const char* key) {
  auto value = tbl[key].value<int64_t>();

  if (!value)
    throw std::runtime_error(std::string("Missing or invalid uint8_t field: ") +
                             key);

  return static_cast<uint8_t>(*value);
}

std::vector<uint8_t> read_u8_neighbours(const toml::table& tbl,
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

uint32_t read_u32(const toml::table& tbl, const char* key) {
  auto value = tbl[key].value<int64_t>();

  if (!value)
    throw std::runtime_error(
        std::string("Missing or invalid uint32_t field: ") + key);

  return static_cast<uint32_t>(*value);
}

float read_float(const toml::table& tbl, const char* key) {
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

    result.drone.neighbours_lastping.insert_or_assign(uuid, Vector3(0, 0, 0));
  }

  result.drone.neighbours = uuids;

  result.drone.internal_uuid = read_u8(*edge, "internal_uuid");

  result.drone.internal_stream_id = read_u32(*edge, "internal_stream_id");

  auto* pos = edge->get("position")->as_table();

  if (!pos) throw std::runtime_error("Missing [edge_drone.position] section");

  result.drone.position[0] = read_float(*pos, "x");
  result.drone.position[1] = read_float(*pos, "y");
  result.drone.position[2] = read_float(*pos, "z");

  return result;
}
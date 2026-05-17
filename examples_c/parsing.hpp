#pragma once

#include "Node.hpp"
#include <toml++/toml.hpp>

struct EdgeDroneConfig {
  EdgeDrone drone{};
  std::string iface_storage;
};

uint8_t read_u8(const toml::table& tbl, const char* key);

std::vector<uint8_t> read_u8_neighbours(const toml::table& tbl,
                                        const char* key);

uint32_t read_u32(const toml::table& tbl, const char* key);

float read_float(const toml::table& tbl, const char* key);

EdgeDroneConfig load_edge_drone_config(const std::string& path);
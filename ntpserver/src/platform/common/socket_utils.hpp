// Copyright (c) 2025
/**
 * @file socket_utils.hpp
 * @brief Platform-independent socket utility functions
 */
#pragma once

// Platform-specific includes for sockaddr_in
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#endif

#include <cstring>
#include "ntpserver/platform/socket_interface.hpp"

namespace ntpserver {
namespace platform {

/**
 * @brief Convert Endpoint to sockaddr_in
 * @param endpoint Platform-independent endpoint
 * @param addr Output sockaddr_in structure
 * @return true on success, false if IP address is invalid
 */
inline bool EndpointToSockaddr(const Endpoint& endpoint, sockaddr_in* addr) {
  std::memset(addr, 0, sizeof(*addr));
  addr->sin_family = AF_INET;
  addr->sin_port = htons(endpoint.port);

  if (inet_pton(AF_INET, endpoint.address.c_str(), &addr->sin_addr) != 1) {
    return false;
  }

  return true;
}

/**
 * @brief Convert sockaddr_in to Endpoint
 * @param addr Input sockaddr_in structure
 * @return Platform-independent endpoint
 */
inline Endpoint SockaddrToEndpoint(const sockaddr_in& addr) {
  Endpoint endpoint;

  char ip[INET_ADDRSTRLEN];
  if (inet_ntop(AF_INET, &addr.sin_addr, ip, sizeof(ip)) != nullptr) {
    endpoint.address = ip;
  } else {
    endpoint.address = "";
  }

  endpoint.port = ntohs(addr.sin_port);

  return endpoint;
}

}  // namespace platform
}  // namespace ntpserver

// Copyright (c) 2025 <Your Name>
#include "ntpserver/ntp_server.hpp"

namespace ntpserver {

uint32_t MakeRefId(const char (&tag)[5]) {
  return (static_cast<uint32_t>(tag[0]) << 24) |
         (static_cast<uint32_t>(tag[1]) << 16) |
         (static_cast<uint32_t>(tag[2]) << 8) | static_cast<uint32_t>(tag[3]);
}

Options::Builder::Builder() {
  stratum_ = Options::kDefaultStratum;
  precision_ = Options::kDefaultPrecision;
  ref_id_ = Options::kDefaultRefId;
  client_retention_ = Options::kDefaultClientRetention;
}

Options::Builder& Options::Builder::Stratum(uint8_t v) {
  stratum_ = v;
  return *this;
}

Options::Builder& Options::Builder::Precision(int8_t v) {
  precision_ = v;
  return *this;
}

Options::Builder& Options::Builder::RefId(uint32_t v) {
  ref_id_ = v;
  return *this;
}

Options::Builder& Options::Builder::ClientRetention(
    std::chrono::steady_clock::duration v) {
  client_retention_ = v;
  return *this;
}

Options Options::Builder::Build() const {
  return Options(stratum_, precision_, ref_id_, client_retention_);
}

Options::Options() {
  stratum_ = kDefaultStratum;
  precision_ = kDefaultPrecision;
  ref_id_ = kDefaultRefId;
  client_retention_ = kDefaultClientRetention;
}

Options::Options(uint8_t stratum, int8_t precision, uint32_t ref_id,
                 std::chrono::steady_clock::duration retention) {
  stratum_ = stratum;
  precision_ = precision;
  ref_id_ = ref_id;
  client_retention_ = retention;
}

uint8_t Options::Stratum() const { return stratum_; }

int8_t Options::Precision() const { return precision_; }

uint32_t Options::RefId() const { return ref_id_; }

std::chrono::steady_clock::duration Options::ClientRetention() const {
  return client_retention_;
}

}  // namespace ntpserver

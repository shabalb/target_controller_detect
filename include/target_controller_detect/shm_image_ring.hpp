#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>

#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>

namespace target_controller_detect {

constexpr std::size_t kShmHeaderSize = 4096;
constexpr std::size_t kShmEncodingSize = 16;
constexpr std::array<char, 8> kShmMagic{{'O', 'A', 'K', 'S', 'H', 'M', '1', '\0'}};

struct ShmImageHeader {
  char magic[8];
  std::uint32_t version;
  std::uint32_t width;
  std::uint32_t height;
  std::uint32_t step;
  std::uint32_t frame_bytes;
  std::uint32_t slots;
  std::uint32_t data_offset;
  std::uint32_t data_size;
  std::uint64_t latest_slot;
  std::uint64_t latest_seq;
};

struct ShmImageView {
  const std::uint8_t *data{};
  std::uint32_t width{};
  std::uint32_t height{};
  std::uint32_t step{};
  std::uint32_t data_size{};
  std::uint64_t seq{};
  std::uint64_t slot{};
  std::string encoding;
};

class ShmImageRingReader {
public:
  explicit ShmImageRingReader(const std::string &name)
  : name_(name),
    shm_(boost::interprocess::open_only, name.c_str(), boost::interprocess::read_only),
    region_(shm_, boost::interprocess::read_only) {
    if (region_.get_size() < kShmHeaderSize) {
      throw std::runtime_error("Shared-memory segment is smaller than header: " + name_);
    }
    refreshHeader();
    if (std::memcmp(header_.magic, kShmMagic.data(), kShmMagic.size()) != 0) {
      throw std::runtime_error("Bad shared-memory image magic: " + name_);
    }
    if (header_.version != 1) {
      throw std::runtime_error("Unsupported shared-memory image version: " + name_);
    }
    const std::size_t min_size =
      static_cast<std::size_t>(header_.data_offset) +
      static_cast<std::size_t>(header_.slots) * header_.frame_bytes;
    if (region_.get_size() < min_size) {
      throw std::runtime_error("Shared-memory segment is truncated: " + name_);
    }
  }

  const ShmImageHeader &header() {
    refreshHeader();
    return header_;
  }

  std::string encoding() const {
    const auto *base = static_cast<const char *>(region_.get_address());
    const auto *enc = base + sizeof(ShmImageHeader);
    return std::string(std::string_view(enc, strnlen(enc, kShmEncodingSize)));
  }

  bool latest(ShmImageView &view) {
    refreshHeader();
    if (header_.slots == 0 || header_.latest_seq == 0) {
      return false;
    }

    const std::uint64_t slot = header_.latest_slot % header_.slots;
    const std::uint64_t before = slotSeq(slot);
    if (before == 0 || (before & 1U) != 0U) {
      return false;
    }

    const auto *base = static_cast<const std::uint8_t *>(region_.get_address());
    const auto *data = base + header_.data_offset + slot * header_.frame_bytes;

    const std::uint64_t after = slotSeq(slot);
    if (before != after || (after & 1U) != 0U) {
      return false;
    }

    view.data = data;
    view.width = header_.width;
    view.height = header_.height;
    view.step = header_.step;
    view.data_size = header_.data_size;
    view.seq = after;
    view.slot = slot;
    view.encoding = encoding();
    return true;
  }

private:
  void refreshHeader() {
    std::memcpy(&header_, region_.get_address(), sizeof(header_));
  }

  std::uint64_t slotSeq(std::uint64_t slot) const {
    const auto *base = static_cast<const std::uint8_t *>(region_.get_address());
    std::uint64_t seq = 0;
    std::memcpy(&seq, base + kShmHeaderSize + slot * sizeof(std::uint64_t), sizeof(seq));
    return seq;
  }

  std::string name_;
  boost::interprocess::shared_memory_object shm_;
  boost::interprocess::mapped_region region_;
  ShmImageHeader header_{};
};

}  // namespace target_controller_detect

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "target_controller_detect/shm_image_ring.hpp"

int main(int argc, char **argv) {
  std::string name = "oak_rgb";
  if (argc > 1) {
    name = argv[1];
  }

  try {
    target_controller_detect::ShmImageRingReader reader(name);
    std::uint64_t last_seq = 0;
    auto last_time = std::chrono::steady_clock::now();
    std::uint32_t count = 0;

    std::cout << "Reading shared-memory image segment: " << name << std::endl;
    while (true) {
      target_controller_detect::ShmImageView view;
      if (reader.latest(view) && view.seq != last_seq) {
        last_seq = view.seq;
        ++count;
      }

      const auto now = std::chrono::steady_clock::now();
      const auto dt = std::chrono::duration<double>(now - last_time).count();
      if (dt >= 2.0) {
        const auto &header = reader.header();
        std::cout << name << ": " << header.width << "x" << header.height
                  << " " << reader.encoding()
                  << " step=" << header.step
                  << " bytes=" << header.data_size
                  << " slots=" << header.slots
                  << " latest_slot=" << header.latest_slot
                  << " fps=" << (static_cast<double>(count) / dt)
                  << std::endl;
        count = 0;
        last_time = now;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  } catch (const std::exception &exc) {
    std::cerr << "shm_image_info error: " << exc.what() << std::endl;
    return 1;
  }
}

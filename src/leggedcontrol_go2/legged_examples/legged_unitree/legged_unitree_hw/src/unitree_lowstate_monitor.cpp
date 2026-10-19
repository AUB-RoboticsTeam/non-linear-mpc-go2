#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace {

std::atomic<bool> running{true};
std::atomic<std::uint64_t> sampleCount{0};
std::atomic<std::int64_t> lastSampleNs{0};
std::atomic<std::int64_t> maximumGapNs{0};

std::int64_t steadyNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

void stop(int) { running.store(false, std::memory_order_release); }

void updateMaximumGap(std::int64_t gapNs) {
  auto maximum = maximumGapNs.load(std::memory_order_relaxed);
  while (gapNs > maximum &&
         !maximumGapNs.compare_exchange_weak(maximum, gapNs)) {
  }
}

void lowStateCallback(const void *message) {
  if (message == nullptr) {
    return;
  }
  const auto nowNs = steadyNowNs();
  const auto previousNs = lastSampleNs.exchange(nowNs, std::memory_order_acq_rel);
  if (previousNs != 0) {
    updateMaximumGap(nowNs - previousNs);
  }
  sampleCount.fetch_add(1, std::memory_order_relaxed);
}

void usage(const char *program) {
  std::cerr << "Usage: " << program
            << " <network_interface> [topic=rt/lowstate] [domain=0] "
               "[duration_seconds=15]\n";
}

} // namespace

int main(int argc, char **argv) {
  if (argc < 2 || argc > 5) {
    usage(argv[0]);
    return 2;
  }

  const std::string interface = argv[1];
  const std::string topic = argc >= 3 ? argv[2] : "rt/lowstate";
  const int domain = argc >= 4 ? std::stoi(argv[3]) : 0;
  const double duration = argc >= 5 ? std::stod(argv[4]) : 15.0;
  if (duration <= 0.0) {
    std::cerr << "duration_seconds must be positive\n";
    return 2;
  }

  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);

  try {
    unitree::robot::ChannelFactory::Instance()->Init(domain, interface);
    auto subscriber = std::make_shared<unitree::robot::ChannelSubscriber<
        unitree_go::msg::dds_::LowState_>>(topic);
    subscriber->InitChannel(lowStateCallback, 1);

    std::cout << "READ-ONLY monitor: interface=" << interface
              << " topic=" << topic << " domain=" << domain
              << " duration=" << duration << " s\n";
    std::cout << "This process does not create or publish rt/lowcmd.\n";

    const auto started = std::chrono::steady_clock::now();
    auto previousReport = started;
    std::uint64_t previousCount = 0;

    while (running.load(std::memory_order_acquire)) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      const auto now = std::chrono::steady_clock::now();
      const auto elapsed = std::chrono::duration<double>(now - started).count();
      const auto interval =
          std::chrono::duration<double>(now - previousReport).count();
      const auto count = sampleCount.load(std::memory_order_acquire);
      const auto intervalCount = count - previousCount;
      const auto lastNs = lastSampleNs.load(std::memory_order_acquire);
      const double ageMs = lastNs == 0
                               ? -1.0
                               : (steadyNowNs() - lastNs) * 1.0e-6;
      const double maximumGapMs =
          maximumGapNs.exchange(0, std::memory_order_acq_rel) * 1.0e-6;

      std::cout << std::fixed << std::setprecision(1) << "t=" << elapsed
                << " s samples=" << count
                << " rate=" << intervalCount / interval << " Hz"
                << " age=";
      if (ageMs < 0.0) {
        std::cout << "no-data";
      } else {
        std::cout << ageMs << " ms";
      }
      std::cout << " max_gap=" << maximumGapMs << " ms\n" << std::flush;

      previousReport = now;
      previousCount = count;
      if (elapsed >= duration) {
        break;
      }
    }

    const auto total = sampleCount.load(std::memory_order_acquire);
    if (total == 0) {
      std::cerr << "FAIL: no " << topic << " samples were received\n";
      return 1;
    }
    std::cout << "DONE: received " << total << " samples\n";
    return 0;
  } catch (const std::exception &exception) {
    std::cerr << "FAIL: " << exception.what() << '\n';
    return 1;
  }
}

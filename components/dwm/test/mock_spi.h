// Written with Claude
#ifndef DWM_MOCK_SPI_H
#define DWM_MOCK_SPI_H

#include <cstddef>
#include <expected>
#include <optional>
#include <span>
#include <vector>

#include "esp_err.h"

struct MockSPI {
  std::vector<std::byte> read_response;
  std::vector<std::byte> last_write;
  std::optional<esp_err_t> fail_with;
  int transfer_count = 0;

  std::expected<void, esp_err_t>
  transfer_halfduplex(std::span<const std::byte> tx, std::span<std::byte> rx) {
    ++transfer_count;

    if (fail_with) {
      return std::unexpected(*fail_with);
    }

    // an empty rx means this is a write transaction
    if (rx.empty()) {
      last_write.assign(tx.begin(), tx.end());
    } else {
      for (size_t i = 0; i < rx.size(); ++i) {
        rx[i] = i < read_response.size() ? read_response[i] : std::byte{0};
      }
    }

    return {};
  }
};

#endif

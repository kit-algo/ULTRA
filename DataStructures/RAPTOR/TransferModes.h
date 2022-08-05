#pragma once

namespace RAPTOR {

enum TransferModes {
    MODE_WALKING = 0,
    MODE_SCOOTER = 1,
    MODE_BICYCLE = 2,
    NUM_TRANSFER_MODES = 3,
};

constexpr const char* TransferModeNames[] = {
    "walking",
    "scooter",
    "bicycle",
};

constexpr int TransferModeOverhead[] = {
        0,
        300,
        300,
};

inline size_t getTransferModeFromName(const std::string& name) noexcept {
    const std::string lowerName = String::toLower(name);
    for (size_t i = 0; i < NUM_TRANSFER_MODES; i++) {
        if (std::string(TransferModeNames[i]) == lowerName) return i;
    }
    Ensure(false, "Could not find transfer mode for name " << name << "!");
    return NUM_TRANSFER_MODES;
}

}

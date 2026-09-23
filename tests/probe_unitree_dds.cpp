// Read-only startup/connectivity probe. Requires a running simulator on domain 0.
// No command publisher is created.
#include "unitree_dds.h"
#include <unitree/idl/hg/LowState_.hpp>
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

int main()
{
    initialize_unitree_dds("lo");
    std::atomic<int> received{0};
    auto channel = unitree::robot::ChannelFactory::Instance()->CreateRecvChannel<
        unitree_hg::msg::dds_::LowState_>("rt/lowstate", [&](const void*) { ++received; }, 1);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "Received " << received.load() << " simulator low-state messages\n";
    return received.load() > 0 ? 0 : 1;
}

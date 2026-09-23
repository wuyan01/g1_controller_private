#pragma once

#include <unitree/robot/channel/channel_factory.hpp>

// The bundled Cyclone DDS defaults to only ten automatic participant slots.
// On loopback, ROS nodes can occupy all of them before Unitree starts. Keep
// domain 0 and the same interface, but allow a larger unicast discovery range.
inline void initialize_unitree_dds(const std::string& network)
{
    auto* factory = unitree::robot::ChannelFactory::Instance();
    if (network != "lo") {
        factory->Init(0, network);
        return;
    }
    const std::string xml =
        "<CycloneDDS><Domain Id=\"any\"><General><Interfaces>"
        "<NetworkInterface name=\"lo\" priority=\"default\" multicast=\"default\" />"
        "</Interfaces></General><Discovery>"
        "<ParticipantIndex>auto</ParticipantIndex>"
        "<MaxAutoParticipantIndex>99</MaxAutoParticipantIndex>"
        "</Discovery></Domain></CycloneDDS>";
    unitree::common::JsonMap config;
    config["DomainId"] = 0;
    config["Config"] = xml;
    factory->Init(config);
}

#include <Arduino.h>
// #include <Wire.h>

#include <hi_can_twai.hpp>

#include "space_resources_parameter_groups.hpp"

using namespace hi_can;
using namespace hi_can::addressing;

PacketManager packetManager;

parameters::ParameterGroup Centrifuge();
parameters::ParameterGroup Magnetometer();
parameters::ParameterGroup SpectralSensor();
parameters::ParameterGroup IlmeniteActuator();

void setup()
{
    auto& interface = TwaiInterface::getInstance(std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
                                                 addressing::filter_t{
                                                     .address = standard_address_t(space_resources::SYSTEM_ID, space_resources::controller::SUBSYSTEM_ID),
                                                     .mask = hi_can::addressing::SUBSYSTEM_MASK,
                                                 });
    packetManager.emplace(interface);
    packetManager->addGroup(Centrifuge);
    packetManager->addGroup(Magnetometer);
    packetManager->addGroup(SpectralSensor);
    packetManager->addGroup(IlmeniteActuator);
}

void loop()
{
    packetManager->handle(false);
}
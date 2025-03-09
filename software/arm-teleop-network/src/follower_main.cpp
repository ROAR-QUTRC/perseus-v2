// We're only showing the specific part that needs to be fixed in this file
// This is the _performHandshake method where the warning occurs

bool LeaderController::_performHandshake()
{
    if (!_client)
    {
        return false;
    }

    try
    {
        // Create handshake message
        protocol::handshake_msg_t msg;
        const char* identifier = "PERSEUS_ARM_CTRL";

        // Fix: Ensure null-termination after strncpy
        std::strncpy(msg.identifier, identifier, sizeof(msg.identifier) - 1);
        msg.identifier[sizeof(msg.identifier) - 1] = '\0';  // Ensure null termination

        // Serialize and send
        auto data = protocol::serializeMessage(msg);
        _client->transmit(data);

        // Wait for response
        constexpr auto HANDSHAKE_TIMEOUT = std::chrono::seconds(3);
        auto start_time = std::chrono::steady_clock::now();

        while (std::chrono::steady_clock::now() - start_time < HANDSHAKE_TIMEOUT)
        {
            auto response = _client->receive(sizeof(protocol::handshake_msg_t), false);
            if (response)
            {
                // Parse the response
                try
                {
                    auto response_msg = protocol::deserializeMessage<protocol::handshake_msg_t>(*response);

                    // Verify the identifier
                    if (std::strncmp(response_msg.identifier, "PERSEUS_ARM_CTRL", sizeof(response_msg.identifier)) == 0)
                    {
                        return true;
                    }
                }
                catch (const std::exception& e)
                {
                    _statusMessage = std::format("Invalid handshake response: {}", e.what());
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        _statusMessage = "Handshake timed out";
        return false;
    }
    catch (const std::exception& e)
    {
        _statusMessage = std::format("Handshake failed: {}", e.what());
        return false;
    }
}
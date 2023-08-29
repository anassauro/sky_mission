-- This Lua script retrieves the battery percentage from ArduPilot using MAVLink and prints it


function batVerify()

    local batteryPercentage = battery:capacity_remaining_pct(0)
    local batteryVoltage = battery:voltage(0)
    if batteryPercentage ~= nil then
        gcs:send_text(6, string.format("Battery Percentage: %.2f", batteryPercentage))


    else
        gcs:send_text(6, string.format("Battery information not available."))
    
    end

    if batteryVoltage ~= nil then
        gcs:send_text(6, string.format("Battery voltage: %.2f", batteryVoltage))


    else
        gcs:send_text(6, string.format("Battery information not available."))
    
    end

    return batVerify, 3000
end
return batVerify()

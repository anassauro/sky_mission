-- example script for using "set_origin()"" and "initialised()"
-- sets the ekf origin if not already set

count = 0

function update ()

    if not ahrs:initialised() then
        return update, 5000
    end
    
    -- origin = assert(not ahrs:get_origin(),"Refused to set EKF origin - already set")
    location = Location() location:lat(425633500) location:lng(126432900) location:alt(163000)
    
    if ahrs:set_origin(location) then
        gcs:send_text(6, string.format("Origin Set - Lat:%.7f Long:%.7f Alt:%.1f", location:lat()/10000000, location:lng()/10000000, location:alt()/100))
    else
        gcs:send_text(6, "Refused to set EKF origin")
    end

    if ahrs:home_is_set() and count < 3 then
        count = count + 1
        ahrs:set_home(ahrs:get_location())
        gcs:send_text(6, "Home position reset")
    end

    return update, 5000
end

return update()
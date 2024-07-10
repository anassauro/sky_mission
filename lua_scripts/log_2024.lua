-- This LUA script takes two zones into account, as well as the antennas (still to be implemented)
local interesting_data = {}
local lat = 1
local lng = 2
local alt = 3
local voltage = 4
local current = 5

local file_name = "teste.csv"
local file

function createPoint(lat, lon, alt, voltage, current)
    return { lat = lat, lon = lon, alt = alt, voltage = voltage, current = current }
end
gcs:send_text(6, "Started..")

function write_to_file()

    if not file then
      error("Could not open file")
    end
    local time_week = tostring(gps:time_week(0))
    local time = tostring(gps:time_week_ms(0))
  
    -- write data
    -- separate with commas and add a carriage return
    file:write(time_week .. "," .. time .. "," .. tostring(millis()) .. ", " .. tostring(interesting_data[1]) .. "," .. tostring(interesting_data[2]) .. "," .. tostring(interesting_data[3]) .. "," .. tostring(interesting_data[4]) .. "," .. tostring(interesting_data[5]) .. "\n")
  
    -- make sure file is up to date
    file:flush()
  
end

function getLocation()
    local current_pos = ahrs:get_position()
    local home = ahrs:get_home()
    local voltage = battery:voltage(0)
    local current = battery:current(0)
    
    if current_pos and home then  
        local point = createPoint(
            math.abs(current_pos:lat()/10000000),
            math.abs(current_pos:lng()/10000000),
            (current_pos:alt() - home:alt())/100,
            voltage,
            current
        )
        gcs:send_text(6, "Altitude: " .. tostring(point.alt) .. " Voltage: " .. tostring(point.voltage) .. " Current: " .. tostring(point.current))
        return point
    end

    local default = createPoint(
        math.abs(home:lat()/10000000),
        math.abs(home:lng()/10000000),
        math.abs(home:alt()),
        voltage,
        current
    )
    return default
end

-- Verify if the location is inside the polygon
-- Define a global variable to keep track of the consecutive times the point is outside the fence

local outsideCount = 0

function update()
    if arming:is_armed() then
        position = getLocation()
        -- get some interesting data
        interesting_data[1] = position.lat
        interesting_data[2] = position.lon
        interesting_data[3] = position.alt
        interesting_data[4] = position.voltage
        interesting_data[5] = position.current

        -- write to the new file on the SD card
        write_to_file()
    end
    
    return update, 1000 -- reschedules the loop
end

-- make a file
-- note that this appends to the same file each time, you will end up with a very big file
-- you may want to make a new file each time using a unique name
file = io.open(file_name, "a")
if not file then
    error("Could not make file")
end

-- write the CSV header
file:write('GPS (week), GPS(week ms), Time(ms), Lat(deg), Lng(deg), Alt(deg), Voltage(V), Current(A)\n')
file:flush()

return update, 1000

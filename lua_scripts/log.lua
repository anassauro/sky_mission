-- Geofence for IMAV Outdoor 2023
-- This LUA script takes two zones into account, as well as the antennas (still to be implemented)
local interesting_data = {}
local lat = 1
local lng = 2
local alt = 3

local file_name = "teste.csv"
local file

function createPoint(lat, lon,alt)
    return { lat = lat, lon = lon, alt = alt }
end
gcs:send_text(6, "Started..")

function write_to_file()

    if not file then
      error("Could not open file")
    end
    local time_week = tostring(gps:time_week(0))
    local time = tostring(gps:time_week_ms(0))
  
    -- write data
    -- separate with comas and add a carriage return
    file:write(time_week .. "," .. time .. tostring(millis()) .. ", " .. tostring(interesting_data[1]) .. "," .. tostring(interesting_data[2]).. ",".. tostring(interesting_data[3]) .."\n")
  
    -- make sure file is upto date
    file:flush()
  
  end

function getLocation()
    -- local location = ahrs:get_position() location:lat() location:lng()
    local current_pos = ahrs:get_position()
    local home = ahrs:get_home() home:lat() home:lng() home:alt()
    -- local position = ahrs:get_position()
    -- local altitude = position:alt()

    if current_pos and home then  
   
        local point = createPoint(math.abs(current_pos:lat()/10000000), math.abs(current_pos:lng()/10000000),(current_pos:alt()-home:alt())/100)
        gcs:send_text(6,tostring(tostring(point.alt)))
        return point
    end

    local default = createPoint( math.abs(home:lat()/10000000), math.abs(home:lng()/10000000, math.abs(home:alt())))
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
    
    -- write to then new file the SD card
    write_to_file()
    end
    
    return update, 1000 -- reschedules the loop
    end



-- make a file
-- note that this appends to the same the file each time, you will end up with a very big file
-- you may want to make a new file each time using a unique name
file = io.open(file_name, "a")
if not file then
  error("Could not make file")
end

-- write the CSV header
file:write('GPS (week), GPS(week ms), Lat(deg), Lng(deg), Alt(deg)\n')
file:flush()

return update, 1000

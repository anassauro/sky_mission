-- -- Esse script faz o drone ir para frente e para trás a uma distância e número de vezes definidas
-- -- Seus estados são :
-- --  0) Muda para o Guided mode
-- --  1) Takeoff até a altura takeoff_alt 
-- --  2) Espera até atingir a altura de takeoff
-- --  3) Aciona a missao

local takeoff_alt = 3 -- Altura de takeoff
local copter_guided_mode_num = 4
local copter_auto_mode_num = 3
local copter_land_mode_num = 9
local stage = 0

local vel = 1                 -- Velocidade do drone (m/s)


function update()

    -- Checando se o drone está armado
    if not arming:is_armed()then
         -- reset state when disarmed
        stage = 0
        gcs:send_text(6, "Arming")
        -- arming:arm()
    else
        if(stage == 0) then      -- Stage0 : Change to guided mode
              if(vehicle:set_mode(copter_guided_mode_num)) then -- change to Guided mode
                  stage = stage + 1
              end
        elseif (stage == 1) then -- Stage1 : takeoff
        gcs:send_text(6, "Taking off")
            if(vehicle:start_takeoff(takeoff_alt)) then
                stage = stage + 1
            end

        elseif (stage == 2) then --  Stage2 : check if vechile has reached target altitude
            local home = ahrs:get_home()
            local curr_loc = ahrs:get_position()
            if home and curr_loc then 
                local vec_from_home = home:get_distance_NED(curr_loc)
                gcs:send_text(6, "Altitude above home: " .. tostring(math.floor(-vec_from_home:z())))
                if(math.abs(takeoff_alt + vec_from_home:z()) < 1) then
                    stage = stage + 1
                end
            end
        elseif (stage == 3) then -- Stage3 : Mission start
            
            gcs:send_text(6, "Mission started")
            gcs:send_text(6,tostring(mission:state()))
            if(vehicle:set_mode(copter_auto_mode_num)) then -- change to Auto mode) then
                mission:set_current_cmd(1)
                stage = stage + 1
            
            end

        end
    end
    return update, 1000
end

return update()

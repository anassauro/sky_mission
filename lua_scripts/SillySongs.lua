local RUN_INTERVAL_MS = 500

-- ALL of the modes!

local COPTER_MODE_STABILIZE =     0
local COPTER_MODE_ACRO =          1
local COPTER_MODE_ALT_HOLD =      2
local COPTER_MODE_AUTO =          3
local COPTER_MODE_GUIDED =        4
local COPTER_MODE_LOITER =        5
local COPTER_MODE_RTL =           6
local COPTER_MODE_CIRCLE =        7
local COPTER_MODE_LAND =          9
local COPTER_MODE_DRIFT =        11
local COPTER_MODE_SPORT =        13
local COPTER_MODE_FLIP =         14
local COPTER_MODE_AUTOTUNE =     15
local COPTER_MODE_POSHOLD =      16
local COPTER_MODE_BRAKE =        17
local COPTER_MODE_THROW =        18
local COPTER_MODE_AVOID_ADSB =   19
local COPTER_MODE_GUIDED_NOGPS = 20
local COPTER_MODE_SMART_RTL =    21
local COPTER_MODE_FLOWHOLD =     22
local COPTER_MODE_FOLLOW =       23
local COPTER_MODE_ZIGZAG =       24
local COPTER_MODE_SYSTEMID =     25
local COPTER_MODE_AUTOROTATE =   26
local COPTER_MODE_AUTO_RTL =     27


local last_mode = vehicle:get_mode()

local function forbidden_riff()
    notify:play_tune(
        'MFT90MLO2L8<A>CEABECB>' ..
        'C<EC>C<F#D<A>F#' ..
        'EC<A>L4CL8EC<A' ..
        'GMSAL2A')
end

local function crazy_train()
    notify:play_tune(
        'MFT160MSO1L8' ..
        'F#F#>C#<F#>D<F#>C#<F#' ..
        'BAG#ABAG#E' ..
        'F#F#>C#<F#>D<F#>MLC#<B' ..
        'MNL2>DE')
end

local function axel_f()
    notify:play_tune(
        'MFT120ML' ..
        'O3L8DP8F.L16DP16DL8GL8DC' ..
        'DP8A.L16DP16DL8B-AF' ..
        'DA>D<L16DCP16CL8<A>EL2D')
end

local function jump()
    notify:play_tune(
        'MFT144ML' ..
        'O3L8DP4EP4' ..
        'CP4CP8DP8' ..
        'L4D.L8EP4CP8' ..
        'O2L4AMNGL2G')
end

local function rock_you()
    notify:play_tune('MFT180MLO3L2GF#EDMSL4EE')
end

local function star_wars()
    notify:play_tune(
        'MFT100' ..
        'O3L4F#F#F#L8D.L16A' ..
        'L4F#L8D.L16AL2F#' ..
        'O4L4C#C#C#L8D.O3L16A' ..
        'L4FL8D.L16AL2F#')
end

local function bob_barker()
    notify:play_tune(
        'MFT120ML' ..
        'O2L16FP16<GP16F' ..
        '>L2CP8.L16D<P16B-P16G' ..
        'L2>CP4L16CL8CL16E-' ..
        'L16FL8FL16E-FL8FL16E-L8F.L16DP16CP16<B-' ..
        '>L2C')
end

local function charge()
    notify:play_tune('MFT180MLL8O2CEMNGT118O3C.O2L16G.L4MLO3C')
end

local function boo()
    notify:play_tune('MFT150MSO2L8E-.E-C.MNFL4E-C')
end

function update()
    local mode = vehicle:get_mode()
    if mode ~= last_mode then
        if mode == COPTER_MODE_ALT_HOLD then rock_you() end
        if mode == COPTER_MODE_AUTO then axel_f() end
        if mode == COPTER_MODE_GUIDED then jump() end
        if mode == COPTER_MODE_ACRO then crazy_train() end
        if mode == COPTER_MODE_LAND then star_wars() end
        if mode == COPTER_MODE_LOITER then charge() end
        if mode == COPTER_MODE_CIRCLE then forbidden_riff() end
        last_mode = mode
    end
    return update, 500
end

return update, 500

-- Copter Follow-mode leader failover example.
--
-- All vehicles must use the same priority list and carry the same mission.
-- MAVLink routing must deliver the registered messages between vehicles.
--
-- This is an example, not a flight-safety-certified failsafe. Test in SITL
-- before use on a vehicle.

local Priority_List = {1, 2, 3}

local UPDATE_INTERVAL_MS = 100
local HEARTBEAT_TIMEOUT_MS = 2000
local SUSPECT_CONFIRM_MS = 5000
local ALIVE_TIMEOUT_MS = 2000
local COOLDOWN_MS = 30000

local MODE_AUTO = 3
local MODE_FOLLOW = 23

local MAV_SEVERITY = {
    CRITICAL = 2,
    ERROR = 3,
    WARNING = 4,
    NOTICE = 5,
    INFO = 6,
}

local MSG_ID = {
    HEARTBEAT = 0,
    ATTITUDE = 30,
    GLOBAL_POSITION_INT = 33,
    MISSION_CURRENT = 42,
    EXTENDED_SYS_STATE = 245,
    STATUSTEXT = 253,
}

local STATE_NORMAL = 0
local STATE_SUSPECT = 1
local STATE_CONFIRMED = 2

local function bind_parameter(...)
    for i = 1, select("#", ...) do
        local parameter = Parameter()
        if parameter:init(select(i, ...)) then
            return parameter
        end
    end
    return nil
end

local MAV_SYSID = bind_parameter("MAV_SYSID", "SYSID_THISMAV")
assert(MAV_SYSID, "leader_failover: MAV_SYSID parameter not found")

local FOLL_SYSID = bind_parameter("FOLL_SYSID")
assert(FOLL_SYSID, "leader_failover: FOLL_SYSID parameter not found")

local own_sysid = math.floor(MAV_SYSID:get())
local configured_leader = math.floor(FOLL_SYSID:get())
local current_leader = configured_leader
if current_leader <= 0 or current_leader == own_sysid then
    current_leader = Priority_List[1]
end

local state = STATE_NORMAL
local suspect_start_ms = nil
local suspect_reason = nil
local cooldown_start_ms = nil
local last_leader_mission_current = nil
local monitoring_start_ms = millis()
local last_failover_attempt_ms = nil

local vehicle_data = {}
for _, sysid in ipairs(Priority_List) do
    vehicle_data[sysid] = {
        heartbeat_ms = nil,
        position_ms = nil,
        mission_current = nil,
        landed_state = nil,
        attitude_ms = nil,
    }
end

local function get_vehicle_data(sysid)
    if vehicle_data[sysid] == nil then
        vehicle_data[sysid] = {}
    end
    return vehicle_data[sysid]
end

-- mavlink:receive_chan() returns a serialized mavlink_message_t. The payload
-- begins at byte 13, matching the generated MAVLink Lua modules.
local function decode_message(message)
    local msgid = string.unpack("<I3", message, 10)
    local decoded = {
        msgid = msgid,
        sysid = string.unpack("<B", message, 8),
        compid = string.unpack("<B", message, 9),
    }

    if msgid == MSG_ID.HEARTBEAT then
        decoded.custom_mode,
        decoded.type,
        decoded.autopilot,
        decoded.base_mode,
        decoded.system_status,
        decoded.mavlink_version = string.unpack("<I4BBBBB", message, 13)
    elseif msgid == MSG_ID.ATTITUDE then
        decoded.time_boot_ms,
        decoded.roll,
        decoded.pitch,
        decoded.yaw,
        decoded.rollspeed,
        decoded.pitchspeed,
        decoded.yawspeed = string.unpack("<I4ffffff", message, 13)
    elseif msgid == MSG_ID.GLOBAL_POSITION_INT then
        decoded.time_boot_ms,
        decoded.lat,
        decoded.lon,
        decoded.alt,
        decoded.relative_alt,
        decoded.vx,
        decoded.vy,
        decoded.vz,
        decoded.hdg = string.unpack("<I4i4i4i4i4i2i2i2I2", message, 13)
    elseif msgid == MSG_ID.MISSION_CURRENT then
        decoded.seq = string.unpack("<I2", message, 13)
    elseif msgid == MSG_ID.EXTENDED_SYS_STATE then
        decoded.vtol_state, decoded.landed_state = string.unpack("<BB", message, 13)
    elseif msgid == MSG_ID.STATUSTEXT then
        decoded.severity, decoded.text = string.unpack("<Bc50", message, 13)
        decoded.text = string.gsub(decoded.text, "%z", "")
    else
        return nil
    end

    return decoded
end

mavlink:init(25, 6)
mavlink:register_rx_msgid(MSG_ID.HEARTBEAT)
mavlink:register_rx_msgid(MSG_ID.GLOBAL_POSITION_INT)
mavlink:register_rx_msgid(MSG_ID.MISSION_CURRENT)
mavlink:register_rx_msgid(MSG_ID.STATUSTEXT)
mavlink:register_rx_msgid(MSG_ID.EXTENDED_SYS_STATE)
mavlink:register_rx_msgid(MSG_ID.ATTITUDE)

local function send_text(severity, text)
    gcs:send_text(severity, "Leader failover: " .. text)
end

local function enter_suspect(now_ms, reason)
    if state ~= STATE_NORMAL or current_leader == own_sysid then
        return
    end

    state = STATE_SUSPECT
    suspect_start_ms = now_ms
    suspect_reason = reason
    send_text(MAV_SEVERITY.WARNING,
              string.format("leader %d lost suspect (%s)", current_leader, reason))
end

local function leader_has_recovered(now_ms)
    local data = get_vehicle_data(current_leader)
    if data.heartbeat_ms == nil or data.heartbeat_ms < suspect_start_ms then
        return false
    end

    -- If this leader has supplied positions, require position recovery too.
    if data.position_ms ~= nil then
        return data.position_ms >= suspect_start_ms and
               now_ms - data.position_ms <= HEARTBEAT_TIMEOUT_MS
    end

    return true
end

local function select_new_leader(now_ms, failed_leader)
    for _, sysid in ipairs(Priority_List) do
        if sysid ~= failed_leader then
            if sysid == own_sysid then
                return sysid
            end

            local heartbeat_ms = get_vehicle_data(sysid).heartbeat_ms
            if heartbeat_ms ~= nil and now_ms - heartbeat_ms <= ALIVE_TIMEOUT_MS then
                return sysid
            end
        end
    end
    return nil
end

local function become_mission_leader(new_leader)
    if last_leader_mission_current == nil then
        send_text(MAV_SEVERITY.ERROR,
                  string.format("SYSID %d selected, mission current unknown", new_leader))
        return false
    end

    if not mission:set_current_cmd(last_leader_mission_current) then
        send_text(MAV_SEVERITY.ERROR,
                  string.format("mission command %d rejected", last_leader_mission_current))
        return false
    end

    if vehicle:get_mode() ~= MODE_AUTO and not vehicle:set_mode(MODE_AUTO) then
        send_text(MAV_SEVERITY.ERROR, "failed to enter AUTO")
        return false
    end

    send_text(MAV_SEVERITY.NOTICE,
              string.format("SYSID %d is leader, AUTO mission %d",
                            new_leader, last_leader_mission_current))
    return true
end

local function follow_new_leader(new_leader)
    if not FOLL_SYSID:set(new_leader) then
        send_text(MAV_SEVERITY.ERROR,
                  string.format("failed to set FOLL_SYSID=%d", new_leader))
        return false
    end

    -- Do not change mode during failover. In Follow mode, invalid target data
    -- causes the standard controller to command zero velocity/acceleration and
    -- yaw rate while waiting for the replacement leader.
    if vehicle:get_mode() ~= MODE_FOLLOW then
        send_text(MAV_SEVERITY.WARNING, "FOLL_SYSID changed while not in Follow mode")
    end

    send_text(MAV_SEVERITY.NOTICE,
              string.format("following new leader SYSID %d", new_leader))
    return true
end

local function attempt_failover(now_ms)
    local failed_leader = current_leader
    last_failover_attempt_ms = now_ms

    local new_leader = select_new_leader(now_ms, failed_leader)
    if new_leader == nil then
        send_text(MAV_SEVERITY.ERROR, "no alive replacement leader")
        return
    end

    local changed
    if new_leader == own_sysid then
        changed = become_mission_leader(new_leader)
    else
        changed = follow_new_leader(new_leader)
    end

    if changed then
        current_leader = new_leader
        state = STATE_NORMAL
        suspect_start_ms = nil
        suspect_reason = nil
        last_failover_attempt_ms = nil
        cooldown_start_ms = now_ms
        send_text(MAV_SEVERITY.INFO, "30 second failover cooldown started")
    end
end

local function confirm_leader_loss(now_ms)
    state = STATE_CONFIRMED
    send_text(MAV_SEVERITY.CRITICAL,
              string.format("leader %d lost confirmed (%s)",
                            current_leader, suspect_reason or "timeout"))
    attempt_failover(now_ms)
end

local function handle_message(message, receive_ms)
    local decoded = decode_message(message)
    if decoded == nil or decoded.sysid == own_sysid then
        return
    end

    local data = get_vehicle_data(decoded.sysid)
    if decoded.msgid == MSG_ID.HEARTBEAT then
        data.heartbeat_ms = receive_ms
    elseif decoded.msgid == MSG_ID.GLOBAL_POSITION_INT then
        data.position_ms = receive_ms
        data.relative_alt = decoded.relative_alt
        data.vx = decoded.vx
        data.vy = decoded.vy
        data.vz = decoded.vz
        -- alt == 0 is deliberately not used as crash evidence.
    elseif decoded.msgid == MSG_ID.MISSION_CURRENT then
        data.mission_current = decoded.seq
        if decoded.sysid == current_leader then
            last_leader_mission_current = decoded.seq
        end
    elseif decoded.msgid == MSG_ID.EXTENDED_SYS_STATE then
        data.landed_state = decoded.landed_state
    elseif decoded.msgid == MSG_ID.ATTITUDE then
        data.attitude_ms = receive_ms
        data.roll = decoded.roll
        data.pitch = decoded.pitch
        data.yaw = decoded.yaw
    elseif decoded.msgid == MSG_ID.STATUSTEXT and
           decoded.sysid == current_leader and
           string.find(decoded.text, "Crash", 1, true) ~= nil then
        enter_suspect(receive_ms, "Crash STATUSTEXT")
    end
end

local update
update = function()
    local message
    local receive_ms
    repeat
        message, _, receive_ms = mavlink:receive_chan()
        if message ~= nil then
            handle_message(message, receive_ms)
        end
    until message == nil

    local now_ms = millis()

    if cooldown_start_ms ~= nil then
        if now_ms - cooldown_start_ms < COOLDOWN_MS then
            return update, UPDATE_INTERVAL_MS
        end
        cooldown_start_ms = nil
        send_text(MAV_SEVERITY.INFO, "failover cooldown complete")
    end

    if current_leader == own_sysid then
        return update, UPDATE_INTERVAL_MS
    end

    local leader_heartbeat_ms = get_vehicle_data(current_leader).heartbeat_ms
    if state == STATE_NORMAL then
        local heartbeat_reference_ms = leader_heartbeat_ms or monitoring_start_ms
        if now_ms - heartbeat_reference_ms > HEARTBEAT_TIMEOUT_MS then
            enter_suspect(now_ms, "heartbeat timeout")
        end
    elseif state == STATE_SUSPECT then
        if leader_has_recovered(now_ms) then
            state = STATE_NORMAL
            suspect_start_ms = nil
            suspect_reason = nil
            send_text(MAV_SEVERITY.INFO,
                      string.format("leader %d recovered", current_leader))
        elseif now_ms - suspect_start_ms >= SUSPECT_CONFIRM_MS then
            confirm_leader_loss(now_ms)
        end
    elseif state == STATE_CONFIRMED and
           (last_failover_attempt_ms == nil or
            now_ms - last_failover_attempt_ms >= 1000) then
        attempt_failover(now_ms)
    end

    return update, UPDATE_INTERVAL_MS
end

send_text(MAV_SEVERITY.INFO,
          string.format("loaded SYSID %d, leader SYSID %d", own_sysid, current_leader))

return update, UPDATE_INTERVAL_MS

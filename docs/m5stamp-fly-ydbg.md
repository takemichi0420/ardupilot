# M5Stamp Fly yaw diagnostic capture

`YDBG` observes the yaw attitude/rate path. It does not tune the controller or
change the mixer, sensors, RC handling, flight modes, or scheduler rates.
This change and acquisition procedure were AI-assisted.

## Diagnostic build

The diagnostic is OFF by default. Enable it only in a diagnostic firmware:

```bash
cd ~/GitHub/ardupilot-m5stamp-fly
printf '%s\n' 'define AC_ATTITUDE_CONTROL_YAW_DIAGNOSTICS_ENABLED 1' > /tmp/m5stamp-ydbg.hwdef
. modules/esp_idf/export.sh
./waf configure --board esp32s3m5stampfly \
  --out ../m5stamp-ydbg-validation-20261107/build \
  --extra-hwdef /tmp/m5stamp-ydbg.hwdef --no-submodule-update
./waf copter -j2
```

This overlay contains only the YDBG enable flag. It does not include the ignored
local `diagnostic.hwdef` or private Wi-Fi credentials. The board's normal public
Wi-Fi defaults therefore apply to this build. Before any separately authorized
flash, confirm that the intended local Wi-Fi configuration is also represented
in the firmware. Do not publish generated headers, binaries, or private overlays.

Without this overlay, the error capture and YDBG writes compile out. The binary
produced with the overlay is a diagnostic build of the current source changes,
not the previously published, unchanged production binary.

## Record and timing

YDBG uses static AP_Logger metadata and `WriteBlock`, with no dynamic format
registration, text output, delays, or allocation added to the control loop.
The control loop only copies the error and its scheduler-loop timestamp.

| Field | Unit | Source |
|---|---|---|
| TimeUS | us since boot | Scheduler attitude-loop timestamp at logging |
| ErrUS | us since boot | Scheduler-loop timestamp when YErr was captured |
| RateUS | us since boot | `_rate_gyro_time_us`, as in RATE |
| YTarg | deg, 0–360 | `get_att_target_euler_rad().z`, as in ANG |
| Yaw | deg, 0–360 | `_ahrs.yaw`, as in ANG |
| YErr | deg | Saved `attitude_error.z` after `thrust_heading_rotation_angles` and before `update_ang_vel_target_from_att_error` |
| RTarg | deg/s | `rate_bf_targets().z`, as in RATE |
| Rate | deg/s | `_rate_gyro_rads.z`, as in RATE |
| YOut | normalized | `_motors.get_yaw() + _motors.get_yaw_ff()`, as in RATE |

The packed record is 51 bytes: three header bytes, three uint64 timestamps,
and six floats (`QQQffffff`). ANG, RATE, PIDY, and PID_TUNING keep their formats.

YDBG is requested from Copter's existing 25 Hz logging task whenever either
Fast Attitude or Medium Attitude logging is selected. No task rate is changed.
At 150 Hz loop rate this nominally gives one sample every six loops (40 ms).
The YDBG payload alone is 1275 bytes/s, before transport/startup metadata overhead.
Actual reception rate and loop impact must be measured on the hardware.

`YErr` is NaN if `ErrUS != TimeUS`, or before the first valid capture. This avoids
presenting a stale angle error in rate-only paths such as Acro. In Stabilize,
finite samples should have `ErrUS == TimeUS`. `RateUS` can precede `TimeUS` because
the latest rate-controller sample/output and the outer-loop update occupy
different points in the existing control pipeline; this is not a synchronized
snapshot of simultaneous sensor measurements.

YErr is a body-frame heading error, including the existing heading-error limit.
It is not a subtraction of Euler yaw display values. RTarg is the final target,
including existing rate limits, feedforward, thrust-error blending and system-ID
input where enabled. Opposite YErr/RTarg signs alone do not prove a controller bug.

## Backend and logging prerequisites

In this source version `LOG_BACKEND_TYPE` is a bitmask: File=1, MAVLink=2,
Block=4. Keep the existing value **2**. It selects `AP_Logger_MAVLink`.

The path is:

```text
YDBG WriteBlock → AP_Logger → AP_Logger_MAVLink buffer
    → REMOTE_LOG_DATA_BLOCK (200-byte blocks)
    → MAVProxy dataflash_logger → numbered .BIN file
```

The receiver sends `REMOTE_LOG_BLOCK_STATUS` START, ACK/NACK and STOP messages.
MAVProxy reconstructs blocks at `200 * seqno`, requests missing blocks and ACKs
received blocks. Receiving ordinary telemetry alone does not start this stream.

Before the test, read and save the current values in Mission Planner or MAVProxy:

```text
param show LOG_BACKEND_TYPE
param show LOG_BITMASK
param show LOG_MAV_RATEMAX
param show LOG_DARM_RATEMAX
param show LOG_DISARMED
```

Do not replace LOG_BITMASK with a guessed value. Relevant bits are 0=Fast
Attitude, 1=Medium Attitude, 6=RCIN, 10=RCOUT, 12=PID. Preserve the other bits.
YDBG requires bit 0 or 1. RCIN/RCOUT are logged by the existing 10 Hz task.
Fast Attitude also allows the existing PIDY/RATE producers to run faster than
their 10 Hz fallback, subject to backend limiting. Prefer preserving the current
mask if it already contains these selections.

YDBG is marked as a streaming record and respects backend rate limits. The
source default `LOG_MAV_RATEMAX=10` would limit it below the diagnostic goal.
If the current value is below 25, the user can select `LOG_MAV_RATEMAX=25` in
Mission Planner (or `param set LOG_MAV_RATEMAX 25`) for the diagnostic run, then
restore the recorded value. This affects all streaming log types, so check link
load and dropped blocks before flight. No source default is changed.

If LOG_DISARMED=0, data production may wait for arming. An optional disarmed
bench capture requires the user's explicit selection of LOG_DISARMED=1 and
review of LOG_DARM_RATEMAX; restore both afterwards. Do not ARM just to start a
bench logger. Do not change LOG_BACKEND_TYPE or controller parameters.

## WSL → Mission Planner and DataFlash capture

Verified installed tools: MAVProxy 1.8.74, pymavlink 2.4.41. The installed
`dataflash_logger` module implements this backend's protocol; an unverified
standalone `dflogger` is not needed.

Use a single MAVProxy upstream connection. Stop Mission Planner's **direct**
connection to the flight controller first, then reconnect Mission Planner as a
UDP listener on **14551**. Leave the flight controller's UDP port at **14550**.
The ESP32 UDP driver replies to the last received peer: simultaneous independent
clients talking directly to the controller can steal each other's reply path.
Mission Planner commands should return through MAVProxy's output link instead.
Do not load a second DataFlash stream requester in Mission Planner.

Set `M5STAMP_FLY_IP` to the confirmed controller address. For WSL2 NAT,
Microsoft documents the default-route gateway as the Windows host address:

```bash
export M5STAMP_FLY_IP='<confirmed-controller-IP>'
m5stamp_windows_ip=$(ip -4 route show default | awk 'NR==1 {print $3}')
test -n "$m5stamp_windows_ip" || { echo 'Confirm Windows host IP first'; exit 1; }
printf 'Mission Planner UDP destination: %s:14551\n' "$m5stamp_windows_ip"
m5stamp_log_dir="$HOME/m5stamp-ydbg-logs/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$m5stamp_log_dir"
mavproxy.py \
  --master="udpout:${M5STAMP_FLY_IP}:14550" \
  --out="udp:${m5stamp_windows_ip}:14551" \
  --source-system=255 --source-component=230 \
  --streamrate=10 \
  --logfile="$m5stamp_log_dir/mav.tlog" \
  --cmd='set flushlogs True'
```

`udpout` sends from a WSL socket and receives the controller's replies through
that same socket; it does not depend on receiving unsolicited broadcasts through
WSL NAT. This still requires network reachability and a Windows firewall rule
permitting Mission Planner's UDP listener. Confirm heartbeat/status in both
applications before attempting a test. No firewall setting is changed here.

Do **not** infer the Windows destination from `/etc/resolv.conf`: with DNS
tunneling its nameserver can be a virtual DNS endpoint, not the Windows host.
For mirrored networking or a different routing arrangement, explicitly confirm
the destination and adapt the variable; the NAT gateway command is not universal.
See [Microsoft WSL networking](https://learn.microsoft.com/en-us/windows/wsl/networking).

Once MAVProxy identifies the vehicle from HEARTBEAT, start the logger:

```text
module load dataflash_logger
dataflash_logger start
dataflash_logger status
```

Loading this module already enables START requests, so `start` is also useful
after a previous stop. Confirm `DFLogger: logging started (.../1.BIN)`, an
increasing Block count, and Missing/Abandoned counts. Files are `1.BIN`, `2.BIN`,
etc., with `LASTLOG.TXT` in the chosen log directory. A flight must not be used
as the first check that DataFlash recording works.

After landing/disarming, allow outstanding blocks to drain and inspect status.
The installed module's `stop` does not immediately send STOP in every state;
explicitly send it first using the installed standard `message` module:

```text
dataflash_logger status
module load message
message REMOTE_LOG_BLOCK_STATUS 0 155 2147483645 1
dataflash_logger stop
```

Here 0 is broadcast system, 155 is MAV_COMP_ID_LOG, 2147483645 is the protocol's
STOP sequence number, and 1 is ACK. These values were checked against this
installed pymavlink. Then **Ctrl-D** closes MAVProxy, flushing/closing the files.
Do not copy an open .BIN file and assume all buffered bytes are present.
The installed MAVProxy 1.8.74 can report a background `log_writer` exception
during process shutdown. The synthetic test's closed BIN was byte-for-byte
correct despite this warning. Check the closed BIN and telemetry files as below
before relying on a hardware capture; no installed MAVProxy code was modified.

## Files and analysis checks

- `mav.tlog`: timestamped MAVLink telemetry, including opaque remote-log blocks.
- `mav.tlog.raw`: raw received MAVLink bytes; not a decoded DataFlash log and
  not a claim of compatibility with every Mission Planner `.rlog` format.
- `1.BIN` (or later number): reconstructed AP_Logger/DataFlash binary, with FMT
  metadata and typed YDBG/ANG/RATE/PIDY/RCIN/RCOU records where enabled.
- Mission Planner can additionally keep its normal `.tlog/.rlog` files.

Check the closed DataFlash file:

```bash
mavlogdump.py --types YDBG,ANG,RATE,PIDY,RCIN,RCOU "$m5stamp_log_dir/1.BIN"
python3 - "$m5stamp_log_dir/1.BIN" <<'PY'
import math
import statistics
import sys
from pymavlink import mavutil
log = mavutil.mavlink_connection(sys.argv[1])
rows = []
while True:
    msg = log.recv_match(type='YDBG')
    if msg is None:
        break
    rows.append(msg)
assert len(rows) >= 2, 'No usable YDBG stream'
dt = [(b.TimeUS-a.TimeUS)*1e-6 for a, b in zip(rows, rows[1:])]
assert min(dt) > 0, 'Non-monotonic YDBG timestamps'
print('samples:', len(rows), 'median Hz:', 1/statistics.median(dt),
      'max gap s:', max(dt), 'fresh finite errors:',
      sum(math.isfinite(r.YErr) and r.ErrUS == r.TimeUS for r in rows))
PY
```

For the Stabilize segment, check approximately 25 Hz reception (at least the
20 Hz goal), gaps/dropped blocks, and fresh finite YErr. Retain DMS logger
statistics and performance logs to assess transport/loop load. No hardware rate
or 150 Hz loop timing is guaranteed by a successful compile or synthetic test.

Align DataFlash TimeUS/ErrUS/RateUS with MAVLink boot-time fields such as
ATTITUDE/RC_CHANNELS/SERVO_OUTPUT_RAW and SYSTEM_TIME, not just the PC arrival
timestamp. Account for reboots, clock offset, stream cadence and UDP latency.
Motor outputs are in RCOU/servo telemetry; RC4 is in RCIN/RC_CHANNELS. The library
does not acquire a new RC-subsystem dependency.

Flash and hardware/flight tests remain separate, user-confirmed work. For the
requested short Stabilize test, capture neutral/right/neutral/left/neutral yaw
segments and landing, then compare RC4 → YTarg/Yaw → YErr → RTarg/Rate → YOut →
motor outputs. Avoid changing gains between compared segments. This procedure
does not by itself establish the cause of the observed yaw motion.

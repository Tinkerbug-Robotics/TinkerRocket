"""What is in *this* flight log — every channel, and everything we cannot read.

The catalog answers three questions the report has never been able to answer:

  1. Which channels does this log actually contain, at what rate, over what span?
  2. What does each one mean, and what will you get wrong if you plot it naively?
  3. What is in the file that this tool cannot decode at all?

It is built by walking `flight.records` — never from a static schema. That
matters: the flight computer has shipped five NonSensor wire formats (42/43/44/
48/50, `plot_flight_data_mini.NONSENSOR_KNOWN_LENS`) and two Guidance formats,
so a hard-coded field list would confidently advertise channels a given
firmware version never logged. Both mobile apps drive their chart pickers from
exactly such a table and, between them, hide sixteen columns they themselves
write. A catalog built from the parsed records cannot drift that way.

`PROVENANCE` supplies the human half — label, unit, and the cautions that
currently live only as comments in the parser. A channel with no PROVENANCE
entry still gets a row (`documented=False`), because the log is the authority
on what exists and the table is only the authority on what it means; a test
asserts the golden flight has no undocumented channels, so adding a field to
the parser without describing it here fails CI rather than shipping a bare name.
"""

from __future__ import annotations

from dataclasses import dataclass, field as _field
from typing import TYPE_CHECKING, Any, Iterable, Optional

if TYPE_CHECKING:
    from .flight import Flight

# Channel kinds. `raw` is called out separately from `float` because an
# uncalibrated ADC/sensor count is only meaningful against another raw count —
# plotting it next to an SI series is a category error, not a scaling one.
KIND_FLOAT = "float"
KIND_BOOL = "bool"
KIND_ENUM = "enum"
KIND_COUNTER = "counter"
KIND_RAW = "raw"

# Frame types that carry no samples, so their absence from `records` is correct
# rather than a decoding gap. Everything else the parser counted but did not
# turn into records is reported to the user as unreadable.
_CONTROL_FRAMES = frozenset({"StartLogging", "EndFlight", "OUT_STATUS_QUERY"})


@dataclass(frozen=True)
class ChannelMeta:
    """The human half of a channel: what it is, and what it is not."""

    label: str
    unit: str = ""          # a units.CONVERSIONS key, or "" when not convertible
    kind: str = KIND_FLOAT
    note: str = ""          # what it actually is, incl. scaling the parser applied
    caution: str = ""       # a trap someone plotting this would fall into
    # Sections where you can actually SEE this channel — its own values on an
    # axis, or, for a flag, its transition drawn as an event marker.
    shown_in: tuple[str, ...] = ()
    # Sections that CONSUME the channel to produce something else. The
    # distinction is the whole point of the catalog: `stability` reads q0..q3 and
    # draws a tilt angle, so a reader told "q0 — shown in: stability" would go
    # looking for a quaternion trace that does not exist and conclude the report
    # already covers it. Derived use is worth knowing about; it is not seeing it.
    derived_in: tuple[str, ...] = ()
    # Which CONVERSIONS row drives the imperial toggle, when it is not `unit`.
    # One SI unit can want two imperial idioms: a vertical rate belongs in ft/s
    # and a horizontal speed in mph, and units.py keeps "m/s:fps" for exactly
    # that. Carrying it here rather than at the call site is the point of a
    # provenance table — the channel knows which idiom it is, the plotter does not.
    conv: str = ""

    @property
    def conv_key(self) -> str:
        return self.conv or self.unit


@dataclass
class Channel:
    """One plottable series in this log: its meaning plus what it measured."""

    stream: str
    field: str
    meta: ChannelMeta
    documented: bool
    n_total: int                     # records in the stream
    n_present: int                   # values that are not None
    rate_hz: Optional[float] = None
    t_start_s: Optional[float] = None  # seconds from the flight's t0
    t_end_s: Optional[float] = None
    vmin: Optional[float] = None
    vmax: Optional[float] = None

    @property
    def key(self) -> str:
        return f"{self.stream}.{self.field}"

    @property
    def label(self) -> str:
        return self.meta.label

    @property
    def is_empty(self) -> bool:
        """True when the channel exists but never carried a value.

        Distinct from a stream being absent: `ekf_ticks` is a real field on a
        50-byte log and `None` on every row of a 48-byte one, and a picker that
        offers it silently draws nothing.
        """
        return self.n_present == 0


@dataclass
class Unreadable:
    """A frame type present in the log that the parser produced nothing from."""

    name: str            # "0xD2", or a name from MSG_NAMES that has no decoder
    count: int
    reason: str


@dataclass
class Catalog:
    channels: list[Channel] = _field(default_factory=list)
    unreadable: list[Unreadable] = _field(default_factory=list)
    dropped: list[str] = _field(default_factory=list)  # decoded then discarded

    def streams(self) -> list[str]:
        """Stream names in catalog order (first appearance)."""
        seen: dict[str, None] = {}
        for c in self.channels:
            seen.setdefault(c.stream, None)
        return list(seen)

    def for_stream(self, stream: str) -> list[Channel]:
        return [c for c in self.channels if c.stream == stream]

    def undocumented(self) -> list[Channel]:
        return [c for c in self.channels if not c.documented]

    def get(self, key: str) -> Optional[Channel]:
        return next((c for c in self.channels if c.key == key), None)


# ---------------------------------------------------------------------------
# Provenance
# ---------------------------------------------------------------------------
# Keyed "STREAM.field". Filled in below by _P(); see the module docstring for
# why an entry is documentation rather than a schema.

PROVENANCE: dict[str, ChannelMeta] = {}


def _P(key: str, label: str, unit: str = "", kind: str = KIND_FLOAT,
       note: str = "", caution: str = "", shown_in: Iterable[str] = (),
       conv: str = "", derived_in: Iterable[str] = ()) -> None:
    PROVENANCE[key] = ChannelMeta(
        label=label, unit=unit, kind=kind, note=note, caution=caution,
        shown_in=tuple(shown_in), conv=conv, derived_in=tuple(derived_in),
    )


# The OutComputer's own INA230 pack monitor (bus voltage, current, and a voltage-derived SOC),
# sampled on a 10 ms schedule but landing at roughly 55-72 Hz in the example logs, and
# timestamped on the flight computer's clock only while the FC link is live — otherwise it
# falls back to the OC's own micros(), a different origin.
_P(
    "POWER.voltage", "Battery voltage",
    unit="V",
    note=
        "INA230 bus voltage of the flight pack, decoded from a uint16 over a 0-10 V full scale. Also "
        "the input to the battery verdict in the health section.",
    caution=
        "Firmware rejects any reading outside 3.0-9.0 V and re-logs the previous good sample, so a "
        "brownout shows up as a flat trace and then a gap, never as a low voltage.",
    shown_in=()
)
_P(
    "POWER.current", "Battery current",
    unit="mA",
    note=
        "INA230 shunt current draw, decoded from an int16 over a +/-10 A full scale. The parser "
        "returns milliamps, not amps.",
    caution=
        "Sign is inverted in firmware, so discharge reads negative and an all-negative trace is "
        "normal draw. The encoder clamps at +/-10000 mA, so a pyro firing above 10 A saturates flat "
        "instead of spiking.",
    shown_in=()
)
_P(
    "POWER.soc", "State of charge",
    unit="%",
    note=
        "Pack state of charge in percent, decoded from an int16 spanning a -25 to +125 % encodable "
        "range. Computed on the OutComputer from bus voltage alone.",
    caution=
        "This is a piecewise-linear lookup of the same bus voltage against a 2S LiPo curve, not "
        "coulomb counting, so it sags and recovers with load. On any other supply it clamps: 5 V "
        "bench runs report a flat 0%.",
    shown_in=()
)

# The old-PCB magnetometer (message 0xA4, roughly 100 Hz), mutually exclusive with the new-PCB
# IIS2MDC stream and timestamped on the flight computer's micros() clock — the same origin as
# the IMU, barometer, and NonSensor streams.
_P(
    "MMC5983MA.mag_x", "Mag X (rocket)",
    unit="µT",
    note=
        "Magnetic field along the rocket's X axis, in microtesla. Raw 18-bit counts are centred and "
        "scaled at 0.006104 µT/count, then rotated out of the chip frame into the rocket frame.",
    caution=
        "The standalone plotting script disables this figure marked \"X axis broken\", so treat X as "
        "suspect. It is also uncalibrated — no hard- or soft-iron correction is applied.",
    shown_in=()
)
_P(
    "MMC5983MA.mag_y", "Mag Y (rocket)",
    unit="µT",
    note=
        "Magnetic field along the rocket's Y axis, in microtesla, from centred 18-bit counts at "
        "0.006104 µT/count. Rotated into the rocket frame before it reaches the report.",
    caution=
        "Uncalibrated: no hard-iron or soft-iron bias is removed, so the field magnitude will not "
        "match the local Earth field and any heading computed from it inherits that bias.",
    shown_in=()
)
_P(
    "MMC5983MA.mag_z", "Mag Z (rocket)",
    unit="µT",
    note=
        "Magnetic field along the rocket's Z axis, in microtesla, on the same 0.006104 µT/count "
        "scaling. The chip rotation is about Z, so only the board-to-rocket quaternion moves this "
        "axis.",
    caution=
        "Uncalibrated like the other two axes — no bias or scale correction anywhere in the parser — "
        "so read it as a relative field trace, not an absolute measurement.",
    shown_in=()
)

# The OutComputer's own log-ring snapshot, emitted into the flight log about once a second
# while a log session is open and stamped with the OC's clock rather than the flight
# computer's, so it shares no time axis with the sensor streams.
_P(
    "LogBufferStats.ring_size", "Ring capacity",
    unit="bytes",
    note=
        "Size of the log ring in bytes. Set once when the session opens and constant for the rest of "
        "the flight.",
    caution=
        "This is the ring's runtime capacity, not the memory chip's size (130048 B on a 131072 B "
        "part), so a \"percent of chip used\" derived from it double-counts the reserved header.",
    shown_in=()
)
_P(
    "LogBufferStats.ring_fill", "Ring fill",
    unit="bytes",
    note=
        "Bytes resident in the ring at the instant of the snapshot, free to fall as well as rise.",
    caution=
        "The flat plateau before launch is the pre-launch rolling window sitting at its firmware cap, "
        "not a stalled flush; activation releases the cap and fill drains to a few hundred bytes.",
    shown_in=()
)
_P(
    "LogBufferStats.ring_highwater", "Ring high-water",
    unit="bytes", kind=KIND_COUNTER,
    note=
        "Highest ring fill in bytes, monotonic. This is the peak the MRAM-sizing viability table "
        "reports.",
    caution=
        "Peak since the OutComputer booted, not since this flight — on a second flight of one power- "
        "up it arrives already elevated and may never move, so check it actually rose during the log.",
    shown_in=()
)
_P(
    "LogBufferStats.ring_overruns", "Ring overruns",
    kind=KIND_COUNTER,
    note="Count of ring overrun events, cumulative since the OutComputer booted.",
    caution=
        "Dominated by the pre-launch window discarding its oldest frame on every push, so the raw "
        "total reads as catastrophic on a healthy flight; only the rise after logging activation "
        "means live frames were lost.",
)
_P(
    "LogBufferStats.ring_drop_oldest_bytes", "Bytes dropped from tail",
    unit="bytes", kind=KIND_COUNTER,
    note=
        "Bytes discarded from the tail of the ring to make room for new frames, cumulative since "
        "boot.",
    caution=
        "Nearly all of the total — often tens of millions of bytes — is ordinary pad turnover before "
        "launch. Only a rise after activation is abnormal, and it means the session restarted mid- "
        "flight.",
)
_P(
    "LogBufferStats.ring_bad_sof_clears", "Bad-SOF ring clears",
    kind=KIND_COUNTER,
    note=
        "Times a corrupt frame header forced the whole ring to be cleared, cumulative since boot. "
        "Zero on every flight log in the repo.",
    caution=
        "A ring clear discards buffered frames wholesale, so this counts destroyed log data. It is "
        "cumulative since OC boot and the report shows only the final value, so a non-zero reading "
        "may have been inherited from an earlier flight — take a delta across the log before blaming "
        "this one.",
)

# New-PCB magnetometer (msg 0xD1), read by a time-gated I2C poll at a nominal 100 Hz — so
# cadence is poll-throttled rather than data-ready strobed — and timestamped on the same
# flight-computer clock as the other sensor streams.
_P(
    "IIS2MDC.mag_x", "Mag X (rocket)",
    unit="µT",
    note=
        "Magnetic field along the rocket's X axis, in µT. Derived rather than logged: raw int16 "
        "counts scaled at 0.15 µT/LSB, rotated about the chip's +Z axis, then mapped into the rocket "
        "frame.",
    caution=
        "Hard-iron calibration is applied inside the chip's offset registers before anything is "
        "logged, so a calibrated and an uncalibrated board produce records that look identical and "
        "nothing in the log says which you have.",
    shown_in=()
)
_P(
    "IIS2MDC.mag_y", "Mag Y (rocket)",
    unit="µT",
    note=
        "Magnetic field along the rocket's Y axis in µT, same 0.15 µT/LSB scaling as X. The chip-Z "
        "rotation mixes the sensor's own X and Y, so neither axis maps one-to-one onto a physical "
        "sensor axis.",
    caution=
        "Same hidden calibration state as mag_x: any hard-iron correction happened in the chip before "
        "logging, and the log carries no flag saying whether it did.",
    shown_in=()
)
_P(
    "IIS2MDC.mag_z", "Mag Z (rocket)",
    unit="µT",
    note=
        "Magnetic field along the rocket's Z axis in µT, from raw int16 counts at 0.15 µT/LSB. A "
        "rotation about Z cannot change Z, so this axis is untouched by the chip-Z rotation.",
    caution=
        "Any hard-iron calibration was applied in the chip's offset registers before logging, so the "
        "record cannot tell you whether the board was calibrated.",
    shown_in=()
)

# Bosch BMP585 barometer logging static pressure and die temperature at a measured ~495 Hz —
# the fastest non-IMU stream — timestamped on the flight computer's micros() clock, the same
# origin as ISM6HG256, GNSS and NonSensor (unlike LogBufferStats).
_P(
    "BMP585.pressure_pa", "Static pressure",
    unit="Pa",
    note=
        "Absolute static pressure at the barometer port. The wire value is Q6 (Pa x 64) and the "
        "parser divides by 64, so this is already pascals.",
    caution=
        "Firmware itself rejects barometer samples through the transonic window (a Mach lockout "
        "against EKF speed), so raw pressure there is data the flight computer refused to use. The "
        "ejection charge also puts a large transient on it.",
    shown_in=("overview", "deployment", "kinematic_checks"),
)
_P(
    "BMP585.temperature", "Barometer die temperature",
    unit="°C",
    note=
        "Internal die temperature the sensor uses to compensate its own pressure reading. The Q16 "
        "wire value is scaled by the parser, so this is degrees Celsius.",
    caution=
        "Not ambient air temperature — the die sits sealed in the airframe beside powered electronics "
        "and reads far above outside air (44-46 C on the pad). Do not use it for a density-altitude "
        "or lapse-rate correction.",
    shown_in=()
)

# Six-axis IMU (low-G accel, high-G accel, gyro) logged as raw counts at ~960 Hz and converted
# in a post-pass into rocket-body axes; its time_us shares the clock origin with GNSS, BMP585,
# NonSensor and POWER, but not with LogBufferStats.
_P(
    "ISM6HG256.low_acc_x", "Low-G Accel X (body)",
    unit="m/s²",
    note=
        "Low-range accelerometer on the rocket's long axis, in body axes after the chip and board "
        "rotations. The finer of the two accelerometers, and the one that carries the boost detail.",
    caution=
        "Rails at ±16 g; the −45° chip rotation smears the clipped value up to ~222 m/s² rather than "
        "a flat top, so read true peaks off high_acc_x.",
    shown_in=("overview", "globe"),
)
_P(
    "ISM6HG256.low_acc_y", "Low-G Accel Y (body)",
    unit="m/s²",
    note=
        "Low-range accelerometer, lateral body Y. Same scaling and rotation chain as the X axis.",
    caution=
        "Same ±16 g rail as X, and the −45° chip rotation gives a saturated stretch a ragged top "
        "rather than a flat one.",
    shown_in=("overview", "globe"),
)
_P(
    "ISM6HG256.low_acc_z", "Low-G Accel Z (body)",
    unit="m/s²",
    note=
        "Low-range accelerometer, lateral body Z. The chip rotation is about Z, so this axis is not "
        "mixed into the other two.",
    caution=
        "Rails at exactly ±16 g (±156.9 m/s²) — a trace sitting flat on that value is saturated, not "
        "measured.",
    shown_in=("overview", "globe"),
)
_P(
    "ISM6HG256.high_acc_x", "High-G Accel X (body)",
    unit="m/s²",
    note=
        "High-range (±256 g) accelerometer on the long axis. Only the high-G axes have the logged "
        "bias subtracted; the low-G ones do not.",
    caution=
        "16× coarser than low-G, so small accelerations here are mostly quantization. It is the only "
        "channel that reads correctly above ~16 g.",
    shown_in=("overview", "globe"),
)
_P(
    "ISM6HG256.high_acc_y", "High-G Accel Y (body)",
    unit="m/s²",
    note=
        "High-range accelerometer, lateral body Y. Same ±256 g scaling, bias subtraction and rotation "
        "chain as the X axis.",
    caution=
        "About 0.077 m/s² per count — a quiet stretch here is quantization noise, not vehicle motion.",
    shown_in=("overview", "globe"),
)
_P(
    "ISM6HG256.high_acc_z", "High-G Accel Z (body)",
    unit="m/s²",
    note=
        "High-range accelerometer, lateral body Z. Same ±256 g scaling, bias subtraction and rotation "
        "chain as the other high-G axes.",
    caution=
        "About 0.077 m/s² per count — a quiet stretch here is quantization noise, not vehicle motion.",
    shown_in=("overview", "globe"),
)
_P(
    "ISM6HG256.gyro_x", "Gyro X (roll rate)",
    unit="deg/s",
    note=
        "Angular rate about the rocket's long axis, i.e. roll rate. Scaled by the ST per-full-scale "
        "sensitivity (0.14 °/s per count at ±4000 dps), not full-scale/32768.",
    caution=
        "A roll RATE, not the roll ANGLE in NonSensor.roll (a ZYX-Euler angle, #514) — never overlay "
        "or difference the two. Rails at ±4000 dps on a fast spin.",
    shown_in=("roll", "roll_pid"),
)
_P(
    "ISM6HG256.gyro_y", "Gyro Y (body)",
    unit="deg/s",
    note=
        "Angular rate about body Y, roughly pitch rate. Same sensitivity and rotation chain as the "
        "other gyro axes, so X and Y are mixed by the chip rotation.",
    caution=
        "A rate in °/s, not the NonSensor pitch angle — the two are not comparable. Saturates at the "
        "same ±4000 dps full scale as the other gyro axes.",
    shown_in=(),
)
_P(
    "ISM6HG256.gyro_z", "Gyro Z (body)",
    unit="deg/s",
    note=
        "Angular rate about body Z, roughly yaw rate. The chip rotation is about this axis, so only "
        "the board-to-rocket quaternion moves it.",
    caution=
        "A rate in °/s, not the NonSensor yaw/azimuth angle — do not plot them together. Saturates at "
        "the same ±4000 dps full scale as the other gyro axes.",
    shown_in=(),
)

# u-blox NAV-PVT solutions read over serial at roughly 18 Hz and timestamped with the flight
# computer's own micros() clock at the moment it read the fix, so every sample trails the
# receiver's solution instant by the output latency.
_P(
    "GNSS.year", "UTC year",
    kind=KIND_COUNTER,
    note=
        "Calendar year of the GNSS solution, UTC. Constant across a flight; it exists to tie a log to "
        "a wall-clock date.",
)
_P(
    "GNSS.month", "UTC month",
    kind=KIND_COUNTER,
    note="Calendar month of the GNSS solution, 1-12, UTC.",
)
_P(
    "GNSS.day", "UTC day",
    kind=KIND_COUNTER,
    note="Day of month of the GNSS solution, 1-31, UTC.",
)
_P(
    "GNSS.hour", "UTC hour",
    kind=KIND_COUNTER,
    note=
        "UTC hour of the solution instant, 0-23. With minute/second/milli_sec it identifies a "
        "solution the receiver repeated rather than recomputed.",
    caution=
        "Wraps to 0 at midnight UTC, so a flight spanning that boundary puts a backward jump in any "
        "time-of-day series built from it.",
)
_P(
    "GNSS.minute", "UTC minute",
    kind=KIND_COUNTER,
    note="UTC minute of the solution instant, 0-59.",
)
_P(
    "GNSS.second", "UTC second",
    kind=KIND_COUNTER,
    note="UTC second of the solution instant, 0-59. No leap-second field is logged.",
)
_P(
    "GNSS.milli_sec", "UTC millisecond",
    kind=KIND_COUNTER,
    note="Millisecond within the solution second, 0-999.",
)
_P(
    "GNSS.fix_mode", "Fix mode",
    kind=KIND_ENUM,
    note=
        "u-blox fix type: 0 none, 1 dead reckoning, 2 2D, 3 3D, 4 GNSS+DR, 5 time-only. Modules that "
        "need a position gate on fix_mode >= 3.",
    caution=
        "A 0 is not only \"no satellites\" — the driver zeroes it whenever the receiver flags the fix "
        "invalid, which is also how u-blox signals a COCOM speed/altitude violation (#562). Sat count "
        "can stay high there.",
    shown_in=()
)
_P(
    "GNSS.num_sats", "Satellites used",
    kind=KIND_COUNTER,
    note="Number of satellites the receiver used in the navigation solution.",
    caution=
        "A healthy count can sit on top of a rejected fix: fix_mode can be forced to 0 while the sat "
        "count stays in the teens, so read the two together.",
    shown_in=("globe"),
)
_P(
    "GNSS.pdop", "PDOP",
    note=
        "Position dilution of precision, dimensionless. Logged as ten times PDOP, so values arrive in "
        "steps of 0.1.",
    caution=
        "The logged value clamps at 25.5, so 25.5 means \"at least that\", and 0 means no solution "
        "rather than a perfect one. Filter to values above 0 before plotting.",
)
_P(
    "GNSS.lat", "Latitude",
    unit="°",
    note=
        "WGS84 latitude in degrees, from an integer field scaled by 1e-7 (about 1 cm resolution).",
    caution=
        "Before the first fix the receiver reports 0/0. Mask on latitude and longitude both being "
        "zero, or the track is dragged to the Gulf of Guinea.",
    shown_in=("globe", "deployment"),
)
_P(
    "GNSS.lon", "Longitude",
    unit="°",
    note="WGS84 longitude in degrees, from an integer field scaled by 1e-7.",
    caution=
        "Same 0/0 pre-fix sentinel as latitude; test both coordinates together so a genuine equator "
        "or prime-meridian fix is not thrown away.",
    shown_in=("globe", "deployment"),
)
_P(
    "GNSS.alt_m", "Altitude (MSL)",
    unit="m",
    note="Receiver altitude above mean sea level, logged in millimetres and scaled to metres.",
    caution=
        "MSL, not above-ground and not ellipsoidal: it sits at field elevation and will not line up "
        "with the barometric trace until you subtract a pad reference.",
    shown_in=("overview", "globe", "deployment"),
)
_P(
    "GNSS.vel_e", "Velocity East",
    unit="m/s",
    note=
        "East component of the receiver's Doppler velocity solution, logged in mm/s and scaled to "
        "m/s.",
    caution=
        "This is the receiver's own Doppler solution, independent of the EKF's e_vel; the two share "
        "an axis in the velocity charts but are not interchangeable.",
    shown_in=("overview"),
)
_P(
    "GNSS.vel_n", "Velocity North",
    unit="m/s",
    note=
        "North component of the receiver's Doppler velocity solution, logged in mm/s and scaled to "
        "m/s.",
    caution=
        "Comes from the receiver's Doppler solution, not the nav filter — agreement with the EKF's "
        "n_vel is a cross-check, not a given, and one cannot stand in for the other.",
    shown_in=("overview"),
)
_P(
    "GNSS.vel_u", "Velocity Up",
    unit="m/s", conv="m/s:fps",
    note=
        "Vertical velocity, positive up: the receiver's down velocity negated, logged in mm/s and "
        "scaled to m/s.",
    caution=
        "Receiver Doppler, not the EKF's u_vel, and not a substitute for it. Trusted over the "
        "barometric apogee peak, which lags it by 0.3-1.3 s (#112) — do not correct one to the other.",
    shown_in=("overview", "apogee"),
)
_P(
    "GNSS.h_acc_m", "GNSS horizontal accuracy",
    unit="m",
    note=
        "Receiver's own estimate of horizontal position accuracy. The firmware rounds it to whole "
        "metres before logging.",
    caution=
        "Quantized to 1 m, clamped at 255, and identically 0 for entire flights on some logs, so it "
        "cannot show a sub-metre trend.",
)
_P(
    "GNSS.v_acc_m", "GNSS vertical accuracy",
    unit="m",
    note=
        "Receiver's own estimate of vertical position accuracy, drawn as the shaded band around the "
        "GNSS altitude trace. Rounded to whole metres before logging.",
    caution=
        "It is 0 for entire flights on some logs, which collapses the band onto the altitude line — "
        "an apparently tight band can mean no estimate was recorded at all.",
    shown_in=()
)

# Flight-computer proportional-navigation telemetry (GUIDANCE_TELEM_MSG 0xCA), emitted only
# while guidance is engaged and at the NonSensor update rate — read the rate from the log,
# since the parser comments saying ~10 Hz are stale — on the same monotonic FC clock as
# NonSensor, so rows align 1:1 with no interpolation.
_P(
    "Guidance.accel_cmd_n", "PN accel command (North)",
    unit="m/s²",
    note=
        "Horizontal North component of the PN acceleration command. There is no vertical component in "
        "the log.",
    caution=
        "A command, not achieved acceleration — do not overlay it on accelerometer data. Past ±327.67 "
        "m/s² it wraps sign instead of clipping, which is what the range→0 terminal singularity "
        "drives it into.",
    shown_in=("guidance",)
)
_P(
    "Guidance.accel_cmd_e", "PN accel command (East)",
    unit="m/s²",
    note="Horizontal East component of the PN acceleration command, the partner of accel_cmd_n.",
    caution=
        "Commanded, not measured, acceleration, and it wraps sign past ±327.67 m/s² rather than "
        "clipping — the same overflow the range→0 singularity produces.",
    shown_in=("guidance",)
)
_P(
    "Guidance.lateral_offset", "Lateral offset from pad",
    unit="m",
    note=
        "Horizontal distance from the vertical line above the pad. Logged in centimetres; the parser "
        "converts to metres.",
    caution=
        "Magnitude caps at ±327.67 m and wraps sign rather than clipping, so a trace that flips "
        "negative near peak drift is an overflow, not a course reversal.",
    shown_in=("guidance",)
)
_P(
    "Guidance.los_angle", "LOS angle to target",
    unit="°",
    note="Line-of-sight angle from the vehicle to the guidance aim point.",
    caution=
        "Packed as degrees×100 in an int16, so it saturates and sign-wraps past ±327.67°. Being an "
        "angle it has no documented branch cut either, so a full-turn jump in the trace is not "
        "necessarily an error.",
    shown_in=("guidance",)
)
_P(
    "Guidance.closing_vel", "Closing velocity",
    unit="m/s",
    note=
        "Closure rate against the guidance aim point — the overhead point above the pad unless a "
        "target was uplinked in flight.",
    caution=
        "Saturates at ±327.67 m/s and wraps sign; a fast flight closing on a near aim point can run "
        "past it.",
    shown_in=("guidance",)
)
_P(
    "Guidance.pitch_fin_cmd", "Pitch fin command",
    unit="°",
    note=
        "The guidance law's pitch deflection demand, the guidance analogue of the NonSensor roll "
        "command.",
    caution=
        "Pre-mix and pre-clamp: this is the demand before the mixer and the ±fin_max limit, so it is "
        "not a servo angle and can exceed one. It is None on every row of a 15-byte legacy log.",
    shown_in=("guidance",)
)
_P(
    "Guidance.yaw_fin_cmd", "Yaw fin command",
    unit="°",
    note=
        "The guidance law's yaw deflection demand; the actual four fin deflections are never logged.",
    caution=
        "Same two traps as the pitch demand: pre-mix and pre-clamp rather than a servo angle, and "
        "None on every row of a 15-byte legacy log.",
    shown_in=("guidance",)
)
_P(
    "Guidance.active", "Guidance active",
    kind=KIND_BOOL,
    note="Guidance-flags bit 0, set from the firmware's guidance_active state.",
    caution=
        "Constant true — the frame is only emitted while guidance runs, so disengagement shows up as "
        "an absence of frames, never as a false sample.",
)
_P(
    "Guidance.burnout", "Burnout detected",
    kind=KIND_BOOL,
    note=
        "The flight computer's latched burnout detection, the same source as the NonSensor burnout "
        "flag.",
    caution=
        "Latched, and sampled only inside the guided window, so it cannot date burnout — if guidance "
        "engaged post-burnout it already reads true on the first frame and never transitions.",
)

# The flight computer's own state frame — EKF attitude, position and velocity plus every
# flight flag — logged at ~470-500 Hz on the FC boot clock shared with the IMU, baro, GNSS and
# magnetometer streams (only LogBufferStats runs on a different origin), with the newest tail
# fields absent on shorter wire formats.
_P(
    "NonSensor.q0", "Quaternion w",
    note="Scalar component of the EKF body-to-nav attitude quaternion, stored to four decimals.",
    caution=
        "If the boost accel direction disagreed with the latched board→rocket orientation by more "
        "than 25°, firmware raises ORIENT_THRUST_MISMATCH and flies on uncorrected — but the parser "
        "drops that bit, so the log gives you no way to tell. Everything attitude-derived is suspect "
        "when it fired.",
    derived_in=("stability", "roll", "roll_pid",),
)
_P(
    "NonSensor.q1", "Quaternion x",
    note="Vector i component of the EKF attitude quaternion.",
    derived_in=("stability", "roll", "roll_pid",),
)
_P(
    "NonSensor.q2", "Quaternion y",
    note="Vector j component of the EKF attitude quaternion.",
    derived_in=("stability", "roll", "roll_pid",),
)
_P(
    "NonSensor.q3", "Quaternion z",
    note="Vector k component of the EKF attitude quaternion.",
    derived_in=("stability", "roll", "roll_pid",),
)
_P(
    "NonSensor.roll", "Euler roll (ZYX)",
    unit="°",
    note="ZYX-Euler roll angle derived by the parser from the attitude quaternion.",
    caution=
        "Not the iOS app CSV's \"Roll\" column (that is body-Z azimuth) and not what the roll "
        "controller regulates — roll/roll_pid rebuild their own \"EKF Roll\" from the quaternion "
        "(#514). Never mix the two.",
    shown_in=()
)
_P(
    "NonSensor.pitch", "Euler pitch (ZYX)",
    unit="°",
    note="ZYX-Euler pitch angle derived by the parser from the attitude quaternion.",
    caution=
        "A near-vertical rocket sits at |pitch| near 90 deg, exactly the ZYX gimbal-lock singularity: "
        "roll and yaw go ill-conditioned and can jump 180 deg during boost.",
    shown_in=()
)
_P(
    "NonSensor.yaw", "Euler yaw (ZYX)",
    unit="°",
    note="ZYX-Euler yaw angle derived by the parser from the attitude quaternion.",
    caution=
        "A nav-frame heading out of the EKF, not a magnetic heading, and it shares pitch's boost- "
        "phase gimbal-lock problem.",
    shown_in=()
)
_P(
    "NonSensor.roll_cmd", "Roll fin command",
    unit="°",
    note=
        "The roll PID's output fin deflection sent to the servos; logged in centidegrees and scaled "
        "to degrees.",
    caution=
        "A fin deflection, not a commanded roll angle — the target angle exists only in the .json "
        "sidecar. Identically 0.0 when roll control is inactive.",
    shown_in=("roll", "roll_pid", "guidance"),
)
_P(
    "NonSensor.e_pos", "EKF position East",
    unit="m",
    note="EKF East position in a local ENU frame; logged in centimetres and scaled to metres.",
    caution=
        "The ENU origin is NOT in the log — globe re-derives it from the GNSS pad fixes and publishes "
        "originResidualM. Any lat/lon computed from e/n is only as good as that reconstruction.",
    shown_in=("globe", "guidance"),
)
_P(
    "NonSensor.n_pos", "EKF position North",
    unit="m",
    note=
        "EKF North position in the same local ENU frame; centimetres on the wire, scaled to metres.",
    caution=
        "The frame's origin is absent from the log; globe reconstructs it from the pad fixes and "
        "reports the fit as originResidualM, so northings are relative to an estimate, not a surveyed "
        "point.",
    shown_in=("globe", "guidance"),
)
_P(
    "NonSensor.u_pos", "EKF altitude (Up)",
    unit="m",
    note=
        "EKF height above the firmware's pad-average altitude, so AGL by construction; centimetres on "
        "the wire, scaled to metres.",
    caution=
        "AGL against the EKF's own pad reference, not the barometric AGL behind the summary, apogee "
        "and deployment numbers. In #741 the two sit ~81 m apart at landing.",
    shown_in=("overview", "globe"),
)
_P(
    "NonSensor.e_vel", "EKF velocity East",
    unit="m/s",
    note="EKF East velocity; logged in cm/s and scaled to m/s.",
    shown_in=("overview", "roll_pid"),
)
_P(
    "NonSensor.n_vel", "EKF velocity North",
    unit="m/s",
    note="EKF North velocity; logged in cm/s and scaled to m/s.",
    shown_in=("overview", "roll_pid"),
)
_P(
    "NonSensor.u_vel", "EKF velocity Up",
    unit="m/s", conv="m/s:fps",
    note=
        "EKF vertical velocity; cm/s on the wire, scaled to m/s. The three-component norm is the "
        "summary card's max speed.",
    caution=
        "Not baro_alt_rate, which is the barometer-only filtered rate the apogee detector votes on.",
    shown_in=("overview", "roll_pid"),
)
_P(
    "NonSensor.flags", "Flight flag byte",
    kind=KIND_RAW,
    note="The raw flight-flag byte; the parser splits bits 0-6 into the booleans below.",
    caution=
        "Bit 7 (simulated-flight active) is decoded into no field, so a hardware-in-the-loop sim run "
        "is indistinguishable from a real flight unless you mask flags & 0x80 yourself.",
    derived_in=(),
)
_P(
    "NonSensor.rocket_state", "Rocket state",
    kind=KIND_ENUM,
    note="Flight state machine: 0 INIT, 1 READY, 2 PRELAUNCH, 3 INFLIGHT, 4 LANDED.",
    caution=
        "An integer code, not a measurement — interpolating or averaging it is a category error. "
        "Several repo logs never leave PRELAUNCH, so a flat line can be a real result.",
    shown_in=(),
)
_P(
    "NonSensor.baro_alt_rate", "Barometric climb rate",
    unit="m/s", conv="m/s:fps",
    note=
        "The flight computer's KF-filtered barometric climb rate; logged in decimetres/s and scaled "
        "to m/s.",
    caution=
        "Not u_vel, and not a derivative of u_pos: this is the flight computer's own filtered "
        "climb rate, so it will not match a rate you difference out of the barometric altitude.",
    shown_in=()
)
_P(
    "NonSensor.alt_landed", "Landing declared",
    kind=KIND_BOOL,
    note=
        "Set when the firmware declares the rocket landed; every consumer takes the first true "
        "record.",
    caution=
        "The firmware's declaration, not the true touchdown instant — deployment's descent-rate "
        "window ends here, so a late declaration stretches it.",
    shown_in=("overview", "roll", "deployment", "apogee", "globe"),
)
_P(
    "NonSensor.alt_apogee", "Apogee vote (baro)",
    kind=KIND_BOOL,
    note=
        "The barometric apogee detector's vote; its first true is the event line the report labels "
        "simply \"apogee\".",
    caution=
        "One detector's vote, not the master call that fires the charge — that is apogee_flag, and "
        "baro usually votes first. Pyro delays measured against this are overstated.",
    shown_in=("overview", "roll", "stability", "deployment", "globe"),
)
_P(
    "NonSensor.vel_apogee", "Apogee vote (velocity)",
    kind=KIND_BOOL,
    note="The vertical-velocity apogee detector's vote.",
    caution=
        "Only one detector's opinion. The call that actually arms the ejection charge is apogee_flag, "
        "so timing a pyro against this vote overstates the delay.",
    shown_in=()
)
_P(
    "NonSensor.launch", "Launch detected",
    kind=KIND_BOOL,
    note=
        "Set when the firmware detects launch; its first true is the t=0 for nearly every window in "
        "the report.",
    caution=
        "The declaration, not the moment of first motion — it latches once the detector is "
        "confident, which on the sample flight is 0.20 s after the motor lit. Every time the "
        "report quotes is measured off the accelerometer instead; this flag only bounds chart "
        "windows, and where it never fires those windows open to the whole log.",
    shown_in=("overview", "roll", "stability", "gaps", "globe"),
)
_P(
    "NonSensor.burnout", "Burnout detected",
    kind=KIND_BOOL,
    note="Set at motor burnout; bounds motor's burn-time and tilt-at-burnout metrics.",
    shown_in=("overview", "roll", "stability"),
)
_P(
    "NonSensor.guidance", "Guidance active",
    kind=KIND_BOOL,
    note="True while proportional-navigation guidance is actively steering.",
    caution=
        "Not guidance_enabled (the configured setting). The guidance module's shaded span comes from "
        "the Guidance telemetry stream's own flag, so a disagreement between them is real "
        "information.",
)
_P(
    "NonSensor.pyro_armed", "Pyro arm line",
    kind=KIND_BOOL,
    note="Live mirror of the single shared arming FET that serves all four pyro channels.",
    caution=
        "Global, not per-channel — there is no per-channel armed state on the wire despite the "
        "pyro1_armed/pyro2_armed keys.",
)
_P(
    "NonSensor.pyro1_cont", "Pyro 1 continuity",
    kind=KIND_BOOL,
    note="Continuity sensed on pyro channel 1 (igniter load present).",
    caution=
        "Reads HIGH on a channel that is disabled in config, so it is not evidence a charge was "
        "fitted: flight_20260705_174532 flew with all four channels off and still carries continuity "
        "on 1-3 for every frame. Confirm against health_pyro1 or the sidecar's pyro.ch1.enabled. "
        "Also forced to 0 on 42-byte logs, where False can mean \"never recorded\".",
)
_P(
    "NonSensor.pyro1_fired", "Pyro 1 fired",
    kind=KIND_BOOL,
    note="Pyro channel 1 completed its fire cycle.",
    caution=
        "Timestamped at the channel's state-to-Done transition, roughly 510 ms after the master "
        "apogee call, not the command instant. Forced False on 42-byte logs.",
    shown_in=("apogee"),
)
_P(
    "NonSensor.pyro2_cont", "Pyro 2 continuity",
    kind=KIND_BOOL,
    note="Continuity sensed on pyro channel 2.",
    caution=
        "Reads HIGH on a disabled channel, so a true here is not evidence a charge was fitted — "
        "confirm against health_pyro2 or the sidecar's pyro.ch2.enabled. On 42-byte logs there is no "
        "pyro status byte and the parser substitutes 0, so False may mean never recorded.",
)
_P(
    "NonSensor.pyro2_fired", "Pyro 2 fired",
    kind=KIND_BOOL,
    note="Pyro channel 2 completed its fire cycle.",
    caution=
        "Marks the state-to-Done transition about half a second after the master apogee call, not the "
        "fire command. Reads False on 42-byte logs, where nothing was recorded.",
    shown_in=("apogee"),
)
_P(
    "NonSensor.pyro3_cont", "Pyro 3 continuity",
    kind=KIND_BOOL,
    note="Continuity sensed on pyro channel 3.",
    caution=
        "Channels 3 and 4 exist only on the new PCB. On a two-channel board these bits are reserved "
        "and decode to meaningless values rather than absent ones. It also reads HIGH on a disabled "
        "channel — confirm against health_pyro3 or pyro.ch3.enabled. Forced to 0 on 42-byte logs.",
)
_P(
    "NonSensor.pyro3_fired", "Pyro 3 fired",
    kind=KIND_BOOL,
    note="Pyro channel 3 completed its fire cycle.",
    caution=
        "New-PCB only — a reserved bit on a two-channel board. When it is real, the timestamp is the "
        "state→Done transition, roughly 510 ms after the master apogee call, not the moment the "
        "charge was commanded.",
    shown_in=("apogee",)
)
_P(
    "NonSensor.pyro4_cont", "Pyro 4 continuity",
    kind=KIND_BOOL,
    note="Continuity sensed on pyro channel 4.",
    caution=
        "New-PCB only: on a two-channel board this decodes a reserved bit and means nothing. Reads "
        "HIGH on a disabled channel too — confirm against health_pyro4 or pyro.ch4.enabled. Forced "
        "to 0 on 42-byte logs as well.",
)
_P(
    "NonSensor.pyro4_fired", "Pyro 4 fired",
    kind=KIND_BOOL,
    note="Pyro channel 4 completed its fire cycle.",
    caution=
        "New-PCB only — a reserved bit on a two-channel board. When real, it marks the state→Done "
        "transition, not the fire command.",
    shown_in=("apogee",)
)
_P(
    "NonSensor.pyro1_armed", "Pyro 1 armed (alias)",
    kind=KIND_BOOL,
    note=
        "Back-compat alias for the global arm bit, kept for code that read the removed per-channel "
        "flag.",
    caution=
        "Identical to pyro_armed and pyro2_armed on every record — plotting all three draws three "
        "coincident lines and says nothing per-channel.",
)
_P(
    "NonSensor.pyro2_armed", "Pyro 2 armed (alias)",
    kind=KIND_BOOL,
    note="The same global arm bit under a second legacy name.",
    caution=
        "Despite the name it is not channel 2's own state — it is the shared arm bit, identical to "
        "pyro1_armed.",
)
_P(
    "NonSensor.has_apogee_flags", "Apogee flags present",
    kind=KIND_BOOL,
    note=
        "Parser flag: true when the frame was long enough to carry the apogee-flags byte. Constant "
        "for a whole log.",
    caution=
        "When false, gps_apogee, pitch_apogee, apogee_flag, reboot_recovery, guidance_enabled, "
        "fc_imu_drop and deployed are all forced False and mean \"not recorded\", not \"no\".",
)
_P(
    "NonSensor.gps_apogee", "Apogee vote (GNSS)",
    kind=KIND_BOOL,
    note="The GNSS apogee detector's vote.",
    caution=
        "One detector's vote, not the master call that fires the charge (apogee_flag). Meaningless "
        "unless has_apogee_flags is true.",
    shown_in=()
)
_P(
    "NonSensor.pitch_apogee", "Apogee vote (pitch)",
    kind=KIND_BOOL,
    note="The pitch-over apogee detector's vote.",
    caution=
        "One detector's vote, not the master call (apogee_flag). Apogee's \"Pitch\" lane is the replay "
        "simulator's reconstruction of the condition, not this logged bit, and the two can disagree.",
)
_P(
    "NonSensor.apogee_flag", "Apogee (master vote)",
    kind=KIND_BOOL,
    note="The voted master apogee result — the call that actually arms the ejection charge.",
    caution=
        "Nothing in the report plots it: apogee's \"Master apogee\" line comes from the replay's "
        "simulated fires. Do not confuse it with alt_apogee, which is only the baro detector's vote.",
    shown_in=("roll_pid", "roll", "apogee"),
)
_P(
    "NonSensor.reboot_recovery", "Reboot recovery",
    kind=KIND_BOOL,
    note="Set when the flight computer rebuilt flight state after an in-flight reboot.",
    caution=
        "If set, timestamps around it were reconstructed rather than continuously measured — treat "
        "nearby timing with suspicion.",
)
_P(
    "NonSensor.guidance_enabled", "Guidance enabled",
    kind=KIND_BOOL,
    note="The flight computer's live guidance-enabled configuration setting.",
    caution=
        "Not the `guidance` flag (steering right now), and not the guidance_enabled row printed by "
        "roll_pid and guidance — those read the .json sidecar, so a sidecar/vehicle mismatch only "
        "shows up here.",
)
_P(
    "NonSensor.fc_imu_drop", "FC IMU queue overflow",
    kind=KIND_BOOL,
    note=
        "Set when the FC loop stalled long enough to overflow the IMU handoff queue and drop samples.",
    caution=
        "Sticky: the first frame carrying it marks when the overflow was noticed, not when samples "
        "were lost. If set anywhere, gaps and sample_rates understate the loss.",
)
_P(
    "NonSensor.deployed", "Deployment detected",
    kind=KIND_BOOL,
    note=
        "Set when the firmware's detector latches a recovery deployment; it also triggers the dynamic "
        "logging-rate step-down.",
    caution=
        "Sticky, and it observes deployment rather than commanding it. False on firmware predating "
        "the detector, so check has_apogee_flags before concluding a flight never deployed.",
)
_P(
    "NonSensor.ekf_ticks", "EKF update ticks",
    kind=KIND_COUNTER,
    note=
        "Free-running EKF update counter, present only on the longest wire format; None on every log "
        "in this repo today.",
    caution=
        "Wraps at 2^16 — difference modulo 65536 or the derived rate goes hugely negative once per "
        "wrap. Frozen at 0 until the EKF initialises on the first good fix.",
)
_P(
    "NonSensor.sensor_health", "Sensor health word",
    kind=KIND_RAW,
    note=
        "Packed subsystem health scorecard, two bits per subsystem; the parser also splits it into "
        "the health_* channels.",
    caution=
        "A bitfield, not a magnitude — plot the health_* splits instead. None rather than 0 on older "
        "logs, because 0 would read as a full card of deliberate \"no verdict\" entries.",
)
_P(
    "NonSensor.health_baro", "Health: barometer",
    kind=KIND_ENUM,
    note="The flight computer's verdict on the barometer: 0 n/a, 1 ok, 2 degraded, 3 bad.",
    caution=
        "0 means \"no verdict\", NOT healthy — 1 is healthy. A step from 0 to 1 is the FC forming an "
        "opinion, not a degradation.",
)
_P(
    "NonSensor.health_imu", "Health: IMU",
    kind=KIND_ENUM,
    note="The flight computer's verdict on the IMU, on the shared 0-3 health scale.",
    caution=
        "Only 1 means healthy; 0 is \"no verdict\" and must be read as missing information, not as a "
        "pass.",
)
_P(
    "NonSensor.health_ekf", "Health: EKF",
    kind=KIND_ENUM,
    note="The FC's verdict on its own filter: initialised, healthy and covariance converged.",
    caution=
        "0 is \"no verdict\", not healthy (1 is). This is the best channel for reading a filter "
        "divergence like #741, and no report section currently surfaces it.",
)
_P(
    "NonSensor.health_mag", "Health: magnetometer",
    kind=KIND_ENUM,
    note="The flight computer's verdict on the magnetometer, on the shared 0-3 scale.",
    caution=
        "Healthy is 1. A 0 means the FC never formed a verdict, which is not the same as good.",
)
_P(
    "NonSensor.health_gnss", "Health: GNSS fix",
    kind=KIND_ENUM,
    note="The flight computer's verdict on GNSS fix quality.",
    caution=
        "0 is \"no verdict\", not healthy (1 is). Fix quality only — the GNSS-absent degraded mode is "
        "the separate health_gnss_absent verdict.",
)
_P(
    "NonSensor.health_batt", "Health: battery",
    kind=KIND_ENUM,
    note=
        "Battery verdict written by the onboard computer from the power telemetry; the flight "
        "computer never reads the pack.",
    caution=
        "0 means no verdict rather than healthy (1 is healthy), and being OC-owned it can sit at 0 on "
        "a log where the FC-owned health fields are populated.",
)
_P(
    "NonSensor.health_pyro1", "Health: pyro 1",
    kind=KIND_ENUM,
    note="Pre-launch verdict for pyro channel 1.",
    caution=
        "The scale inverts here: 0 = channel not configured, 2 = configured but continuity untested, "
        "3 = configured with NO continuity. Reading 0 as \"fine\" inverts the go/no-go.",
)
_P(
    "NonSensor.health_pyro2", "Health: pyro 2",
    kind=KIND_ENUM,
    note="Pre-launch verdict for pyro channel 2.",
    caution=
        "Same inverted scale as channel 1: 0 means the channel was not configured at all, 2 "
        "configured but untested, 3 configured and failing continuity.",
)
_P(
    "NonSensor.health_pyro3", "Health: pyro 3",
    kind=KIND_ENUM,
    note="Pre-launch verdict for pyro channel 3.",
    caution=
        "Inverted per-channel scale: 0 is \"not configured\" rather than healthy, 2 is configured but "
        "continuity untested, 3 is configured with no continuity.",
)
_P(
    "NonSensor.health_pyro4", "Health: pyro 4",
    kind=KIND_ENUM,
    note="Pre-launch verdict for pyro channel 4.",
    caution=
        "Read on the inverted per-channel scale: 0 = not configured for this flight, 2 = configured "
        "but untested, 3 = configured and no continuity. A 0 is not a pass.",
)
_P(
    "NonSensor.health_storage", "Health: log storage",
    kind=KIND_ENUM,
    note=
        "Onboard-computer verdict on flight-log storage: bad = NAND writes failing or no room, "
        "degraded = room for fewer than two more flights.",
    caution=
        "0 is \"no verdict\", not healthy. Bad means the NEXT flight will not be recorded — it says "
        "nothing about the integrity of the log you are reading.",
)
_P(
    "NonSensor.health_gnss_absent", "Health: GNSS-absent mode",
    kind=KIND_ENUM,
    note=
        "Set to bad when the FC committed to the baro+IMU-only EKF path because the GNSS module "
        "failed bring-up.",
    caution=
        "Inverted against the rest of the family: 3 means the degraded mode is ACTIVE, 0 means normal "
        "(not \"no verdict\"), 1 and 2 never occur. At 3 there is no absolute position at all.",
)


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def _numeric(v: Any) -> Optional[float]:
    """A plottable float, or None. Booleans count — a flag plots as 0/1."""
    if v is None or isinstance(v, str):
        return None
    if isinstance(v, bool):
        return float(v)
    if isinstance(v, (int, float)):
        f = float(v)
        return f if f == f and f not in (float("inf"), float("-inf")) else None
    return None


def _field_order(rows: list[dict[str, Any]], sample: int = 64) -> list[str]:
    """Field names in insertion order, unioned over the first few rows.

    One row is nearly always enough — the parser writes a dict literal, so every
    record of a stream has the same keys — but the union is cheap and covers a
    format change mid-log without dropping the fields only one half carries.
    """
    order: dict[str, None] = {}
    for r in rows[:sample]:
        for k in r:
            order.setdefault(k, None)
    return list(order)


def _measure(rows: list[dict[str, Any]], name: str) -> tuple[int, Optional[float], Optional[float]]:
    """(non-None count, min, max) over one field."""
    n = 0
    lo = hi = None
    for r in rows:
        v = _numeric(r.get(name))
        if v is None:
            continue
        n += 1
        if lo is None or v < lo:
            lo = v
        if hi is None or v > hi:
            hi = v
    return n, lo, hi


def _timespan(rows: list[dict[str, Any]], t0_us: Optional[int]
              ) -> tuple[Optional[float], Optional[float], Optional[float]]:
    """(t_start_s, t_end_s, rate_hz) for a stream, relative to the flight t0.

    The rate is the median inter-sample interval rather than count/span: a
    stream with a dropout has a mean rate that describes no part of the flight,
    and the median is what `gaps._per_sensor_summary` reports for the same
    reason.
    """
    ts = [r["time_us"] for r in rows if r.get("time_us") is not None]
    if not ts:
        return None, None, None
    base = t0_us if t0_us is not None else ts[0]
    t_start = (ts[0] - base) / 1e6
    t_end = (ts[-1] - base) / 1e6
    rate = None
    if len(ts) > 1:
        deltas = sorted(b - a for a, b in zip(ts, ts[1:]) if b > a)
        if deltas:
            med = deltas[len(deltas) // 2]
            rate = 1e6 / med if med else None
    return t_start, t_end, rate


def _humanize(name: str) -> str:
    """A readable label for a channel with no PROVENANCE entry."""
    return name.replace("_", " ").strip().capitalize()


def _unreadable(stats: dict[str, Any], records: dict[str, list]) -> list[Unreadable]:
    """Frame types counted by the parser that produced no records.

    This is the part no current tool tells the user. `report.html.j2` already
    prints a row labelled `0xD2` with its frame count in the parser table; here
    it is named as something the tool cannot read, rather than left looking like
    a sensor nobody recognises.
    """
    out: list[Unreadable] = []
    for name, count in sorted(stats.get("type_counts", {}).items(), key=lambda kv: -kv[1]):
        if name in _CONTROL_FRAMES or records.get(name):
            continue
        if name.startswith("0x"):
            reason = "no decoder — the frame type is not in the parser's message table"
        else:
            reason = "recognised, but the parser has no branch that decodes it"
        out.append(Unreadable(name=name, count=count, reason=reason))
    return out


# Fields the parser decodes off the wire and then discards, so they are absent
# from `records` and no amount of walking it will find them. Listed so the
# inventory can say "this log contains it, we throw it away" rather than stay
# silent — that is the difference between a known gap and an invisible one.
DROPPED_FIELDS: tuple[str, ...] = (
    "ISM6HG256.raw counts — the int16 accel/gyro counts are staged in a local "
    "and only the scaled SI values are kept, so an uncalibrated-counts check is "
    "not possible from the record dict.",
    "MMC5983MA / IIS2MDC raw_x/raw_y/raw_z on the calibrated path — popped off "
    "the record once the corrected values exist.",
    "NonSensor.apogee_flags bit 5 (NSF2_ORIENT_THRUST_MISMATCH) — the constant "
    "is defined but never decoded into a field.",
)


def build(flight: "Flight") -> Catalog:
    """The catalog for one flight. Cheap: one pass per field, no plotting."""
    records = flight.records
    t0_us = flight.t0_us

    cat = Catalog(dropped=list(DROPPED_FIELDS))

    for stream, rows in records.items():
        if not rows:
            # An empty stream has no channels *in this log*. Saying so is the
            # whole point — MMC5983MA and Guidance parse empty on every flight
            # in the repo, and a static field list would offer them anyway.
            continue
        t_start, t_end, rate = _timespan(rows, t0_us)
        for name in _field_order(rows):
            if name == "time_us":
                # The x axis, not a channel. Its span and rate are carried on
                # every channel of the stream instead.
                continue
            n_present, lo, hi = _measure(rows, name)
            key = f"{stream}.{name}"
            meta = PROVENANCE.get(key)
            cat.channels.append(Channel(
                stream=stream,
                field=name,
                meta=meta or ChannelMeta(label=_humanize(name)),
                documented=meta is not None,
                n_total=len(rows),
                n_present=n_present,
                rate_hz=rate,
                t_start_s=t_start,
                t_end_s=t_end,
                vmin=lo,
                vmax=hi,
            ))

    cat.unreadable = _unreadable(flight.stats, records)
    return cat


# ---------------------------------------------------------------------------
# Text rendering — shared by the report section and the `fields` subcommand so
# the two cannot describe the same log differently.
# ---------------------------------------------------------------------------

def _span_note(chans: list[Channel]) -> str:
    c = chans[0]
    if c.rate_hz is None or c.t_start_s is None:
        return ""
    return f"{c.rate_hz:,.1f} Hz · T{c.t_start_s:+.1f}s to T{c.t_end_s:+.1f}s · {c.n_total:,} records"


def _range_text(c: Channel) -> str:
    if c.is_empty:
        return "(never set)"
    if c.vmin is None:
        return ""
    if c.meta.kind == KIND_BOOL:
        return "true for some" if c.vmax else "always false"
    # Counters and coded states are integers, and 4 significant figures turns a
    # 262,144-byte ring size into "2.621e+05" — which reads as a measurement
    # rather than the exact constant it is.
    if c.meta.kind in (KIND_COUNTER, KIND_ENUM):
        lo, hi = int(c.vmin), int(c.vmax)
        return f"{lo:,} (constant)" if lo == hi else f"{lo:,} … {hi:,}"
    if c.vmin == c.vmax:
        return f"{c.vmin:,.4g} (constant)"
    return f"{c.vmin:,.4g} … {c.vmax:,.4g}"


def format_text(cat: Catalog, width: int = 100) -> str:
    """The inventory as a monospace block.

    Deliberately plain text: it is read as a reference, it greps, and it pastes
    into an issue.

    Each channel gets its headline row — name, unit, the range it actually
    covered on this flight, and which report sections chart it — then its note,
    then any caution. The note and caution are the reason the catalog exists; a
    row that shows only a name and a range tells you a channel is there without
    telling you what you would be looking at, which is how someone ends up
    comparing the nav filter's Euler roll against the app's body-Z azimuth.
    """
    import textwrap

    out: list[str] = []
    for stream in cat.streams():
        chans = cat.for_stream(stream)
        head = f"{stream}  —  {len(chans)} channels"
        note = _span_note(chans)
        out.append(head + (f"  ({note})" if note else ""))
        out.append("=" * min(width, max(len(head), 60)))
        w_name = max(len(c.field) for c in chans)
        w_unit = max((len(c.meta.unit) for c in chans), default=0)
        body_indent = " " * 6
        for c in chans:
            if c.meta.shown_in:
                shown = "shown in: " + ", ".join(c.meta.shown_in)
            elif c.meta.derived_in:
                shown = "not plotted — consumed by " + ", ".join(c.meta.derived_in)
            else:
                shown = "not plotted in this report"
            out.append(
                f"  {c.field:<{w_name}}  {c.meta.unit:<{w_unit}}  "
                f"{_range_text(c):<22}  {shown}"
            )
            if c.meta.label and c.meta.label.lower() != _humanize(c.field).lower():
                out.append(f"{body_indent}{c.meta.label}")
            for para, prefix in ((c.meta.note, ""), (c.meta.caution, "⚠ ")):
                if not para:
                    continue
                wrapped = textwrap.wrap(
                    prefix + para, width=width - len(body_indent),
                    initial_indent=body_indent,
                    subsequent_indent=body_indent + ("  " if prefix else ""),
                )
                out.extend(wrapped)
            out.append("")
        out.append("")

    if cat.unreadable:
        out.append("In the log, not readable by this tool")
        out.append("-" * 60)
        for u in cat.unreadable:
            out.append(f"  {u.name:<10} {u.count:>7,} frames   {u.reason}")
        out.append("")

    if cat.dropped:
        out.append("Decoded off the wire, then discarded by the parser")
        out.append("-" * 60)
        for d in cat.dropped:
            out.append(f"  · {d}")
        out.append("")

    return "\n".join(out).rstrip() + "\n"

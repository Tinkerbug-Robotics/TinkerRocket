# Inter-board cables — which jumper to order, and the trap in its name

Three links in the fleet use JST SR/SH ribbon jumpers, and all three are wired
**pad 1 to pad 1** at both ends. The daughterboards and the hosts were checked
pin-for-pin from their netlists (rocket J5/J1, base station J6, LoRa
daughterboard J6, both GNSS carriers' J3), so one straight cable convention
serves every board.

| Link | Host header | Cable to order |
|---|---|---|
| LoRa daughterboard — rocket J5, base station J6 | 4-pin `BM04B-SRSS-TB` | `A04SR04SR30K51A` |
| GNSS carrier — rocket J1 | 5-pin `BM05B-SRSS-TB` | `A05SR05SR30K51A` |

## The trap: Digi-Key's "Reversed" is the cable we want

For these jumpers the catalogue attribute describes the **housing
orientation**, not the pin mapping. Two same-orientation housings must rotate
180° to mate at both ends, and that rotation flips the pin order:

| Catalogue description | Suffix | Actual mapping | Use it? |
|---|---|---|---|
| "Socket to Socket, **Reversed**" | **A** | pad 1 → pad 1 | **yes** |
| "Socket to Socket" | **B** | pad 1 → pad N | **no** |

The wrong part reads as the right one on the order page. It happened on
2026-07-31: a B-suffix 4-pin jumper between the base station and a LoRa
daughterboard put the host's supply (V_LORA ≈ 9.6 V on the base station, up to
8.4 V from a rocket's pack) straight onto the daughterboard's ESP32-S3 GPIO6,
which conducted it through its ESD clamp into the 3.3 V rail that also feeds
the radio. The board got hot immediately; the processor survived, the E220
radio did not. Swapping the wires end-for-end made the board power up
normally.

## What a reversed 5-pin GNSS cable does

Power is on pin 3, the centre pin of five, so it self-aligns under reversal and
the carrier still gets its supply. Everything else moves, and the move that
matters is ground: the carrier's ground (pin 4) lands on the rocket's pin 2,
GNSS_TX. On the rocket that pin is the host P4's UART **receive** input
(GPIO3), with no series resistor, so the carrier's whole supply return goes
through that pin's ESD clamp into the host's 3.3 V rail, and the carrier's
ground floats about 4 V above the host's.

- **A light carrier** (a bare receiver module, tens of mA) keeps running on
  what is left and can present as a module that answers once and then stops
  parsing, not as a dead board. When a GNSS carrier is "deaf" on the bench,
  check the cable before the firmware.
- **The PX1105R carrier** (`legacy/gnss-px1105r-18mm-highpower-ext-ant`) draws
  about 0.2 A, and all of it goes into that clamp. Expect a damaged GPIO3 pad on
  a chip-down P4 that cannot be reworked, and a host 3.3 V rail pushed up if the
  host draws less than the injected current. This failure is not silent.

The carrier cannot guard against this: the fault arrives on its ground pin,
which carries the full supply current and cannot take a series resistor (its
signal pins already have 1 kΩ). The connectors are keyed, so a cable cannot go
in rotated; only the wrong cable type causes this, and the rule above is the
protection.

## Before every bench session and every airframe build

- Check each jumper pad 1 to pad 1 with a meter, or by eye against the
  contact side of the ribbon. Label the cable once it is verified.
- The camera link (rocket J4, JST PH) is not an SR jumper, but it has the same
  pin-1-is-ground / pin-2-is-supply ordering and the same reverse-polarity
  consequence; see the rocket's `FABRICATION-NOTES.md` block B6 for its pinout,
  and B7 for the servo port, both of which changed ends at the high-side switch
  rework.
- A pack plug cannot mis-mate into the camera port any more: the battery
  connector is a JST-VH (`B2P-VH`) since the V9 close-out, a different family
  from the PH camera port. The older concern (#677) was written against a PH-2
  battery plug.

## Future spin, for consideration

Putting the supply on the centre pin of an odd-count connector and ground on
both end pins would make the LoRa link tolerant of a reversed cable. Centre
power alone is not enough: on the 5-pin GNSS link the supply self-aligns, but
ground still lands on a signal pin (above). Not planned; the cable rule above
is the mitigation in force.

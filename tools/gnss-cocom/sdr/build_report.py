#!/usr/bin/env python3
"""Rebuild report.html around the receiver comparison.

The page grew by accretion: it began as one receiver's AND-vs-OR question and
five parts later was still shaped like that investigation, with the comparison
table buried in section 06 and each part's story told as a narrative of what
went wrong on the bench. This rebuilds it around what it is actually for --
comparing where different hobby-grade receivers stop publishing position -- and
puts the gate table first.

Run it after editing results/receivers.json or re-plotting; it regenerates the
whole body from the archived figures and the receiver data.
"""
import json, subprocess, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
FIG = HERE / "results" / "figures"


def fig(name):
    p = FIG / name
    return p.read_text().strip() if p.exists() else f"<!-- missing {name} -->"


d = json.loads((HERE / "results" / "receivers.json").read_text())
table = subprocess.run([sys.executable, str(HERE / "receiver_table.py"), "--html"],
                       capture_output=True, text=True, check=True).stdout.strip()

# Per-receiver panels: (id, figure prefix, one-line verdict, extra note)
PARTS = [
    ("px1125r", "spaceshot.svg", "gentle_alt.svg",
     "Textbook behaviour: both export limits enforced independently, "
     "velocity at 510&ndash;517&nbsp;m/s and altitude at 80&nbsp;km, re-opening "
     "within 1.5&nbsp;s whenever four or more satellites were held."),
    ("sam_m10q", "ublox_m10_spaceshot.svg", "ublox_m10_gentle_alt.svg",
     "The flight computer's own receiver, and the tightest data on the bench: "
     "never fewer than four satellites in either flight, so every withheld "
     "epoch is unambiguously the gate."),
    ("zed_f9p", "zed_f9p_spaceshot.svg", "zed_f9p_gentle_alt.svg",
     "Same thresholds as the others, but 2&ndash;3&nbsp;s slow to <em>close</em> "
     "the altitude gate &mdash; 400&ndash;600&nbsp;m of overshoot above 80&nbsp;km "
     "with position still being published."),
    ("neo_m8t", "neo_m8t_spaceshot.svg", "neo_m8t_gentle_alt.svg",
     "Velocity gate normal at 510&ndash;524&nbsp;m/s, but it stops publishing at "
     "50&nbsp;km &mdash; the u-blox dynamic-model ceiling, not an export limit. "
     "Its true altitude behaviour is therefore unmeasurable."),
    ("air530", "air530_spaceshot.svg", "air530_gentle_alt.svg",
     "The outlier. <strong>No velocity gate at all</strong>, and it stops "
     "publishing at 10&nbsp;km &mdash; below every candidate export altitude, and "
     "below apogee for most high-power flights."),
]
by_id = {r["id"]: r for r in d["receivers"]}

parts_html = []
for pid, f1, f2, verdict in PARTS:
    r = by_id[pid]
    vel = (f"none to {r['velocity_fix_max_mps']:.0f} m/s"
           if r.get("velocity_gate_present") is False else
           f"{r['velocity_fix_max_mps']}&ndash;{r['velocity_blocked_min_mps']} m/s")
    alt = ("&mdash;" if r.get("altitude_fix_max_km") is None else
           f"{r['altitude_fix_max_km']:.2f}&ndash;{r['altitude_blocked_min_km']:.2f} km")
    cause = r.get("altitude_gate_cause", "cocom")
    if cause != "cocom":
        alt += f" <em>({cause})</em>"
    parts_html.append(f'''
    <div class="prose">
      <h3>{r['part']}</h3>
      <p class="note">{r['bands']} &middot; {r['protocol']} &middot; {r['path']} &middot;
        {r['rf']} &middot; {r['runs']} run(s)</p>
      <p>{verdict}</p>
      <p><strong>Velocity gate</strong> {vel} &nbsp;&middot;&nbsp;
         <strong>Altitude gate</strong> {alt} &nbsp;&middot;&nbsp;
         <strong>Satellites</strong> min {r['sats_min']}, median {r['sats_median']}</p>
    </div>

    <figure>
      <div class="panels" style="grid-template-columns:1fr">
        <div class="panel">
          <h4>{r['part']} &mdash; 15 g boost <span class="badge">spaceshot</span></h4>
          <p>82.5 km apogee, 1343 m/s peak &mdash; both limits exceeded</p>
{fig(f1)}
        </div>
        <div class="panel">
          <h4>{r['part']} &mdash; 3 g boost <span class="badge">gentle_alt</span></h4>
          <p>same apogee reached slowly &mdash; the run that brackets the thresholds</p>
{fig(f2)}
        </div>
      </div>
      <figcaption>{r['part']}: the two standard profiles. Shaded bands mark where
        the injected trajectory exceeds a COCOM limit &mdash; red for velocity,
        amber for altitude. Read the lock strip against those bands, and the
        satellite bar underneath to confirm the receiver was still tracking.</figcaption>
    </figure>
''')

body = f'''
<div class="page">

  <header class="masthead">
    <div class="kicker">
      <span>Tinkerbug Robotics</span>
      <span>Issue #491</span>
      <span>Bench report</span>
      <span>21 Aug 2026</span>
    </div>
    <h1>Where hobby GNSS receivers stop</h1>
    <p class="standfirst">Five hobby-grade GNSS receivers, flown against identical
      simulated rocket trajectories, to find where each one stops publishing
      position &mdash; and whether the reason is the COCOM export limit or
      something else entirely. <strong>Two of the five stop well below the export
      limit for unrelated reasons</strong>, and one implements no velocity limit
      at all.</p>
    <span class="status status-live">Five parts measured &middot; 25 runs</span>
  </header>

  <section>
    <h2><span class="n">01</span>The gates</h2>
    <div class="prose">
      <p>Every receiver here was flown against the same two trajectories through
        the same signal generator. This is where each one stopped publishing a
        position, and why.</p>
    </div>

    <div class="scroll">
      <table class="cmp">
        <caption>Measured gates, by receiver</caption>
{table.split(chr(10), 1)[1].rsplit(chr(10), 1)[0]}
      </table>
    </div>

    <div class="prose">
      <p class="note">Brackets are <em>(highest value that still held a fix,
        lowest value withheld]</em>, taken across every gate edge in every run for
        that part. &dagger; marks an inverted bracket &mdash; a value that held a
        fix sitting above one that was withheld, which happens when a receiver is
        slow to close. A parenthesised cause marks a ceiling that is <em>not</em>
        an export gate.</p>

      <h3>What the table says</h3>
      <p><strong>The export limits themselves are consistent.</strong> Of the four
        parts that implement a velocity gate, all four put it at the same place:
        taking every gate edge from every flight, the highest speed that still
        held a fix is 514&nbsp;m/s and the lowest that was withheld is
        516&nbsp;m/s, straddling the quoted 515. Where an altitude gate is
        genuinely an export gate, it is at <strong>80&nbsp;km</strong>, never at
        the commonly-quoted 18&nbsp;km. Both limits act
        <strong>independently</strong> &mdash; either closes the gate on its own,
        which contradicts SkyTraq's own FAQ.</p>
      <p><strong>What differs between parts is everything else.</strong> Two of
        the five stop publishing far below the export limit, for reasons that have
        nothing to do with export control: the NEO-M8T at 50&nbsp;km because of
        its u-blox dynamic model, and the Air530 at 10&nbsp;km for reasons unknown.
        A third, the Air530 again, implements no velocity limit at all &mdash; it
        held a fix to 900&nbsp;m/s. And re-open latency spans two orders of
        magnitude across the set.</p>
      <p><strong>For choosing a receiver, the export gate is the least
        interesting number on the page.</strong> It is the same everywhere. The
        useful columns are the ceiling and its cause: a part that stops at
        10&nbsp;km is unusable for high-power flight regardless of what the export
        rule says, and a part that stops at 50&nbsp;km cannot be fixed by
        configuration because no u-blox dynamic model goes higher.</p>
    </div>
  </section>

  <section>
    <h2><span class="n">02</span>How this was measured</h2>
    <div class="prose">
      <p>A HackRF One drives a simulated GPS L1 C/A constellation into each
        receiver through a fixed attenuator chain &mdash; conducted, never
        radiated, except for the flight computer's receiver which is fed by a
        quarter-wave inside a Faraday cage. The baseband comes from
        <code>gps-sdr-sim</code> against real broadcast ephemeris, so the
        satellites are where they actually were on the day.</p>

      <h3>Blocked is not the same as lost</h3>
      <p>The whole measurement rests on one distinction. A receiver that has been
        <em>gated</em> keeps tracking satellites and simply declines to publish a
        position; a receiver that has <em>lost signal</em> loses the satellites
        first. Every capture therefore records both the fix state and the
        per-satellite carrier-to-noise ratio, and each epoch is classified:</p>
      <div class="scroll">
        <table>
          <thead><tr><th>Verdict</th><th>Fix</th><th>Satellites</th><th>Meaning</th></tr></thead>
          <tbody>
            <tr><td><strong>FIX</strong></td><td>yes</td><td>tracked</td><td>publishing normally</td></tr>
            <tr><td><strong>BLOCKED</strong></td><td>no</td><td><strong>still tracked</strong></td><td>withholding &mdash; a gate</td></tr>
            <tr><td><strong>NO_LOCK</strong></td><td>no</td><td>gone</td><td>lost the signal &mdash; not a gate</td></tr>
          </tbody>
        </table>
      </div>
      <p>A result is only reported as a gate when the satellites stayed. Where a
        receiver was starved &mdash; fewer than four satellites through a wait
        &mdash; the analysis says so rather than quoting a latency.</p>

      <h3>The two standard trajectories</h3>
      <p>Every receiver flies both. They reach the same 82.5&nbsp;km apogee and
        cross the same limits; only the boost differs, and that difference is the
        point.</p>
      <div class="scroll">
        <table>
          <thead><tr><th>Scenario</th><th class="num">Boost</th><th class="num">Peak speed</th><th class="num">Apogee</th><th>Purpose</th></tr></thead>
          <tbody>
            <tr><td><code>spaceshot</code></td><td class="num">15 g</td><td class="num">1343 m/s</td><td class="num">82.5 km</td><td>realistic high-power flight; hardest case for tracking</td></tr>
            <tr><td><code>gentle_alt</code></td><td class="num">3 g</td><td class="num">997 m/s</td><td class="num">82.5 km</td><td>same envelope crossed slowly &mdash; this is what brackets the thresholds</td></tr>
          </tbody>
        </table>
      </div>
      <p><strong>Why two.</strong> At a 1&nbsp;Hz navigation rate a 15&nbsp;g boost
        covers 118&nbsp;m/s between one epoch and the next, so a fast flight can
        only ever bracket the velocity gate coarsely no matter how carefully it is
        flown. The 3&nbsp;g ascent crosses the same threshold slowly enough to pin
        it. Both are integrated from thrust, drag and gravity rather than drawn as
        ramps, and flown to the ground under a drogue-then-main recovery.</p>
      <p>Each flight opens with a 180&nbsp;s pad hold so the receiver has acquired
        and settled before anything moves. Trajectory time is recovered from the
        GPS time in the receiver's own output, so no clock synchronisation between
        the transmitter and the analysis is needed.</p>
    </div>
  </section>

  <section>
    <h2><span class="n">03</span>Receiver by receiver</h2>
    <div class="prose">
      <p>The same two profiles, part by part. Each pair of plots shows altitude,
        speed and acceleration against time, with a lock strip and satellite count
        underneath.</p>
    </div>
{"".join(parts_html)}
  </section>

  <section>
    <h2><span class="n">04</span>Excursion tests</h2>
    <div class="prose">
      <p>The flight profiles were enough for four of the five parts. They were not
        enough for the Air530, and the reason generalises: <strong>a ramp cannot
        measure a threshold that a receiver reacts to slowly.</strong> On a
        3&nbsp;g climb the vehicle spends about one second within
        &plusmn;15&nbsp;m/s of the velocity limit, so a receiver lagging a few
        seconds smears the answer across hundreds of m/s. What comes back is the
        latency, not the threshold &mdash; and read as a threshold it is simply
        wrong.</p>
      <p>The fix is to stop ramping and <strong>dwell</strong>: hold a value steady
        for far longer than any plausible lag, and the first level that blocks is
        the threshold. Five scenarios were built for this.</p>

      <div class="scroll">
        <table>
          <caption>Excursion scenarios</caption>
          <thead><tr><th>Scenario</th><th>Shape</th><th>Question it answers</th></tr></thead>
          <tbody>
            <tr><td><code>vel_stair</code></td><td>90 s dwells, 495&ndash;530 m/s at 5 km</td><td>where is the velocity gate, latency removed</td></tr>
            <tr><td><code>alt_stair</code></td><td>90 s dwells, 76&ndash;82 km</td><td>where is the altitude gate near 80 km</td></tr>
            <tr><td><code>alt_stair_low</code></td><td>90 s dwells, 12&ndash;22 km</td><td>is there a gate near the quoted 18 km</td></tr>
            <tr><td><code>alt_stair_vlow</code></td><td>90 s dwells, 8&ndash;13 km</td><td>brackets a ceiling below any export limit</td></tr>
            <tr><td><code>blockdur</code></td><td>5 / 30 / 150 s excursions, 155 s clear between</td><td>is a slow recovery a gate, or the receiver re-converging</td></tr>
          </tbody>
        </table>
      </div>

      <h3>What they established for the Air530</h3>
      <p>Three tests, three ways of asking the same question, all agreeing there
        is <strong>no velocity gate</strong>: 100% of epochs held a fix at every
        90&nbsp;s dwell from 495 to 530&nbsp;m/s; a ramp to 900&nbsp;m/s at
        5&nbsp;km never lost the fix and reported 899&nbsp;m/s against 900
        injected; and on <code>blockdur</code> the receiver held a fix at
        560&nbsp;m/s for <strong>148 continuous seconds</strong>.</p>
      <p>What it has instead is an altitude ceiling at
        <strong>10&ndash;11&nbsp;km</strong>. The dwell staircase is unambiguous
        &mdash; 100% of epochs fixed at 8&nbsp;km, 97% at 9, 91% at 10, and not one
        epoch at 11, 12 or 13&nbsp;km, with ten or eleven satellites tracked
        throughout &mdash; and two independent ramps agree (fix at 9.90&nbsp;km,
        blocked at 10.25&nbsp;km at a constant 354&nbsp;m/s; fix at 10.10, blocked
        at 10.44 at 394&nbsp;m/s).</p>
      <p><strong>That ceiling is not an export gate.</strong> It sits below both
        candidate COCOM altitudes, and <code>alt_stair_vlow</code> crosses no
        export limit at all &mdash; 13&nbsp;km and 156&nbsp;m/s are inside the
        envelope on any reading of the rule.</p>
      <p class="note">On a flight profile the ceiling and the velocity limit fire
        within moments of each other, because a rocket crosses 10&nbsp;km while
        travelling fast. The distinguishing observation is on <code>spaceshot</code>,
        where the fix stops while speed is <em>falling</em> (1334 &rarr;
        1304&nbsp;m/s) as altitude rises through 9.83&nbsp;km. No velocity gate
        fires on decreasing speed.</p>

      <h3>The same trap on the NEO-M8T</h3>
      <p>The M8T stops publishing at 50&nbsp;km, which is the documented maximum
        altitude of u-blox's airborne dynamic models rather than an export limit.
        The test that separates the two is to change the model instead of the
        trajectory: switching from airborne &lt;4&nbsp;g to portable moved the same
        ceiling from 49.80&nbsp;km to 5.04&nbsp;km. An export gate does not track
        the platform model.</p>
      <p>No u-blox model exceeds 50&nbsp;km, and airborne &lt;4&nbsp;g is already
        both the highest ceiling and the highest velocity limit available, so this
        part's true altitude behaviour cannot be measured at all &mdash; the model
        stops it first. The SAM-M10Q and ZED-F9P held fixes at 68.8&nbsp;km on that
        same model, so it is an M8-generation behaviour.</p>
    </div>

    <figure>
      <div class="panels" style="grid-template-columns:1fr">
        <div class="panel">
          <h4>u-blox NEO-M8T &mdash; altitude ramp <span class="badge">t2_altramp</span></h4>
          <p>85 km at a constant 354 m/s, so altitude is the only variable</p>
{fig("neo_m8t_t2_altramp.svg")}
        </div>
      </div>
      <figcaption>The M8T against an altitude-only ramp. The lock strip ends at
        50 km and never resumes while the satellite bar stays full &mdash; a
        receiver declining to publish, not one losing signal.</figcaption>
    </figure>
  </section>

  <section>
    <h2><span class="n">05</span>A receiver-side artifact</h2>
    <div class="prose">
      <p>The Air530's satellite count dips on a tight <strong>18&nbsp;s</strong>
        cycle, and a regular artifact on a bench is guilty until proven innocent
        &mdash; a loose connector or a playback stall would look like this and
        would invalidate every number here. Three observations place it inside the
        receiver.</p>
      <p><strong>A control part on the same path shows none of it.</strong> The
        ZED-F9P ran the identical scenario files through the identical
        70&nbsp;dB conducted chain with zero periodic dips.
        <strong>Every satellite blanks in the same epoch</strong> and returns
        together within three &mdash; attenuation is graded, so a real signal
        problem takes the weakest first; instead all twelve go from a flat
        49&nbsp;dBHz to an <em>empty</em> C/N&#8320; field at once, which is a
        reporting decision, not a level. And it happens
        <strong>while the receiver is withholding</strong>: 19 of 677 such epochs
        on <code>gentle_alt</code> against 0 of 170 while publishing, and on
        <code>blockdur</code>, where it holds a fix almost throughout, 2 events in
        988 epochs.</p>
      <p class="note">18&nbsp;s is three GPS subframes, the span of subframes
        1&ndash;3 that carry the ephemeris, so the part may re-validate ephemeris
        on that cadence and blank C/N&#8320; while doing so. That is a hypothesis;
        the three observations above stand without it.</p>
    </div>

    <figure>
      <div class="panels" style="grid-template-columns:1fr">
        <div class="panel">
          <h4>Air530 vs ZED-F9P &mdash; satellites tracked <span class="badge">control</span></h4>
          <p>same scenario file, same conducted path; only the part differs</p>
{fig("air530_dip_periodicity.svg")}
        </div>
      </div>
      <figcaption>The comb in the upper trace is the artifact; the lower trace is
        what the bench looks like without it.</figcaption>
    </figure>
  </section>

  <section>
    <h2><span class="n">06</span>What this cannot tell us</h2>
    <div class="prose">
      <p><strong>One signal, one band.</strong> The injection is GPS L1 C/A only.
        Every receiver here is multi-constellation and two are multi-band, so all
        of them are being tested well below what they can do. That is a property
        of the rig, not of any part, and it applies equally to all five &mdash;
        which is what makes the comparison fair even though it is not
        representative of open-sky performance.</p>
      <p><strong>One firmware, one sample.</strong> Each result is one physical
        unit at one firmware version. Nothing here says a different sample of the
        same part behaves identically, and the two ceilings that are not export
        gates are exactly the kind of thing a firmware revision could move.</p>
      <p><strong>Thresholds, not certification.</strong> These are bench
        measurements of where a receiver stops publishing. They are not a
        statement about what any receiver is licensed or required to do.</p>
      <p><strong>Where a ceiling masks the export gate, the export gate is
        unmeasured.</strong> The M8T's COCOM altitude behaviour above 50&nbsp;km
        and the Air530's above 10&nbsp;km cannot be reached on this rig, because
        the receivers stop for other reasons first.</p>
    </div>
  </section>

</div>
'''

# The stylesheet and design system are kept in report_head.html rather than in
# this script: they are hand-authored, they change rarely, and inlining ~9 kB of
# CSS in a generator makes the generator unreadable.
head = (HERE / "report_head.html").read_text()
out = head + "\n" + body
Path(HERE / "report.html").write_text(out)
print(f"  report.html rebuilt: {len(out)} bytes")

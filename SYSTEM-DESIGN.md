# Robotic Arm Drawing System — system design

> How a line of G-code becomes a pen stroke, one 0.2-unit step at a time.
>
> A command like `G01X4.0Y12.0` is read character by character, turned into a
> chain of Cartesian points 0.2 units apart, and each point is pushed through
> the same narrow gate: **two-link inverse kinematics** over two 9-unit arms,
> degrees quantized to **0.12° encoder clicks**, and a **differential motor
> command** — the difference between where each joint is remembered to be and
> where it must go, with the elbow's command corrected by the shoulder's move.
> What leaves the program is a stream of short serial commands —
> `E-50`, `D+19` — pacing a Rhino educational arm over COM1 at 9600 baud, 7E2.
> Every primitive's travel ends with an unconditional snap to the exact target, which
> is the real reason the drawings close.

This document is the developer-facing map of the whole system — every component
and how data moves between them. The companion [README](README.md) covers the
per-layer detail, building, and running.

---

## End-to-end flowchart

<p align="center"><img src="docs/system-design-flowchart.svg" alt="Robotic Arm Drawing System end-to-end flowchart. ParseGCodeD.cpp main builds Parse with the hardcoded file test5.txt (one of three face-drawing variants, test3, test5 and test6, each ending in an m80 line) and loops: parseLine, then wait for Enter. parseLine reads one line with getline, takes the G digit from CommandLine[2] and scans X, Y, I, J and Z (digits, dot and minus, then atof) into modal member state that is never reset, so the m80 line, which has no G word, re-runs the last move. Dispatch on the G digit: 0 is a jump (F −20 quill off, one MoveToXY to the target, F +20 quill on); 1 is Line, a relative vector in ±0.2 steps at an angle from the +Y axis, loop bound int(Length / 0.2) re-read each step, whose points Parse sends to MoveToXY; 2 and 3 are circle, with I and J as the absolute center and 360 / (2πR / 0.2) degrees per 0.2-unit arc step, which calls MoveToXY itself. Line and arc end with an endpoint snap; any other digit causes no motion. MoveToPoint::MoveToXY runs RhinoMath::getAngpair (two-link inverse kinematics, A = B = 9.0, degrees) and converts degrees to clicks with int(θ / 0.12), always motor E then motor D. RhrinoSpecific sends TickAng minus target and stores the target (seeded at 750 clicks = 90°, open loop); only the D command subtracts the E differential just issued; every command is chunked into ±50-click writes plus a remainder, with Sleep(120) before and after each write. The quill moves F −20 and F +20 skip the IK and TickAng. Tserial (tserial.h/.cpp, modified 2013) sends over COM1 at 9600 baud 7E2 with CR LF appended, transmit only, and a failed open turns every write into a no-op; RS-232 carries the commands to the Rhino arm: E shoulder, D elbow, F waist for quill off and on. Present but unused: RhinoMath.h (uncompilable, skipped because of a duplicate RHINOMAN_H guard), RhrinoSpecificold.cpp and ParseGCodeE4_27.cpp (not in the build), Z, QuilOutFlag and Line's MoveToPoint pointer. A legend maps the colours to input, parser, motion primitives, IK gate, motor layer and serial, hardware and unused parts." width="100%"></p>

---

## How to read it: the three ideas that matter

1. **Position state is layered, and every layer dead-reckons its own copy.**
   `Parse` holds the Cartesian pen position (twice, in fact: `CurrentX`/`CurrentY`
   and a `Current` struct); `Line` holds `UnitDistInc`, its progress along the
   current segment; `RhrinoSpecific` holds the arm's absolute pose in clicks
   (`TickAng1`/`TickAng2`). Nothing ever flows back up — the arm is never
   queried, the serial receive path is never used. Correctness at power-on
   rests on a three-way agreement between the seeds: Cartesian (9, 9) solves
   to exactly 90°/90°, which is exactly the 750/750 clicks the motor layer is
   born with. The one crack: `Parse`'s duplicate `Current` struct is seeded
   (0, 0), so a program whose *first* stroke is a `G01` — [test6.txt](test6.txt)
   is one — interpolates its first segment from the wrong origin until the
   endpoint snap corrects it.

2. **Absolute in, differential out.** Everything above the motor layer speaks
   absolute targets — Cartesian points, then absolute joint angles, then
   absolute click counts. `MotoMoveServo` is where absolute becomes relative:
   the command sent is `TickAng − target`, the new absolute is stored, and the
   elbow's command additionally subtracts the shoulder's just-issued
   differential — the correction you need when the joints' drives are
   mechanically coupled, and the reason `MoveToXY` must always send `E`
   immediately before `D` (the coupling reads state left behind by the
   previous call). The move then leaves as `±50`-click chunks plus a
   remainder, each write bracketed by 120 ms sleeps — pacing by delay, not by
   handshake.

3. **The endpoint snap is the real contract.** Every primitive — jump, line,
   arc — ends its travel with an unconditional `MoveToXY` straight to the
   exact target (a jump then only drops the quill with `F+20`).
   That single call is what the system actually guarantees; the interpolation
   in between only shapes the path. This is why several latent geometry bugs
   (the stale first loop bound, the leftward-horizontal walk, the vanishing
   CW arc) degrade drawings instead of breaking them: the pen always *arrives*,
   and `Parse`'s bookkeeping is updated from the target, not from the steps.
   It's also why the program is honest without hardware — if COM1 fails to
   open, every write silently no-ops and the console trace (parsed words,
   interpolated points, IK angles) becomes a full dry-run simulator.

---

## Deep dive 1 — one G01 line, end to end

Line 9 of [test5.txt](test5.txt) is `G01X4.0Y12.0`, arriving with the pen at
(3.77, 10.0) — the left ear stroke of the face:

<p align="center"><img src="docs/one-g01-line.svg" alt="Robotic Arm Drawing System, one G01 line end to end: a 21-step sequence across main, Parse, Line, MoveToPoint, RhinoMath, RhrinoSpecific and Tserial on COM1. Line 9 of test5.txt, G01X4.0Y12.0, starts with the pen down at (3.77, 10.0) and TickAng1/2 at 1024/892 clicks. main calls parseLine, which reads GCODE '1', X 4.0 and Y 12.0, calls Line's LineReset and sets WorkingPt to (0.23, 2.0). A loop runs 10 passes; its bound int(Length / 0.2) first reads the stale constructor Length of 1.0. Each pass, getNextPt adds 0.2 to UnitDistInc along atan(0.23 / 2.0), about 6.56° from +Y, and returns the next point ((3.79, 10.20) on pass 1); MoveToXY asks RhinoMath for the two-link IK angles in degrees (122.41° and 105.61° on pass 1), calls MotoMoveServo('E', 1020), which writes the differential E+4 with CR LF, then MotoMoveServo('D', 880), whose differential 12 minus E's 4 is written as D+8; Parse prints the point. After the loop the endpoint snap MoveToXY(4.0, 12.0) gives 116.92° and 90.71°, absolute clicks E 974 and D 755, so it writes only E-0 and D+1. Parse sets Current to (4.0, 12.0) and returns to main, which waits for the next Enter; every write sits between two 120 ms sleeps, about 5.3 s for this stroke." width="100%"></p>

Things worth noticing:

- **The loop bound heals itself.** `getLineSteps(0.2)` divides a `Length`
  member that is only recomputed inside `getNextPt`, so the *first* loop-entry
  check uses whatever the previous segment left behind (the constructor's 1.0
  here, since this is the file's first `G01`). From iteration one onward the
  bound is the true `int(2.013 / 0.2) = 10`.
- **Each interpolated point costs at least ~480 ms of sleep** — two motors,
  each write bracketed by 120 ms `Sleep`s. Ten steps
  plus a snap make this one stroke a multi-second affair by design.
- **The IK trace is the observable output.** `getAngpair` prints both angles
  on every call; with the arm unplugged this sequence is exactly what you see,
  minus the serial writes.

## Deep dive 2 — the differential tick pipeline

A hypothetical first move straight from home to (4, 12), worked through
[RhrinoSpecific::MotoMoveServo](RhrinoSpecific.cpp)'s arithmetic (in
[test5.txt](test5.txt) the arm arrives there from (3.77, 10.0), so the real
differences are smaller):

<p align="center"><img src="docs/tick-pipeline.svg" alt="Robotic Arm Drawing System, the differential tick pipeline for one MoveToXY call, worked for (4.0, 12.0) as a hypothetical first move from home. RhinoMath::getAngpair, with ArmA = ArmB = 9.0, gives θ2 = acos(−2 / 162) = 90.71° and θ1 = 45.35° + 71.57° = 116.92°; MoveToXY truncates θ / 0.12 to absolute clicks: 974 for motor E, the shoulder, and 755 for motor D, the elbow. Call 1, MotoMoveServo('E', 974): remembered TickAng1 = 750 (the seed), diff = 750 − 974 = −224 (old minus new), TickAng1 becomes 974, and Distance −224 is sent unchanged and kept as EmotorDistance and EmotorDistance2. Call 2, MotoMoveServo('D', 755): diff = 750 − 755 = −5, TickAng2 becomes 755, and Distance = −5 − (−224) = +219, the elbow's command corrected by the shoulder's move. Each call chunks its distance into 50-click commands plus a remainder that is always written: E-50 four times and E-24 (writes 1 to 5, all before call 2), then D+50 four times and D+19 (writes 6 to 10). A zoomed write shows Sleep(120), the bytes E, minus, 5, 0 (0x45 0x2D 0x35 0x30), CR and LF, then Sleep(120), on COM1 at 9600 baud, 7 data bits, even parity, 2 stop bits. The whole MoveToXY is 10 writes and 60 bytes, about 69 ms on the line, against 2.4 s of Sleep; if COM1 fails to open the writes do nothing but the sleeps still run. Notes add that in test5.txt the pen reaches (4, 12) at the end of line 9's interpolation, so the real snap sends only E-0 and D+1, and that the quill calls F-20 and F+20 match neither branch and leave the remembered state alone." width="100%"></p>

- **`TickAng` stores the absolute, sends the difference.** After this move the
  layer remembers 974/755; the next target is diffed against that. The arm is
  trusted to have executed everything — pure open-loop dead reckoning.
- **The coupling is stateful and order-dependent.** `EmotorDistance2` is
  refreshed after *every* call's E-block, so the D calculation reads the E
  differential from the immediately preceding call. `MoveToXY`'s fixed
  E-then-D order is load-bearing; a standalone D move would subtract a stale
  shoulder differential.
- **The quill moves skip all of it.** The waist motor's `F-20` / `F+20`
  (quill off / on) fall through both branches and go out as-is, relative
  every time, no remembered state.

---

## Component inventory

| Component | Layer | Provenance | Where |
|---|---|---|---|
| Driver `main` — Enter-per-line stepper, hardcoded `test5.txt` | Input | project code | [ParseGCodeD.cpp](ParseGCodeD.cpp) |
| `Parse` — char-switch parser, modal words, G dispatch | Parser | scaffolding skeleton, filled in here | [Parse.h](Parse.h) / [Parse.cpp](Parse.cpp) |
| `Line` — 0.2-unit linear interpolation | Geometry | ✅ the marked student piece ("add your line code") | [Line.h](Line.h) / [Line.cpp](Line.cpp) |
| `circle` — degree-swept arc interpolation | Geometry | instructor demo code ("converted to degrees for demonstration") | [CircleC.h](CircleC.h) / [CircleC.cpp](CircleC.cpp) |
| `MoveToPoint` — the IK gate, E-then-D ordering | Gate | project code | [MoveToPoint.h](MoveToPoint.h) / [MoveToPoint.cpp](MoveToPoint.cpp) |
| `RhinoMath` — two-link IK, degree output | Kinematics | project code | [RhinoMan.h](RhinoMan.h) / [RinoMath.cpp](RinoMath.cpp) |
| `RhrinoSpecific` — differential ticks, coupling, chunked commands | Motor | demo scaffolding + in-class rework ("Prof and Rebecca") | [RhrinoSpecific.h](RhrinoSpecific.h) / [RhrinoSpecific.cpp](RhrinoSpecific.cpp) |
| `Tserial` — Win32 serial wrapper, TX-only in practice | Wire | third-party, vendored (header dated 2013) | [tserial.h](tserial.h) / [tserial.cpp](tserial.cpp) |
| Face drawings ×3 | Test data | project code | [test3.txt](test3.txt) / [test5.txt](test5.txt) / [test6.txt](test6.txt) |
| Build — VS 2019, v142, Win32 + x64, console | Build | Visual Studio template | [ParseGCodeE4_27.sln](ParseGCodeE4_27.sln) / [.vcxproj](ParseGCodeE4_27.vcxproj) |
| `MasterHeader` class declaration | — | ⬜ dead header, shielded by duplicate include guard | [RhinoMath.h](RhinoMath.h) |
| Older motor layer with debug trace | — | ⬜ superseded, excluded from build | [RhrinoSpecificold.cpp](RhrinoSpecificold.cpp) |
| Template "Hello World" `main` | — | ⬜ stub, excluded from build | [ParseGCodeE4_27.cpp](ParseGCodeE4_27.cpp) |
| Parser-source snapshot dated 2022-04-26 | — | ⬜ archive | [ParseGCodeE4_27.zip](ParseGCodeE4_27.zip) |

---

## The numbers that matter

| Value | What it is |
|---|---|
| 9.0 / 9.0 | link lengths `ArmA` / `ArmB` — `RhinoMath` constructor defaults |
| 0.12° | one encoder click; `MoveToXY` divides degrees by `.12` |
| 750 / 750 | `TickAng1` / `TickAng2` seeds — "clicks to get from 0° to 90°" |
| (9, 9) | `Parse`'s Cartesian home — solves to exactly 90°/90° = 750/750 clicks, closing the three-way seed agreement |
| 0.2 | the interpolation quantum: units per line step and per arc step — a literal in three places (both call sites in `Parse.cpp`, plus `Line::getNextPt`); `RhrinoSpecific`'s settable `unitstepVal = 0.2` is never read |
| 360 / (2πR / 0.2) | degrees per arc step, derived from radius so each step covers 0.2 units of arc |
| 50 | maximum clicks per serial command; moves chunked into `±50`s + remainder |
| ±20 | waist (`F`) clicks: −20 quill off, +20 quill on — always relative |
| 120 ms | `Sleep` before *and* after every serial write (~480 ms floor per interpolated point; the old motor layer used 250 ms) |
| 9600 · 7E2 | COM1 configuration: `BaudRate` 9600, `ByteSize` 7, `EVENPARITY`, `TWOSTOPBITS` — not the 8N1 the old README claimed |
| 80 | `getline` buffer size (at most 79 characters per G-code line) — and, coincidentally, the `RobCmd` buffer size |
| `CommandLine[2]` | where the G digit is read from — zero-padded two-digit codes only |
| `'8'` | the stop sentinel (third char of a G line); the `m80` trailer never matches it |
| 3 / 0 | sample drawings / automated tests |

---

## Verification status

There are **no automated tests** — no unit tests, no harness, no asserts. The
code was validated the way classroom robot code usually is, and the repo shows
its work:

- **The driver is the debugger.** `while(1) { parseLine(); cin.get(); }`
  single-steps the program one G-code line per Enter, and the console traces
  each step (the arc and motor layers print nothing):
  the parser echoes `G0# X… Y… I… J…`, the line loop prints each interpolated
  point, and the IK prints both joint angles per solve. `circle` still carries
  a `test` variable kept "for break point to be removed".
- **Dry-run by default.** If COM1 can't be opened, the failed connect leaves an
  invalid handle and every write silently no-ops, so the full pipeline runs as
  a console trace on any Windows machine without a COM1 port.
- **Hardware runs happened but aren't captured here** — the in-class comments
  ("work explained in class with Prof and Rebecca", tuning notes like
  "Needs to be 1 for line but 50 for point-to-point") are the surviving
  evidence of iteration against the physical arm.
- What the three test files *don't* cover is exactly where the latent bugs
  live: no leftward-horizontal `G01`, no CCW arc with start angle below end
  angle, no CW arc crossing 0°, no out-of-reach target.

---

## Design trade-offs & sharp edges

- **Open-loop dead reckoning over feedback** — the serial receive path exists
  in `Tserial` but is never called; the arm is never asked where it is. Cheap
  and simple, but any lost command desynchronizes `TickAng` from reality until
  restart re-seeds home.
- **The endpoint snap over correct interpolation** — the unconditional final
  `MoveToXY` converts geometry bugs into path-quality bugs. It is why the
  stale first loop bound, the leftward-horizontal walk (ΔY = 0, ΔX < 0 steps
  *away* from the target), and the vanishing CW-across-0° arc all still
  produce closed drawings.
- **Degrees end to end** — every interpolation angle (`Line`'s heading,
  `circle`'s sweep) is converted to degrees at birth ("for demonstration",
  per the comments) and back to radians inside each trig call; the IK
  solves in radians and converts to degrees only on output. Readable in the
  console, wasteful in the math, and the reason
  the arc sweep logic can use `+360` bookkeeping — which is also where the
  CCW extra-lap bug lives (`G03` with start < end draws a full pen-down
  revolution).
- **An absolute-center `I,J` dialect** — `circle` subtracts `(I, J)` from both
  endpoints, so the files encode arc centers absolutely, not as the standard
  offsets-from-start. The test files and the parser agree with each other and
  disagree with vanilla G-code; feeding this interpreter real CNC output would
  draw the wrong arcs. (`Parse` then reconstructs the current position as
  `Target + Center` — correct only because `DoitC` mutates its by-reference
  arguments first. Fragile, but verified.)
- **Sleep-paced serial over handshake** — flow control disabled, timeouts
  zeroed, 120 ms sleeps bracketing every write. Robust against nothing,
  but simple, and the ~480 ms-per-point floor doubles as the arm's settling
  time.
- **Stateful coupling over stateless commands** — the elbow correction reads
  the shoulder differential left by the previous call, hard-wiring the
  E-then-D calling convention into correctness. Refactor `MoveToXY`'s two
  lines apart and the arm draws garbage.
- **Two integration styles side by side** — `circle` drives the arm itself
  through an injected `MoveToPoint*`; `Line` hands points back and lets
  `Parse` drive. `Line` also has an injected pointer — never set, never used.
  The asymmetry is the visible seam between provided demo code and the
  student-built primitive.
- **One bug shielding another** — [RhinoMath.h](RhinoMath.h) cannot compile
  (nonexistent `AngPair` type), but its copy-pasted `RHINOMAN_H` guard makes
  every inclusion after [RhinoMan.h](RhinoMan.h) a no-op. Delete the "unused"
  guard collision and the build breaks.
- **The `m80` trailer is not recognized** — it leaves all state stale and the
  dispatch replays the previous command once per trailer line before EOF ends
  things. In test3/test5 that is a near no-op (a repeated `G00` home, a
  zero-sweep `G02` that just snaps to (9, 9)). In [test6.txt](test6.txt) it
  replays the final `G01X1.1Y1.0`: rounding in Parse's `CurrentX` update,
  (1.1 − 9.0) + 9.0, leaves a residual ΔX of about 4e-16 with ΔY = 0, so
  the angle is `atan(ΔX / 0)` = 90°. The stale `Length` lets the loop run
  once, and the arm makes a stray 0.2-unit move to (1.3, 1.0) before the
  endpoint snap returns it to (1.1, 1.0).

---

## Provenance

A classroom robotics project from spring 2022, targeting a Rhino-brand
educational arm — dated by the "Did not get 3-D model in 2022" comment, the
"5_17 add" fix, and the 2022-04-26 source snapshot in
[ParseGCodeE4_27.zip](ParseGCodeE4_27.zip). Instructor-provided scaffolding
(the parser skeleton, `circle` demo, motor-command demo — first-person
instructor comments throughout, including a "see page 153" course-text
reference) with the student work layered on top: the `Line` interpolator
(marked by `//add your line code` in [Parse.h](Parse.h)), the parser's
dispatch wiring, and the differential-tick / shoulder-elbow coupling rework of
[RhrinoSpecific.cpp](RhrinoSpecific.cpp), developed in class with the
professor and a collaborator named Rebecca (credited in
[RhrinoSpecificold.cpp](RhrinoSpecificold.cpp)'s comments). The serial layer,
[tserial](tserial.cpp), is vendored third-party Win32 code whose header block
dates it to April 2013.

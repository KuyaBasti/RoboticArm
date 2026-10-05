# Robotic Arm Drawing System

<p align="center"><img src="docs/system-overview.svg" alt="Robotic Arm Drawing System overview. The main() console loop calls Parse once at launch and again after each Enter key press; each call reads one line of the hardcoded G-code file test5.txt and scans it for G, X, Y, I, J and Z words. Parse dispatches on the G digit: G00 rapid (quill off, jump, quill on), Line for G01 in 0.2-unit linear steps, or circle for G02/G03 in 0.2-unit arc steps, clockwise or counter-clockwise. Every XY point, ending with an endpoint snap to the target, goes to MoveToPoint, the one gate to the motors. MoveToPoint asks RhinoMath for two-link inverse kinematics with 9.0 + 9.0 arms and gets back θ1 and θ2 in degrees, then passes RhrinoSpecific absolute clicks (θ ÷ 0.12°) for motors E and D, or ±20 on F for quill off and on. RhrinoSpecific sends only the click difference, takes the shoulder's difference off the elbow's, and splits each move into ±50 chunks; commands such as E-50 and D+19, with 120 ms sleeps, go through Tserial on COM1 at 9600 baud 7E2 as CR-LF lines, transmit only, to the Rhino arm's E shoulder, D elbow and F waist motors. A legend marks program code in green, console and serial I/O in orange, the G-code input file in gray and hardware outside the repo in blue." width="100%"></p>

A Windows C++ console program that reads **CNC G-code** and draws with a **two-link Rhino educational robot arm** over RS-232. [Parse](Parse.cpp) walks each command line character by character (`G`/`X`/`Y`/`I`/`J`/`Z`), fans out to motion primitives — a single jump for `G00`, **0.2-unit linear interpolation** for `G01`, **degree-swept arcs** for `G02`/`G03` — and every generated point funnels through one gate: `MoveToXY`, which solves **two-link inverse kinematics** (two 9.0-unit arms), quantizes the joint angles at **0.12° per encoder click**, and emits chunked serial commands (`E-50`, `D+19`, …) to the arm's shoulder (`E`) and elbow (`D`) motors on COM1. The waist motor (`F`) gets fixed `F-20` / `F+20` moves around each `G00` jump to take the quill off the board and put it back.

The interesting part isn't the G-code parsing — it's the bottom of the stack: **the motor layer converts absolute poses into differential moves**. [RhrinoSpecific](RhrinoSpecific.cpp) remembers where each joint is in clicks (`TickAng1`/`TickAng2`, seeded at 750 = 90°), sends only the difference, and **subtracts the shoulder's move from the elbow's command** — evidently compensating for a mechanically coupled drive. Above it, `Parse` dead-reckons its own copy of "where the pen is" in Cartesian units (`CurrentX`/`CurrentY`), and the system stays consistent because the home pose agrees three ways: Cartesian (9, 9) ↔ joints 90°/90° ↔ 750/750 clicks — except `Parse`'s second copy, the `Current` struct, seeded (0, 0), which a leading `G01` ([test6.txt](test6.txt)) interpolates from until the endpoint snap corrects it.

---

## Table of Contents

1. [From G-code to Serial Bytes](#from-g-code-to-serial-bytes)
2. [Repository Map](#repository-map)
3. [The Parser](#the-parser)
4. [Motion Primitives — Lines and Arcs](#motion-primitives--lines-and-arcs)
5. [Inverse Kinematics](#inverse-kinematics)
6. [The Motor Layer — Absolute In, Differential Out](#the-motor-layer--absolute-in-differential-out)
7. [The Serial Wire](#the-serial-wire)
8. [The Test Drawings](#the-test-drawings)
9. [Build & Run](#build--run)
10. [Known Limitations & Sharp Edges](#known-limitations--sharp-edges)
11. [Provenance](#provenance)

---

## From G-code to Serial Bytes

<p align="center"><img src="docs/gcode-to-serial.svg" alt="Robotic Arm Drawing System, from G-code to serial bytes. main reads the hardcoded test5.txt one line at launch and then one per Enter key, until end of file or a G digit of 8. Parse::parseLine switches on each character (G, X, Y, I, J, Z) and takes the G digit from CommandLine[2]; X, Y, I and J persist across lines, and Z is parsed but never used. A switch on the G digit branches three ways. G00: F-20 quill off, one MoveToXY to the target, F+20 quill on. G01: Line::getNextPt returns points in steps of ±0.2 from Current, and Parse loops int(Length / 0.2) times. G02/G03: circle::DoitC treats I, J as the absolute center (G02 clockwise, G03 counter-clockwise) and steps 0.2 units of arc. Parse sends each line step, the line's endpoint snap and the G00 jump to MoveToPoint::MoveToXY; DoitC sends each arc step and its endpoint snap itself. MoveToXY, the single entry for every X/Y point, calls RhinoMath::getAngpair (two-link inverse kinematics, arms of 9.0, θ1 and θ2 in degrees), then MotoMoveServo('E', int(θ1/.12)) and MotoMoveServo('D', int(θ2/.12)). In RhrinoSpecific.cpp, E and D send TickAng1 or TickAng2 minus the target and then store the target (seeded at 750 clicks = 90°); D also subtracts E's differential from the call just before. The quill calls F-20 and F+20, from MotoMoveServoFout and MotoMoveServoFin, skip MoveToXY, the IK and this bookkeeping. Every motor's move goes out as |cmd|/50 commands of ±50 plus one remainder command, with Sleep(120) before and after each write. Tserial sends each command to COM1 at 9600 baud, 7E2, terminated CR LF, and skips writes if COM1 failed to open; for example, home to (4, 12) sends E-50 ×4, E-24, then D+50 ×4, D+19. The Rhino educational arm (hardware) has E shoulder, D elbow and F waist (−20 quill off, +20 quill on) and runs open loop: nothing is read back." width="100%"></p>

One `G01` line becomes: a relative vector, `int(length / 0.2)` interpolated points, one IK solve and two `MotoMoveServo` calls (E then D) per point, each sent as zero or more ±50-click serial commands plus one remainder command (for a 0.2-unit step usually just the remainder), then an unconditional **endpoint snap** — a final `MoveToXY` straight to the exact target, which is what actually guarantees the pen arrives even when the interpolation wanders (see [sharp edges](#known-limitations--sharp-edges)).

## Repository Map

```text
RoboticArm/
├── README.md                  # you are here
├── SYSTEM-DESIGN.md           # the architecture-level view
├── docs/                      # SVG diagrams embedded in the two docs above
│   ├── system-overview.svg    # one-screen overview, top of this README
│   ├── gcode-to-serial.svg    # one G-code line → serial bytes (above)
│   ├── system-design-flowchart.svg # SYSTEM-DESIGN's end-to-end flowchart
│   ├── one-g01-line.svg       # deep dive 1: one G01 line, step by step
│   └── tick-pipeline.svg      # deep dive 2: the differential tick pipeline
├── ParseGCodeE4_27.sln        # Visual Studio 2019 solution (v142 toolset, Win32 + x64)
├── ParseGCodeE4_27.vcxproj    # the build: 8 .cpp files — note who's missing (below)
├── ParseGCodeD.cpp            # the real main(): open test5.txt, run line 1, then one line per Enter
├── ParseGCodeE4_27.cpp        # VS "Hello World" template stub — NOT in the build
├── Parse.h / Parse.cpp        # char-switch G-code parser + G00/G01/G02-G03 dispatch
├── Line.h / Line.cpp          # linear interpolation in 0.2-unit steps
├── CircleC.h / CircleC.cpp    # arc interpolation — angles swept in degrees
├── MoveToPoint.h / .cpp       # the gate: IK → 0.12° clicks → E/D/F motor calls
├── RhinoMan.h                 # AnglePair struct + RhinoMath class declaration
├── RinoMath.cpp               # two-link inverse kinematics (yes, spelled "Rino")
├── RhinoMath.h                # DEAD header — uncompilable, shielded by a duplicate include guard
├── RhrinoSpecific.h / .cpp    # motor layer: differential clicks, chunked "E±50" commands
├── RhrinoSpecificold.cpp      # older motor layer kept for reference — NOT in the build
├── tserial.h / tserial.cpp    # vendored Win32 serial-port wrapper (header dated 2013)
├── test3.txt / test5.txt      # the face drawing, travel moves pen-up (test3 adds a G00 home)
├── test6.txt                  # same figure with every travel move drawn (all G01)
└── ParseGCodeE4_27.zip        # snapshot of the parser sources dated 2022-04-26
```

The [.vcxproj](ParseGCodeE4_27.vcxproj) compiles exactly eight `.cpp` files. Two are deliberately excluded: [ParseGCodeE4_27.cpp](ParseGCodeE4_27.cpp) (the project's namesake, still the untouched Visual Studio template — it would add a second `main`) and [RhrinoSpecificold.cpp](RhrinoSpecificold.cpp) (a superseded motor layer — it would add a second `MotoMoveServo`).

## The Parser

[Parse::parseLine](Parse.cpp) processes **one line per call** (the driver waits for Enter between lines). It reads at most 79 characters (an 80-char `getline` buffer), then walks the string with a `switch` on each character: `G` grabs the command digit, `X`/`Y`/`I`/`J`/`Z` each run a little scanner that accepts digits, `.`, and `-` and converts with `atof`.

- **The G command is read from `CommandLine[2]`** — the parser assumes a zero-padded two-digit code at the start of the line (`G00`…`G03`). `G1X…` would store `'X'` as the command and silently draw nothing.
- **Words are modal by construction**: `X`, `Y`, `I`, `J` are member variables that persist across lines, so a line that omits a word reuses the previous value — standard G-code modality, achieved for free.
- **Dispatch**: `G00` lifts the quill (`F-20`), jumps to the target with a single `MoveToXY`, and drops it (`F+20`); `G01` runs the line interpolator; `G02`/`G03` run the arc interpolator with `Direction = "CW"` / `"CCW"`. `Z` is scanned but drives nothing.
- **Termination** is end-of-file or a command whose third character is `'8'` (e.g. a `G28`). The `m80` trailer that ends every test file matches *neither* — parsing it changes no state, so the motion switch re-executes the **previous** command with stale values before the file finally hits EOF.

## Motion Primitives — Lines and Arcs

**[Line](Line.cpp)** works on the *relative* vector from current position to target. `getNextPt` accumulates `UnitDistInc` in ±0.2 steps — the sign chosen by which half-plane the endpoint lies in — and converts to XY with an angle measured **from the +Y axis**: `Angle = atan(ΔX/ΔY)`, then `X = d·sin(Angle)`, `Y = d·cos(Angle)`. The sign trick plus `atan`'s folding covers all four quadrants correctly — except one degenerate ray (see sharp edges). The loop bound `int(Length / 0.2)` is re-evaluated every iteration against a `Length` that is only computed *inside* `getNextPt`, so the first iteration's bound uses the **previous `G01` line's length** (1.0, the constructor default, for the very first line). The endpoint snap papers over both quirks.

**[circle](CircleC.cpp)** treats `I`,`J` as the **absolute center coordinates** (not the standard relative offsets — verifiable from `Pt1 -= Cpt; Pt2 -= Cpt`). It translates both endpoints into the center's frame, computes start/end angles in degrees with quadrant-fixed `atan`, radius from the start point, and an angular step worth 0.2 units of arc: `360 / (2πR / 0.2)` degrees. CCW sweeps `Angle1` **up to `Angle2 + 360`**; CW sweeps down to `Angle2`. Each plotted point is translated back (`+ Cpt`) and fed to `MoveToXY`, ending — as always — with the endpoint snap.

## Inverse Kinematics

[RinoMath.cpp](RinoMath.cpp) solves the classic two-link planar arm (`ArmA = ArmB = 9.0` by default, set in the [RhinoMan.h](RhinoMan.h) constructor):

```cpp
// elbow:    cos(θ2) = (x² + y² − A² − B²) / (2·A·B)
// shoulder: θ1 = asin(A·sin(θ2) / √(x² + y²)) + atan(y/x)
```

Results are converted to degrees and printed to the console on every call — the IK trace is half the program's observable output. Note the shoulder formula uses `ArmA` where the general form wants the *distal* link length; with equal 9/9 arms the two are identical, so it only matters if you change the geometry. There is **no reachability guard**: a target outside the annulus hands `acos` an argument beyond ±1, and the resulting NaN flows straight into `int(NaN / 0.12)`.

The home pose is self-consistent three ways: Cartesian (9, 9) gives exactly θ1 = θ2 = 90°, which is exactly the 750 clicks that `TickAng1`/`TickAng2` are seeded with (90° / 0.12° = 750). The first real move therefore transmits only a genuine difference.

## The Motor Layer — Absolute In, Differential Out

[MoveToPoint::MoveToXY](MoveToPoint.cpp) passes **absolute** click counts down: `MotoMoveServo('E', int(θ1/.12))` then `('D', int(θ2/.12))`. [RhrinoSpecific::MotoMoveServo](RhrinoSpecific.cpp) turns them into moves:

1. **Differential** — the command is `TickAng − target`; the new absolute is stored back into `TickAng1`/`TickAng2`. The arm is driven open-loop from remembered state, never queried.
2. **Coupling** — the elbow's move has the shoulder's just-computed differential subtracted (`DmotorDistance = diff − EmotorDistance2`), tying D's motion to E's. This only works because `MoveToXY` always sends E immediately before D.
3. **Chunking** — the move is emitted as `|Distance| / 50` commands of `±50` clicks plus one remainder command (`E-50`, `E-50`, …, `E-24`), each terminated CR-LF, with a **120 ms `Sleep` before and after every write**. Two motors per point means a hard floor of ~480 ms per interpolated point.

The waist motor `F` is simpler: hardcoded relative `F-20` (waist out, quill off) and `F+20` (waist in, quill on), no bookkeeping.

## The Serial Wire

[tserial](tserial.cpp) is a vendored Win32 wrapper (`CreateFileA`, `WriteFile`, DCB configuration; its header block dates it to April 2013 and Windows 95–7). The constructor of `RhrinoSpecific` connects at object construction: **COM1, 9600 baud** — and, per the DCB it builds, **7 data bits, even parity, 2 stop bits (7E2)**, flow control disabled, all timeouts zero. The `operator<<(Tserial&, char*)` appends CR LF to every command.

If COM1 fails to open, the object keeps an invalid handle and **every write silently no-ops** — the program degrades into a console-only simulator, which is genuinely useful: the parse echo, the `G01` interpolated points, and the IK angles of every point (arc steps included) still print, so the whole pipeline can be exercised without an arm attached.

## The Test Drawings

| File | Lines | What it is |
|---|---|---|
| [test5.txt](test5.txt) | 21 + `m80` | The main drawing: a cartoon face — a large ~267° head arc centered (7, 7), two pointed ears drawn with `G01` spikes up to Y = 12, two upward eye arcs, a triangular nose at (7, 7), and a two-scallop mouth. Travel moves are pen-up `G00`. |
| [test3.txt](test3.txt) | 22 + `m80` | Same drawing plus a final `G00X9.0Y9.0` return to home. |
| [test6.txt](test6.txt) | 22 + `m80` | Same figure with **every** `G00` replaced by `G01` — the pen never lifts, so all travel moves are drawn — plus a final stroke to (1.1, 1.0). |

All three files contain the typo `J8..0`; the scanner accepts it and `atof` stops at the second dot, yielding 8.0 — accidentally harmless. All three end with `m80`, which the parser does not recognize (see sharp edges).

## Build & Run

Honestly: this is a **Windows + MSVC project**, and nothing else will do — it uses `windows.h`, `Sleep`, `strcpy_s`/`_itoa_s`, `system("chdir")`, and Win32 COM-port I/O.

- **Visual Studio 2019 or later** with the **v142 platform toolset** and Windows 10 SDK (per the [.vcxproj](ParseGCodeE4_27.vcxproj)). Debug/Release, Win32/x64 all configured.
- Open [ParseGCodeE4_27.sln](ParseGCodeE4_27.sln), Build Solution (Ctrl+Shift+B).
- **Run with the repo as the working directory** — the input path is the relative, hardcoded `"test5.txt"` in [ParseGCodeD.cpp](ParseGCodeD.cpp) (`main` even calls `system("chdir")` first to show you where it's looking). Switching drawings means editing that string and rebuilding.
- The driver is an intentional single-stepper: `while(1) { DataIn.parseLine(); cin.get(); }` — the first G-code line runs immediately at launch, then **press Enter to execute each next line**. There is no clean exit; close the console when done.
- **With hardware**: a Rhino-style arm controller on COM1 at 9600/7E2 (port name hardcoded in [RhrinoSpecific.h](RhrinoSpecific.h)). **Without hardware**: the failed COM1 open silently disables writes and the program runs as a console trace of the full pipeline.

## Known Limitations & Sharp Edges

Honest notes — all verified in the code; several are latent because the three test files never trigger them:

- **`m80` replays the last command.** The end-of-program marker matches no parser case, so nothing updates — and the motion switch then re-executes the previous G-code with stale values. Termination actually relies on EOF (or a `G` line whose third char is `'8'`, which no test file contains).
- **Leftward horizontal lines walk the wrong way.** For a `G01` with ΔY = 0 and ΔX < 0, the half-plane sign trick yields −0.2 while `atan(ΔX/0)` folds to −90°; the two sign flips cancel and every interpolated point steps **+X**, away from the target, until the endpoint snap yanks the pen to the true destination.
- **The first loop bound is stale.** `getLineSteps` divides a `Length` that is only set inside `getNextPt`, so entering the loop is gated on the last `Length` that `getNextPt` computed, which is the *previous* `G01`'s length (1.0 for the first line ever; `G00` jumps and arcs never touch it). Once a `G01` shorter than 0.2 has run, the bound is 0, `getNextPt` never runs again to refresh `Length`, and every later `G01` draws no intermediate points at all — just the snap.
- **CCW arcs can lap; CW arcs can vanish.** `G03` sweeps to `Angle2 + 360`, so a CCW arc whose start angle is below its end angle draws a **full extra pen-down revolution**. `G02` sweeps down to `Angle2`, so a CW arc crossing 0° fails the loop condition entirely and degenerates to a straight endpoint snap. The test files happen to avoid both cases.
- **No reachability or quadrant guards in the IK.** Unreachable targets produce NaN (unguarded `acos`); `atan(y/x)` instead of `atan2` mis-folds targets with x < 0. The drawings' targets all sit in the first quadrant.
- **A dead header shielded by a duplicated include guard.** [RhinoMath.h](RhinoMath.h) declares a `MasterHeader` class that cannot compile (it uses a nonexistent `AngPair` type) — but its copy-pasted guard `RHINOMAN_H` matches [RhinoMan.h](RhinoMan.h)'s, so [MoveToPoint.cpp](MoveToPoint.cpp), the only file that includes it, silently skips it. One bug hides another.
- **Case-mismatched includes.** [Parse.h](Parse.h) includes `"circleC.h"` and `"line.h"`; the files are `CircleC.h` and `Line.h`. Fine on Windows, fatal on a case-sensitive filesystem.
- **Everything is hardcoded**: `COM1`, `"test5.txt"`, the 0.2 step (a literal in three places — both call sites in [Parse.cpp](Parse.cpp) and again inside `Line::getNextPt` — despite the old README calling it configurable; the settable `unitstepVal = 0.2` in [RhrinoSpecific.h](RhrinoSpecific.h) is never read), 9/9 arm lengths, ±20 waist (`F`) clicks.
- **No failure checks**: a missing input file is not detected (`parseLine` prints an all-zero echo line and moves nothing, forever, one Enter at a time), serial connect errors are computed and then ignored, and `MotoMoveServo`'s return value is always `false` and always discarded.
- **Dead weight**: `Z` is parsed but unused, `QuilOutFlag` is never read, `circle` carries unused `CCAngle` math and a `test` variable kept "for break point", `Line`'s injected `MoveToPoint*` is never set *or* used, and `Tserial`'s `operator>>(Tserial&, char*)` reads zero bytes per pass and never advances, so it would loop forever on any non-empty buffer if anything called it (nothing does — the wire is transmit-only).

## Provenance

This is a **classroom robotics project from spring 2022**, evidenced by the repo itself: `MoveToPoint.h` notes "Did not get 3-D model in 2022", [RhrinoSpecificold.cpp](RhrinoSpecificold.cpp) credits "work explained in class with Prof and Rebecca" (with change-log comments like "Rebecca changed this to a 2 … I changed to 1"), a "5_17 add" fix is dated in [RhrinoSpecific.cpp](RhrinoSpecific.cpp), and the bundled [ParseGCodeE4_27.zip](ParseGCodeE4_27.zip) snapshots the parser sources at 2022-04-26. The target hardware is a **Rhino-brand educational arm** (the `Rhino*` class names and the "C string cmd for Rhino" comment).

The division of labor is legible in the comments: the parser skeleton, `circle` class, and motor-command demo read as **instructor-provided scaffolding** (first-person instructor voice: "I have converted to degrees for demonstration", "You may pick one", "see page 153"), while the marked student work is the **`Line` interpolator** ([Parse.h](Parse.h): `#include "line.h" //add your line code`) and the **differential-tick / shoulder-elbow coupling rework** of the motor layer worked out in class. [tserial](tserial.h) is vendored third-party Win32 serial code (header block dated April 8, 2013).

Textbook references carried forward from the original README: Craig, *Introduction to Robotics: Mechanics and Control*; Smid, *CNC Programming Handbook*; Siciliano, *Robotics: Modelling, Planning and Control*. This system is educational — expect hardware-specific modification before any production use.

See [SYSTEM-DESIGN.md](SYSTEM-DESIGN.md) for the architecture-level view: the full data-flow diagram, the ideas behind the design, and the numbers that matter.

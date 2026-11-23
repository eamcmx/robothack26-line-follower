# A line-follower robot, from one prompt

Prompts and checks from the master class *From a sentence to a machine* at RobotHack '26, Transport and
Telecommunication Institute, Riga, 24 November 2026. Prof. Emmanuel A. Merchán-Cruz.

A language model does not see Fusion and does not measure anything. It writes a program; Autodesk Fusion builds
what the program says; a check measures what Fusion built. The prompt decides most of what comes back. These are the
prompts we used, so you can try them yourself.

**Prompt checker:** https://eamcmx.github.io/robothack26-line-follower/ . Paste a prompt and see which of the five
parts it has and what it leaves the model to guess.

## What is here

| File | What it is |
|---|---|
| `prompts/LineFollower_full.txt` | The full line follower: chassis, N20 gear motors in steel brackets, wheels with set screws and treaded tyres, a screwed ball caster, a sensor bar on L-brackets, an Arduino Nano on a carrier and brass standoffs, an 18650 battery holder, every M3 screw, nut and clearance hole, and a test board with a black line |
| `prompts/LineFollower_basic.txt` | A first, simpler line follower (no fasteners): useful to compare |
| `checks/linefollower_full_check.py` | The check of the full prompt: a Fusion script that runs your script's `build()` and prints ok or FAIL for each acceptance line |
| `checks/linefollower_basic_check.py` | The check of the basic prompt |
| `results/` | The two scripts Claude wrote from these prompts on 23 November 2026, and what the checks printed |
| `docs/` | The prompt checker page |

## How to use them

1. Give Claude (or another model) a way to run scripts in Autodesk Fusion. Fusion has a local MCP server: Preferences >
   General > API > "Fusion MCP Server".
2. Give it one prompt file, whole. Let it write the script, run it, and read what Fusion says.
3. Run the check on what it wrote: open the check in Fusion's Scripts and Add-Ins, set `PATH` at its top to the script,
   run it, and read the ok and FAIL lines in the Text Commands window. Give the FAIL lines back to the model and let it
   correct its script. Repeat until it prints ALL PASS.
4. Then change one number in the prompt and do it again.

## What happened when we did it (23 November 2026)

- The basic prompt, Claude Sonnet: 540 lines; it ran the check 8 times and corrected what failed; 11 of 11 passed after
  27 min 55 s. Asked by the prompt's last line, it listed four decisions it had made on its own.
- The full prompt, Claude Opus: 622 lines; the first run of the check stopped on a material name the library does
  not have; corrected, the second run passed all 12 lines, 10 min 26 s after the prompt: 68 parts, 20 screws, 14 nuts,
  435 g.

Both scripts and their check reports are in `results/`, as the models wrote them.

Before trusting any of it, measure: the model can say "built" when parts sit inside each other, or "involute gears"
when there are no teeth at all. The check, run in Fusion, is the judge.

## The five parts of a prompt that works

From the book *Mechanical Design in Autodesk Fusion with Claude*:

1. **Inputs**: every part, every size, every material.
2. **Derived values**: what follows from what, worked out in the script, never typed.
3. **Hard constraints**: what must never happen, and what must be refused.
4. **Fusion conventions**: a new design, cm and radians, one named component per part, which joints.
5. **Acceptance**: what will be measured, each with a number and a tolerance.

And a last line: *tell me every decision you made that this prompt did not.*

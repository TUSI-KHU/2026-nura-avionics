Behavioral guidelines to reduce common LLM coding mistakes. Use when writing, reviewing, or refactoring code to avoid overcomplication, make surgical changes, surface assumptions, and define verifiable success criteria.
1. Think Before Coding
Don't assume. Don't hide confusion. Surface tradeoffs.
Before implementing:
    State your assumptions explicitly. If uncertain, ask.
    If multiple interpretations exist, present them - don't pick silently.
    If a simpler approach exists, say so. Push back when warranted.
    If something is unclear, stop. Name what's confusing. Ask.
2. Simplicity First

Minimum code that solves the problem. Nothing speculative.

    No features beyond what was asked.
    No abstractions for single-use code.
    No "flexibility" or "configurability" that wasn't requested.
    No error handling for impossible scenarios.
    If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.
3. Surgical Changes

Touch only what you must. Clean up only your own mess.

When editing existing code:

    Don't "improve" adjacent code, comments, or formatting.
    Don't refactor things that aren't broken.
    Match existing style, even if you'd do it differently.
    If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:

    Remove imports/variables/functions that YOUR changes made unused.
    Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.
4. Goal-Driven Execution

Define success criteria. Loop until verified.

Transform tasks into verifiable goals:

    "Add validation" → "Write tests for invalid inputs, then make them pass"
    "Fix the bug" → "Write a test that reproduces it, then make it pass"
    "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:

1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

5. Flight Logic Safety

Flight logic is safety-critical. Do not add or change mission, state-machine,
deployment, pyro, arming, abort, fallback, sensor-fusion, or threshold logic
just because it "seems reasonable."

Before implementing flight logic:

    Identify the exact state transition or safety decision being changed.
    Name the sensor inputs used by the decision.
    State the physical assumption behind the logic.
    State the source of every threshold: datasheet, OpenRocket/RASAero,
    measured flight log, ground test, team decision, or explicit TODO.
    If the source is unknown, do not invent a number silently. Leave a named
    placeholder, mark it unsafe for flight, or ask for the missing value.

When changing flight logic:

    Prefer conservative, observable conditions over clever but unverified
    prediction-only triggers.
    Keep prediction algorithms behind confidence checks and fallbacks.
    Never energize pyro directly from packet parsing, telemetry reception,
    debug commands, or raw state assignment.
    Never bypass arming, continuity, battery, sensor health, timeout, or
    state guards unless the user explicitly asks for a bench-only test path.
    Keep bench/test-only behavior behind clearly named build flags or test
    entrypoints so it cannot be mistaken for flight behavior.

The test: A reviewer should be able to answer "why is this safe enough to try?"
from the code, tests, and documents without guessing.

6. Documentation Required For Logic Changes

Any person or agent who adds or changes flight behavior must document the logic
in `documents/` in the same change.

This applies to:

    State-machine transitions.
    Launch, burnout, coast, apogee, drogue, main, landing, or fault detection.
    Pyro firing, inhibit, retry, or fallback policy.
    Sensor filtering, calibration, coordinate transforms, fusion, and health
    decisions used by flight logic.
    Telemetry or uplink commands that can affect flight state or recovery.
    Logging formats that are needed to validate flight decisions.

The document must include:

    Purpose of the logic.
    Inputs and units.
    State(s) where it is allowed to run.
    State(s) where it is forbidden.
    Thresholds and their sources.
    Failure modes considered.
    Fallback behavior.
    Verification plan: simulation, replay, bench test, hardware integration
    test, or flight-log comparison.

If the implementation is experimental, label it as experimental in the document
and in code comments or names. Experimental logic must not be enabled in flight
builds unless the document states the acceptance criteria and those criteria
have been met.

# plc-code transpiler limitations — RESOLVED

> **Status (2026-05-29): all limitations below are fixed upstream in
> [`siemens-plc-tools`](../../siemens-plc-tools) (branch `fix/transpiler-limitations`).**
> `ruckig-scl` now depends on `siemens-plc-tools/packages/plc-code` and no longer
> needs any of the workarounds. This file is kept as a historical record of the
> issues found while porting Ruckig to SCL and how they were resolved.

The SCL code itself was always 100% valid TIA Portal syntax — the limitations
only affected the **Python test bench**. They have now been fixed in the
transpiler, so the idiomatic SCL forms are used directly throughout `ruckig-scl`.

## What was fixed

| # | Symptom in the test bench | Root cause | Fix |
|---|---------------------------|-----------|-----|
| 1 | Inter-FC call `"IsFiniteLreal"(x := #v)` in an expression became a string literal with `:=`→`==` | named-block calls were only handled in statement position; FC return value not captured | `ExpressionTranslator` extracts the call as a protected placeholder; `call_named_block` returns the `FUNCTION` value |
| 2 | `"dbRuckigConst".RESULT_X` stayed a string / `int = 0` | parser inserts spaces around the dot (`"db" . X`); DB pattern didn't match, and the enum-string collector grabbed `< "db"` | DB pattern tolerates whitespace; collector ignores quoted names followed by `.`/`(`; constant DBs auto-load from search paths |
| 3 | `REGION Per-axis validation` leaked `- axis validation` as code | `_parse_region` stopped the name at the first non-identifier token | name now consumes the whole line |
| 5 | Multi-line RHS in a REGION dropped continuation lines | `_preprocess` only joined a continuation ending in `:=` | also joins operator-led continuations |
| 6 | `REGION Set 7 phase durations` leaked `7 phase durations` | same as #3 (digit) | same fix |
| 7 | Hex literal `16#8201` in code became `16 self.8201` | `#` parsed as instance-var prefix before hex translation | hex translation runs first and tolerates parser-inserted spaces |
| 9 | An identifier starting with `IF` (e.g. `"dbRuckigConst".IFACE_VELOCITY`) is mis-lexed — the lexer reads `IF` as the keyword, producing invalid Python | The DB-constant pattern splits on `.`, leaving the bare `IFACE_VELOCITY` token which starts with the keyword `IF` | Workaround: use the literal integer value (e.g. `1`) with an inline comment naming the constant (`// IFACE_VELOCITY`) |
| 10 | An inline comment on the same line as `END_IF;` (e.g. `END_IF; // closes ...`) silently drops the entire enclosing block | The control-flow depth counter matches `END_IF` only after `rstrip` of `; ` and spaces, not a trailing `//` comment, so the `END_IF` is not recognised and the block is discarded | Workaround: put the comment on its own line before or after `END_IF;`, never inline with it |

(Items 4 and 8 in the original notes were conventions, not bugs.)

## Idiomatic forms now used in ruckig-scl

- **Shared constants** live only in `dbRuckigConst` and are referenced as
  `"dbRuckigConst".RESULT_WORKING`, `"dbRuckigConst".EPS_POSITION`, etc.
  The test bench auto-loads the DB from `src/data-blocks` via the runtime's
  block search paths (see `tests/conftest.py`).
- **Finite checks** call the `IsFiniteLreal` sub-block:
  `IF NOT "IsFiniteLreal"(x := #input.currentPosition[#i]) THEN ...`.
- **REGION names** use their natural form (`Per-axis validation`,
  `Set 7 phase durations and jerks`).
- **Long expressions** (e.g. the boundary-state integration) span multiple lines.

## Test harness setup

`tests/conftest.py` exposes a `make_harness` factory with the standard search
paths (`src/blocks`, `src/data-types`, `src/data-blocks`) so inter-FC calls,
UDTs and the constant DB all resolve automatically.

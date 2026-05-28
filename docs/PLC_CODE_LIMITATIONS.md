# plc-code transpiler limitations (discovered during ruckig-scl v0.1)

This file documents limitations of the `plc-code` SCL-to-Python transpiler
(from `203-plc-tools/packages/plc-code`) discovered while implementing the
v0.1 SCL blocks of `ruckig-scl`. These are **transpiler bugs or unsupported
features**, not bugs in our SCL code. They affect what SCL constructs can
be tested via the `plc-code` executor harness.

The SCL code itself remains 100% valid TIA Portal syntax — the limitations
only apply to the **Python test bench**. When deployed to a real TIA Portal
project, the constructs we work around here can be re-introduced in their
ideal form.

## Limitation 1 — Inter-FC calls with named parameters

**Problem.** Calling another FC from inside a block, with the standard SCL
named-parameter syntax, transpiles incorrectly:

```scl
IF NOT "IsFiniteLreal"(x := #input.currentPosition[#i]) THEN
```

Becomes (in generated Python):

```python
if not "IsFiniteLreal" ( x == self.input.currentPosition[self.i] ):
```

Two errors at once:
- `"IsFiniteLreal"` is treated as a Python string literal (not a function
  call dispatched to the runtime)
- `:=` (SCL named-parameter assignment) is transpiled to `==` (Python
  comparison) instead of `=` (Python keyword argument)

**Workaround.** Inline the called FC's logic in the calling block. Yes, this
duplicates code — but it lets the test bench run. Move the duplication
back into a real call when deploying to TIA Portal.

## Limitation 2 — External DB references

**Problem.** References to a global DB like `"dbRuckigConst".RESULT_WORKING`
do not resolve. The transpiler treats `"dbRuckigConst"` as a string constant
(typed `int = 0` in the generated dataclass).

**Workaround.** Use **local** `VAR CONSTANT` blocks in each FC/FB that needs
the codes. Duplicates the constant definitions across blocks, but
self-contained — no external DB needed at test time. When deploying to TIA
Portal, the local constants can stay, or be replaced by DB references if
desired.

## Limitation 3 — REGION names with hyphens

**Problem.** A `REGION Per-axis validation` produces a transpiled comment
`# Per-axis validation`, but the transpiler splits this on the hyphen,
yielding two lines: `# Per` then `- axis validation`, which is invalid
Python (a bare `- name` expression).

**Workaround.** Use spaces or underscores in REGION names instead of
hyphens. `REGION Per axis validation` works, `REGION Per_axis_validation`
works.

## Limitation 4 — `harness.get_output("FunctionName")` for FCs

**Not a limitation, just an undocumented convention.** For an FC (typed
FUNCTION), the return value is exposed by the harness under the **function
name itself**:

```python
result = harness.get_output("ValidateInput")  # not "result" or anything else
```

## Pattern that works

```scl
{
    S7_Author := "Martin C";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.1.0"
}
FUNCTION "MyFunc" : Word
    VAR_IN_OUT
        someStruct : _.typeMyUdt;        // _. prefix for UDT references
    END_VAR
    VAR_TEMP
        i : DInt;
        tmp : LReal;
    END_VAR
    VAR CONSTANT
        // Inline all status codes locally instead of DB references
        OK_CODE  : Word := 16#7000;
        ERR_CODE : Word := 16#8201;
    END_VAR

    { S7_Language := "SCL" }
    NETWORK
        REGION Block header
            // ...
        END_REGION

        REGION Validation                                 // No hyphens in REGION names
            FOR #i := 0 TO #someStruct.count - 1 DO
                // Inline what would otherwise be a function call
                #tmp := #someStruct.values[#i];
                IF #tmp <> #tmp THEN                       // NaN check inlined
                    #MyFunc := #ERR_CODE;
                    RETURN;
                END_IF;
            END_FOR;
        END_REGION

        #MyFunc := #OK_CODE;
    END_NETWORK
END_FUNCTION
```

## Test harness setup with search paths

When a block references UDTs defined in `src/data-types/`, the harness
needs `block_search_paths` configured:

```python
from plc_code.executor import create_harness
from plc_code.executor.runtime import PLCRuntime

PROJECT_ROOT = Path(__file__).parent.parent.parent
SEARCH_PATHS = [
    PROJECT_ROOT / "src/blocks",
    PROJECT_ROOT / "src/data-blocks",
    PROJECT_ROOT / "src/data-types",
]

@pytest.fixture
def harness():
    rt = PLCRuntime(block_search_paths=SEARCH_PATHS)
    return create_harness(BLOCK_PATH, runtime=rt)
```

## Improvements to upstream

These limitations should be fixed in `plc-code` upstream (in `203-plc-tools/packages/plc-code`)
to avoid the workarounds here. Until then, follow the patterns documented
in this file.

Tracked improvement candidates:
- Inter-FC calls dispatched via `runtime.call_named_block()`
- External DB references resolved through `block_search_paths`
- REGION names containing hyphens or other operator characters

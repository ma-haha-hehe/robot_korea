# Explicit alternate designs

These seven products are new targets for the current primitive geometry and
19.2 mm snap layer pitch. They do **not** replace the original catalogue files,
and success on a variant must not be reported as success on its original.
Part IDs, types, colours and counts are retained.

`changes.json` records every target position and yaw change, before and after.

| Original | Design changes |
|---|---|
| `final_product`, `final_product2` | Put feet on the base, use contiguous layers and 16 mm spacing, rotate hips/torso to span the feet; hands and head share the upper supported row. |
| `final_product_burger` | Rotate the long base, separate the upper pair by 32 mm and remove the missing layer. |
| `final_product_door` | Use full layer spacing, rotate the middle pair and separate the red pair by 32 mm. |
| `final_product_flower` | Use 19.2 mm layers and 16 mm lateral spacing; rotate the two branches so the outer parts have support. |
| `final_product_simple` | Start at base height, use contiguous layers and rotate the base to support the blue pair. |
| `final_product_t` | Rotate long parts by 90 degrees so same-layer parts no longer overlap. |

All seven pass the nominal geometry audit. This alone is not an assembly result.
Execution evidence, when available, is recorded separately in the validation report.

```bash
source enter_sim_env.sh
export PYTHONPATH="$PWD/src/mj_bridge:$PYTHONPATH"
python scripts/test_product_suite.py \
  --products examples/products/supported_variants --seeds 42 \
  --jobs 2 --timeout 300 --output-dir runs/supported-variants
```

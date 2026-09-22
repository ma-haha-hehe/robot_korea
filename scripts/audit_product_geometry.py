#!/usr/bin/env python3
"""Report unsupported nominal targets without silently rewriting legacy products."""
import json
from pathlib import Path
from mj_bridge.benchmark_cli import normalized_from_path
from mj_bridge.product_geometry import audit_product


def audit(product):
    return audit_product(product)


if __name__ == '__main__':
    results = [audit(normalized_from_path(str(p)))
               for p in sorted(Path('examples/products/catalog').glob('*.yaml'))]
    print(json.dumps({'kind':'nominal_geometry_audit',
                      'products':len(results),
                      'valid_for_snap':sum(r['valid_for_snap'] for r in results),
                      'results':results}, indent=2))

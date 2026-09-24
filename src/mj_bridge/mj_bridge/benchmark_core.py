"""Pure-Python public benchmark core: products, seeded spawning and scoring."""
from __future__ import annotations

import json
import copy
import hashlib
import math
import random
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_REGISTRY = PACKAGE_DIR / "part_registry.yaml"
TABLE_TOP_Z = 0.04
ASSEMBLY_ORIGIN = (0.35, 0.35)
BRICK_COLLISION_CENTER_OFFSET_Z = -0.009
BASE_STUD_TOP_Z = 0.050
DEFAULT_SPAWN_REGION = {"x": [0.30, 0.68], "y": [-0.42, 0.12]}


class ProductError(ValueError):
    pass


def body_name(part_type: str, block_id: str) -> str:
    """Create an XML-safe name without collisions after character replacement."""
    safe = re.sub(r"[^A-Za-z0-9_-]", "_", block_id).strip("_") or "part"
    digest = hashlib.sha1(block_id.encode("utf-8")).hexdigest()[:8]
    return f"bench__{part_type}__{safe}__{digest}"


def load_yaml(path: str | Path) -> dict:
    with Path(path).open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ProductError(f"YAML root must be a mapping: {path}")
    return data


def load_registry(path: str | Path = DEFAULT_REGISTRY) -> dict:
    data = load_yaml(path)
    parts = data.get("parts")
    if not isinstance(parts, dict) or not parts:
        raise ProductError("part registry must contain a non-empty 'parts' mapping")
    return parts


def infer_type(block: dict) -> str:
    explicit = str(block.get("type", "")).strip()
    if explicit:
        return explicit
    text = str(block.get("name", block.get("id", ""))).lower().replace("×", "x")
    if "2x4" in text or "4x2" in text:
        return "brick_4x2"
    if "2x2" in text:
        return "brick_2x2"
    dims = block.get("dims")
    if isinstance(dims, list) and len(dims) == 3:
        xy = sorted(abs(float(v)) for v in dims[:2])
        if xy[1] >= 0.050 and xy[0] <= 0.040:
            return "brick_4x2"
        if xy[1] <= 0.040:
            return "brick_2x2"
    raise ProductError(f"cannot infer part type for block: {block.get('name', block.get('id'))}")


def _yaw_deg(rotation: Any) -> float:
    if rotation is None:
        return 0.0
    value = rotation[2] if isinstance(rotation, (list, tuple)) else rotation
    value = float(value)
    # Legacy files mix radians and degrees. Fractional values within 2π are radians;
    # integer values (for example 1 degree) remain degrees.
    is_fractional = not math.isclose(value, round(value), abs_tol=1e-9)
    return math.degrees(value) if 0 < abs(value) <= 2 * math.pi + 1e-6 and is_fractional else value


def normalize_product(data: dict, *, name: str = "product") -> dict:
    """Normalize the public schema and legacy blocks/tasks YAML into schema v1."""
    if "schema_version" in data and data["schema_version"] != 1:
        raise ProductError(f"unsupported schema_version: {data['schema_version']}")
    if data.get("schema_version") == 1 and isinstance(data.get("blocks"), list):
        blocks = []
        for raw in data["blocks"]:
            target = raw.get("target", {})
            blocks.append({
                "id": str(raw.get("id", raw.get("name", ""))).strip(),
                "type": infer_type(raw),
                "color": str(raw.get("color", "gray")),
                "target": {
                    "position": [float(x) for x in target.get("position", [])],
                    "yaw_deg": float(target.get("yaw_deg", 0.0)),
                },
            })
        product_name = str(data.get("product", {}).get("name", name))
    elif isinstance(data.get("blocks"), list):
        blocks = []
        used_ids: dict[str, int] = {}
        for index, raw in enumerate(data["blocks"]):
            base_id = str(raw.get("id", raw.get("name", f"block_{index}"))).strip()
            occurrence = used_ids.get(base_id, 0)
            used_ids[base_id] = occurrence + 1
            block_id = base_id if occurrence == 0 else f"{base_id}_{occurrence + 1}"
            blocks.append({
                "id": block_id,
                "type": infer_type(raw),
                "color": str(raw.get("color", "gray")),
                "target": {
                    "position": [float(x) for x in raw.get("pos", [0, 0, 0])],
                    "yaw_deg": _yaw_deg(raw.get("rotation", 0.0)),
                },
            })
        product_name = str(data.get("name", name))
    elif isinstance(data.get("tasks"), list) or isinstance(data.get("tasksh"), list):
        blocks = []
        # "tasksh" is supported because it appears in an early exported planner format.
        for index, raw in enumerate(data.get("tasks", data.get("tasksh"))):
            block_id = str(raw.get("id", raw.get("name", f"block_{index}"))).strip().replace(" ", "_")
            place = raw.get("place", {})
            blocks.append({
                "id": f"{block_id}_{index}",
                "type": infer_type(raw),
                "color": str(raw.get("color", str(raw.get("name", "gray")).split(" ")[0])),
                "target": {
                    "position": [float(x) for x in place.get("pos", [0, 0, 0])],
                    "yaw_deg": 0.0,
                },
            })
        product_name = str(data.get("name", name))
    else:
        raise ProductError("product YAML must contain a 'blocks' or legacy 'tasks' list")

    normalized = {"schema_version": 1, "product": {"name": product_name}, "blocks": blocks}
    if "initial_layout" in data:
        normalized["initial_layout"] = copy.deepcopy(data["initial_layout"])
    validate_product(normalized)
    from .recorded_layout import validate_layout
    validate_layout(normalized)
    return normalized


def validate_product(product: dict, registry: dict | None = None) -> None:
    registry = registry or load_registry()
    blocks = product.get("blocks")
    if not isinstance(blocks, list) or not blocks:
        raise ProductError("product requires at least one block")
    ids = set()
    for block in blocks:
        if not isinstance(block, dict):
            raise ProductError("every block must be a mapping")
        block_id = block.get("id")
        if not isinstance(block_id, str) or not block_id:
            raise ProductError("every block requires a non-empty id")
        if block_id in ids:
            raise ProductError(f"duplicate block id: {block_id}")
        ids.add(block_id)
        part_type = block.get("type")
        if part_type not in registry:
            raise ProductError(f"unsupported part type '{part_type}'; available: {', '.join(sorted(registry))}")
        pos = block.get("target", {}).get("position")
        if not isinstance(pos, list) or len(pos) != 3:
            raise ProductError(f"block {block_id} target.position must contain 3 numbers")
        values = pos + [block["target"].get("yaw_deg", 0.0)]
        if any(isinstance(v, bool) or not isinstance(v, (int, float))
               or not math.isfinite(v) for v in values):
            raise ProductError(f"block {block_id} position and yaw must be finite numbers")


def _aabb(size: list[float], x: float, y: float, yaw: float, margin: float):
    hx, hy = size[0] / 2, size[1] / 2
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    ex, ey = c * hx + s * hy + margin, s * hx + c * hy + margin
    return x - ex, x + ex, y - ey, y + ey


def _overlap(a, b) -> bool:
    return not (a[1] <= b[0] or b[1] <= a[0] or a[3] <= b[2] or b[3] <= a[2])


def plastic_fixture_offset(product):
    """Align the mounting plate to the product's lowest-layer stud grid.

    Product coordinates remain unchanged. The fixed fixture may translate by
    at most half a stud pitch on either axis before the episode is created.
    """
    pitch = .016
    bottom = min(b['target']['position'][2] for b in product['blocks'])
    base = [b for b in product['blocks'] if abs(b['target']['position'][2] - bottom) < 1e-5]
    first = base[0]['target']['position']
    offset = [round((float(value) + pitch/2) % pitch - pitch/2, 9) for value in first[:2]]
    for block in base:
        yaw = float(block['target'].get('yaw_deg', 0.))
        if abs((yaw + 45.) % 90. - 45.) > 1e-5:
            raise ProductError('plastic baseplate requires cardinal lowest-layer parts')
        for axis in (0, 1):
            delta = float(block['target']['position'][axis]) - offset[axis]
            if abs(delta - round(delta / pitch) * pitch) > 1e-5:
                raise ProductError('lowest-layer parts do not share one plastic stud grid')
    return offset


def generate_episode(product: dict, *, seed: int, registry: dict | None = None,
                     spawn_region: dict | None = None, margin_m: float = 0.018,
                     max_attempts: int = 5000, connection_mode: str = "snap",
                     contact_profile: str = "loose") -> dict:
    registry = registry or load_registry()
    if connection_mode not in {"snap", "physics"}:
        raise ProductError("unknown connection mode")
    if contact_profile not in {'loose', 'plastic'}:
        raise ProductError('unknown contact profile')
    if contact_profile == 'plastic' and connection_mode != 'physics':
        raise ProductError('plastic contact requires physics connection mode')
    from .plastic_contact import PlasticContact
    geometry = ('hollow_plastic_clutch_v2' if contact_profile == 'plastic' else
                'hollow_primitives_v1' if connection_mode == 'physics' else 'solid_primitives_v1')
    validate_product(product, registry)
    from .recorded_layout import validate_layout
    recorded = validate_layout(product, registry)
    region = spawn_region or DEFAULT_SPAWN_REGION
    rng = random.Random(int(seed))
    occupied = []
    spawned = []
    for block in product["blocks"]:
        spec = registry[block["type"]]
        size = [float(v) for v in spec["size_m"]]
        if connection_mode == "physics":
            size[2] = .0192
        if recorded is not None:
            pose = recorded[block['id']]
            spawned.append({
                'id': block['id'], 'type': block['type'], 'color': block['color'],
                'body_name': body_name(block['type'], block['id']),
                'geometry': geometry,
                'position': [*pose['position_xy_m'], TABLE_TOP_Z + size[2] / 2 - BRICK_COLLISION_CENTER_OFFSET_Z],
                'yaw_rad': math.radians(pose['yaw_deg']),
            })
            continue
        for _ in range(max_attempts):
            yaw = rng.uniform(-math.pi, math.pi)
            # Sample conservatively away from table edges; exact rotated AABB is checked below.
            x = rng.uniform(float(region["x"][0]), float(region["x"][1]))
            y = rng.uniform(float(region["y"][0]), float(region["y"][1]))
            bounds = _aabb(size, x, y, yaw, margin_m)
            inside = (bounds[0] >= region["x"][0] and bounds[1] <= region["x"][1]
                      and bounds[2] >= region["y"][0] and bounds[3] <= region["y"][1])
            if inside and not any(_overlap(bounds, old) for old in occupied):
                occupied.append(bounds)
                spawned.append({
                    "id": block["id"], "type": block["type"], "color": block["color"],
                    "body_name": body_name(block["type"], block["id"]),
                    "geometry": geometry,
                    "position": [x, y, TABLE_TOP_Z + size[2] / 2 - BRICK_COLLISION_CENTER_OFFSET_Z],
                    "yaw_rad": yaw,
                })
                break
        else:
            raise ProductError(f"cannot place {len(product['blocks'])} parts in spawn region with margin {margin_m}")

    fixture_offset = plastic_fixture_offset(product) if contact_profile == 'plastic' else [0., 0.]
    targets = []
    for block in product["blocks"]:
        p = block["target"]["position"]
        spec = registry[block["type"]]
        targets.append({
            "id": block["id"], "type": block["type"], "color": block["color"],
            "body_name": body_name(block["type"], block["id"]),
            "position": [ASSEMBLY_ORIGIN[0] + p[0], ASSEMBLY_ORIGIN[1] + p[1],
                         (.046 if connection_mode == "physics" else BASE_STUD_TOP_Z)
                         + (.0192 if connection_mode == "physics" else float(spec["size_m"][2])) / 2
                         - BRICK_COLLISION_CENTER_OFFSET_Z + p[2]],
            "yaw_rad": math.radians(float(block["target"].get("yaw_deg", 0.0))),
        })
    return {
        "schema_version": 1,
        "connection_mode": connection_mode,
        "contact_profile": contact_profile,
        "contact_parameters": (vars(PlasticContact())
            if contact_profile == "plastic" else None),
        "geometry": geometry,
        "assembly_fixture": {"position_offset_xy_m": fixture_offset,
                             "reason": "align lowest-layer stud grid" if contact_profile == 'plastic' else "default"},
        "episode_id": f"{product['product']['name']}-seed-{int(seed)}",
        "initial_layout_mode": "recorded" if recorded is not None else "seeded",
        "seed": int(seed), "product": product, "spawn_region": region,
        "spawned_blocks": spawned, "target_blocks": targets,
    }


def angular_error(actual: float, target: float, symmetry_deg: float) -> float:
    period = math.radians(float(symmetry_deg))
    return abs((actual - target + period / 2) % period - period / 2)


def score_episode(manifest: dict, actual: dict, registry: dict | None = None,
                  xy_tol=0.006, z_tol=0.004, yaw_tol_deg=8.0) -> dict:
    registry = registry or load_registry()
    if not manifest.get("target_blocks"):
        raise ProductError("cannot score an episode without target blocks")
    per_block, success = [], True
    actual_blocks = actual.get("blocks", actual)
    for target in manifest["target_blocks"]:
        observed = actual_blocks.get(target["id"])
        if observed is None:
            item = {"id": target["id"], "success": False, "reason": "missing"}
        else:
            dx = float(observed["position"][0]) - target["position"][0]
            dy = float(observed["position"][1]) - target["position"][1]
            dz = float(observed["position"][2]) - target["position"][2]
            xy = math.hypot(dx, dy)
            yaw_error = angular_error(float(observed.get("yaw_rad", 0.0)), target["yaw_rad"],
                                      registry[target["type"]]["yaw_symmetry_deg"])
            upright = True
            if "quaternion_wxyz" in observed:
                q = observed["quaternion_wxyz"]
                norm = sum(float(v) ** 2 for v in q)
                upright = (len(q) == 4 and math.isfinite(norm) and norm > 0
                           and 1 - 2 * (q[1] ** 2 + q[2] ** 2) / norm >= math.cos(math.radians(8)))
            ok = (xy <= xy_tol and abs(dz) <= z_tol
                  and yaw_error <= math.radians(yaw_tol_deg) and upright)
            item = {"id": target["id"], "success": ok, "xy_error_m": xy,
                    "z_error_m": abs(dz), "yaw_error_deg": math.degrees(yaw_error),
                    "upright": upright}
        success = success and item["success"]
        per_block.append(item)
    return {"success": success, "placed": sum(x["success"] for x in per_block),
            "total": len(per_block), "completion": sum(x["success"] for x in per_block) / len(per_block),
            "blocks": per_block}


def dump_yaml(path: str | Path, data: dict) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)


def dump_json(path: str | Path, data: dict) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")

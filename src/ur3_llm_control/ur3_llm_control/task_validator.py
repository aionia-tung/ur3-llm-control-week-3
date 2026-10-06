from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Set

VALID_OBJECTS = {"red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube"}
VALID_ZONES = {"zone_a", "zone_b", "zone_c", "temporary_position"}
VALID_SKILLS = {"check_zone", "pick", "place", "home"}


@dataclass
class ValidationResult:
    valid: bool
    plan: List[Dict[str, str]]
    error: str = ""


class PlanValidator:
    def __init__(self, objects: Optional[Set[str]] = None, zones: Optional[Set[str]] = None):
        self.objects = set(objects or VALID_OBJECTS)
        self.zones = set(zones or VALID_ZONES)

    def validate(self, payload: Any) -> ValidationResult:
        if not isinstance(payload, dict) or not isinstance(payload.get("plan"), list):
            return ValidationResult(False, [], "Expected JSON object with a plan array")

        result = []
        gripper_holding: Optional[str] = None  # Theo dõi trạng thái gripper trong chuỗi hành động

        for i, step in enumerate(payload["plan"], 1):
            if not isinstance(step, dict):
                return ValidationResult(False, [], f"Step {i} must be an object")

            skill = step.get("skill")
            if skill not in VALID_SKILLS:
                return ValidationResult(False, [], f"Step {i}: invalid skill {skill!r}")

            if skill == "home":
                if set(step) != {"skill"}:
                    return ValidationResult(False, [], f"Step {i}: home takes no arguments")
                result.append({"skill": "home"})

            elif skill == "check_zone":
                zone = step.get("zone")
                if zone not in self.zones:
                    return ValidationResult(False, [], f"Step {i}: invalid zone {zone!r}")
                if set(step) != {"skill", "zone"}:
                    return ValidationResult(False, [], f"Step {i}: check_zone has unexpected arguments")
                result.append({"skill": "check_zone", "zone": zone})

            elif skill == "pick":
                if gripper_holding is not None:
                    return ValidationResult(
                        False, [], f"Step {i}: cannot pick {step.get('object')} while already holding {gripper_holding}"
                    )
                obj = step.get("object")
                if obj not in self.objects:
                    return ValidationResult(False, [], f"Step {i}: invalid object {obj!r}")
                if set(step) != {"skill", "object"}:
                    return ValidationResult(False, [], f"Step {i}: pick has unexpected arguments")
                gripper_holding = obj
                result.append({"skill": "pick", "object": obj})

            elif skill == "place":
                obj, zone = step.get("object"), step.get("zone")
                if obj not in self.objects:
                    return ValidationResult(False, [], f"Step {i}: invalid object {obj!r}")
                if zone not in self.zones:
                    return ValidationResult(False, [], f"Step {i}: invalid zone {zone!r}")
                if gripper_holding != obj:
                    return ValidationResult(
                        False, [], f"Step {i}: cannot place {obj} because robot is holding {gripper_holding!r}"
                    )
                if set(step) != {"skill", "object", "zone"}:
                    return ValidationResult(False, [], f"Step {i}: place has unexpected arguments")
                gripper_holding = None
                result.append({"skill": "place", "object": obj, "zone": zone})

        if not result:
            return ValidationResult(False, [], "Plan is empty")

        if result[-1]["skill"] != "home":
            return ValidationResult(False, [], "Plan must end with skill 'home'")

        return ValidationResult(True, result)
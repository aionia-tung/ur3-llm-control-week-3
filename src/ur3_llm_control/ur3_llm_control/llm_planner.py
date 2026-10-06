import json
import os
import re
import time
import urllib.error
import urllib.request

SYSTEM_PROMPT = """Translate the user's request into a robot execution plan based on the camera world state. Return JSON only:
{"plan":[
  {"skill":"check_zone","zone":"zone_b"},
  {"skill":"pick","object":"blue_cube"},
  {"skill":"place","object":"blue_cube","zone":"temporary_position"},
  {"skill":"pick","object":"red_cube"},
  {"skill":"place","object":"red_cube","zone":"zone_b"},
  {"skill":"home"}
]}

Allowed skills:
- check_zone(zone)
- pick(object)
- place(object, zone)
- home()

Allowed objects: red_cube, yellow_cube, blue_cube, green_cube, purple_cube.
Allowed zones / targets: zone_a, zone_b, zone_c, temporary_position.

CRITICAL RULES:
1. Examine the provided "Zone Occupancy" from the camera state.
2. If the user wants to place an object X into a zone that is ALREADY OCCUPIED by another object Y:
- First emit: check_zone(target_zone)
- Then emit: pick(Y)
- Then emit: place(Y, "temporary_position")
- Then proceed with the main task: pick(X) -> place(X, target_zone)
3. If the target zone is EMPTY/FREE (null/None):
- You may emit pick(X) -> place(X, target_zone) directly.
4. Emit home exactly once at the end of the entire plan.
5. Never emit poses, joint values, trajectories, explanations, or any markdown outside JSON."""


class PlannerError(RuntimeError):
    pass


class LLMPlanner:
    def __init__(self, endpoint="https://9router.com/v1/chat/completions",
                 model="gpt-4o-mini", api_key="", timeout=90.0,
                 student_id="23020770"):
        self.endpoint = endpoint
        self.model = model
        self.api_key = api_key or os.environ.get("NINEROUTER_API_KEY", "")
        self.timeout = timeout
        mappings = (
            ("red_cube", "yellow_cube", "blue_cube"),
            ("red_cube", "blue_cube", "yellow_cube"),
            ("yellow_cube", "red_cube", "blue_cube"),
            ("yellow_cube", "blue_cube", "red_cube"),
            ("blue_cube", "red_cube", "yellow_cube"),
            ("blue_cube", "yellow_cube", "red_cube"),
        )
        digits = "".join(ch for ch in str(student_id) if ch.isdigit())
        p = int(digits[-2:]) % 6 if len(digits) >= 2 else 0
        self.student_id = str(student_id)
        self.zone_mapping = dict(zip(("zone_a", "zone_b", "zone_c"), mappings[p]))

    @staticmethod
    def _extract_content(raw, content_type):
        """Read either a regular chat-completion JSON body or an SSE stream."""
        if content_type == "text/event-stream" or raw.lstrip().startswith("data:"):
            chunks = []
            for line in raw.splitlines():
                line = line.strip()
                if not line.startswith("data:"):
                    continue
                event_data = line[5:].strip()
                if not event_data or event_data == "[DONE]":
                    continue
                event = json.loads(event_data)
                choices = event.get("choices", [])
                if not choices:
                    if "error" in event:
                        raise ValueError(f"9Router stream error: {event['error']}")
                    continue
                choice = choices[0]
                delta = choice.get("delta", {})
                chunk = delta.get("content")
                if chunk is None:
                    chunk = choice.get("message", {}).get("content")
                if isinstance(chunk, str):
                    chunks.append(chunk)
            content = "".join(chunks).strip()
            if not content:
                raise ValueError("9Router SSE response contained no assistant text")
            return content

        data = json.loads(raw)
        content = data["choices"][0]["message"]["content"]
        if not isinstance(content, str) or not content.strip():
            raise ValueError("9Router response contained no assistant text")
        return content.strip()

    def create_plan(self, command, world_state=None):
        if not command.strip():
            raise PlannerError("Command is empty")
        if not self.api_key:
            raise PlannerError("Set NINEROUTER_API_KEY or pass api_key")

        state_info = ""
        if world_state:
            occupancy = world_state.get("zone_occupancy", {})
            state_info = f"\nCurrent Camera Zone Occupancy: {json.dumps(occupancy)}"

        user_content = f"Command: {command}{state_info}"

        body = {
            "model": self.model,
            "temperature": 0,
            "response_format": {"type": "json_object"},
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT +
                 f"\nStudent ID: {self.student_id}; zone mapping for arrange-all: " +
                 json.dumps(self.zone_mapping)},
                {"role": "user", "content": user_content},
            ],
        }

        max_retries = 3
        last_error = None

        for attempt in range(max_retries):
            request = urllib.request.Request(
                self.endpoint,
                data=json.dumps(body).encode("utf-8"),
                headers={"Authorization": f"Bearer {self.api_key}",
                         "Content-Type": "application/json"},
                method="POST",
            )
            try:
                with urllib.request.urlopen(request, timeout=self.timeout) as response:
                    raw = response.read().decode("utf-8")
                    content_type = response.headers.get_content_type()
                content = self._extract_content(raw, content_type)
                fenced = re.search(r"```(?:json)?\s*(.*?)\s*```", content, re.S | re.I)
                payload = json.loads(fenced.group(1) if fenced else content)
                if not isinstance(payload, dict):
                    raise ValueError("JSON root is not an object")
                return payload
            except urllib.error.HTTPError as err:
                err_body = err.read().decode("utf-8", errors="ignore")
                last_error = f"HTTP {err.code}: {err_body}"
                if err.code == 429:
                    time.sleep(4)
                    continue
                break
            except (TimeoutError, urllib.error.URLError) as exc:
                last_error = f"Timeout/URLError: {exc}"
                time.sleep(3)
                continue
            except (KeyError, IndexError, TypeError, ValueError, json.JSONDecodeError) as exc:
                last_error = f"Parse error: {exc}"
                break

        raise PlannerError(f"9Router request/response failed: {last_error}")

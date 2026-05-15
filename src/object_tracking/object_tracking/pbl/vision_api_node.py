#!/usr/bin/env python3
# coding=utf-8

"""ROS 2 service node that delegates difficult PBL vision tasks to Gemini."""

import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from tracking_msg.srv import VisionAnalyze

from object_tracking.pbl.vision_api_client import (
    GeminiVisionClient,
    api_key_from_environment,
    parse_json_response,
)
from object_tracking.pbl.vision_api_prompts import prompt_for_task


class PBLVisionAPINode(Node):
    """Gemini-backed JSON vision analyzer for PBL inspection images."""

    def __init__(self):
        super().__init__("pbl_vision_api")

        self.declare_parameter("gemini_model", "gemini-3-flash-preview")
        self.declare_parameter(
            "api_key_env_json",
            '["GEMINI_API_KEY", "GOOGLE_API_KEY"]',
        )
        self.declare_parameter("api_timeout_sec", 8.0)
        self.declare_parameter("max_api_calls", 4)
        self.declare_parameter("save_dir", "/tmp/pbl_inspection")
        self.declare_parameter("save_raw", True)
        self.declare_parameter("mock_mode", False)

        self.call_count = 0
        self.save_dir = Path(self.get_parameter("save_dir").value)
        self.request_dir = self.save_dir / "api_requests"
        self.response_dir = self.save_dir / "api_responses"
        self.request_dir.mkdir(parents=True, exist_ok=True)
        self.response_dir.mkdir(parents=True, exist_ok=True)

        self.mock_mode = bool(self.get_parameter("mock_mode").value)
        self.client = None
        if not self.mock_mode:
            self.client = self.create_gemini_client()
        else:
            self.get_logger().warn("Gemini API mock_mode is enabled.")

        self.service = self.create_service(
            VisionAnalyze,
            "/pbl_vision_api/analyze",
            self.handle_analyze,
        )
        self.get_logger().info("PBL Gemini Vision API node is ready.")

    def create_gemini_client(self):
        """Create the Gemini client from environment variables."""
        try:
            env_names = json.loads(self.get_parameter("api_key_env_json").value)
        except json.JSONDecodeError:
            env_names = ["GEMINI_API_KEY", "GOOGLE_API_KEY"]

        api_key = api_key_from_environment(env_names)
        try:
            return GeminiVisionClient(
                api_key=api_key,
                model=self.get_parameter("gemini_model").value,
                timeout_sec=float(self.get_parameter("api_timeout_sec").value),
            )
        except Exception as exc:
            self.get_logger().error(f"Gemini client is unavailable: {exc}")
            return None

    def handle_analyze(self, request, response):
        """Handle one JSON image-analysis request."""
        started = time.time()
        try:
            payload = json.loads(request.request_json)
        except json.JSONDecodeError as exc:
            response.response_json = json.dumps({
                "ok": False,
                "source": "vision_api",
                "reason": f"invalid_request_json:{exc}",
            })
            return response

        task = str(payload.get("task", "")).lower()
        image_path = payload.get("image_path", "")
        local_result = payload.get("local_result", {})

        max_calls = int(self.get_parameter("max_api_calls").value)
        if self.call_count >= max_calls:
            response.response_json = json.dumps({
                "ok": False,
                "task": task,
                "source": "vision_api",
                "reason": "api_call_limit_reached",
                "api_calls_used": self.call_count,
                "api_call_limit": max_calls,
            })
            return response

        if not image_path or not Path(image_path).exists():
            response.response_json = json.dumps({
                "ok": False,
                "task": task,
                "source": "vision_api",
                "reason": "image_path_not_found",
                "image_path": image_path,
            })
            return response

        prompt = prompt_for_task(task, local_result=local_result)
        request_path = self.save_request(task, image_path, prompt, payload)

        self.call_count += 1
        if self.mock_mode:
            parsed = self.mock_response(task)
            raw_text = json.dumps(parsed, ensure_ascii=False)
            latency = time.time() - started
        elif self.client is None:
            parsed = {
                "ok": False,
                "task": task,
                "source": "vision_api",
                "reason": "gemini_client_unavailable",
            }
            raw_text = json.dumps(parsed)
            latency = time.time() - started
        else:
            try:
                raw_text, latency = self.client.analyze_image(prompt, image_path)
                parsed = parse_json_response(raw_text)
            except Exception as exc:
                raw_text = str(exc)
                latency = time.time() - started
                parsed = {
                    "ok": False,
                    "task": task,
                    "source": "vision_api",
                    "reason": f"gemini_request_failed:{exc}",
                }

        result = self.normalize_result(task, parsed)
        result.update({
            "task": task,
            "source": "vision_api",
            "api_latency_sec": round(float(latency), 3),
            "api_calls_used": self.call_count,
            "api_call_limit": max_calls,
            "api_request_path": str(request_path),
        })
        result["raw_response_path"] = str(
            self.save_response(task, raw_text, result)
        )

        response.response_json = json.dumps(result, ensure_ascii=False)
        return response

    def save_request(self, task, image_path, prompt, payload):
        """Persist one API request for debugging."""
        path = self.request_dir / f"{int(time.time() * 1000)}_{task}.json"
        data = {
            "task": task,
            "image_path": image_path,
            "prompt": prompt,
            "payload": payload,
        }
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        return path

    def save_response(self, task, raw_text, parsed):
        """Persist raw and parsed API response for debugging."""
        path = self.response_dir / f"{int(time.time() * 1000)}_{task}.json"
        data = {
            "task": task,
            "raw_text": raw_text,
            "parsed": parsed,
        }
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        return path

    def normalize_result(self, task, parsed):
        """Clamp Gemini output to the schemas used by the mission node."""
        if not isinstance(parsed, dict):
            return {"ok": False, "reason": "api_response_not_object"}

        if task == "water_level":
            cups = parsed.get("cups", [])
            normalized_cups = []
            for index, cup in enumerate(cups, start=1):
                level = self.quantize_level(cup.get("water_level_percent"))
                if level is None:
                    continue
                normalized_cups.append({
                    "index": int(cup.get("index", index)),
                    "water_level_percent": level,
                })
            return {
                "ok": len(normalized_cups) > 0,
                "cup_count": int(parsed.get("cup_count", len(normalized_cups))),
                "water_level_percent": [
                    cup["water_level_percent"] for cup in normalized_cups
                ],
                "cups": normalized_cups,
            }

        if task == "multimeter":
            count = int(parsed.get("multimeter_count", 0))
            return {"ok": count > 0, "multimeter_count": count}

        if task == "tower_light":
            color = str(parsed.get("light_color", "")).lower()
            ok = color in {"red", "yellow", "green"}
            return {"ok": ok, "light_color": color if ok else "unknown"}

        if task == "baseball":
            count = int(parsed.get("baseball_count", 0))
            return {"ok": count >= 1, "baseball_count": count, "color": "orange"}

        parsed.setdefault("ok", False)
        return parsed

    def quantize_level(self, value):
        """Return one of the allowed water levels."""
        try:
            percent = int(round(float(value) / 20.0) * 20)
        except (TypeError, ValueError):
            return None
        return int(max(20, min(100, percent)))

    def mock_response(self, task):
        """Return predictable responses for integration testing without API usage."""
        if task == "water_level":
            return {
                "ok": True,
                "cup_count": 1,
                "cups": [{"index": 1, "water_level_percent": 60}],
            }
        if task == "multimeter":
            return {"ok": True, "multimeter_count": 1}
        if task == "tower_light":
            return {"ok": True, "light_color": "red"}
        if task == "baseball":
            return {"ok": True, "baseball_count": 1, "color": "orange"}
        return {"ok": False, "reason": "unknown_task"}


def main():
    """Run the Gemini Vision API service node."""
    rclpy.init()
    node = PBLVisionAPINode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

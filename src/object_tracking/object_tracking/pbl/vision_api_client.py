"""Gemini Vision API client used by the PBL API ROS node."""

import json
import mimetypes
import os
import re
import time
from pathlib import Path

try:
    from google import genai
    from google.genai import types
except ImportError:  # pragma: no cover - depends on robot environment
    genai = None
    types = None


class GeminiVisionClient:
    """Small wrapper around Gemini image analysis calls."""

    def __init__(self, api_key, model, timeout_sec=8.0):
        if genai is None or types is None:
            raise RuntimeError(
                "google-genai is not installed. Run: python3 -m pip install google-genai"
            )
        if not api_key:
            raise RuntimeError(
                "Gemini API key is missing. Set GEMINI_API_KEY or GOOGLE_API_KEY."
            )

        self.client = genai.Client(api_key=api_key)
        self.model = model
        self.timeout_sec = float(timeout_sec)

    def analyze_image(self, prompt, image_path):
        """Send one image and prompt to Gemini, returning raw text and latency."""
        image_path = Path(image_path)
        mime_type = mimetypes.guess_type(str(image_path))[0] or "image/jpeg"
        start_time = time.time()

        with image_path.open("rb") as image_file:
            image_part = types.Part.from_bytes(
                data=image_file.read(),
                mime_type=mime_type,
            )

        response = self.client.models.generate_content(
            model=self.model,
            contents=[prompt, image_part],
            config=types.GenerateContentConfig(
                temperature=0.1,
                response_mime_type="application/json",
            ),
        )
        return response.text, time.time() - start_time


def api_key_from_environment(env_names):
    """Return the first configured API key from a list of environment variables."""
    for env_name in env_names:
        value = os.environ.get(env_name)
        if value:
            return value
    return ""


def clean_json_string(text):
    """Remove common Markdown fences from model output."""
    text = re.sub(r"```json", "", text or "")
    text = re.sub(r"```", "", text)
    return text.strip()


def parse_json_response(text):
    """Parse a Gemini JSON response, including simple fenced-output recovery."""
    cleaned = clean_json_string(text)
    try:
        return json.loads(cleaned)
    except json.JSONDecodeError:
        match = re.search(r"\{.*\}", cleaned, flags=re.DOTALL)
        if not match:
            raise
        return json.loads(match.group(0))


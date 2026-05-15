"""Prompt templates for PBL Gemini vision inspection tasks."""

TASK_PROMPTS = {
    "water_level": """
You are analyzing an AMR inspection photo.
Task: count all visible cups and estimate each cup's water level.

Allowed water levels are only: 20, 40, 60, 80, 100.
Choose the closest allowed level if uncertain.

Return JSON only:
{
  "ok": true,
  "cup_count": number,
  "cups": [
    {"index": number, "water_level_percent": 20}
  ]
}
""".strip(),
    "multimeter": """
You are analyzing an AMR inspection photo.
Task: count only digital multimeters.
Do not count remote controls, phones, calculators, or other similar rectangular devices.

Return JSON only:
{
  "ok": true,
  "multimeter_count": number
}
""".strip(),
    "tower_light": """
You are analyzing an AMR inspection photo.
Task: identify which tower light color is currently on.
Allowed colors are only: red, yellow, green.

Return JSON only:
{
  "ok": true,
  "light_color": "red"
}
""".strip(),
    "baseball": """
You are analyzing an AMR inspection photo.
Task: find the orange baseball.
Return whether one orange baseball is visible.

Return JSON only:
{
  "ok": true,
  "baseball_count": 1,
  "color": "orange"
}
""".strip(),
}


def prompt_for_task(task, local_result=None):
    """Return the Gemini prompt for a task with optional local detector context."""
    prompt = TASK_PROMPTS.get(task)
    if prompt is None:
        prompt = """
You are analyzing an AMR inspection photo.
Return concise JSON only for the requested inspection task.
""".strip()

    if local_result:
        prompt += (
            "\n\nA local detector produced this preliminary JSON. "
            "Use it only as a hint, and correct it if the image disagrees:\n"
            f"{local_result}"
        )

    return prompt


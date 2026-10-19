#!/usr/bin/env python3

import re
from dataclasses import dataclass
from pathlib import Path

from ocs2_msgs.msg import ModeSchedule


MODE_NAME_TO_NUMBER = {
    "FLY": 0,
    "RH": 1,
    "LH": 2,
    "LH_RH": 3,
    "RF": 4,
    "RF_RH": 5,
    "RF_LH": 6,
    "RF_LH_RH": 7,
    "LF": 8,
    "LF_RH": 9,
    "LF_LH": 10,
    "LF_LH_RH": 11,
    "LF_RF": 12,
    "LF_RF_RH": 13,
    "LF_RF_LH": 14,
    "STANCE": 15,
}


@dataclass(frozen=True)
class GaitTemplate:
    name: str
    switching_times: list[float]
    mode_sequence: list[int]


class GaitCommandFile:
    def __init__(self, path: str):
        self.path = Path(path)
        if not self.path.is_file():
            raise FileNotFoundError(f"gait command file does not exist: {self.path}")
        self._text = self._strip_comments(self.path.read_text())
        self.gait_names = self._parse_list()
        self.templates = {name: self._parse_template(name) for name in self.gait_names}

    def create_message(self, gait_name: str) -> ModeSchedule:
        key = gait_name.strip().lower()
        if key not in self.templates:
            raise KeyError(f"unknown gait '{gait_name}'")
        template = self.templates[key]
        msg = ModeSchedule()
        msg.event_times = list(template.switching_times)
        msg.mode_sequence = list(template.mode_sequence)
        return msg

    @staticmethod
    def _strip_comments(text: str) -> str:
        return "\n".join(line.split(";", 1)[0] for line in text.splitlines())

    def _parse_list(self) -> list[str]:
        block = self._extract_block("list")
        names = []
        for _, value in self._indexed_values(block):
            names.append(value.lower())
        if not names:
            raise ValueError(f"no gait list entries found in {self.path}")
        return names

    def _parse_template(self, name: str) -> GaitTemplate:
        block = self._extract_block(name)
        mode_block = self._extract_block("modeSequence", block)
        time_block = self._extract_block("switchingTimes", block)

        modes = []
        for _, value in self._indexed_values(mode_block):
            mode_name = value.upper()
            if mode_name not in MODE_NAME_TO_NUMBER:
                raise ValueError(f"unknown mode '{value}' in gait '{name}'")
            modes.append(MODE_NAME_TO_NUMBER[mode_name])

        switching_times = [float(value) for _, value in self._indexed_values(time_block)]
        if len(switching_times) != len(modes) + 1:
            raise ValueError(
                f"gait '{name}' has {len(modes)} modes but {len(switching_times)} switching times"
            )
        return GaitTemplate(name=name, switching_times=switching_times, mode_sequence=modes)

    def _extract_block(self, name: str, text: str | None = None) -> str:
        source = self._text if text is None else text
        match = re.search(rf"(^|\n)\s*{re.escape(name)}\s*\{{", source)
        if match is None:
            raise ValueError(f"block '{name}' not found in {self.path}")

        open_brace = source.find("{", match.start())
        depth = 0
        for index in range(open_brace, len(source)):
            char = source[index]
            if char == "{":
                depth += 1
            elif char == "}":
                depth -= 1
                if depth == 0:
                    return source[open_brace + 1:index]
        raise ValueError(f"block '{name}' is missing a closing brace in {self.path}")

    @staticmethod
    def _indexed_values(block: str) -> list[tuple[int, str]]:
        values = []
        for match in re.finditer(r"\[(\d+)\]\s+([A-Za-z0-9_+.\-]+)", block):
            values.append((int(match.group(1)), match.group(2)))
        return sorted(values, key=lambda item: item[0])

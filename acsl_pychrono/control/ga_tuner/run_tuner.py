"""Entry point for running GA tuning."""

from __future__ import annotations

import os
import sys
from pathlib import Path

from acsl_pychrono.control.ga_tuner.ga_config.config import (
    DEFAULT_TUNING_CONFIG,
    TuningConfig,
)
from acsl_pychrono.control.ga_tuner.runner import run_mrac_ga, summarize_result

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
os.chdir(PROJECT_ROOT)


def run_from_config(config: TuningConfig) -> None:
    result, tuned_names, all_names, evaluator_type = run_mrac_ga(config)
    summarize_result(result, tuned_names, all_names, evaluator_type)


def main() -> None:
    config = DEFAULT_TUNING_CONFIG
    run_from_config(config)


if __name__ == "__main__":
    main()

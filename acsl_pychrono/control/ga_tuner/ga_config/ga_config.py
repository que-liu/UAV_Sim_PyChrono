from dataclasses import dataclass
from typing import Any, Dict, Optional


@dataclass
class GAConfig:
    """Central configuration for GA tuning runs."""

    controller_type: str = "MRAC"  # 'PID' or 'MRAC'
    algorithm: str = "PYMOO"  # 'DEAP' or 'PYMOO'

    population_size: int = 5
    num_generations: int = 2
    crossover_rate: float = 0.8
    mutation_rate: float = 0.1

    selection_method: str = "tournament"
    tournament_size: int = 5
    random_seed: Optional[int] = None
    multi_objective: bool = True

    # Pymoo-specific parameters
    pymoo_variant: str = "NSGA2"
    pymoo_algorithm_params: Optional[Dict[str, Any]] = None

    def normalized_controller_type(self) -> str:
        return self.controller_type.upper()

    def normalized_algorithm(self) -> str:
        return self.algorithm.upper()

    def __post_init__(self):
        self.algorithm = self.algorithm.upper()
        self.controller_type = self.controller_type.upper()


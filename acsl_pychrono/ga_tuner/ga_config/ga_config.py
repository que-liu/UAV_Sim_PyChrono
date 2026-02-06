from dataclasses import dataclass, field
from typing import Any, Dict, Optional


# Custom reference vector for iterative tuning. Set to None to use default gains from mrac_gains.py
# NOTE: This must match the number of parameters selected in param_config.py
_CUSTOM_REFERENCE_VECTOR = None


@dataclass
class GAConfig:
    """Central configuration for GA tuning runs."""

    controller_type: str = "PID"  # 'PID', 'MRAC'
    algorithm: str = "PYMOO"  # 'DEAP' or 'PYMOO'

    population_size: int = 91
    num_generations: int = 10
    crossover_rate: float = 0.6
    mutation_rate: float = 0.05

    selection_method: str = "tournament"
    tournament_size: int = 5
    random_seed: Optional[int] = None
    verbose: bool = True  # Print generation progress during optimization

    # Search space configuration
    # - 'local': Relative bounds around reference (±10× scaling) - for fine-tuning existing gains
    # - 'global': Absolute bounds for exploring diverse parameter regions
    search_space_type: str = "local"  # Changed to 'local' for refinement around global search results
    
    # Custom reference for iterative tuning (use tuned parameters as new reference point for fine-tuning)
    # Set to None to use default gains from mrac_gains.py, set to a list to use custom reference (see _CUSTOM_REFERENCE_VECTOR above)
    custom_reference_vector: Optional[list] = None  # Set to _CUSTOM_REFERENCE_VECTOR to use it

    # Pymoo-specific parameters
    pymoo_variant: str = "NSGA3"
    # For NSGA3 with many objectives: manually set partitions to reduce reference directions
    pymoo_algorithm_params: Optional[Dict[str, Any]] = None
    
    def __post_init__(self):
        self.algorithm = self.algorithm.upper()
        # Set default n_partitions for NSGA3 if not specified
        if self.pymoo_variant == "NSGA3" and self.pymoo_algorithm_params is None:
            self.pymoo_algorithm_params = {"n_partitions": 3}  # Reduces ref_dirs from 6188 to ~84


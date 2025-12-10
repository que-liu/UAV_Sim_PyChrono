import numpy as np

# Default fallback MRAC gains
_FALLBACK_MRAC_DEFAULTS = {
    'Gamma_x_tran': np.diag([1e1, 1e1, 1e2, 1e1, 1e1, 1e2]),
    'Gamma_r_tran': np.diag([3e-2, 3e-2, 12e-2]),
    'Gamma_Theta_tran': np.diag([1e1, 1e1, 2e1, 1e1, 1e1, 2e1]),
    'Gamma_x_rot': np.diag([1e4, 1e4, 1e4]),
    'Gamma_r_rot': np.diag([5e0, 5e0, 5e0]),
    'Gamma_Theta_rot': np.diag([2e3, 2e3, 2e3, 2e3, 2e3, 2e3]),
    'Q_tran': np.diag([6e-2, 6e-2, 72e-2, 6e-2, 6e-2, 12e-2]),  # 6e-2 * [1,1,12,1,1,2]
    'K_P_omega_ref': np.diag([3.8e1 * 0.8, 3.8e1 * 0.8, 3.8e1 * 1.2]),
    'K_I_omega_ref': np.diag([5e-1, 5e-1, 1e-1]),
    'sigma_x_tran': 0.5,
    'sigma_r_tran': 0.5,
    'sigma_Theta_tran': 0.5,
    'sigma_x_rot': 0.5,
    'sigma_r_rot': 0.5,
    'sigma_Theta_rot': 0.5,
    'dead_zone_delta_tran': 0.5,
    'dead_zone_e0_tran': 0.01,
    'dead_zone_delta_rot': 0.5,
    'dead_zone_e0_rot': 0.002,
}
# Configuration for Gamma matrices, dynamically inferring sizes from fallback gains
GAMMA_MATRIX_CONFIGS = {
    name: {
        # keep the 'Gamma_' -> 'gamma_' prefix but preserve the remainder's original case
        'prefix': name.replace('Gamma_', 'gamma_').lower() if name.startswith('Gamma_') else name.lower(),
        'size': _FALLBACK_MRAC_DEFAULTS[name].shape[0] if isinstance(_FALLBACK_MRAC_DEFAULTS[name], np.ndarray) else None
    }
    for name in _FALLBACK_MRAC_DEFAULTS
    if name.startswith('Gamma_') or name == 'Q_tran'
}

# Mapping from diagonal matrix names in MRAC gains to individual scalar parameter names
DIAGONAL_MATRIX_PARAMS = {
    'K_P_omega_ref': ('K_P_omega_ref_1', 'K_P_omega_ref_2', 'K_P_omega_ref_3'),
    'K_I_omega_ref': ('K_I_omega_ref_1', 'K_I_omega_ref_2', 'K_I_omega_ref_3'),
    'KP_tran': ('KP_tran_1', 'KP_tran_2', 'KP_tran_3'),
    'KD_tran': ('KD_tran_1', 'KD_tran_2', 'KD_tran_3'),
    'KI_tran': ('KI_tran_1', 'KI_tran_2', 'KI_tran_3'),
    'KP_rot': ('KP_rot_1', 'KP_rot_2', 'KP_rot_3'),
}

# Groups of scalar parameters that are handled together
SCALAR_PARAMETER_GROUPS = [
    (
        'sigma_params',
        (
            'sigma_x_tran', 'sigma_r_tran', 'sigma_Theta_tran',
            'sigma_x_rot', 'sigma_r_rot', 'sigma_Theta_rot'
        )
    ),
    (
        'dead_zone_params',
        (
            'dead_zone_delta_tran', 'dead_zone_e0_tran',
            'dead_zone_delta_rot', 'dead_zone_e0_rot'
        )
    )
]

# Configuration for target names used in MRAC parameter handling

target_names = {
    'Gamma_x_tran', 'Gamma_r_tran', 'Gamma_Theta_tran',
    'Gamma_x_rot', 'Gamma_r_rot', 'Gamma_Theta_rot',
    'Q_tran',
    'K_P_omega_ref', 'K_I_omega_ref',
    'sigma_x_tran', 'sigma_r_tran', 'sigma_Theta_tran',
    'sigma_x_rot', 'sigma_r_rot', 'sigma_Theta_rot',
    'dead_zone_delta_tran', 'dead_zone_e0_tran',
    'dead_zone_delta_rot', 'dead_zone_e0_rot'
}
import math
import numpy as np  

class M_NonAdaptiveEBCI:
  @staticmethod
  def computeErrorBoundingControlInput(
    xi_bar_d: float,
    lambda_bar: float,
    delta_ebci: float,
    B: np.ndarray,
    P: np.ndarray,
    e: np.ndarray,
    use_ebci: bool
    ) -> np.ndarray:
    """
    Compute the error bounding control input
    """    
    m = B.shape[1]

    if not use_ebci:
      return np.zeros((m, 1))

    BPe = B.T * P * e
    BPe_norm = np.linalg.norm(BPe)

    if BPe_norm < delta_ebci:
      control_input = np.zeros((m, 1))
    else:
      sum_Pe = np.sum(abs(P * e))
      control_input = - (xi_bar_d / lambda_bar) * (BPe / BPe_norm) * sum_Pe

    return control_input


  
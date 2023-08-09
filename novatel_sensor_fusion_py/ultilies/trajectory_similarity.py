from scipy import interpolate
import numpy as np


def align_trajectories(t_a, t_b, trajectory_b):
    """
    Trajectory a is the reference.
    Interpolate trajectory b to align with the timestamps of trajectory a.
    """
    interp_b = interpolate.interp1d(t_b, trajectory_b, kind='linear', fill_value='extrapolate')
    b_aligned = interp_b(t_a)
    return b_aligned


def compute_rmse(trajectory_a, b_aligned):
    """
    Compute the Root Mean Square Error between two trajectories.
    """
    return np.sqrt(np.mean((trajectory_a - b_aligned)**2))


def compute_rmse_between_trajectories(t_a, trajectory_a, t_b, trajectory_b):
    return compute_rmse(
        trajectory_a, align_trajectories(
            t_a=t_a, t_b=t_b, trajectory_b=trajectory_b
        )
    )
import pickle
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import plotly.express as px

from run_parallel_pid_deap import generatePIDConfig
from acsl_pychrono.simulation.simulation import Simulation
from acsl_pychrono.executor.simulate_mission import simulateMission
from acsl_pychrono.control.logging import Logging
import os

def load_pareto_front(filename='multi_obj_pareto_front.pkl'):
    """Load Pareto front solutions from pickle file."""
    try:
        with open(filename, 'rb') as f:
            pareto_solutions = pickle.load(f)
        return pareto_solutions
    except FileNotFoundError:
        print(f"Error: {filename} not found. Please run multi_objective_pid_tuning.py first.")
        return None
    except Exception as e:
        print(f"Error loading {filename}: {e}")
        return None

def plot_pareto_front_3d(pareto_solutions):
    """Create 3D scatter plot of Pareto front."""
    if not pareto_solutions:
        print("No Pareto solutions to plot.")
        return
    
    # Extract cost values
    j_positions = [sol['fitness'][0] for sol in pareto_solutions]
    j_velocities = [sol['fitness'][1] for sol in pareto_solutions]
    j_thrusts = [sol['fitness'][2] for sol in pareto_solutions]
    
    # Create 3D scatter plot
    fig = go.Figure(data=[go.Scatter3d(
        x=j_positions,
        y=j_velocities,
        z=j_thrusts,
        mode='markers+text',
        marker=dict(
            size=8,
            color=j_positions,  # Color by position cost
            colorscale='Viridis',
            opacity=0.8
        ),
        text=[f'Sol {i}' for i in range(len(pareto_solutions))],
        textposition="middle center",
        hovertemplate='<b>Solution %{text}</b><br>' +
                      'J_position: %{x:.4f}<br>' +
                      'J_velocity: %{y:.4f}<br>' +
                      'J_thrust: %{z:.4f}<br>' +
                      '<extra></extra>'
    )])
    
    # Update layout
    fig.update_layout(
        title='Pareto Front: Multi-Objective PID Optimization',
        scene=dict(
            xaxis_title='J_position (Position Error)',
            yaxis_title='J_velocity (Velocity Error)',
            zaxis_title='J_thrust (Thrust Cost)',
            camera=dict(
                eye=dict(x=1.5, y=1.5, z=1.5)
            )
        ),
        width=800,
        height=600,
        showlegend=False
    )
    
    return fig

def plot_pareto_front_2d_projections(pareto_solutions):
    """Create 2D projections of the Pareto front."""
    if not pareto_solutions:
        print("No Pareto solutions to plot.")
        return
    
    # Extract cost values
    j_positions = [sol['fitness'][0] for sol in pareto_solutions]
    j_velocities = [sol['fitness'][1] for sol in pareto_solutions]
    j_thrusts = [sol['fitness'][2] for sol in pareto_solutions]
    
    # Create subplots for 2D projections
    fig = make_subplots(
        rows=1, cols=3,
        subplot_titles=('Position vs Velocity', 'Position vs Thrust', 'Velocity vs Thrust'),
        specs=[[{"secondary_y": False}, {"secondary_y": False}, {"secondary_y": False}]]
    )
    
    # Position vs Velocity
    fig.add_trace(
        go.Scatter(
            x=j_positions, y=j_velocities,
            mode='markers+text',
            name='Pareto Solutions',
            text=[f'Sol {i}' for i in range(len(pareto_solutions))],
            textposition="top center",
            marker=dict(size=10, color='blue', opacity=0.7),
            hovertemplate='<b>Solution %{text}</b><br>' +
                          'J_position: %{x:.4f}<br>' +
                          'J_velocity: %{y:.4f}<br>' +
                          '<extra></extra>'
        ),
        row=1, col=1
    )
    
    # Position vs Thrust
    fig.add_trace(
        go.Scatter(
            x=j_positions, y=j_thrusts,
            mode='markers+text',
            name='Pareto Solutions',
            text=[f'Sol {i}' for i in range(len(pareto_solutions))],
            textposition="top center",
            marker=dict(size=10, color='red', opacity=0.7),
            hovertemplate='<b>Solution %{text}</b><br>' +
                          'J_position: %{x:.4f}<br>' +
                          'J_thrust: %{y:.4f}<br>' +
                          '<extra></extra>',
            showlegend=False
        ),
        row=1, col=2
    )
    
    # Velocity vs Thrust
    fig.add_trace(
        go.Scatter(
            x=j_velocities, y=j_thrusts,
            mode='markers+text',
            name='Pareto Solutions',
            text=[f'Sol {i}' for i in range(len(pareto_solutions))],
            textposition="top center",
            marker=dict(size=10, color='green', opacity=0.7),
            hovertemplate='<b>Solution %{text}</b><br>' +
                          'J_velocity: %{x:.4f}<br>' +
                          'J_thrust: %{y:.4f}<br>' +
                          '<extra></extra>',
            showlegend=False
        ),
        row=1, col=3
    )
    
    # Update layout
    fig.update_layout(
        title='Pareto Front: 2D Projections',
        width=1200,
        height=400,
        showlegend=False
    )
    
    # Update axes labels
    fig.update_xaxes(title_text="J_position", row=1, col=1)
    fig.update_yaxes(title_text="J_velocity", row=1, col=1)
    fig.update_xaxes(title_text="J_position", row=1, col=2)
    fig.update_yaxes(title_text="J_thrust", row=1, col=2)
    fig.update_xaxes(title_text="J_velocity", row=1, col=3)
    fig.update_yaxes(title_text="J_thrust", row=1, col=3)
    
    return fig

def print_pareto_summary(pareto_solutions):
    """Print a summary of the Pareto front solutions."""
    if not pareto_solutions:
        print("No Pareto solutions to summarize.")
        return
    
    print("\n" + "="*80)
    print("PARETO FRONT SUMMARY")
    print("="*80)
    print(f"Number of Pareto optimal solutions: {len(pareto_solutions)}")
    print("\nDetailed Solutions:")
    print("-"*80)
    
    for i, solution in enumerate(pareto_solutions):
        gains = solution['gains']
        fitness = solution['fitness']
        print(f"Solution {i}:")
        print(f"  Gains: {gains}")
        print(f"  J_position: {fitness[0]:.6f}")
        print(f"  J_velocity: {fitness[1]:.6f}")
        print(f"  J_thrust: {fitness[2]:.6f}")
        print()
    
    # Find extreme solutions
    j_positions = [sol['fitness'][0] for sol in pareto_solutions]
    j_velocities = [sol['fitness'][1] for sol in pareto_solutions]
    j_thrusts = [sol['fitness'][2] for sol in pareto_solutions]
    
    best_position_idx = np.argmin(j_positions)
    best_velocity_idx = np.argmin(j_velocities)
    best_thrust_idx = np.argmin(j_thrusts)
    
    print("Extreme Solutions:")
    print("-"*80)
    print(f"Best Position Tracking (Solution {best_position_idx}): J_pos={j_positions[best_position_idx]:.6f}")
    print(f"Best Velocity Tracking (Solution {best_velocity_idx}): J_vel={j_velocities[best_velocity_idx]:.6f}")
    print(f"Most Energy Efficient (Solution {best_thrust_idx}): J_thrust={j_thrusts[best_thrust_idx]:.6f}")
    print("="*80)

    extreme_indices = [best_position_idx, best_velocity_idx, best_thrust_idx]
    # Simulate and visualize each one
    for i in extreme_indices:
        solution = pareto_solutions[i]
        gains = solution["gains"]
        print(f"\n=== Running Simulation for Extreme Solution {i} ===")
        print(f"Gains: {gains}")
        print(f"J_position: {solution['fitness'][0]}, J_velocity: {solution['fitness'][1]}, J_thrust: {solution['fitness'][2]}")

        log_dir = "simulation_logs/extreme_solutions"
        os.makedirs(log_dir, exist_ok=True)

        # Generate config
        sim_cfg = generatePIDConfig(list(gains), log_dir)
        sim = Simulation(sim_cfg)
        sim_cfg.mission_config.visualization_flag = True
        sim_cfg.mission_config.wrapper_flag = False
        simulateMission(sim, Logging.getGitRepoInfo())

def main():
    """Main function to load and visualize Pareto front."""
    print("Loading Pareto front solutions...")
    pareto_solutions = load_pareto_front()
    
    if pareto_solutions is None:
        return
    
    print(f"Loaded {len(pareto_solutions)} Pareto optimal solutions.")
    
    # Print summary
    print_pareto_summary(pareto_solutions)
    
    # Create 3D plot
    print("\nCreating 3D Pareto front visualization...")
    fig_3d = plot_pareto_front_3d(pareto_solutions)
    if fig_3d:
        fig_3d.write_html("pareto_front_3d.html")
        print("3D plot saved to pareto_front_3d.html")
    
    # Create 2D projections
    print("Creating 2D projection plots...")
    fig_2d = plot_pareto_front_2d_projections(pareto_solutions)
    if fig_2d:
        fig_2d.write_html("pareto_front_2d_projections.html")
        print("2D projections saved to pareto_front_2d_projections.html")
    
    print("\nVisualization complete!")
    print("Open the HTML files in your browser to view the interactive plots.")

if __name__ == "__main__":
    main() 
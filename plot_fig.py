import pickle
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# Load the log
# with open('/Users/queliu/miniconda3/envs/pychrono/simulation_logs/log_dict.pkl', 'rb') as f:
with open('simulation_logs/log_dict.pkl', 'rb') as f:
    log_dict = pickle.load(f)

gains = None
# Try to get gains from the log if present
if 'gains' in log_dict:
    gains = log_dict['gains']
else:
    # Try to load from best_pid_gains.pkl if available
    try:
        with open('best_pid_gains.pkl', 'rb') as f:
            gains = pickle.load(f)
    except Exception:
        gains = None

if gains is not None:
    with open('best_gains.txt', 'w') as f:
        f.write('Best PID Gains:\n')
        f.write(str(gains) + '\n')

# Extract time
time = np.array(log_dict['time']).flatten()

# Axes to process
axes = ['x', 'y', 'z']
labels = {'x': 'X', 'y': 'Y', 'z': 'Z'}

# Store RMSEs
rmse_pos = {}
rmse_vel = {}

# Create subplot grid: 3 rows per axis, 1 col
fig = make_subplots(
    rows=9, cols=1,
    shared_xaxes=True,
    vertical_spacing=0.02,
    subplot_titles=[
        f"{labels[ax]}-Position Tracking" for ax in axes
    ] + [
        f"{labels[ax]}-Velocity Tracking" for ax in axes
    ] + [
        f"{labels[ax]}-Velocity Tracking Error" for ax in axes
    ]
)

row = 1
for ax in axes:
    # Load position and velocity data
    pos_des = np.array(log_dict['user_defined_position'][ax]).flatten()
    pos_act = np.array(log_dict['position'][ax]).flatten()
    vel_des = np.array(log_dict['user_defined_velocity'][ax]).flatten()
    vel_act = np.array(log_dict['velocity'][ax]).flatten()

    # Error
    pos_err = pos_des - pos_act
    vel_err = vel_des - vel_act

    # RMSE
    rmse_pos[ax] = np.sqrt(np.mean(pos_err ** 2))
    rmse_vel[ax] = np.sqrt(np.mean(vel_err ** 2))

    # --- Position Tracking ---
    fig.add_trace(go.Scatter(x=time, y=pos_des, mode='lines', name=f'Desired {labels[ax]}', line=dict(dash='dash')), row=row, col=1)
    fig.add_trace(go.Scatter(x=time, y=pos_act, mode='lines', name=f'Actual {labels[ax]}'), row=row, col=1)

    # --- Velocity Tracking ---
    fig.add_trace(go.Scatter(x=time, y=vel_des, mode='lines', name=f'Desired {labels[ax]} Velocity', line=dict(dash='dash')), row=row+3, col=1)
    fig.add_trace(go.Scatter(x=time, y=vel_act, mode='lines', name=f'Actual {labels[ax]} Velocity'), row=row+3, col=1)

    # --- Velocity Error ---
    fig.add_trace(go.Scatter(x=time, y=vel_err, mode='lines', name=f'{labels[ax]} Velocity Error', line=dict(color='red')), row=row+6, col=1)
    fig.add_trace(go.Scatter(x=time, y=np.zeros_like(time), mode='lines', name='Zero Line', line=dict(dash='dot', color='black')), row=row+6, col=1)

    row += 1

# Update layout
fig.update_layout(
    height=1600,
    width=1000,
    title_text=(
        f"3D Axis Tracking with RMSEs — "
        f"Pos: X={rmse_pos['x']:.3f}, Y={rmse_pos['y']:.3f}, Z={rmse_pos['z']:.3f} m | "
        f"Vel: X={rmse_vel['x']:.3f}, Y={rmse_vel['y']:.3f}, Z={rmse_vel['z']:.3f} m/s"
    ),
    hovermode='x unified'
)

# Axis labels
for i, ax in enumerate(axes):
    fig.update_yaxes(title_text=f"{labels[ax]} Position [m]", row=i+1, col=1)
    fig.update_yaxes(title_text=f"{labels[ax]} Velocity [m/s]", row=i+4, col=1)
    fig.update_yaxes(title_text=f"{labels[ax]} Velocity Error [m/s]", row=i+7, col=1)

# Final x-axis
fig.update_xaxes(title_text="Time [s]", row=9, col=1)

# Format gains for filename
if gains is not None:
    # If gains is a list or tuple, flatten and round
    if isinstance(gains, (list, tuple, np.ndarray)):
        gains_str = "_".join([f"{float(g):.3f}" for g in gains])
    else:
        # If gains is a DEAP Individual or other object, convert to list first
        try:
            gains_str = "_".join([f"{float(g):.3f}" for g in list(gains)])
        except Exception:
            gains_str = "best"
    html_filename = f"best_pid_{gains_str}.html"
else:
    html_filename = "best_pid.html"

fig.write_html(html_filename)
print(f"Plot saved to {html_filename}")

# Pure Pursuit vs Stanley Controller: Lane Keeping in CARLA

**A Comparative Study of Lateral Control Algorithms for Autonomous Lane Keeping**

Final Project for EE267 - Autonomous Vehicles

---

## 📋 Project Overview

This project implements and compares two widely-used lateral control algorithms for autonomous vehicles:
- **Pure Pursuit Controller**: Geometric path tracking using lookahead distance
- **Stanley Controller**: Cross-track error and heading error-based control

Both controllers are evaluated on lane-keeping tasks in the CARLA simulator, with comprehensive performance analysis across multiple hyperparameter configurations.

---

## 🎯 Project Objectives

1. **Implementation**: Develop working Pure Pursuit and Stanley controllers
2. **Planning**: Use CARLA's built-in waypoint generation for path planning
3. **Control**: Apply controllers to lane-keeping tasks
4. **Evaluation**: Compare performance using multiple metrics
5. **Ablation Studies**: Analyze impact of hyperparameter variations
6. **Analysis**: Generate insights and design recommendations

---

## 🗂️ Project Structure

```
.
├── pure_pursuit.py          # Pure Pursuit controller implementation
├── stanley.py               # Stanley controller implementation
├── experiment_runner.py     # Main experiment orchestration script
├── evaluate_results.py      # Analysis and plotting script
├── README.md                # This file
├── requirements.txt         # Python dependencies
├── results/                 # Experiment results (JSON files)
│   ├── pure_pursuit_ld2.0.json
│   ├── pure_pursuit_ld3.0.json
│   ├── pure_pursuit_ld5.0.json
│   ├── stanley_k0.5.json
│   ├── stanley_k1.0.json
│   ├── stanley_k2.0.json
│   └── all_experiments.json
└── plots/                   # Generated visualizations
    ├── lateral_error_comparison.png
    ├── heading_error_comparison.png
    ├── steering_smoothness.png
    ├── summary_comparison.png
    └── summary_table.png
```

---

## 🚀 Getting Started

### Prerequisites

1. **CARLA Simulator** (version 0.9.13 or later)
   - Download from: https://github.com/carla-simulator/carla/releases
   - Extract to a directory (e.g., `~/CARLA_0.9.13`)

2. **Python 3.7+**

3. **Required Python packages**:
   ```bash
   pip install numpy matplotlib
   ```

### Installation

1. **Clone or download this project**:
   ```bash
   git clone <your-repo-url>
   cd lane-keeping-controllers
   ```

2. **Verify CARLA is accessible**:
   - Make sure CARLA's Python API is in your Python path
   - The `experiment_runner.py` script will attempt to find it automatically

---

## 🎮 Running the Experiments

### Step 1: Start CARLA Simulator

Open a terminal and start the CARLA server:

```bash
cd ~/CARLA_0.9.13  # Or your CARLA installation directory
./CarlaUE4.sh      # On Linux
# or
CarlaUE4.exe       # On Windows
```

Wait for CARLA to fully load (you'll see the town environment).

### Step 2: Run Experiments

In a separate terminal, run the experiment script:

```bash
python experiment_runner.py
```

This will:
- Load Town01 in CARLA
- Generate waypoints for the road network
- Spawn a Tesla Model 3 vehicle
- Run 6 experiments (3 Pure Pursuit + 3 Stanley configurations)
- Save results to `results/` directory

**Expected Runtime**: ~6-8 minutes for all experiments (60 seconds each)

### Step 3: Generate Analysis and Plots

After experiments complete, run the evaluation script:

```bash
python evaluate_results.py
```

This will generate:
- Performance comparison plots
- Summary statistics tables
- Detailed visualizations in `plots/` directory

---

## 📊 Experiments Conducted

### Pure Pursuit Ablation Study

| Experiment | Lookahead Distance (Ld) | Expected Behavior |
|------------|------------------------|-------------------|
| PP-Small   | 2.0 m | Tight tracking, potential oscillations |
| PP-Medium  | 3.0 m | Balanced performance |
| PP-Large   | 5.0 m | Smooth but may cut corners |

### Stanley Controller Ablation Study

| Experiment | Gain (K) | Expected Behavior |
|------------|----------|-------------------|
| Stanley-Low | 0.5 | Slow error correction |
| Stanley-Med | 1.0 | Balanced response |
| Stanley-High | 2.0 | Aggressive steering, possible oscillations |

---

## 📈 Evaluation Metrics

The following metrics are collected for each experiment:

1. **Lateral Error** (Cross-track error)
   - Distance from vehicle to nearest waypoint
   - Lower is better

2. **Heading Error**
   - Difference between vehicle heading and path heading
   - Lower is better

3. **Steering Smoothness**
   - Standard deviation of steering angle changes
   - Lower indicates smoother control

4. **Speed Profile**
   - Vehicle speed over time
   - Should be stable and consistent

5. **Completion Metrics**
   - Total distance traveled
   - Success on sharp curves

---

## 🔬 Key Findings (Expected)

### Pure Pursuit
- ✅ **Strengths**: Simple, stable, smooth steering
- ❌ **Weaknesses**: Cuts corners at high speeds, lookahead tuning critical
- 🎯 **Best Config**: Ld = 3.0 m for balanced performance

### Stanley
- ✅ **Strengths**: Excellent lane centering, no corner cutting
- ❌ **Weaknesses**: Can oscillate at high speeds, more sensitive to noise
- 🎯 **Best Config**: K = 1.0 for balanced responsiveness

### Overall Comparison
- Pure Pursuit is better for **smooth, comfortable driving**
- Stanley is better for **precise lane centering**
- Hybrid approaches may combine strengths of both

---

## 🛠️ Customization

### Modify Experiment Parameters

Edit `experiment_runner.py`:

```python
# Change experiment duration
metrics = runner.run_experiment(controller, name, duration=120.0)  # 2 minutes

# Change target speed
controller.target_speed = 50.0  # km/h

# Try different towns
runner.setup_world(town='Town02')

# Adjust waypoint density
runner.generate_waypoints(distance=1.0)  # More dense waypoints
```

### Add New Controllers

1. Create a new controller file (e.g., `pid_controller.py`)
2. Implement the `run_step(vehicle, waypoints)` method
3. Add to `experiment_runner.py`:

```python
from pid_controller import PIDController

controller = PIDController(kp=1.0, ki=0.1, kd=0.5)
metrics = runner.run_experiment(controller, "PID_Controller", duration=60.0)
```

---

## 📝 Report Components

Your final report should include:

1. **Introduction**
   - Problem statement
   - Controller background

2. **Methodology**
   - Controller algorithms (with equations)
   - Experimental setup
   - Evaluation metrics

3. **Results**
   - Quantitative comparisons (tables)
   - Visualizations (plots from `plots/` directory)
   - Ablation study findings

4. **Discussion**
   - Design insights
   - Tradeoffs between controllers
   - Recommendations for different scenarios

5. **Conclusion**
   - Summary of findings
   - Future work

---

## 🐛 Troubleshooting

### CARLA Connection Error
```
Error: Connection refused
```
**Solution**: Make sure CARLA is running before executing the experiment script.

### Import Error: No module named 'carla'
```
ModuleNotFoundError: No module named 'carla'
```
**Solution**: 
- Make sure CARLA's PythonAPI is in your path
- Or manually add it:
  ```python
  sys.path.append('/path/to/CARLA/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg')
  ```

### Vehicle Doesn't Move
**Potential Issues**:
- Check that synchronous mode is enabled
- Verify waypoints are generated correctly
- Ensure vehicle is spawned at a valid location

### Poor Performance
**Try**:
- Reduce simulation quality in CARLA settings
- Lower the fixed_delta_seconds value
- Run on a dedicated GPU

---

## 📚 References

1. **Pure Pursuit**:
   - Coulter, R. C. (1992). "Implementation of the Pure Pursuit Path Tracking Algorithm"
   
2. **Stanley Controller**:
   - Hoffmann, G. M., et al. (2007). "Autonomous Automobile Trajectory Tracking for Off-Road Driving: Controller Design, Experimental Validation and Racing"

3. **CARLA Simulator**:
   - Dosovitskiy, A., et al. (2017). "CARLA: An Open Urban Driving Simulator"

---

## 👥 Author

[Satyadev Gangineni & Nagarjun HR]
EE267 Final Project
[12-7-2025]

---

## 📄 License

This project is for educational purposes as part of the EE267 course.

---

## 🙏 Acknowledgments

- CARLA development team for the simulator
- Course instructors Prof.Hang Qiu and TA Shilpa Mukhopadhyay
- Pure Pursuit and Stanley algorithm original authors

---

**Good luck with your project! 🚗💨**

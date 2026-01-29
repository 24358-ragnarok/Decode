# Data Collection and Visualization

This directory stores scripts and data used for recording and analyzing projectile tests during the
FTC season.  
Collected data is used to train models, demonstrate relationships, and produce visualizations for
judges and team strategy.

## Environment Setup

Run all commands from inside this directory:

```bash
cd TeamCode/src/main/java/org/firstinspires/ftc/teamcode/data
python3 -m venv .venv
```

Activate virtual environment

macOS/Linux:

```source .venv/bin/activate```

Windows (PowerShell):

```.venv\Scripts\Activate.ps1```

Upgrade pip and setuptools
```pip install --upgrade pip setuptools wheel```

Install dependencies
```pip install -r requirements.txt```

Usage

Start the collector to capture packets from the robot and save them into the database:

```python3 trajectory_collectory.py```

Visualize the dataset in 3D:

```python3 trajectory_visualizer.py --show```

Database

All recorded trajectories are stored in trajectories.json (TinyDB format).
This file is tracked in Git to keep a shared dataset across the team.
Everyone contributes to the same pool of test results, making visualizations and analysis consistent
regardless of who collected the data.
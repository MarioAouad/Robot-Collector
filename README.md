# Pathfinding & TSP Collector Visualizer

A robust interactive visualization tool built in Python that simulates a robot navigating a grid to collect targets ("Green" and "Red" nodes). The project demonstrates the implementation of low-level pathfinding algorithms (A*, UCS) integrated with high-level route optimization (Greedy TSP).

## 🚀 Features

* **Pathfinding Algorithms:**
    * **A* (A-Star):** Heuristic-based search for optimal pathing.
    * **UCS (Uniform Cost Search):** Explores equally in all directions (Dijkstra variant).
* **Heuristics:** Includes Manhattan, Euclidean, and Chebyshev distance calculations.
* **Route Optimization:** Solves the traversal order using a Greedy TSP (Traveling Salesperson Problem) approach to visit all targets efficiently.
* **Real-Time Visualization:**
    * Live rendering of the "Frontier" (yellow) and "Explored" (light blue) sets.
    * Dynamic target spawning (Red nodes appear over time).
    * Visual trail tracking.
* **Interactive UI:** Built with **Tkinter**, featuring controls for grid size, speed, pausing/resuming, and algorithm selection.

## 🛠️ Project Structure

* `run.py`: The entry point of the application.
* `app.py`: Handles the GUI logic, simulation loop, and event handling (Tkinter).
* `lowlevel.py`: Contains the core pathfinding algorithms (A* and UCS).
* `tsp_opt.py`: Calculates pairwise distances and determines the optimal visit order.
* `heuristics.py`: Mathematical heuristic functions for grid traversal.
* `grid.py`: Data structure defining the grid environment and collision logic.

## 📦 Installation & Usage

This project relies on the Python standard library, so no external `pip` installations are required.

### Prerequisites
* Python 3.8+
* Tkinter (Included with standard Python installs on Windows/macOS. On Linux, you may need `sudo apt-get install python3-tk`).

### Running the App
1. Clone the repository:
   ```bash
   git clone [https://github.com/MarioAouad/Robot-Collector.git](https://github.com/MarioAouad/Robot-Collector.git)
   cd REPO-NAME
Run the application:

Bash
python run.py
🎮 How to Use
Select Settings: Choose your grid size (10, 20, or 50) and Algorithm (A* or UCS).

Run Optimal: Click "Run Optimal" to let the AI calculate the most efficient path to collect all current targets.

Manual Control: You can pause, resume, or restart the simulation at any time.

Stats: Click "Stats" to view performance metrics like time elapsed and nodes expanded.

🧠 Technical Details
The AI operates on two levels:

High-Level Planning: It calculates the distance matrix between all targets and uses a Greedy Nearest Neighbor approach to determine the visit order.

Low-Level Execution: It executes the path between specific targets using A* to avoid obstacles and minimize movement cost.

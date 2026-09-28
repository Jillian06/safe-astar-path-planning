# Safe A* Path Planning

[![Python](https://img.shields.io/badge/Python-3.x-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![Tests](https://github.com/Jillian06/safe-astar-path-planning/actions/workflows/test.yml/badge.svg)](https://github.com/Jillian06/safe-astar-path-planning/actions/workflows/test.yml)

A public research-portfolio implementation accompanying the IEEE paper **“Research on the star algorithm for safe path planning”** (ICCECT 2023). The work studies how A* can be modified to reduce search effort while preserving a safety margin around obstacles.

> **Provenance:** the published paper is the research record. The Python code in this repository is a clean public reimplementation of the documented method for reproducibility and portfolio use; it is not presented as the original 2022–2023 experimental source tree.

## Paper

**B. Cao, Z. Yang, L. Yu, Y. Zhang**, “Research on the star algorithm for safe path planning,” *2023 IEEE International Conference on Control, Electronics and Computer Technology (ICCECT)*, pp. 105–109. DOI: [10.1109/ICCECT57938.2023.10141167](https://doi.org/10.1109/ICCECT57938.2023.10141167)

## Method

The paper develops three modifications around conventional A*:

1. **Adaptive weighting with GA** — changes the relative influence of the accumulated path cost in the evaluation function.
2. **Obstacle-neighborhood penalty z(s)** — rejects or strongly penalizes cells too close to obstacles.
3. **Invisible obstacle expansion** — inflates occupied cells during planning so the returned route maintains a configurable safety margin.

The documented combined evaluation is:

```text
f(s) = (1 / GA) g(s) + h(s) + z(s)
```

where `g(s)` is accumulated cost, `h(s)` is the goal heuristic, `GA` reflects the obstacle-free search context, and `z(s)` increases cost near obstacles.

## What this repository demonstrates

- baseline 8-connected A*
- obstacle inflation for configurable clearance
- optional near-obstacle rejection
- an adaptive GA-style weighting policy
- traversed-node counting for search-efficiency comparison
- reproducible unit tests

## Quick start

```bash
python -m pip install pytest
PYTHONPATH=src pytest -q
```

Minimal example:

```python
from safe_astar import plan

grid = [[0] * 12 for _ in range(12)]
for y in range(2, 10):
    grid[y][6] = 1

result = plan(grid, (1, 1), (10, 10), inflation_radius=1, adaptive_ga=True)
print(result.path)
print(result.expanded_nodes)
```

## Research result context

In the published experiments, obstacle inflation improved clearance while the GA-weighted variants substantially reduced the number of traversed nodes. The paper reports reductions exceeding 80% in some map comparisons, with the GA + obstacle-expansion combination identified as the preferred trade-off between safety and search effort.

See [docs/PAPER_NOTES.md](docs/PAPER_NOTES.md) for a source-faithful summary of the reported method and experiments.

## Connection to current robotics work

This project is the earlier research thread behind my newer [ROS 2 autonomous navigation project](https://github.com/Jillian06/ros2-autonomous-navigation), where a custom planner is integrated with ROS 2 messages, feedback control, LiDAR interfaces, and Gazebo simulation.

## License

MIT for the code in this repository. The IEEE paper remains subject to IEEE copyright.

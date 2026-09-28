from safe_astar import inflate_obstacles, plan

def test_baseline_path_exists():
    grid = [[0] * 8 for _ in range(8)]
    r = plan(grid, (0, 0), (7, 7))
    assert r.path[0] == (0, 0)
    assert r.path[-1] == (7, 7)

def test_inflation_adds_clearance():
    grid = [[0] * 7 for _ in range(7)]
    grid[3][3] = 1
    inflated = inflate_obstacles(grid, 1)
    assert inflated[2][3] == 1
    assert inflated[3][2] == 1
    assert inflated[4][4] == 1

def test_blocked_goal_returns_none():
    grid = [[0] * 5 for _ in range(5)]
    grid[4][4] = 1
    assert plan(grid, (0, 0), (4, 4)).path is None

def test_diagonal_corner_cutting_is_rejected():
    grid = [[0] * 3 for _ in range(3)]
    grid[0][1] = 1
    grid[1][0] = 1
    assert plan(grid, (0, 0), (1, 1)).path is None

def test_adaptive_ga_remains_complete_on_simple_map():
    grid = [[0] * 10 for _ in range(10)]
    r = plan(grid, (0, 0), (9, 9), adaptive_ga=True)
    assert r.path is not None

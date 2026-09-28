# Paper notes

## Documented evaluation functions

Conventional A*:

`f(s) = g(s) + h(s)`

Paper variants:

- GA weighting: `f(s) = (1/GA) g(s) + h(s)`
- obstacle-neighborhood penalty: `f(s) = g(s) + h(s) + z(s)`
- combined: `f(s) = (1/GA) g(s) + h(s) + z(s)`

The paper defines `z(s)` as zero away from obstacles and tending toward infinity when an obstacle occurs in the current node's 8-neighborhood. It separately proposes invisible obstacle expansion so all cells in the selected safety region are treated as occupied during planning.

## Reported experiments

Two example map comparisons in the paper report:

| Method | Map 1 path area | Map 1 search area | Map 2 path area | Map 2 search area |
|---|---:|---:|---:|---:|
| Expansion barrier | 45 | 1069 | 50 | 1158 |
| GA | 43 | 129 | 44 | 356 |
| GA + expansion | 45 | 52 | 50 | 349 |
| Conventional A* | 43 | 1349 | 44 | 1895 |

The discussion reports that GA weighting can reduce traversed nodes by more than 80% in some comparisons while keeping path length nearly unchanged, and identifies **GA + invisible obstacle expansion** as the preferred combined method.

## Interpretation

The central trade-off is not simply shortest path length. The work explicitly treats **clearance** and **search effort** as additional design objectives for practical robots such as medical and service robots.

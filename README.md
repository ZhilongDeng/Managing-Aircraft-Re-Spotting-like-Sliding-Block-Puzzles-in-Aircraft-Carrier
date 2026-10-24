# Managing Aircraft Re-Spotting like Sliding-Block Puzzles in an Aircraft Carrier

MATLAB code for re-spotting aircraft on a carrier's flight deck and hangar. The
problem is modelled as a sliding-block puzzle: an A\* search over whole layouts
finds the sequence of aircraft moves, and a serial scheduler turns that sequence
into a timed plan with tractors and elevators assigned.

## Citation

If you use this code, please cite:

> Deng Z, Wang L, Fang Z, Li B, Wang X*. Managing Aircraft Re-Spotting like
> Sliding-Block Puzzles in Aircraft Carrier. Journal of Computational Design
> and Engineering, In Press.

```

The `*` marks the corresponding author.

## Requirements

MATLAB **R2021a or later** — the plotting code uses the `name=value` syntax.
Verified on R2021b. No toolboxes required.

## Running

```matlab
demo_aircraft_respotting
```

The demo loads the spot geometry and trajectory library, moves one aircraft
from spot 137 to spot 140, and plots the resulting Gantt chart. The A\* search
takes roughly 5 s for this example.

Note: `TrajLibrary.mat` is about **52 MB**. That is under GitHub's 100 MB
per-file limit, so a plain `git add` works, but clones will be large.

## Files
The relevant programs will be sorted out and uploaded in the near future.



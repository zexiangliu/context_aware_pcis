# Context Aware Pcis
Synthesize context-aware pcis  and test with real data.

### Setup

1. Install MPT3 (and optional third-party solvers: [Mosek](https://www.mosek.com/downloads/), [Gurobi](https://www.gurobi.com/)).

2. Add folders `indicate_nonredundant_halfplanes`, `pcis/lib` and `takeover` into the search path of MATLAB.
```
addpath(genpath('pcis/lib/'));
addpath(genpath('indicate_nonredundant_halfplanes'));
addpath(genpath('takeover'));
```
### Usage

*To turn on/off the fast (but less stable) minHRep implementation, go to `pcis/lib/minHRep2.m` and change the flag according to the comments in line 11-12.*

### Thrid-party libraries

- MPT3 (project webpage)[https://www.mpt3.org/]
- A fast minHRep implementation [(github page)](https://github.com/mageecoe/indicate_nonredundant_halfplanes)

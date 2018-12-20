# ltl_fragment_planning

This contains an implementation of feasible policy synthesis for a subset of LTL in a system with uncertainty. The sample implementation is demonstrated on a 2D grid where a controllable agent tries to traverse the grid in order to satisfy LTL specifications and a moving obstruction introduces uncertainty aspect of the system.

The implementation is based on the paper, Optimal Control of Non-deterministic Systems for a Computationally Efficient Fragment of Temporal Logic, by Wolff et al.(link:https://ieeexplore.ieee.org/abstract/document/6760371).

Current implementation produces feasible policy but does not optimize for ordering of state traversal related to task constraints. The synthesized policies are subsequently simulated and recorded for analysis.

See test/sample_obstacle/main.rs for synthesis and simulation. To run, install Rust via rustup package manager, clone the repository, and execute "cargo run" in the cloned repository.

See project.pdf for problem formulation and information on the related LTL fragment type used for synthesis.

Todo:
- code refactor as a reusable general purpose module for arbitrary dimension state space
- optimization to internal algorithms
- future investigation in MDP and monte carlo methods for controller robustness evaluation





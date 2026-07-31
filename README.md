# Robust Stability Analysis of Positive Lur'e Systems with Neural Network Feedback

MATLAB code accompanying the paper:

> H. Montazeri Hedesh, M. K. Wafi, B. Shafai, and M. Siami,
> **"Robust Stability Analysis of Positive Lur'e System with Neural Network Feedback,"**
> *2025 IEEE Conference on Control Technology and Applications (CCTA)*,
> San Diego, CA, USA, Aug. 25–27, 2025, pp. 976–981.
> DOI: [10.1109/CCTA53793.2025.11151493](https://doi.org/10.1109/CCTA53793.2025.11151493)

The repository reproduces the two numerical examples in Section V of the paper:
a positive Lur'e system with a structured (Metzler) perturbation, and a
neural-network–controlled system whose sector bound is refined via the
paper's stability-radius formula.

---

## What the paper does (in one paragraph)

The paper studies robust stability of *positive* Lur'e systems — feedback
interconnections of a linear plant and a sector-bounded nonlinearity — where
both the linear part carries a structured uncertainty and the nonlinearity's
sector is uncertain. Using the positive Aizerman conjecture together with the
stability radius of Metzler matrices, it derives a closed-form stability
radius, extends it to systems with feed-forward neural-network (NN) feedback,
and gives a procedure to tighten (refine) an otherwise conservative NN sector
bound by observing when the closed loop is driven to instability.

---

## Repository layout

```
positive-lure-nn-robustness/
├── examples/
│   ├── example_A_lure_structured_perturbation.m   % Figures 2 and 3
│   └── example_B_nn_controlled_system.m           % Figures 4, 5 and 6
├── data/
│   ├── W1.csv   % NN layer 1 weights  ( 5 x 1 )
│   ├── W2.csv   % NN layer 2 weights  (10 x 5 )
│   └── W3.csv   % NN layer 3 weights  ( 1 x 10)
├── CITATION.bib
├── LICENSE
└── README.md
```

Each script in `examples/` is **self-contained** (all helper routines are
local functions at the bottom of the file) and requires only base MATLAB —
no toolboxes. `ode45` is the only solver used.

---

## How to run

```matlab
% from the repository root, or with examples/ on the path
run examples/example_A_lure_structured_perturbation.m
run examples/example_B_nn_controlled_system.m
```

Both scripts set a fixed random seed (`rng(0,'twister')`) so the randomly
initialised trajectories are reproducible across runs. Example B locates
`data/` automatically relative to the script, so it can be launched from any
working directory.

Tested with MATLAB R2021a+ (`tiledlayout`, `fillBetween`, and `\` matrix
solve). It should also run on older releases; if `tiledlayout` is unavailable,
replace the Figure 6 block with `subplot(2,2,k)`.

---

## Script ↔ figure ↔ result map

| Script | Figures | Key quantities reproduced |
|---|---|---|
| `example_A_lure_structured_perturbation.m` | Fig. 2 (sector), Fig. 3 (trajectories) | stability radius **r ≈ 0.26**; stable for Δ ≤ 0.26 |
| `example_B_nn_controlled_system.m` | Fig. 4, Fig. 5 (trajectories), Fig. 6 (refined sector) | NN sector **[Γ₁,Γ₂] ≈ [−0.91, 0.91]**; radius **r ≈ 2.04**; refined **Γ₂,refined = 0.67 / 0.36 / 0.25 / 0.18** at Δ = 2.5 / 3 / 3.15 / 3.25 |

The neural network in Example B has architecture **1 → 5 → 10 → 1** with
`tanh` activations; its scalar input is the plant output `y = C x` and its
scalar output is the control. The a-priori sector bound follows Theorem 4 with
`c = 1` (since `tanh` is sector bounded in `[0, 1]`):
`Γ₂ = |W3|·|W2|·|W1|` and `Γ₁ = −Γ₂`.

---

## Notes

- **Paper PDF is not included.** The IEEE-formatted article is © 2025 IEEE;
  please obtain it from [IEEE Xplore](https://doi.org/10.1109/CCTA53793.2025.11151493).
  If you want a copy in the repo, add the *author's accepted manuscript* or an
  arXiv preprint rather than the published PDF.
- **Exploratory / follow-up scripts are not included** (e.g. the 2×2 SISO
  variants and controller-training data). They belong to separate experiments
  and are not needed to reproduce this paper's figures. They can be added under
  a separate folder or branch if desired.

---

## Citation

If you use this code, please cite the paper (see [`CITATION.bib`](CITATION.bib)):

```bibtex
@inproceedings{hedesh2025robust,
  author    = {Montazeri Hedesh, Hamidreza and Wafi, Moh Kamalul and Shafai, Bahram and Siami, Milad},
  title     = {Robust Stability Analysis of Positive {Lur'e} System with Neural Network Feedback},
  booktitle = {2025 IEEE Conference on Control Technology and Applications (CCTA)},
  year      = {2025},
  pages     = {976--981},
  address   = {San Diego, CA, USA},
  doi       = {10.1109/CCTA53793.2025.11151493}
}
```

---

## Acknowledgment

This material is based upon work supported in part by the U.S. Office of Naval
Research and the DEVCOM Analysis Center, as described in the paper.

## License

Released under the MIT License (see [`LICENSE`](LICENSE)). If you would prefer a
different license (e.g. BSD-3-Clause or Apache-2.0), replace this file before
publishing.

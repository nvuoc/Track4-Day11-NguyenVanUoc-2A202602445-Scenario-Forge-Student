"""Part C/D: search for failures with the cross-entropy method. TODO 5; the sampling helpers are given."""
from __future__ import annotations

import math

import numpy as np

from avsim import space


def cem_update(Z, Bn, score, prev: dict, rho: float = 0.1, smooth: float = 0.7, min_std: float = 0.25,
               p_clip=(0.03, 0.97)) -> dict:
    """TODO 5. One cross-entropy step. Keep the elite = the top ceil(rho * n) rows by score (at least 2;
    higher score = more dangerous; on equal scores the earlier row wins, i.e. np.argsort(-score, kind="stable")),
    then return a new proposal dict:

      mean  = smooth * elite mean + (1 - smooth) * prev["mean"]           (per continuous coordinate)
      std   = max(min_std, smooth * elite std + (1 - smooth) * prev["std"])
      p_bin = clip(smooth * elite frequency + (1 - smooth) * prev["p_bin"], *p_clip)  (per binary coordinate)
      level = the lowest score inside the elite (how dangerous the elite already is)

    Why the floors: a std that collapses to 0 or a probability of exactly 0/1 makes importance weights explode."""
    n = len(score)
    n_elite = max(2, math.ceil(rho * n))
    order = np.argsort(-score, kind="stable")
    elite_idx = order[:n_elite]

    elite_Z = Z[elite_idx]
    elite_Bn = Bn[elite_idx]

    elite_mean = np.mean(elite_Z, axis=0)
    elite_std = np.std(elite_Z, axis=0)
    elite_p_bin = np.mean(elite_Bn, axis=0)

    new_mean = smooth * elite_mean + (1.0 - smooth) * prev["mean"]
    new_std = np.maximum(min_std, smooth * elite_std + (1.0 - smooth) * prev["std"])
    new_p_bin = np.clip(smooth * elite_p_bin + (1.0 - smooth) * prev["p_bin"], p_clip[0], p_clip[1])
    level = float(np.min(score[elite_idx]))

    return dict(mean=new_mean, std=new_std, p_bin=new_p_bin, level=level)


# ---------------------------------------------------------------- given

def natural_proposal(nat: space.Natural) -> dict:
    return dict(mean=np.zeros(space.D), std=np.ones(space.D), p_bin=nat.p_bin.copy(), level=-math.inf)


def proposal_sample(prop, n, rng):
    Z = prop["mean"] + prop["std"] * rng.standard_normal((n, space.D))
    Bn = (rng.random((n, space.B)) < prop["p_bin"]).astype(int)
    return Z, Bn


def proposal_logpdf(prop, Z, Bn):
    lz = (-0.5 * np.sum(((Z - prop["mean"]) / prop["std"]) ** 2, 1) - np.sum(np.log(prop["std"]))
          - 0.5 * space.D * math.log(2 * math.pi))
    lb = np.sum(Bn * np.log(prop["p_bin"]) + (1 - Bn) * np.log1p(-prop["p_bin"]), 1)
    return lz + lb


def mixture_sample(prop, nat: space.Natural, n, rng, beta=0.2):
    """Draw n rows from the defensive mixture (1 - beta) * proposal + beta * natural."""
    Z, Bn = proposal_sample(prop, n, rng)
    m = rng.random(n) < beta
    Zn, Bnn = nat.sample(int(m.sum()), rng)
    Z[m], Bn[m] = Zn, Bnn
    return Z, Bn

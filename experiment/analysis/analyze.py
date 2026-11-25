#!/usr/bin/env python3
"""Gray-rollout significance analysis for the portal-cooldown experiment.

Pure standard library so it runs under `python3 -I` in a locked-down sandbox.

Pipeline
  1. load empirical latencies measured by bench/ (real A*, real cache)
  2. calibrate repeat-request rate p from the observed ~20% group difference
  3. write the expectation expressions for the two groups and prove the sign
  4. invert the two-standard-error rule to get required sample size N*(g)
  5. sweep canary fraction g: detection-speed curve vs exposure (impact) curve
  6. draw one stratified sample per group at g=0.10 and give a binary verdict
  7. verify the analytic SE against a 10k-run Monte Carlo at the same design
"""
import csv
import json
import math
import os
import random

BENCH_DIR = os.path.join(os.path.dirname(__file__), "..", "bench", "results")
OUT_DIR = os.path.join(os.path.dirname(__file__), "..", "results")
SEED = 20261126
Z_CRIT = 2.0                      # |D_hat| > 2 SE rule given in the task
Z_POWER80 = 0.8416                # extra z so a 5%-level test reaches 80% power
OBSERVED_REL_DIFF = 0.20          # the "two-cheng" gap reported from the field
G_CANARY = 0.10                   # current rollout: 10% of servers have cooldown


def load_ns(name):
    path = os.path.join(BENCH_DIR, f"times_{name}.csv")
    with open(path) as fh:
        return [int(row[0]) for row in csv.reader(fh) if row]


def round_moments(kind):
    """mean/variance of each independent measurement round, for stability."""
    out = []
    rd = 0
    while True:
        p = os.path.join(BENCH_DIR, f"r{rd}", f"times_{kind}.csv")
        if not os.path.exists(p):
            break
        vals = load_ns_path(p)
        n = len(vals)
        m = sum(vals) / n
        v = sum((x - m) ** 2 for x in vals) / (n - 1)
        out.append({"round": rd, "n": n, "mean": m, "var": v})
        rd += 1
    return out


def load_ns_path(path):
    with open(path) as fh:
        return [int(row[0]) for row in csv.reader(fh) if row]


def moments(x):
    n = len(x)
    m = sum(x) / n
    var = sum((v - m) ** 2 for v in x) / (n - 1)          # unbiased
    m2 = sum(v * v for v in x) / n                        # E[X^2], for mixtures
    return n, m, var, m2


def mixture_moments(p, m_a, m2_a, m_b, m2_b):
    """Bernoulli mixture: component A with prob p, B with prob 1-p."""
    m = p * m_a + (1 - p) * m_b
    m2 = p * m2_a + (1 - p) * m2_b
    var = m2 - m * m
    return m, var


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    rng = random.Random(SEED)

    full = load_ns("full")
    miss = load_ns("miss")
    hit = load_ns("cache")
    n_f, mu_c, var_c, m2_c = moments(full)
    _, mu_m, var_m, m2_m = moments(miss)
    _, mu_h, var_h, m2_h = moments(hit)

    # ---- 0b. cross-round stability: pooled moments vs spread across rounds
    stability = {}
    for kind in ("full", "miss", "cache"):
        rms = round_moments(kind)
        if rms:
            means = [r["mean"] for r in rms]
            vars_ = [r["var"] for r in rms]
            mm = sum(means) / len(means)
            cv_mean = (math.sqrt(sum((x - mm) ** 2 for x in means) / max(1, len(means) - 1)) / mm) if mm else 0.0
            stability[kind] = {
                "rounds": len(rms),
                "round_means": means,
                "round_vars": vars_,
                "mean_cv_across_rounds": cv_mean,
                "var_minmax_ratio": (max(vars_) / min(vars_)) if min(vars_) > 0 else None,
            }

    # ---- 1. calibrate repeat rate p so the analytic gap equals the observed 20%
    # exact equation:  mu_C - mu_T(p) = 0.20 * mu_C
    #   mu_T(p) = (1-p)*mu_m + p*mu_h = mu_m + p*(mu_h - mu_m)
    #   -> p* = (mu_m - target_mu_t) / (mu_m - mu_h)
    target_mu_t = (1.0 - OBSERVED_REL_DIFF) * mu_c
    p_star = (mu_m - target_mu_t) / (mu_m - mu_h)
    p = min(max(p_star, 0.0), 1.0)

    mu_t, var_t = mixture_moments(p, mu_h, m2_h, mu_m, m2_m)
    delta = mu_c - mu_t                                    # expected decrease
    rel = delta / mu_c
    assert abs(rel - OBSERVED_REL_DIFF) < 1e-9, "calibration must reproduce the observed 20% gap"

    # ---- 2. sign proof: miss path contains a full A* strictly more than a hit
    sign_slack = mu_m - mu_h                                # always > 0
    miss_premium = mu_m - mu_c                              # should be >= 0

    # ---- 3. invert the 2-SE rule
    # N* total requests, canary share g -> n_T = gN, n_C = (1-g)N
    # SE(D_hat) = sqrt(var_t/(gN) + var_c/((1-g)N)); require delta >= Z_CRIT*SE
    def n_required(g, z=Z_CRIT):
        se_unit = math.sqrt(var_t / g + var_c / (1 - g))    # SE * sqrt(N)
        n_total = (z * se_unit / delta) ** 2
        return n_total, g * n_total, (1 - g) * n_total

    n_star_total, n_star_t, n_star_c = n_required(G_CANARY)
    n_p80_total, n_p80_t, n_p80_c = n_required(G_CANARY, Z_CRIT + Z_POWER80)

    # ---- 4. canary-fraction sweep: detection speed vs impact surface
    grid = [x / 100 for x in range(5, 51, 5)]
    sweep = []
    for g in grid:
        n_total, n_t, n_c = n_required(g)
        n80, _, _ = n_required(g, Z_CRIT + Z_POWER80)
        sweep.append({
            "g": g,
            "n_total": n_total,
            "n_treatment": n_t,
            "n_control": n_c,
            "n_total_power80": n80,
            "exposure_pct": g * 100.0,
        })

    # ---- 5. one real stratified sample per group at the current 10% rollout
    # main window sized for 80% power (the 2-SE boundary alone is ~50% power)
    N_obs = math.ceil(n_p80_total / 10.0) * 10
    n_t_obs = max(1, round(G_CANARY * N_obs))
    n_c_obs = N_obs - n_t_obs

    def draw(pool, k):
        return [pool[rng.randrange(len(pool))] for _ in range(k)]

    def sample_groups(n_t, n_c, p_rep):
        tt = [hit[rng.randrange(len(hit))] if rng.random() < p_rep
              else miss[rng.randrange(len(miss))] for _ in range(n_t)]
        cc = draw(full, n_c)
        return tt, cc

    def diff_stats(tt, cc):
        mt = sum(tt) / len(tt)
        mc = sum(cc) / len(cc)
        v_t = sum((v - mt) ** 2 for v in tt) / (len(tt) - 1)
        v_c = sum((v - mc) ** 2 for v in cc) / (len(cc) - 1)
        d = mc - mt
        se = math.sqrt(v_t / len(tt) + v_c / len(cc))
        return d, se, mt, mc, v_t, v_c

    treatment, control = sample_groups(n_t_obs, n_c_obs, p)
    d_hat, se_hat, xbar_t, xbar_c, _, _ = diff_stats(treatment, control)
    z = d_hat / se_hat
    ci_lo, ci_hi = d_hat - Z_CRIT * se_hat, d_hat + Z_CRIT * se_hat
    verdict = "significant" if abs(z) > Z_CRIT and d_hat > 0 else "inconclusive"

    # ---- 6. Monte Carlo check of the analytic SE and realized power
    reps = 10000
    mc_d = []
    hits = 0
    for _ in range(reps):
        tt, cc = sample_groups(n_t_obs, n_c_obs, p)
        d, se, _, _, _, _ = diff_stats(tt, cc)
        mc_d.append(d)
        if d > Z_CRIT * se:
            hits += 1
    mc_mean = sum(mc_d) / reps
    mc_var = sum((v - mc_mean) ** 2 for v in mc_d) / (reps - 1)
    mc_se = math.sqrt(mc_var)
    power = hits / reps

    # ---- 7. window-size behavior via Monte Carlo (no cherry-picked single draws)
    # rejection rate of the 2-SE rule at N*/2, N* (boundary) and N80 windows
    def mc_rejection(n_total, reps=4000):
        n_t = max(1, round(G_CANARY * n_total))
        n_c = n_total - n_t
        reject = 0
        for _ in range(reps):
            tt, cc = sample_groups(n_t, n_c, p)
            d, se, _, _, _, _ = diff_stats(tt, cc)
            if d > Z_CRIT * se:
                reject += 1
        return reject / reps, n_t, n_c

    N_bound = math.ceil(n_star_total / 10.0) * 10
    window_power = []
    for label, n_total in (("half_N_star", N_bound // 2),
                           ("N_star_2SE_boundary", N_bound),
                           ("N_power80", N_obs)):
        rate, n_t_w, n_c_w = mc_rejection(n_total)
        window_power.append({"window": label, "total_n": n_total,
                             "treatment_n": n_t_w, "control_n": n_c_w,
                             "mc_rejection_rate": rate})

    # full power curve: rejection rate as the observation window grows
    fractions = (0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0)
    power_curve = []
    for frac in fractions:
        n_total = max(20, round(N_obs * frac / 10.0) * 10)
        rate, _, _ = mc_rejection(n_total, reps=4000)
        power_curve.append({"fraction_of_N80": frac, "total_n": n_total,
                            "mc_rejection_rate": rate})

    # how much more data would make the actually observed difference significant?
    inflate = (Z_CRIT * se_hat / d_hat) ** 2 if d_hat > 0 else float("inf")
    n_for_observed = N_obs * inflate

    # ---- 8. sensitivity: conclusions under alternative repeat rates
    sensitivity = []
    for p_alt in sorted(set((0.10, 0.20, round(p, 4), 0.50))):
        m_t_a, v_t_a = mixture_moments(min(p_alt, 1.0), mu_h, m2_h, mu_m, m2_m)
        d_a = mu_c - m_t_a
        se_unit_a = math.sqrt(v_t_a / G_CANARY + var_c / (1 - G_CANARY))
        nstar_a = (Z_CRIT * se_unit_a / d_a) ** 2 if d_a > 0 else float("inf")
        n80_a = ((Z_CRIT + Z_POWER80) * se_unit_a / d_a) ** 2 if d_a > 0 else float("inf")
        sensitivity.append({
            "repeat_rate_p": p_alt,
            "is_calibrated": abs(p_alt - round(p, 4)) < 1e-9,
            "expected_relative_decrease": d_a / mu_c,
            "n_star_total_at_g010": nstar_a,
            "n_power80_total_at_g010": n80_a,
        })

    report = {
        "inputs": {
            "samples_per_distribution": n_f,
            "z_critical": Z_CRIT,
            "observed_relative_gap": OBSERVED_REL_DIFF,
            "canary_fraction": G_CANARY,
            "monte_carlo_reps": reps,
            "seed": SEED,
        },
        "empirical_ns": {
            "full_astar_mean": mu_c, "full_astar_var": var_c,
            "miss_path_mean": mu_m, "miss_path_var": var_m,
            "cache_hit_mean": mu_h, "cache_hit_var": var_h,
            "miss_premium_over_full": miss_premium,
            "pooled_samples_per_kind": n_f,
            "cross_round_stability": stability,
        },
        "model": {
            "repeat_rate_p": p,
            "treatment_mean": mu_t, "treatment_var": var_t,
            "control_mean": mu_c, "control_var": var_c,
            "expected_decrease_delta_ns": delta,
            "expected_relative_decrease": rel,
            "sign_slack_mu_m_minus_mu_h": sign_slack,
        },
        "design_at_10pct": {
            "required_total_n": n_star_total,
            "required_treatment_n": n_star_t,
            "required_control_n": n_star_c,
            "power80_total_n": n_p80_total,
            "power80_treatment_n": n_p80_t,
            "power80_control_n": n_p80_c,
            "window_used_total_n": N_obs,
            "window_treatment_n": n_t_obs,
            "window_control_n": n_c_obs,
        },
        "sweep": sweep,
        "realized_sample": {
            "treatment_n": n_t_obs, "control_n": n_c_obs,
            "treatment_mean_ns": xbar_t, "control_mean_ns": xbar_c,
            "difference_ns": d_hat,
            "standard_error_ns": se_hat,
            "z": z,
            "ci95_ns": [ci_lo, ci_hi],
            "verdict": verdict,
        },
        "monte_carlo": {
            "analytic_se_ns": math.sqrt(var_t / n_t_obs + var_c / n_c_obs),
            "mc_se_ns": mc_se,
            "mc_mean_difference_ns": mc_mean,
            "realized_power": power,
        },
        "window_power": window_power,
        "power_curve": power_curve,
        "observed_difference_needs_n": {
            "current_total_n": N_obs,
            "required_total_n_for_this_d_hat": n_for_observed,
            "inflation_factor": inflate,
        },
        "p_sensitivity": sensitivity,
    }
    with open(os.path.join(OUT_DIR, "analysis.json"), "w") as fh:
        json.dump(report, fh, indent=2)

    print(json.dumps(report, indent=2))
    return report


if __name__ == "__main__":
    main()

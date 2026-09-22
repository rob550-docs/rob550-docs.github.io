---
layout: default
title: Factory Calibration
nav_order: 6
parent: Armlab
last_modified_at: 2026-09-22 12:00:00 -0400
---

> Why your forward kinematics does not exactly match the arm's, and what you can do about it. **Everything on this page is optional.** Checkpoint 2 does not require any of it. Read it if your FK is correct and you want to know where the last few millimeters go.

### Contents
* TOC
{:toc}

## The observation

You write forward kinematics from the published Lite 6 DH table, sample joint configurations inside the joint limits, and compare your end effector position against the one the controller reports through `get_forward_kinematics()`.

On our arms the two differ by **2 to 7 mm**. The difference changes with the joint configuration, and it does not look like a constant offset.

That is expected, and it does not mean your FK is wrong.

## The cause

Every Lite 6 in the lab carries a **factory kinematic calibration** in its controller. After assembly, UFactory measures each arm and records small corrections to the geometry of every joint. The firmware uses those corrected values for FK, IK and motion planning. The published DH table describes the ideal arm and does not include them.

Four things follow from that:

- **The calibration is different for every arm**, because machining and assembly errors differ from one to the next.
- **`get_dh_params()` does not report it.** That call returns the nominal table, which is why running your chaining code on the firmware's parameters does not close the gap either.
- **The error is deterministic.** The same joint angles always produce the same difference. It looks random only because the joint samples are random.
- **The firmware is closer to the truth.** The calibration exists to make the arm more accurate, so where the two disagree, the controller is nearer to where the flange actually is.

{: .important}
A student FK that uses the nominal DH table correctly will **not** match the firmware exactly, and cannot. That is why Checkpoint 2 asks you to agree within 10 mm rather than to floating point precision. Your two implementations, DH and PoX, should still agree with **each other** to numerical precision, because they are two descriptions of the same nominal arm.

## First, rule out the easy causes

Before concluding that the remaining error is calibration, check the three things that produce a similar looking mismatch and are entirely in your control.

**Use the same nominal table as everyone else.** UFactory publishes one set of numbers, and its ROS package ships a slightly different default file (`lite6_default_kinematics.yaml`, with 243.5 mm for `d1` and 62.5 mm for `d6`). The two nominal models differ by **up to about 1.4 mm**, which is a fifth of the gap you are chasing. The table in [Section 6](#the-nominal-table) is the one this lab uses.

**Check the TCP offset is zero.** The SDK's FK includes whatever tool offset is configured on the arm; your FK does not model it. A non zero offset shows up as a constant shift along the tool axis, which is exactly what a wrong `d6` looks like.

**Check the error actually has structure.** Plot your position error against each joint angle. Calibration error varies smoothly with configuration. If yours jumps, or is large only in one region, or grows without bound, you have a real bug underneath the calibration floor.

## Reading your arm's calibration

UFactory ships a script called `gen_kinematics_params.py` in its ROS packages (`xarm_ros` and `xarm_ros2`, under `xarm_description/config/kinematics/`). It is a plain Python file and does not need ROS. It opens a TCP connection to the arm on **port 502**, sends one read request, and receives **42 floating point numbers**.

Those are six values per joint:

| Value | Unit | Meaning |
| ----- | ---- | ------- |
| x, y, z | meters | Position of the joint frame in the previous frame |
| roll, pitch, yaw | radians | Orientation of the joint frame in the previous frame |

That is the same format as a URDF `<origin xyz="..." rpy="..."/>` tag: six parameters per joint, where DH uses four.

{: .warning}
The read is passive. It does not move the arm or change any setting. Even so, do not run it while another program is commanding the arm.

## FK from the calibration

With joint angles `q1` through `q6` in radians, the flange pose in the base frame is:

```
T = T1 * T2 * T3 * T4 * T5 * T6

Ti = Trans(xi, yi, zi) * Rz(yawi) * Ry(pitchi) * Rx(rolli) * Rz(qi)
```

Four notes on that chain:

- The rotation order `Rz(yaw) Ry(pitch) Rx(roll)` is the URDF convention.
- Each joint rotates about its own local z axis, which is the final `Rz(qi)`.
- The result is in **meters**. The SDK reports millimeters.
- The flange frame is the frame after joint 6. No tool transform is added.

Set every calibration correction to zero and this chain puts the flange at about **(87, 0, 154) mm** with all joints at zero, which is the nominal zero pose from the [Hardware page](/docs/armlab/hardware#zero-position).

This is the version to write if you want your FK to match the firmware. It is a different parameterization from DH, not a correction to it.

## Optional: expressing the calibration as a DH table

You already think in DH, so it is natural to want the calibrated arm as a DH table with small corrections. A plain DH table with four parameters per joint **cannot represent it accurately**.

In a simulated test, random calibration errors of about 0.8 mm and 0.15 degrees per joint were added to the nominal model:

| Model | Maximum position error |
| ----- | ---------------------- |
| Nominal DH table | about 6.4 mm |
| Best fit standard DH table, 4 parameters per joint | about 2.8 mm |
| Extended model described below | about 1e-13 mm, which is numerical precision |

The extended model adds five parameters to standard DH: a base transform with four, and one Hayati parameter on joint 2. It also fixes `d2` at zero. The next two sections say why each is needed.

### Why a base transform is needed

Standard DH assumes the base frame's z axis is exactly the axis of joint 1. On a real arm, joint 1's axis can be shifted slightly in x and y, or tilted slightly in roll and pitch, relative to the mounting surface that defines the base frame. DH has no parameters for that, so a separate transform goes in front of the chain:

```
Base = Trans(x, y, 0) * Ry(pitch) * Rx(roll)
```

The base z translation and yaw are not needed, because `d1` and joint 1's angle offset already describe them.

### Why the Hayati parameter is needed

**How DH describes two consecutive joints.** DH relates two joint axes through the **common normal**, the unique line meeting both axes at a right angle. The four parameters describe that line: `a` is its length, `alpha` the angle between the axes measured around it, `d` the distance along the previous axis to where it starts, and `theta` the rotation about the previous axis.

**The problem when two axes are parallel.** On the Lite 6, joints 2 and 3 are designed to be exactly parallel (`alpha2` = 180 degrees). For two parallel lines there is no unique common normal: any line perpendicular to both is valid, and there is one at every position along the axes. DH handles this by picking one, usually at `d = 0`.

The trouble starts when the calibration shows the axes are *not* quite parallel. Suppose axis 3 is tilted by a small angle `epsilon` relative to axis 2, in the plane containing both. The axes now meet at a point far away, at roughly:

```
distance = a2 / epsilon        (epsilon in radians)
```

With `a2` = 200 mm and a tilt of only 0.1 degree (0.00175 rad):

```
distance = 200 / 0.00175 = about 115,000 mm = 115 m
```

The DH parameters must then describe a common normal about 115 m from the robot, which causes three problems:

1. **Huge parameter values.** `d2` and `d3` become about 115 m each with opposite signs, so they nearly cancel. A real error of 0.1 degree is buried inside two enormous numbers.
2. **A discontinuity.** Change the tilt from +0.1 to −0.1 degree and the meeting point jumps from 115 m on one side of the robot to 115 m on the other. `d2` swings from large positive to large negative. The DH parameters are not a continuous function of the geometry near parallel axes.
3. **Bad numerical behavior.** A least squares fit cannot converge well against parameters that behave like that, which is why the best fit standard DH table above still left 2.8 mm.

**The Hayati solution.** Hayati (1983) proposed a modification for nearly parallel axes. For that joint, drop `d` (fix it at zero, since position along the axis is arbitrary for parallel axes) and add a rotation `beta` about the y axis, which describes the small tilt directly:

```
A2 = Rz(q2 + theta_off2) * Tx(a2) * Rx(alpha2) * Ry(beta2)
```

Every other joint keeps the standard form:

```
Ai = Rz(qi + theta_offi) * Tz(di) * Tx(ai) * Rx(alphai)
```

Now a 0.1 degree tilt between axes 2 and 3 simply reads `beta2` = 0.1 degree. The parameter is small when the error is small and changes smoothly when the geometry does, so the fit converges to values close to nominal and reproduces the calibration exactly.

| Situation | Standard DH | Hayati modification |
| --------- | ----------- | ------------------- |
| Axes exactly parallel | Common normal not unique, `d` chosen arbitrarily | `d` fixed at 0 |
| Axes tilted by 0.1 degree | `d` values of about 115 m that nearly cancel | `beta` = 0.1 degree |
| Tilt changes sign | `d` jumps from large positive to large negative | `beta` passes smoothly through 0 |

Use Hayati parameters for any pair of consecutive axes that are nominally parallel. On the Lite 6 that is only joints 2 and 3.

### The complete effective model

```
T = Base * A1 * A2 * A3 * A4 * A5 * A6
```

with `Base` using x, y, roll, pitch; `A2` in Hayati form with `d2` = 0 and `beta2`; and `A1, A3, A4, A5, A6` standard. That is 4 + 24 − 1 + 1 = **28 parameters**, and the fitted values sit close to the nominal table, so you can read them as small corrections.

## The nominal table

| Joint | theta offset (deg) | d (mm) | a (mm) | alpha (deg) |
| ----- | ------------------ | ------ | ------ | ----------- |
| 1 | 0 | 243.3 | 0 | −90 |
| 2 | −90 | 0 | 200 | 180 |
| 3 | −90 | 0 | 87 | 90 |
| 4 | 0 | 227.6 | 0 | 90 |
| 5 | 0 | 0 | 0 | −90 |
| 6 | 0 | 61.5 | 0 | 0 |

{: .warning}
UFactory's ROS repository ships `lite6_default_kinematics.yaml` with slightly different numbers, notably 243.5 mm for `d1` and 62.5 mm for `d6`. The two nominal models differ by up to about 1.4 mm. Make sure your whole team compares against the same nominal source.

## Ways to close the gap

Roughly in order of effort. None of this is required.

1. **Confirm the easy causes first.** Same nominal table across the team, TCP offset zero, and an error that varies smoothly with configuration. This alone can remove a millimeter or more.
2. **Report the error properly.** Give the mean, median, maximum and 95th percentile, and plot the error against each joint angle. A calibration floor looks like smooth structure; a bug does not. Being able to tell them apart is worth more than a smaller number.
3. **Read your arm's calibration and write the six parameter chain** from [FK from the calibration](#fk-from-the-calibration). This is the direct route to matching the firmware, and it is a short function.
4. **Fit the effective DH model** with the base transform and the Hayati parameter, so you keep working in DH while absorbing the calibration. This is the most involved option and the most interesting one.
5. **Compare arms.** Read the calibration from two or three stations and look at how much the corrections vary. That variation is the manufacturing tolerance of the arm, made visible.

## Verification status

**Measured on our arms.** Reading the stored calibration from several of the lab's Lite 6 units and comparing it against the nominal DH table gives a maximum position difference of **2 to 7 mm**, consistently, on every arm tested. That is the same size as the gap students see between their own FK and the firmware, which is the confirmation the explanation on this page needed. It also shows the calibration is substantial rather than a token: these arms really are individually measured, and the corrections really do move the flange by millimeters.

{: .note}
The remaining number worth collecting per station is step 5, the calibration model against the SDK's own `get_forward_kinematics`. UFactory describes the stored file as the arm's calibrated parameters, so that comparison should come out well under 0.1 mm. If it does not, the six-parameter chain in [FK from the calibration](#fk-from-the-calibration) is not the whole story and the difference is worth reporting.

The rest of the model was checked offline: the FK formula against the nominal model at zero joint angles, and the effective DH fit against both the default file and simulated calibrations, reproducing each to numerical precision in position and orientation.

## Running the script

Before you run it:

- Nothing else should be commanding the arm. The script only reads.
- The TCP offset should be zero for the SDK comparison. The script warns if it is not.
- The joint limits near the top of the file (`JOINT_LIMITS_DEG`) should match your firmware.

Requires `numpy`, `scipy` and `pyyaml`, plus `xarm-python-sdk` for the SDK comparison. All are in `env550lab`.

```bash
python lite6_calibrated_dh.py --ip 192.168.1.xxx
python lite6_calibrated_dh.py --ip 192.168.1.xxx --no-sdk
python lite6_calibrated_dh.py --yaml lite6_kinematics_calibrated.yaml
```

It will:

1. Read the calibration from the arm and save it as `lite6_kinematics_calibrated.yaml`.
2. Print the maximum difference between the nominal DH model and the calibration.
3. Fit the effective model, print its table in mm and degrees, and save `lite6_effective_dh.json`.
4. Check the fit on 1000 fresh joint samples.
5. Compare the calibration FK against the SDK's `get_forward_kinematics` and print the maximum and mean error.

## `lite6_calibrated_dh.py`

```python
"""
lite6_calibrated_dh.py

Read the per-arm kinematic calibration of a UFactory Lite6, compute forward
kinematics from it, and fit an "effective DH" table that reproduces it.

Effective model (all lengths in mm, angles in degrees in the printed table):
    T = Base(x, y, roll, pitch) * prod_i [ Rz(q_i + theta_off_i) Tz(d_i) Tx(a_i) Rx(alpha_i) ]
    with one extra rotation Ry(beta2) appended to joint 2 (Hayati parameter,
    needed because joints 2 and 3 are nominally parallel), and d2 fixed at 0.

Usage:
    python lite6_calibrated_dh.py --ip 192.168.1.xxx            # read arm, fit, verify vs SDK
    python lite6_calibrated_dh.py --ip 192.168.1.xxx --no-sdk   # read arm and fit only
    python lite6_calibrated_dh.py --yaml lite6_kinematics_X.yaml # use a saved file (offline)

Requires numpy, scipy, pyyaml. The SDK check also requires xarm-python-sdk.
"""
import argparse, json, socket, struct, sys
import numpy as np
from scipy.optimize import least_squares

# Lite6 joint limits in degrees (published). Edit if your firmware reports different values.
JOINT_LIMITS_DEG = [(-360, 360), (-150, 150), (-3.5, 300), (-360, 360), (-124, 124), (-360, 360)]

# ---------------------------------------------------------------- reading
def read_params_from_arm(ip):
    """Same read-only query used by UFactory's gen_kinematics_params.py (xarm_ros2)."""
    s = socket.create_connection((ip, 502), timeout=5)
    s.send(bytes([0x00, 0x01, 0x00, 0x02, 0x00, 0x01, 0x08]))
    buf = b""
    while len(buf) < 179:
        chunk = s.recv(179 - len(buf))
        if not chunk:
            break
        buf += chunk
    s.close()
    if len(buf) != 179 or not buf[8]:
        raise RuntimeError("Arm returned no valid calibration data. It may not have "
                           "factory kinematic calibration (common on older units).")
    dof, rtype = buf[9], buf[10]
    if not (dof == 6 and rtype == 9):
        print(f"Warning: robot reports dof={dof}, type={rtype}; expected a Lite6 (6, 9).")
    p = struct.unpack("<42f", buf[11:])
    return np.array([p[i * 6:i * 6 + 6] for i in range(6)], dtype=float)  # x y z (m), r p y (rad)

def read_params_from_yaml(path):
    import yaml
    k = yaml.safe_load(open(path))["kinematics"]
    return np.array([[k[f"joint{i}"][n] for n in ("x", "y", "z", "roll", "pitch", "yaw")]
                     for i in range(1, 7)], dtype=float)

def save_yaml(P, path):
    import yaml
    data = {"kinematics": {f"joint{i+1}": dict(zip(("x", "y", "z", "roll", "pitch", "yaw"),
                                                   map(float, P[i]))) for i in range(6)}}
    yaml.safe_dump(data, open(path, "w"), sort_keys=False)

# ---------------------------------------------------------------- math
def rot_rpy(r, p, y):  # URDF convention: R = Rz(yaw) Ry(pitch) Rx(roll)
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @
            np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @
            np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))

def rotz(q):
    T = np.eye(4); c, s = np.cos(q), np.sin(q); T[:2, :2] = [[c, -s], [s, c]]; return T

def roty(b):
    T = np.eye(4); c, s = np.cos(b), np.sin(b); T[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]; return T

def dh(th, d, a, al):
    ct, st, ca, sa = np.cos(th), np.sin(th), np.cos(al), np.sin(al)
    return np.array([[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st],
                     [0, sa, ca, d], [0, 0, 0, 1]])

def fk_calibrated(P, q):
    """FK from the controller's calibration parameters (URDF-style chain). q in rad. Returns 4x4 in m."""
    T = np.eye(4)
    for i in range(6):
        O = np.eye(4); O[:3, :3] = rot_rpy(*P[i, 3:]); O[:3, 3] = P[i, :3]
        T = T @ O @ rotz(q[i])
    return T

# Nominal Lite6 DH (theta_offset rad, d m, a m, alpha rad), UFactory published table
DH_NOMINAL = np.array([[0, 0.2433, 0, -np.pi / 2],
                       [-np.pi / 2, 0, 0.200, np.pi],
                       [-np.pi / 2, 0, 0.087, np.pi / 2],
                       [0, 0.2276, 0, np.pi / 2],
                       [0, 0, 0, -np.pi / 2],
                       [0, 0.0615, 0, 0]])

def unpack(x):
    base = np.eye(4); base[:3, :3] = rot_rpy(x[2], x[3], 0.0); base[:3, 3] = [x[0], x[1], 0.0]
    D = np.insert(x[4:27], 5, 0.0).reshape(6, 4)  # d2 fixed at 0
    return base, D, x[27]

def fk_effective_dh(x, q):
    base, D, beta2 = unpack(x)
    T = base.copy()
    for i in range(6):
        A = dh(q[i] + D[i, 0], D[i, 1], D[i, 2], D[i, 3])
        if i == 1:
            A = A @ roty(beta2)
        T = T @ A
    return T

def log_rot(R):
    c = np.clip((np.trace(R) - 1) / 2, -1, 1); th = np.arccos(c)
    if th < 1e-12:
        return np.zeros(3)
    return th / (2 * np.sin(th)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])

def sample_q(n, rng):
    lo = np.deg2rad([l for l, _ in JOINT_LIMITS_DEG]); hi = np.deg2rad([h for _, h in JOINT_LIMITS_DEG])
    return rng.uniform(lo, hi, (n, 6))

def fit_effective_dh(P, n=300, seed=0):
    Q = sample_q(n, np.random.default_rng(seed))
    targets = [fk_calibrated(P, q) for q in Q]
    def res(x):
        out = []
        for q, T in zip(Q, targets):
            F = fk_effective_dh(x, q)
            out.append(F[:3, 3] - T[:3, 3])
            out.append(0.1 * log_rot(T[:3, :3].T @ F[:3, :3]))  # 0.1 m per rad weighting
        return np.concatenate(out)
    x0 = np.concatenate([np.zeros(4), np.delete(DH_NOMINAL.ravel(), 5), [0.0]])
    return least_squares(res, x0, x_scale="jac", xtol=1e-15, ftol=1e-15, gtol=1e-15).x

def print_table(x):
    base, D, beta2 = unpack(x)
    print("\nEffective model (mm, deg)")
    print(f"Base transform: x={x[0]*1000:.4f}  y={x[1]*1000:.4f}  z=0  "
          f"roll={np.rad2deg(x[2]):.4f}  pitch={np.rad2deg(x[3]):.4f}  yaw=0")
    print(" joint  theta_off        d          a      alpha")
    for i in range(6):
        print(f"  J{i+1}  {np.rad2deg(D[i,0]):9.4f}  {D[i,1]*1000:9.4f}  {D[i,2]*1000:9.4f}  {np.rad2deg(D[i,3]):9.4f}")
    print(f"Hayati beta on joint 2 (extra Ry after the joint 2 DH transform): {np.rad2deg(beta2):.5f} deg")

def as_dict(x):
    base, D, beta2 = unpack(x)
    return {"units": "mm and deg",
            "base": {"x": x[0] * 1000, "y": x[1] * 1000, "z": 0.0,
                     "roll": np.rad2deg(x[2]), "pitch": np.rad2deg(x[3]), "yaw": 0.0},
            "dh": [{"joint": i + 1, "theta_off": np.rad2deg(D[i, 0]), "d": D[i, 1] * 1000,
                    "a": D[i, 2] * 1000, "alpha": np.rad2deg(D[i, 3])} for i in range(6)],
            "beta_joint2": np.rad2deg(beta2)}

# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip"); ap.add_argument("--yaml")
    ap.add_argument("--no-sdk", action="store_true"); ap.add_argument("--n-check", type=int, default=200)
    ap.add_argument("--out", default="lite6_effective_dh.json")
    a = ap.parse_args()
    if not a.ip and not a.yaml:
        ap.error("give --ip or --yaml")

    P = read_params_from_yaml(a.yaml) if a.yaml else read_params_from_arm(a.ip)
    if a.ip and not a.yaml:
        save_yaml(P, "lite6_kinematics_calibrated.yaml")
        print("Saved raw calibration to lite6_kinematics_calibrated.yaml")

    rng = np.random.default_rng(1)
    Qt = sample_q(1000, rng)
    nom = max(np.linalg.norm(fk_calibrated(P, q)[:3, 3] - fk_effective_dh(
        np.concatenate([np.zeros(4), np.delete(DH_NOMINAL.ravel(), 5), [0.0]]), q)[:3, 3]) for q in Qt)
    print(f"Max position difference, nominal DH vs calibration: {nom*1000:.3f} mm")

    x = fit_effective_dh(P)
    pos = max(np.linalg.norm(fk_effective_dh(x, q)[:3, 3] - fk_calibrated(P, q)[:3, 3]) for q in Qt)
    ang = max(np.linalg.norm(log_rot(fk_calibrated(P, q)[:3, :3].T @ fk_effective_dh(x, q)[:3, :3])) for q in Qt)
    print(f"Max difference, effective DH vs calibration (1000 new samples): "
          f"{pos*1000:.2e} mm, {np.rad2deg(ang):.2e} deg")
    print_table(x)
    json.dump(as_dict(x), open(a.out, "w"), indent=2)
    print(f"Saved effective table to {a.out}")

    if a.ip and not a.no_sdk:
        from xarm.wrapper import XArmAPI
        arm = XArmAPI(a.ip, is_radian=False)
        tcp = arm.tcp_offset
        if any(abs(v) > 1e-6 for v in tcp):
            print(f"Warning: TCP offset is {tcp}. The SDK FK includes it and this model does not. "
                  "Comparison below will show that offset.")
        errs = []
        for q in sample_q(a.n_check, rng):
            code, pose = arm.get_forward_kinematics(list(np.rad2deg(q)), input_is_radian=False,
                                                    return_is_radian=False)
            if code != 0:
                print("SDK FK returned code", code); continue
            errs.append(np.linalg.norm(np.array(pose[:3]) - fk_calibrated(P, q)[:3, 3] * 1000))
        arm.disconnect()
        if errs:
            print(f"SDK FK vs calibrated model over {len(errs)} samples: "
                  f"max {max(errs):.4f} mm, mean {np.mean(errs):.4f} mm")

if __name__ == "__main__":
    main()
```

## References

- [UFactory Lite 6 kinematic and dynamic parameters (PDF)](https://www.ufactory.cc/wp-content/uploads/2023/04/Kinematic-and-Dynamic-Parameters-of-UFACTORY-Lite-6.pdf)
- [xArm Python SDK issue 115](https://github.com/xArm-Developer/xArm-Python-SDK/issues/115), where UFactory explains the difference
- [xarm_ros2](https://github.com/xArm-Developer/xarm_ros2), which contains `gen_kinematics_params.py`
- S. Hayati, "Robot arm geometric link parameter estimation," *Proceedings of the 22nd IEEE Conference on Decision and Control*, 1983.

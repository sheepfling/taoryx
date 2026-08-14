from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from typing import Literal, Sequence

InterpPolicy = Literal["strict", "source_clamp"]

CDMIN = ((0.0,0.061),(0.1,0.061),(0.5,0.061),(0.7,0.062),(0.8,0.065),(0.9,0.068),(0.99,0.09),(1.0,0.09),(1.01,0.09),(1.1,0.13),(1.2,0.12),(1.3,0.11),(1.4,0.10),(1.5,0.093),(2.0,0.08),(3.0,0.062),(4.0,0.048),(5.0,0.04),(6.0,0.038),(7.0,0.037),(8.0,0.037),(9.0,0.037))
K = ((0.0,0.2),(0.5,0.2),(1.0,0.23),(1.5,0.4),(2.0,0.5),(3.0,0.8),(4.0,0.93),(5.0,1.05),(6.0,1.15),(9.0,1.33))
CLA = ((0.0,4.5),(0.4,3.8),(0.6,3.6),(1.05,4.5),(1.4,4.0),(2.8,2.5),(6.0,1.1),(9.0,1.0))
CLDE = ((0.0,1.0),(0.6,1.05),(1.0,1.15),(1.2,1.0),(1.6,0.66),(2.0,0.5),(2.4,0.4),(3.0,0.31),(5.6,0.21),(6.0,0.2),(9.0,0.2))
CMDE = ((0.0,-1.5),(0.8,-1.6),(1.0,-1.75),(1.6,-1.3),(2.8,-0.6),(6.0,-0.25),(9.0,-0.2))

CLM_M = (0.0,0.6,0.8,1.0,1.2,1.4,1.6,9.0)
ALT = (0.0,40000.0,60000.0,80000.0)
CLM = ((0,0,0,0),(0,0,0,0),(0,0,0.2,0),(0.01,0.04,0.6,0),(0,0.02,0.15,0),(0,0,0,0),(0,0,0,0),(0,0,0,0))
CLDA_M = (0.0,0.4,0.6,1.0,1.4,1.6,2.4,6.0)
CLDA = ((0.05,0.05),(0.07,0.05),(0.08,0.05),(0.11,0.05),(0.08,0.06),(0.07,0.06),(0.06,0.05),(0.03,0.02))
CMM_M = (0.0,0.6,0.8,1.0,1.1,1.2,1.4,1.7,2.0,9.0)
CMM = ((0,0,0,0),(0,-0.01,-0.01,0),(0,-0.13,-0.13,0),(0,-0.11,-0.18,0),(0,-0.16,-0.18,0),(0,-0.25,-0.17,0),(0,-0.06,-0.15,0),(0,-0.03,-0.03,0),(0,-0.01,-0.01,0),(0,0,0,0))


@dataclass(frozen=True)
class Coefficients:
    CX: float
    CY: float
    CZ: float
    CMX: float
    CMY: float
    CMZ: float
    CL: float
    CD: float
    CY_wind: float
    clamping_used: bool
####
####


def interp1(x: float, points: Sequence[tuple[float, float]], policy: InterpPolicy) -> tuple[float, bool]:
    clamped = False
    if x < points[0][0]:
        if policy == "strict":
            raise ValueError("Below table range")
        x = points[0][0]
        clamped = True
    elif x > points[-1][0]:
        if policy == "strict":
            raise ValueError("Above table range")
        x = points[-1][0]
        clamped = True
    ####
    if x == points[-1][0]:
        return points[-1][1], clamped
    for (x0, y0), (x1, y1) in zip(points[:-1], points[1:], strict=True):
        if x0 <= x <= x1:
            f = (x - x0) / (x1 - x0)
            return y0 + f * (y1 - y0), clamped
        ####
    raise RuntimeError("Interpolation failure")
####


def interp2(x: float, y: float, xs: Sequence[float], ys: Sequence[float], values: Sequence[Sequence[float]], policy: InterpPolicy) -> tuple[float, bool]:
    clamped = False
    if x < xs[0] or x > xs[-1] or y < ys[0] or y > ys[-1]:
        if policy == "strict":
            raise ValueError("Outside table range")
        x = min(max(x, xs[0]), xs[-1])
        y = min(max(y, ys[0]), ys[-1])
        clamped = True
    ####
    def locate(v: float, nodes: Sequence[float]) -> tuple[int, int, float]:
        if v == nodes[-1]:
            return len(nodes)-1, len(nodes)-1, 0.0
        for i, (a, b) in enumerate(zip(nodes[:-1], nodes[1:], strict=True)):
            if a <= v <= b:
                return i, i+1, (v-a)/(b-a)
            ####
        raise RuntimeError("No bracket")
    ####
    i0, i1, fx = locate(x, xs)
    j0, j1, fy = locate(y, ys)
    v00 = values[i0][j0]
    if i0 == i1 and j0 == j1:
        return float(v00), clamped
    if i0 == i1:
        return float(v00 + fy*(values[i0][j1]-v00)), clamped
    if j0 == j1:
        return float(v00 + fx*(values[i1][j0]-v00)), clamped
    value = (
        v00*(1-fx)*(1-fy)
        + values[i1][j0]*fx*(1-fy)
        + values[i0][j1]*(1-fx)*fy
        + values[i1][j1]*fx*fy
    )
    return float(value), clamped
####


def evaluate(
    mach: float,
    altitude_ft: float,
    alpha_deg: float,
    beta_deg: float,
    de_deg: float = 0.0,
    da_deg: float = 0.0,
    dr_deg: float = 0.0,
    p_hat: float = 0.0,
    q_hat: float = 0.0,
    r_hat: float = 0.0,
    policy: InterpPolicy = "source_clamp",
) -> Coefficients:
    a = math.radians(alpha_deg)
    b = math.radians(beta_deg)
    de = math.radians(de_deg)
    da = math.radians(da_deg)
    dr = math.radians(dr_deg)
    cd0, c0 = interp1(mach, CDMIN, policy)
    k, c1 = interp1(mach, K, policy)
    cla, c2 = interp1(mach, CLA, policy)
    clde, c3 = interp1(mach, CLDE, policy)
    cmde, c4 = interp1(mach, CMDE, policy)
    clm, c5 = interp2(mach, altitude_ft, CLM_M, ALT, CLM, policy)
    clda, c6 = interp2(mach, altitude_ft, CLDA_M, (0.0,80000.0), CLDA, policy)
    cmm, c7 = interp2(mach, altitude_ft, CMM_M, ALT, CMM, policy)

    cl = cla*a + mach*clm + clde*de
    cd = cd0 + k*cl*cl
    cyw = -1.4*b - 0.05*da + 0.45*dr
    cmx = -0.01*b + clda*da + 0.012*dr - 0.35*p_hat + 0.04*r_hat
    cmy = -1.2*a + mach*cmm + cmde*de - 6.2*q_hat
    cmz = 0.5*b + 0.04*da - 0.3*dr - 1.5*r_hat

    ca, sa = math.cos(a), math.sin(a)
    cb, sb = math.cos(b), math.sin(b)
    cx = -cd*ca*cb - cyw*ca*sb + cl*sa
    cy = -cd*sb + cyw*cb
    cz = -cd*sa*cb - cyw*sa*sb - cl*ca
    return Coefficients(cx, cy, cz, cmx, cmy, cmz, cl, cd, cyw, any((c0,c1,c2,c3,c4,c5,c6,c7)))
####


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mach", type=float, required=True)
    parser.add_argument("--altitude-ft", type=float, required=True)
    parser.add_argument("--alpha-deg", type=float, required=True)
    parser.add_argument("--beta-deg", type=float, required=True)
    parser.add_argument("--de-deg", type=float, default=0.0)
    parser.add_argument("--da-deg", type=float, default=0.0)
    parser.add_argument("--dr-deg", type=float, default=0.0)
    parser.add_argument("--p-hat", type=float, default=0.0)
    parser.add_argument("--q-hat", type=float, default=0.0)
    parser.add_argument("--r-hat", type=float, default=0.0)
    parser.add_argument("--policy", choices=("strict", "source_clamp"), default="source_clamp")
    args = parser.parse_args()
    result = evaluate(
        args.mach,
        args.altitude_ft,
        args.alpha_deg,
        args.beta_deg,
        args.de_deg,
        args.da_deg,
        args.dr_deg,
        args.p_hat,
        args.q_hat,
        args.r_hat,
        args.policy,
    )
    print(json.dumps(asdict(result), indent=2, sort_keys=True))
    return 0
####
####


if __name__ == "__main__":
    raise SystemExit(main())
####

"""
Given the desired force needed to fly a trajectory,
compute the optimal pitch for producing that force.
"""

from os import error
import sys
import numpy as np
import scipy.optimize as sco
from enum import Enum
sys.path.append('..')

from tools.rotations import Quaternion2Rotation, vee
from tools.wrap import wrap
import parameters.convergence_parameters as VTOL
import parameters.optimal_pitch_parameters as OPT_PARAM
import parameters.landing_parameters_precomputed as LANDING

# import pyoptsparse - set flag if it is available
try:
    from pyoptsparse import Optimization, OPT
    pyoptsparse_imported = True
except ImportError:
    pyoptsparse_imported = False

class OPTIMAL_PITCH_METHOD(Enum):
    Sampled = 1
    Optimizer = 2
    ZThrust = 3

class AERO_TYPE(Enum):
    SMALL_ANGLE = 1
    FLAT_PLATE_1 = 2
    FLAT_PLATE_2 = 3
    BLENDED_1 = 4
    BLENDED_2 = 5
    SMALL_ANGLE_CONTINUOUS = 6


def CL_small_angle(alpha):
    return VTOL.C_L_0 + VTOL.C_L_alpha * alpha

def CL_discontinuous(alpha, CL_func):
    if isinstance(alpha, np.ndarray):
        # vectorized
        CL_ret = CL_func(alpha)
        CL_ret[alpha > OPT_PARAM.alpha_aero_max] = 0.0
        CL_ret[alpha < OPT_PARAM.alpha_aero_min] = 0.0
        return CL_ret
    else:
        if alpha >= OPT_PARAM.alpha_aero_min and alpha <= OPT_PARAM.alpha_aero_max:
            return CL_func(alpha)
        else:
            return 0.0


def CD_small_angle(alpha):
    eta_2 = np.pi*VTOL.e_oswald*VTOL.AR_wing
    return VTOL.C_D_p + (CL_small_angle(alpha)**2)/eta_2

## Alternative L/D models
def CL_plate_stengel(alpha):
    return 2 * np.sign(alpha) * np.sin(alpha)**2 * np.cos(alpha)

def CL_plate_tedrake(alpha):
    return 2 * np.sin(alpha) * np.cos(alpha)

def CD_plate_stengel(alpha):
    return 2 * np.sign(alpha) * np.sin(alpha)**3

def CD_plate_tedrake(alpha):
    return 2 * np.sin(alpha)**2 + VTOL.C_D_p

def sigma(alpha):
    tmp1 = np.exp(-VTOL.M * (alpha - VTOL.alpha0))
    tmp2 = np.exp(VTOL.M * (alpha + VTOL.alpha0))
    return (1 + tmp1 + tmp2) / ((1 + tmp1) * (1 + tmp2))

def blended(alpha, CLCD_small, CLCD_large):
    # use sigma function to blend a CL/CD curve
    sig = sigma(alpha)
    CLCD_small_res = CLCD_small(alpha)
    CLCD_large_res = CLCD_large(alpha)

    return (1 - sig)*CLCD_small_res + sig*CLCD_large_res

def CD(alpha, model=AERO_TYPE.SMALL_ANGLE):
    if model is AERO_TYPE.SMALL_ANGLE:
        return CD_small_angle(alpha)
    elif model is AERO_TYPE.FLAT_PLATE_1:
        return CD_plate_tedrake(alpha)
    elif model is AERO_TYPE.FLAT_PLATE_2:
        return CD_plate_stengel(alpha)
    elif model is AERO_TYPE.BLENDED_1:
        return blended(alpha, CD_small_angle, CD_plate_tedrake)
    elif model is AERO_TYPE.BLENDED_2:
        return blended(alpha, CD_small_angle, CD_plate_stengel)
    elif model is AERO_TYPE.SMALL_ANGLE_CONTINUOUS:
        return CD_small_angle(alpha)
    else:
        error("Unknown AERO_TYPE")

def CL(alpha, model=AERO_TYPE.SMALL_ANGLE):
    if model is AERO_TYPE.SMALL_ANGLE:
        return CL_discontinuous(alpha, CL_small_angle)
    elif model is AERO_TYPE.FLAT_PLATE_1:
        return CL_plate_tedrake(alpha)
    elif model is AERO_TYPE.FLAT_PLATE_2:
        return CL_plate_stengel(alpha)
    elif model is AERO_TYPE.BLENDED_1:
        return blended(alpha, CL_small_angle, CL_plate_tedrake)
    elif model is AERO_TYPE.BLENDED_2:
        return blended(alpha, CL_small_angle, CL_plate_stengel)
    elif model is AERO_TYPE.SMALL_ANGLE_CONTINUOUS:
        return CL_small_angle(alpha)
    else:
        error("Unknown AERO_TYPE")

def F_lift(alpha, Va, model=AERO_TYPE.SMALL_ANGLE):

    return .5 * VTOL.rho * Va**2 * VTOL.S_wing * CL(alpha, model=model)

def F_drag(alpha, Va, model=AERO_TYPE.SMALL_ANGLE):
    return .5 * VTOL.rho * Va**2 * VTOL.S_wing * CD(alpha, model=model)

def compute_thrust(Va, alphas, F_d_gamma, model=AERO_TYPE.SMALL_ANGLE):
    R_alpha = R_bar(alphas)

    F_err = np.array([ F_drag(alphas, Va, model=model) + F_d_gamma[0],
                                 F_lift(alphas, Va, model=model) + F_d_gamma[1]])

    if isinstance(alphas, np.ndarray):
        T_d_p = np.einsum("ijn,jn->in", R_alpha, F_err) # R_alpha[:,:,n] @ F_err[:,n]
    else:
        T_d_p = R_alpha @ F_err

    return T_d_p

def R_bar(ang):
    return np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])

def thrust_angle(thrusts):
    if thrusts.ndim == 2:
        T_angle = np.arctan2(-thrusts[1,:], thrusts[0,:])
    elif thrusts.ndim == 1:
        T_angle = np.arctan2(-thrusts[1], thrusts[0])
    return T_angle

def thrust_magnitude(thrusts):
    if thrusts.ndim == 2:
        T_mag = np.sqrt(thrusts[0,:]**2 + thrusts[1,:]**2)
    elif thrusts.ndim == 1:
        T_mag = np.sqrt(thrusts[0]**2 + thrusts[1]**2)
    return T_mag

def find_pitch_thrust_sampled_thetas(thetas, Va, gamma, F_d_gamma, T_mag_weight=1.0, theta_weight=.01, model=AERO_TYPE.SMALL_ANGLE):
    alphas = thetas - gamma

    T_d_p = compute_thrust(Va, alphas, F_d_gamma, model=model)

    T_angle = thrust_angle(T_d_p)
    T_mag = thrust_magnitude(T_d_p)

    T_good = np.logical_and(T_angle >= OPT_PARAM.xi_T_min, T_angle <= OPT_PARAM.xi_T_max)

    thetas_good = thetas[T_good]
    T_angle_good = T_angle[T_good]
    T_mag_good = T_mag[T_good]
    T_d_p_good = T_d_p[:,T_good]

    if len(T_mag_good) > 0:
        T_best_index = np.argmin(T_mag_weight*T_mag_good + theta_weight*thetas_good**2)

        T_d_p_best = T_d_p_good[:,T_best_index]
        theta_best = thetas_good[T_best_index]
        return theta_best, T_d_p_best
    else:
        # No feasible thrust
        return None, None

def find_thrust_from_theta(v_d_d, F_d_d, theta, model=AERO_TYPE.SMALL_ANGLE):
    gamma = np.arctan2(-v_d_d[2], v_d_d[0])
    Va = np.linalg.norm(v_d_d)
    if Va < 1e-8:
        gamma = 0.0
    R_gamma = R_bar(gamma)

    if len(F_d_d) == 3:
        F_d_d = F_d_d[[0,2]]

    F_d_gamma = R_gamma @ F_d_d

    alpha = theta - gamma

    T_d_p = compute_thrust(Va, alpha, F_d_gamma, model=model)

    return T_d_p

def find_pitch_thrust_optimized(v_d_d, F_d_d, previous_theta=None, model=AERO_TYPE.BLENDED_2, theta_weight=1e-4):
    # this is an implementation of the pitch-thrust optimization from the CDC/L-CSS 2021 paper
    if not(pyoptsparse_imported):
        error("Pyoptsparse not available on this system")

    gamma = np.arctan2(-v_d_d[2], v_d_d[0])
    Va = np.linalg.norm(v_d_d)
    if Va < 1e-8:
        gamma = 0.0
    R_gamma = R_bar(gamma)

    if len(F_d_d) == 3:
        F_d_d = F_d_d[[0,2]]

    F_d_gamma = R_gamma @ F_d_d

    def objconfunc(xdict):
        theta = xdict["xvars"][0]
        funcs = {}

        alpha = theta - gamma

        T_d_p = compute_thrust(Va, alpha, F_d_gamma, model=model)

        funcs["obj"] = T_d_p[0]**2 + T_d_p[1]**2 + theta_weight*theta**2

        angle_T = thrust_angle(T_d_p)

        ineq_convec = [
            # xi_T_min <= angle_T <= xi_T_max
            angle_T - OPT_PARAM.xi_T_min,
            OPT_PARAM.xi_T_max - angle_T
        ]

        funcs["ineq_con"] = ineq_convec

        return funcs, False

    # find force angle
    xi_F_d = np.arctan2(-F_d_d[1], F_d_d[0])

    theta_max = max(OPT_PARAM.theta_max, xi_F_d - np.pi/2)

    optProb = Optimization("Optimal Pitch", objconfunc)
    optProb.addVarGroup("xvars", 1, "c", lower=[OPT_PARAM.theta_min], upper=[theta_max])
    optProb.addConGroup("ineq_con", 2, lower=0.0, upper=None)

    optProb.addObj("obj")

    optOptions = {"Major feasibility tolerance":1e-4, "Major optimality tolerance":1e-4, "Iterations limit":100}
    opt = OPT("SNOPT", options=optOptions)
    sol = opt(optProb, sens="FD")

    xStar = sol.xStar['xvars']
    success_code = sol.optInform['value'][0]
    if  success_code > 2 and success_code != 13:
        print("Failed with Va = {}, gamma = {}, F_d_d = {}\n\t{}".format(
            Va, gamma, F_d_d, sol.optInform['text']))
        alpha = previous_theta - gamma
        T_fallback = compute_thrust(Va, alpha, F_d_gamma)
        return previous_theta, T_fallback

    theta_best = xStar[0]
    alpha = theta_best - gamma
    T_d_p_best = compute_thrust(Va, alpha, F_d_gamma)

    return theta_best, T_d_p_best


def find_pitch_thrust_sampled(v_d_d, F_d_d, previous_theta=None, nsamples=50, model=AERO_TYPE.SMALL_ANGLE, transition=False):

    if transition:
        theta_d = LANDING.theta
        F_d_p = R_bar(theta_d) @ F_d_d
        return theta_d, F_d_p

    
    gamma = np.arctan2(-v_d_d[2], v_d_d[0])
    Va = np.linalg.norm(v_d_d)
    if Va < 1e-8:
        gamma = 0.0
    R_gamma = R_bar(gamma)

    if len(F_d_d) == 3:
        F_d_d = F_d_d[[0,2]]

    F_d_gamma = R_gamma @ F_d_d

    # search across the theta_min/theta_max pitch range first
    if previous_theta is None or previous_theta >= OPT_PARAM.theta_max or previous_theta <= OPT_PARAM.theta_min:
        thetas = np.concatenate((np.linspace(OPT_PARAM.theta_min, OPT_PARAM.theta_max, nsamples, endpoint=True), np.array([0])))
    else:
        theta_spacing = (OPT_PARAM.theta_max-OPT_PARAM.theta_min)/nsamples

        pt_samples = theta_spacing*(np.linspace(-.9, .9, int(nsamples/2))**3) + previous_theta
        pt_samples = pt_samples[np.logical_and(pt_samples >= OPT_PARAM.theta_min, pt_samples <= OPT_PARAM.theta_max)]

        thetas = np.concatenate([
            np.linspace(OPT_PARAM.theta_min, OPT_PARAM.theta_max, int(nsamples/2), endpoint=True),
            np.array([0]),
            pt_samples])

    theta_best, T_d_p_best = find_pitch_thrust_sampled_thetas(thetas, Va, gamma, F_d_gamma, theta_weight=1e-4, model=model)

    if theta_best is None:
        # didn't find feasible thrust  - need to search at higher/lower pitch angles
        if previous_theta is None or (previous_theta >= OPT_PARAM.theta_min and previous_theta <= OPT_PARAM.theta_max):
            thetas = np.concatenate((np.linspace(-np.pi/2, OPT_PARAM.theta_min, int(nsamples/2), endpoint=True), np.linspace(OPT_PARAM.theta_max, np.pi/2, int(nsamples/2), endpoint=True)))
        else:
            nsamples_search = nsamples/2
            nsamples_pt = nsamples/2
            theta_spacing = (np.pi/2 - OPT_PARAM.theta_max)/(nsamples/2)

            thetas = np.concatenate([
                np.linspace(-np.pi/2, OPT_PARAM.theta_min, int(nsamples_search), endpoint=True),
                np.linspace(OPT_PARAM.theta_max, np.pi/2, int(nsamples_search), endpoint=True),
                theta_spacing*(np.linspace(-.9, .9, int(nsamples_pt))**3) + previous_theta])


        theta_best, T_d_p_best = find_pitch_thrust_sampled_thetas(thetas, Va, gamma, F_d_gamma, T_mag_weight=1e-4, theta_weight=1.0, model=model) # find smallest theta

    if theta_best is None:
        if previous_theta is None:
            previous_theta = VTOL.theta0
        alpha = previous_theta - gamma
        T_fallback = compute_thrust(Va, alpha, F_d_gamma)
        return previous_theta, T_fallback

    return theta_best, T_d_p_best

def find_pitch_thrust_ZThrust(v_d_d, F_d_d, model=AERO_TYPE.SMALL_ANGLE):
    # for plotting/comparison purposes
    gamma = np.arctan2(-v_d_d[2], v_d_d[0])
    Va = np.linalg.norm(v_d_d)
    if Va < 1e-8:
        gamma = 0.0
    R_gamma = R_bar(gamma)

    if len(F_d_d) == 3:
        F_d_d = F_d_d[[0,2]]

    F_d_gamma = R_gamma @ F_d_d

    xi_F_d = np.arctan2(-F_d_d[1], F_d_d[0])

    theta = xi_F_d - np.pi/2

    alpha = theta - gamma

    T_d_p = compute_thrust(Va, alpha, F_d_gamma, model=model)

    # # zero a negative Tx component, since it can't be produced
    # if T_d_p[0] < 0.:
    #     T_d_p[0] = 0.

    # T_angle = thrust_angle(T_d_p)

    if Va < 10.:
        # valid T_angle
        return theta, T_d_p
    else:
        return np.nan, np.array([np.nan, np.nan])

def find_pitch_thrust(v_d_d, F_d_d, previous_theta, method=OPTIMAL_PITCH_METHOD.Sampled, model=AERO_TYPE.SMALL_ANGLE, transition=False):
    if method is OPTIMAL_PITCH_METHOD.Sampled:
        return find_pitch_thrust_sampled(v_d_d, F_d_d, previous_theta, model=model, transition=transition)
    elif method is OPTIMAL_PITCH_METHOD.Optimizer:
        return find_pitch_thrust_optimized(v_d_d, F_d_d, previous_theta, model=model)
    elif method is OPTIMAL_PITCH_METHOD.ZThrust:
        return find_pitch_thrust_ZThrust(v_d_d, F_d_d, model=model)
    else:
        error("Unknown OPTIMAL_PITCH_METHOD ", method)

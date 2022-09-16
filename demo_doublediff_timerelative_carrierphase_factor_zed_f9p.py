#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Example of a factor graph with time-relative carrier-phase factors.

Created on Sun Oct 17 12:03:26 2021

@author: Jonas Beuchert
"""
from pyubx2 import UBXReader
import os
import gpstk
# GTSAM from https://github.com/borglab/gtsam for latest features
# Downloads/gtsam
import gtsam
import matplotlib.pyplot as plt
import numpy as np
from functools import partial  # For GTSAM's custom factor
from typing import List, Optional  # For type checking in error function of factor
import ctypes  # To access content of GPSTk's matrices
from geographiclib.geodesic import Geodesic  # Coordinate trafos
import util  # Custom utilities
import collections  # For queue with maximum length that stores recent observations
import math


# Incremental smoothing or offline smoothing
INCREMENTAL = False
# Optimizer type
DOG_LEG = False
# Down-weight previous observations by scaling standard deviation linear with time
WEIGHT = True
# Use Huber loss instead of squared one
HUBER = True
# Use dynamic covariance scaling instaed of squared loss
DCS = False

if HUBER and DCS:
    raise Exception("Can only use Huber loss *or* DCS.")

# Satellite systems to use
GNSS_LIST = gpstk.vector_GNSS([gpstk.SatelliteSystem.GPS,
                               gpstk.SatelliteSystem.Glonass,
                               gpstk.SatelliteSystem.Galileo,
                               gpstk.SatelliteSystem.BeiDou
                              ])

# Plot factors
PLOT_FACTORS = True  # False

# Desired GNSS obseravtion rate
obs_frequency = 1.0  # [Hz]

# Meta data
dataset = {
    "rhb-handheld": {
        "directory": "raw_data",
        "observation_file": "nhm.ubx",
        "coarse_start_time": gpstk.CivilTime(2021, 10, 24, 11, 0, 0),
        "coarse_end_time": gpstk.CivilTime(2021, 10, 24, 11, 30, 0),
        "coarse_latitude": 51.759127,
        "coarse_longitude": -1.256574,
        "coarse_height": 100.0,
        "start_key": 0,
        "end_key": 113,
        "observation_frequency": 1.0,
        "temperature": 14.0,  # [C]
        "pressure": 1017.0,  # [mB]
        "humidity": 67.0  # [%]
    }
}

# Select dataset
sequence = dataset["rhb-handheld"]

# Extract data
obs_folder = sequence["directory"]
civil_time = sequence["coarse_start_time"]
obs_file = sequence["observation_file"]
obs_path = os.path.join(obs_folder, obs_file)
step = int(round(sequence["observation_frequency"] / obs_frequency))
obs_frequency = sequence['observation_frequency'] / step
print(f"Use observation frequency {obs_frequency} Hz.")
start_key = sequence["start_key"]
end_key = sequence["end_key"]
lat0, lon0, h0 = util.read_lat_lon_height_key(obs_path, start_key)

# Navigation data
ephStore, header = util.read_nav(civil_time)

# Number of observed/used satellite systems
n_gnss = len(GNSS_LIST)
# State dimension
# Double differences do not require estimation of bias for each GNSS
n_state_vars = 3
# The state to estimate is the 3D position relative to the start position in
# earth center earth fixed (ECEF) coordinates

# Factor graph

# Maximum age for carrier phases to create differenced carrier-phase factors [s]
# Cannot generate constraints between long time separated nodes because the
# satellite clock bias changes with time
max_age = 95.0

# Create initial/reference (geodetic) location and transform to ECEF XYZ
pos_init_geo = gpstk.Position(lat0, lon0, h0, gpstk.Position.Geodetic)
pos_init_ecef = pos_init_geo.transformTo(gpstk.Position.Cartesian)

# Create noise model, TODO: tune
# Initial receiver clock uncertainty [m]
sigma_delta_t_times_c_prior = 0.02 * gpstk.C_MPS
# Initial location uncertainty in each direction [m]
sigma_pos_prior = 20.0*1e-6
# Uncertainty of initial state
prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
    sigma_pos_prior * np.ones(3)
    )
# Prior at origin (relative XYZ coordinates)
prior_mean = np.zeros(3)
# [dx, dy, dz]

# Create an empty nonlinear factor graph
graph = gtsam.NonlinearFactorGraph()

# Add a prior on the first point, setting it to the origin
# A prior factor consists of a mean and a noise model (covariance matrix)
prior_factor = gtsam.PriorFactorVector(start_key, prior_mean, prior_noise)
graph.add(prior_factor)

# Motion model, TODO: tune
# Receiver clock drift noise [m/s]
sigma_delta_t_times_c_drift = 0.1 * gpstk.C_MPS
# Receiver velocity noise in each direction [m/s]
sigma_velocity = 5.0
# Uncertainty of state change [m]
motion_noise = gtsam.noiseModel.Diagonal.Sigmas(
    sigma_velocity * np.ones(3) / obs_frequency)
# Random motion (without bias)
motion_mean = np.zeros(n_state_vars)

# Create the data structure to hold the initial estimate to the solution
initial = gtsam.Values()

if INCREMENTAL:
    # Create (incremental) ISAM2 solver
    isam_parameters = gtsam.ISAM2Params()
    if DOG_LEG:
        isam_parameters.setOptimizationParams(gtsam.ISAM2DoglegParams())
    print()
    print("ISAM2 parameters:")
    print(isam_parameters)
    print()
    isam = gtsam.ISAM2(isam_parameters)
    previous_state = np.zeros(3)

# Configure GPSTk helper functions
raim_solver = gpstk.PRSolution()
raim_solver.allowedGNSS = GNSS_LIST

# Reference ellipsoid for latitude and longitude
geod = Geodesic.WGS84

# Variable key, start with x_0
key = 0

# For debugging
debug_info = []

def _error_cp(measurement: np.ndarray,
             this: gtsam.CustomFactor,
             values,
             jacobians: Optional[List[np.ndarray]]):
    """Time-relative carrier phase factor error function
    :param measurement: XYZ satellite locations [m] x4,
                        receiver clock bias selector vector (binary),
                        carrier phase difference [m],
                        to be filled with `partial`
    :param this: gtsam.CustomFactor handle
    :param values: gtsam.Values
    :param jacobians: Optional list of Jacobians
    :return: the unwhitened error
    """    
    state = [values.atVector(key) for key in this.keys()]
    receiver_pos = [state[idx][0:3] + np.array([pos_init_ecef[i] for i in range(3)]) for idx in range(2)]
    sat_pos = [measurement[(3*idx):(3*idx+3)] for idx in range(4)]
    geo_range = [sat_pos[idx] - receiver_pos[int(idx/2)] for idx in range(4)]
    geo_range_norm = [np.linalg.norm(geo_range[idx]) for idx in range(4)]
    cp = [geo_range_norm[idx] for idx in range(4)]
    cp_diff_estimate = (cp[3] - cp[2]) - (cp[1] - cp[0])
    cp_diff_measurement = measurement[-1]
    error = cp_diff_estimate - cp_diff_measurement
    
    if jacobians is not None:
    
        for idx in range(2):
            
            unit_vector = [geo_range[idx_] / geo_range_norm[idx_] for idx_ in range(idx*0*2, (idx*0+1)*2)]
            
            jacobians[idx] = np.array([(-1)**(idx+1) * (-unit_vector[1] - (-unit_vector[0]))])
    
    return np.array([error])

def weight_cp(std, dt):
    return std + dt * std

# Observations

# Open UBX file
with open(obs_path, "rb") as stream:
    
    # UBX file contains only UBX messages, no NMEA messages (adjust if necessary)
    ubx_reader = UBXReader(stream, ubxonly=False)
    
    # Storage for most recent observations
    data_storage = {}
    
    # Loop over all messages
    for (raw_data, parsed_data) in ubx_reader:
        
        # Check if message holds raw observations (like carrier phases)
        if parsed_data.identity == "RXM-RAWX":
            
            # Check if messgae index is in selected data range
            if key >= start_key and (key-start_key)%step == 0:
            
                raw_msg = parsed_data
                
                # Number of observations
                n_obs = raw_msg.__dict__["numMeas"]
                # GPS week
                week = raw_msg.__dict__["week"]
                # Time of week
                tow = raw_msg.__dict__["rcvTow"]
                # Leap seconds
                # leap_sec = raw_msg.__dict__["leapS"]
                
                # Get timestamp of observation
                gps_time = gpstk.GPSWeekSecond(week,
                                               tow,
                                               gpstk.TimeSystem.GPS)
                time = gps_time.toCommonTime()  # Leap seconds?
                
                # Loop over all observations for this timestamp/message
                for i_obs in range(n_obs):
                
                    # Check if carrier phase measurement is valid
                    cp_valid = raw_msg.__dict__[f"trkStat_{(i_obs+1):02d}"][0] & 0b00000010  
                    # pr_valid = raw_msg.__dict__[f"trkStat_{(i_obs+1):02d}"][0] & 0b00000001
                    if cp_valid:
                    
                        # Code phase [cycles]
                        cp = raw_msg.__dict__[f"cpMes_{(i_obs+1):02d}"]
                        
                        # System
                        gnss = raw_msg.__dict__[f"gnssId_{(i_obs+1):02d}"]
                        # 0: GPS
                        # 1: SBAS
                        # 2: Galileo
                        # 3: BeiDou
                        # 5: QZSS
                        # 6: GLONASS
                        
                        # Map u-blox GNSS index to GPSTk GNSS index
                        gnss = util.gnss_map[gnss]
                        
                        # Satellite index
                        sat = raw_msg.__dict__[f"svId_{(i_obs+1):02d}"]

                         # Map u-blox satellite index to GPSTk satellite object
                        sat = gpstk.SatID(sat, gnss)
                                
                        # Carrier phase standard deviation [0.004 cycles]
                        cp_std_raw = raw_msg.__dict__[f"cpStdev_{(i_obs+1):02d}"][0]
                        cp_std_valid = cp_std_raw < int(0x0F)
                        if not cp_std_valid:
                            print(f"State {key}: Invalid standard deviation for {sat}.")

                        # Check if navigation data is available for satellite
                        sat_valid = ephStore.isPresent(sat)
                        # ...and GNSS shall be used
                        if sat_valid and gnss in GNSS_LIST and cp_std_valid:
                        
                            # Signal type (band)
                            signal_type = raw_msg.__dict__[f"reserved2_{(i_obs+1):02d}"]
                            # https://www.u-blox.com/sites/default/files/ZED-F9P_InterfaceDescription_%28UBX-18010854%29.pdf
                            # 1.5.4 Signal identifiers

                            # Carrier frequency [Hz]
                            carrier_frequency = util.frequency_map[(sat.system, signal_type)]
                            
                            # GLONASS frequency slot                
                            if gnss == gpstk.SatelliteSystem.Glonass:
                                slot = raw_msg.__dict__[f"freqId_{(i_obs+1):02d}"] - 7
                                frequency_step = util.frequency_step_map[signal_type]
                                frequency_offset = frequency_step * slot
                                carrier_frequency += frequency_offset
                                
                            # # Carrier-to-noise density ratio (signal strength) [dB-Hz]
                            # signal_strength = raw_msg.__dict__[f"cno_{(i_obs+1):02d}"]

                            # Carrier phase standard deviation [cycles]
                            cp_std = cp_std_raw * 0.004
                            
                            # Carrier phase locktime counter [s] (maximum 64.5 s)
                            locktime = raw_msg.__dict__[f"locktime_{(i_obs+1):02d}"] * 1e-3
                            
                            # Convert carrier phase observations in cycles to carrier phase
                            # observations in meters
                            wavelength = gpstk.C_MPS / carrier_frequency
                            cp = cp * wavelength
                            
                            # Get satellite position at transmit time
                            # and pseudorange corrected for satellite clock
                            # offset and relativity
                            svp = gpstk.matrix_double(1, 4)
                            sat_vec = gpstk.seqToVector([sat], outtype='vector_SatID')
                            cp_vec = gpstk.seqToVector([cp])
                            ok = raim_solver.PreparePRSolution(
                                Tr=time,
                                Sats=sat_vec,
                                Pseudorange=cp_vec,
                                pEph=ephStore,
                                SVP=svp)
                            # CommonTime Tr, vector_SatID Sats, vector_double Pseudorange, XvtStore_SatID pEph, matrix_double SVP) -> int
                            # SVP = {SV position at transmit time}, raw range + clk + rel
                            
                            cp = (ctypes.c_double).from_address(int(svp(0, 3))).value
                                
                            # Convert GPSTk matrix to ECEF coordinates
                            sat_x = (ctypes.c_double).from_address(int(svp(0, 0))).value
                            sat_y = (ctypes.c_double).from_address(int(svp(0, 1))).value
                            sat_z = (ctypes.c_double).from_address(int(svp(0, 2))).value
                            
                            receiver_x = pos_init_ecef[0]
                            receiver_y = pos_init_ecef[1]
                            receiver_z = pos_init_ecef[2]
                            
                            # rho [s]
                            time_of_flight = np.sqrt((sat_x - receiver_x)**2
                                                      + (sat_y - receiver_y)**2
                                                      + (sat_z - receiver_z)**2) \
                                / gpstk.WGS84Ellipsoid().c()
                            
                            # Correct for Earth rotation        
                            wt = gpstk.WGS84Ellipsoid().angVelocity() * time_of_flight  # [rad]
                            sat_x = np.cos(wt) * sat_x + np.sin(wt) * sat_y
                            sat_y = -np.sin(wt) * sat_x + np.cos(wt) * sat_y
                                    
                            # Add observation to storage
                            # For use as reference later
                            # Each observation is identified by satellite ID,
                            # signal type, and timestamp

                            # Check if GNSS and signal type are not already present
                            if not (gnss, signal_type) in data_storage:
                                # Create empty dictionary for this combination
                                data_storage[(gnss, signal_type)] = {}

                            # Check if satellite is already present
                            if not sat in data_storage[(gnss, signal_type)]:
                                # Create empty queue for this satellite
                                # (list with finite length)
                                data_storage[(gnss, signal_type)][sat] = collections.deque(
                                    maxlen=math.ceil(obs_frequency * max_age / step))
                            
                            # Append observation
                            data_storage[(gnss, signal_type)][sat].append({"key": key,
                                                                           "time": time,
                                                                           "locktime": locktime,
                                                                           "cp": cp,
                                                                           "cp_std": cp_std,
                                                                           "sat_x": sat_x,
                                                                           "sat_y": sat_y,
                                                                           "sat_z": sat_z})

                            # Remove obsolete observations
                            while time - data_storage[(gnss, signal_type)][sat][0]["time"] > locktime:
                                # print(f"Removed obs because locktime is {locktime} s.")
                                del data_storage[(gnss, signal_type)][sat][0]
                
                # For each GNSS and band, find satellites that have been visible for the longest time
                # Among these satellites, find the one with highest elevation or lowest standard deviation
                for gnss, signal_type in data_storage.keys():
                    # # (Only use those where at least 2 signals and no cycle slip)
                    # print(f"GNSS {gnss}, band {signal_type}")
                    # Iterate over all satellites and delete all data that
                    # belongs to satellites that are not visible anymore
                    for sat in list(data_storage[(gnss, signal_type)].keys()):
                        # Check most recent timestamp
                        if data_storage[(gnss, signal_type)][sat][-1]["time"] < time:
                            # Timestamp is old, delete data
                            del data_storage[(gnss, signal_type)][sat]
                    # Select sata for all satellites of this GNSS in this signal band
                    data = data_storage[(gnss, signal_type)]
                    n_sat = len(data)
                    if n_sat > 1:
                        max_locktime = -float("inf")
                        min_std = float("inf")
                        best_sat = None
                        for sat in data.keys():
                            sat_data = data[sat]
                            locktime = len(sat_data)
                            std = np.mean([sat_data_["cp_std"] for sat_data_ in sat_data])
                            if (locktime > max_locktime or (locktime == max_locktime and std < min_std)
                                ):
                                best_sat = sat
                                max_locktime = locktime
                                min_std = std
                            if sat_data[-1]["time"] != time:
                                raise AssertionError("Timestamps does not match.")

                        # Iterate over all satellites except for the best one
                        for sat in data.keys():
                            
                            if sat == best_sat:
                                # Skip satellite
                                continue
                            # Use satellite
                            # Differencing across satellites
                            cp_diff_curr = data[sat][-1]["cp"] - data[best_sat][-1]["cp"]
                            cp_diff_std_curr = np.sqrt(data[sat][-1]["cp_std"]**2 + data[best_sat][-1]["cp_std"]**2)
                            # Differencing across time
                            for idx in range(2, len(data[sat]) + 1):
                                if data[sat][-idx]["time"] != data[best_sat][-idx]["time"]:
                                    print("Timestamps of previous satellite observations do not match.")
                                    continue
                                cp_diff_old = data[sat][-idx]["cp"] - data[best_sat][-idx]["cp"]
                                cp_diff = cp_diff_curr - cp_diff_old
                                # Merge standard devations of current observation
                                # and reference observation
                                cp_diff_std_old = np.sqrt(data[sat][-idx]["cp_std"]**2 + data[best_sat][-idx]["cp_std"]**2)
                                cp_diff_std = np.sqrt(cp_diff_std_curr**2 + cp_diff_std_old**2)

                                if WEIGHT:
                                    # Down-weight older sats based on time diff
                                    dt = data[best_sat][-1]["time"] - data[sat][-idx]["time"]
                                    cp_diff_std = weight_cp(cp_diff_std, dt)
                                    
                                # Create noise model
                                cp_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([cp_diff_std]))
                                if HUBER:
                                    cp_noise = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345), cp_noise)
                                elif DCS:
                                    cp_noise = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.DCS.Create(1.345), cp_noise)
                                
                                # Add time-relative carrierphase factor
                                # https://github.com/borglab/gtsam/blob/develop/python/gtsam/examples/CustomFactorExample.py
                                measurement_sat_pos = np.array([data[best_sat][-idx]["sat_x"],
                                                                data[best_sat][-idx]["sat_y"],
                                                                data[best_sat][-idx]["sat_z"],
                                                                data[sat][-idx]["sat_x"],
                                                                data[sat][-idx]["sat_y"],
                                                                data[sat][-idx]["sat_z"],
                                                                data[best_sat][-1]["sat_x"],
                                                                data[best_sat][-1]["sat_y"],
                                                                data[best_sat][-1]["sat_z"],
                                                                data[sat][-1]["sat_x"],
                                                                data[sat][-1]["sat_y"],
                                                                data[sat][-1]["sat_z"]])
                                measurement = np.concatenate((measurement_sat_pos,
                                                              np.array([cp_diff])))
                                cp_factor = gtsam.CustomFactor(
                                    noiseModel=cp_noise,
                                    keys=[data[sat][-idx]["key"], data[sat][-1]["key"]],
                                    errorFunction=partial(
                                        _error_cp, measurement
                                        )
                                    )
                                graph.add(cp_factor)
                                debug_info.append({"start_key": data[sat][-idx]["key"],
                                                   "end_key": data[sat][-1]["key"],
                                                   "start_time": data[sat][-idx]["time"].getSecondOfDay(),
                                                   "end_time": data[best_sat][-1]["time"].getSecondOfDay(),
                                                   "gnss": gnss,
                                                   "signal": signal_type,
                                                   "sat": sat})
                        
                # Add between factor (motion model), except for x_0
                if key-step >= start_key:
                    motion_factor = gtsam.gtsam.BetweenFactorVector(
                        key - step, key, motion_mean, motion_noise)
                    graph.add(motion_factor)
                
                if not INCREMENTAL:
                    # Add initial value (origin)
                    initial.insert(key, np.zeros(3))

                else:
                    # Add initial value (solution obatined in last valid
                    # optimization step for last state)
                    initial.insert(key, previous_state)
                    # Solve incrementally
                    try:
                        try:
                            isam.update(graph, initial)
                        except MemoryError:
                            print(f"Out of memory. Aborting before key {key}.")
                            break
                        result = isam.calculateEstimate()
                        previous_state = result.atVector(key)

                        # Find carrier-phase factors
                        cp_factor_bool_list = np.array([type(graph.at(idx)) is gtsam.CustomFactor
                                                        for idx in range(graph.nrFactors())])
                        # Find indices of carrier-phase factors
                        cp_factor_idx_list = np.where(cp_factor_bool_list)[0]
                        if len(cp_factor_idx_list) > 0:
                            # Get corresponding residuals
                            residual_list = np.array([graph.at(idx).unwhitenedError(result)[0]
                                                      for idx in cp_factor_idx_list])
                            max_residual = np.max(np.abs(residual_list))
                            print(f"State {key:>4}: maximum residual {max_residual:.2f} m")
                            threshold = 1e9  # Threshold for bad observation
                            if max_residual > threshold:                                

                                # Plot 2D solution
                                x = []
                                y = []
                                h = []
                                for i in range(start_key, key+step, step):
                                    pos_ecef = gpstk.Position(
                                        result.atVector(i)[0] + pos_init_ecef.X(),
                                        result.atVector(i)[1] + pos_init_ecef.Y(),
                                        result.atVector(i)[2] + pos_init_ecef.Z())
                    
                                    g = geod.Inverse(lat0, lon0,
                                                    pos_ecef.getGeodeticLatitude(),
                                                    pos_ecef.getLongitude())
                                    x.append(g["s12"] * np.sin(np.pi * g["azi1"] / 180.0))
                                    y.append(g["s12"] * np.cos(np.pi * g["azi1"] / 180.0))
                                    h.append(pos_ecef.getAltitude() - h0)
                                # Plot reference and track
                                plt.figure(0)
                                plt.clf()
                                plt.grid(True)
                                plt.plot(x, y, "-xr", label="factor-graph solution")
                                plt.xlabel("east [m]")
                                plt.ylabel("north [m]")
                                for i, xi, yi in zip(range(start_key, key+step, step), x, y):
                                    plt.annotate(i, (xi, yi))
                                plt.legend(loc="best")
                                plt.gca().set_aspect("equal", adjustable="datalim")
                                plt.show()

                                debug_info_key = [info for info in debug_info if info["end_key"] == key]
                                bad_idx = np.where(np.abs(residual_list) > threshold)[0]
                                print(f"{len(bad_idx)} residuals above threshold {threshold} m.")
                                for idx in bad_idx:
                                    info_bad = debug_info_key[idx]
                                    print(info_bad)
                                print(f"Large residual!")

                    except RuntimeError as e:
                        print()
                        print(f"ISAM cannot update FG at state {key} because of: {e}")
                        print()

                    # Reset
                    graph = gtsam.NonlinearFactorGraph()
                    initial.clear()
                
            # Go to next pose/state        
            key += 1
            
            # Check if end of selected data range is reached
            if not key < end_key:
                break

end_key = key
            
# print("\nFactor Graph:\n{}".format(graph))

if not INCREMENTAL:
    # Optimize using Levenberg-Marquardt optimization
    params = gtsam.LevenbergMarquardtParams()
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
    print(f"Initial error: {optimizer.error()}")
    print("Optimizing...")
    result = optimizer.optimize()
    print(f"Final error: {optimizer.error()}")
    # print("\nFinal result:\n{}".format(result))

x = []
y = []
h = []

for i in range(start_key, end_key, step):
    
    pos_ecef = gpstk.Position(result.atVector(i)[0] + pos_init_ecef.X(),
                              result.atVector(i)[1] + pos_init_ecef.Y(),
                              result.atVector(i)[2] + pos_init_ecef.Z())
    
    g = geod.Inverse(lat0, lon0,
                      pos_ecef.getGeodeticLatitude(), pos_ecef.getLongitude())
    x.append(g["s12"] * np.sin(np.pi * g["azi1"] / 180.0))
    y.append(g["s12"] * np.cos(np.pi * g["azi1"] / 180.0))
    h.append(pos_ecef.getAltitude() - h0)

# Plot reference and track
plt.figure(1)
plt.clf()
plt.plot(0, 0, "+b", label="reference location", ms=12)
plt.grid(True)
plt.plot(x, y, "-xr", label="factor-graph solution")
plt.xlabel("east [m]")
plt.ylabel("north [m]")
plt.legend(loc="best")
plt.show()

if INCREMENTAL:
    graph = isam.getFactorsUnsafe()

if PLOT_FACTORS:
    # Plot factors
    fig = plt.figure(10)
    plt.clf()
    plt.plot(0, 0, "+b", label="reference location", ms=12)
    plt.grid(True)
    # Find carrier-phase factors factors
    cp_factor_bool_list = np.array([type(graph.at(idx)) is gtsam.CustomFactor
                                    for idx in range(graph.nrFactors())])
    # Find indices of carrier-phase factors
    cp_factor_idx_list = np.where(cp_factor_bool_list)[0]
    # Find start and end nodes for the factors of each GNSS
    start_key_list = [graph.at(idx).keys()[0]
                    for idx in cp_factor_idx_list]
    end_key_list = [graph.at(idx).keys()[1]
                    for idx in cp_factor_idx_list]
    # Plot factors
    cmap = plt.cm.get_cmap('Spectral')
    for start_state, end_state in zip(start_key_list, end_key_list):
        plt.plot([x[int((start_state - start_key) / step)],
                x[int((end_state - start_key) / step)]],
                [y[int((start_state - start_key) / step)],
                y[int((end_state - start_key) / step)]], "-x",
                color=cmap((start_state - start_key) / (end_key - start_key)),
                linewidth=0.5)
    plt.xlabel("east [m]")
    plt.ylabel("north [m]")
    plt.title("binary carrier-phase factors")
    plt.gca().set_aspect("equal", adjustable="datalim")
    plt.show()

plt.figure(2)
plt.clf()
for ecef_idx in range(3):
    plt.subplot(1, 3, ecef_idx + 1)
    plt.grid(True)
    plt.xlabel("variable key")
    plt.ylabel(f"{['x', 'y', 'z'][ecef_idx]} [m]")
    plt.plot([key for key in range(start_key, end_key, step)],
             [result.atVector(key)[ecef_idx] for key in range(start_key, end_key, step)],
             "-x")
    plt.title(['ECEF - x', 'ECEF - y', 'ECEF - z'][ecef_idx])
plt.show()

# Plot residuals
plt.figure(3)
plt.clf()
# Find carrier-phase factors factors
cp_factor_bool_list = np.array([type(graph.at(idx)) is gtsam.CustomFactor
                                for idx in range(graph.nrFactors())])
# Find indices of carrier-phase factors
cp_factor_idx_list = np.where(cp_factor_bool_list)[0]
if len(debug_info) == 0:
    # Find GNSSs of carrier-pase factors
    gnss_selector_list = np.array([np.where(graph.at(idx).linearize(result).jacobian()[0][0][3:(3+len(GNSS_LIST))])[0][0]
                                for idx in cp_factor_idx_list])
    # Find start and end nodes for the factors of each GNSS
    start_key_list = [np.array([graph.at(cp_factor_idx_list[idx]).keys()[0]
                                for idx in np.where(np.array(gnss_selector_list)==gnss_idx)[0]])
                    for gnss_idx in range(len(GNSS_LIST))]
    end_key_list = [np.array([graph.at(cp_factor_idx_list[idx]).keys()[1]
                            for idx in np.where(np.array(gnss_selector_list)==gnss_idx)[0]])
                    for gnss_idx in range(len(GNSS_LIST))]
    # Get corresponding residuals
    residual_list = [np.array([graph.at(cp_factor_idx_list[idx]).unwhitenedError(result)[0]
                            for idx in np.where(np.array(gnss_selector_list)==gnss_idx)[0]])
                    for gnss_idx in range(len(GNSS_LIST))]
else:
    # Find GNSSs of carrier-pase factors
    gnss_selector_list = np.array([info["gnss"]
                                   for info in debug_info if info["end_key"] < end_key])
    # Find start and end nodes for the factors of each GNSS
    start_key_list = [np.array([graph.at(cp_factor_idx_list[idx]).keys()[0]
                                for idx in np.where(gnss_selector_list==gnss)[0]])
                    for gnss in GNSS_LIST]
    end_key_list = [np.array([graph.at(cp_factor_idx_list[idx]).keys()[1]
                            for idx in np.where(gnss_selector_list==gnss)[0]])
                    for gnss in GNSS_LIST]
    # Get corresponding residuals
    residual_list = [np.array([graph.at(cp_factor_idx_list[idx]).unwhitenedError(result)[0]
                            for idx in np.where(gnss_selector_list==gnss)[0]])
                    for gnss in GNSS_LIST]
    # Find problematic satellites
    threshold = 1.0  # Threshold for bad observation
    info_list = [[info for info in debug_info if info["gnss"] == gnss]
                for gnss in GNSS_LIST]
    bad_cp = [[info[bad_idx] for bad_idx in np.where(res > threshold)[0]] for res, info in zip(residual_list, info_list)]
    bad_res_cp = [[res[bad_idx] for bad_idx in np.where(res > threshold)[0]] for res, info in zip(residual_list, info_list)]
    for i_gnss, r_gnss in zip(bad_cp, bad_res_cp):
        print()
        for i, r in zip(i_gnss, r_gnss):
            print(f"Large cp residual between x_{i['start_key']} ({i['start_time']:.1f} s) and x_{i['end_key']} ({i['end_time']:.1f} s) for {i['sat']}, {util.band_map[(i['gnss'], i['signal'])].name}: {r:.1f} m")
# Plot residuals for each GNSS
for gnss_idx in range(len(GNSS_LIST)):
    plt.scatter(start_key_list[gnss_idx], 
                residual_list[gnss_idx],
                marker="+", label=gpstk.SatelliteSystem(GNSS_LIST[gnss_idx]).name)
plt.title("residuals (start key)")
plt.ylabel("unwhitened error [m]")
plt.xlabel("variable key")
plt.grid()
plt.legend()
plt.show()
# 3D plot
# ax = plt.figure(4).add_subplot(projection='3d')
# for gnss_idx in range(len(GNSS_LIST)):
#     n = len(end_key_list[gnss_idx])
#     ax.scatter(xs=start_key_list[gnss_idx],
#             ys=end_key_list[gnss_idx],
#             zs=residual_list[gnss_idx],
#             label=gpstk.SatelliteSystem(GNSS_LIST[gnss_idx]).name)
# plt.title("residuals")
# ax.set_zlabel("unwhitened error [m]")
# ax.set_xlabel("start key")
# ax.set_ylabel("end key")
# plt.legend()
# plt.show()

# Plot residuals (sorted by end key)
plt.figure(5)
plt.clf()
# Plot residuals for each GNSS
for gnss_idx in range(len(GNSS_LIST)):
    plt.scatter(end_key_list[gnss_idx], 
                residual_list[gnss_idx],
                marker="+", label=gpstk.SatelliteSystem(GNSS_LIST[gnss_idx]).name)
plt.title("residuals (end key)")
plt.ylabel("unwhitened error [m]")
plt.xlabel("variable key")
plt.grid()
plt.legend()
plt.show()

# Plot as HTML map
map_name = obs_file[:-4] + "-DD-CP"
util.plot_map(result, lat0, lon0, pos_init_ecef, name=map_name)
print(f"Saved map as {map_name}.")

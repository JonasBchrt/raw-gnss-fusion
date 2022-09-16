#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Oct  1 10:03:34 2021

@author: jonasbeuchert
"""
from pyubx2 import UBXReader
from pyubx2.exceptions import UBXParseError
import gpstk
import os
import folium


# Map u-blox GNSS indices to GPSTk GNSS index
# https://www.u-blox.com/sites/default/files/ZED-F9P_InterfaceDescription_%28UBX-18010854%29.pdf
# 1.5.2 GNSS identifiers
gnss_map = {0: gpstk.SatelliteSystem.GPS,
            1: gpstk.SatelliteSystem.Geosync,
            2: gpstk.SatelliteSystem.Galileo,
            3: gpstk.SatelliteSystem.BeiDou,
            5: gpstk.SatelliteSystem.QZSS,
            6: gpstk.SatelliteSystem.Glonass}

# Map u-blox signal identifiers to GPSTk
# https://www.u-blox.com/sites/default/files/ZED-F9P_InterfaceDescription_%28UBX-18010854%29.pdf
# 1.5.4 Signal identifiers
frequency_map = {(gpstk.SatelliteSystem.GPS, 0): gpstk.FREQ_GPS_L1,
                 (gpstk.SatelliteSystem.GPS, 3): gpstk.FREQ_GPS_L2,
                 (gpstk.SatelliteSystem.GPS, 4): gpstk.FREQ_GPS_L2,
                 (gpstk.SatelliteSystem.Geosync, 0): gpstk.FREQ_SBAS_L1,
                 (gpstk.SatelliteSystem.Galileo, 0): gpstk.FREQ_GALILEO_E1,
                 (gpstk.SatelliteSystem.Galileo, 1): gpstk.FREQ_GALILEO_E1,
                 (gpstk.SatelliteSystem.Galileo, 5): gpstk.FREQ_GALILEO_E5b,
                 (gpstk.SatelliteSystem.Galileo, 6): gpstk.FREQ_GALILEO_E5b,
                 (gpstk.SatelliteSystem.BeiDou, 0): gpstk.FREQ_BEIDOU_B1,
                 (gpstk.SatelliteSystem.BeiDou, 1): gpstk.FREQ_BEIDOU_B1,
                 (gpstk.SatelliteSystem.BeiDou, 2): 1207.14e6,
                 (gpstk.SatelliteSystem.BeiDou, 3): 1207.14e6,
                 (gpstk.SatelliteSystem.QZSS, 0): gpstk.FREQ_QZSS_L1,
                 (gpstk.SatelliteSystem.QZSS, 1): gpstk.FREQ_QZSS_L1,
                 (gpstk.SatelliteSystem.QZSS, 4): gpstk.FREQ_QZSS_L2,
                 (gpstk.SatelliteSystem.QZSS, 5): gpstk.FREQ_QZSS_L2,
                 (gpstk.SatelliteSystem.Glonass, 0): gpstk.FREQ_GLONASS_G1,
                 (gpstk.SatelliteSystem.Glonass, 2): gpstk.FREQ_GLONASS_G2
                 }

# Map u-blox GLONASS frequency step identifiers to GPSTk
frequency_step_map = {0: gpstk.L1_FREQ_STEP_GLO,
                      2: gpstk.L2_FREQ_STEP_GLO}

# Map u-blox signal band identifiers to GPSTk
band_map = {(gpstk.SatelliteSystem.GPS, 0): gpstk.CarrierBand.L1,
            (gpstk.SatelliteSystem.GPS, 3): gpstk.CarrierBand.L2,
            (gpstk.SatelliteSystem.GPS, 4): gpstk.CarrierBand.L2,
            (gpstk.SatelliteSystem.Geosync, 0): gpstk.CarrierBand.L1,
            (gpstk.SatelliteSystem.Galileo, 0): gpstk.CarrierBand.L1,
            (gpstk.SatelliteSystem.Galileo, 1): gpstk.CarrierBand.L1,
            (gpstk.SatelliteSystem.Galileo, 5): gpstk.CarrierBand.E5b,
            (gpstk.SatelliteSystem.Galileo, 6): gpstk.CarrierBand.E5b,
            (gpstk.SatelliteSystem.BeiDou, 0): gpstk.CarrierBand.B1,
            (gpstk.SatelliteSystem.BeiDou, 1): gpstk.CarrierBand.B1,
            (gpstk.SatelliteSystem.BeiDou, 2): gpstk.CarrierBand.B2,
            (gpstk.SatelliteSystem.BeiDou, 3): gpstk.CarrierBand.B2,
            (gpstk.SatelliteSystem.QZSS, 0): gpstk.CarrierBand.L1,
            (gpstk.SatelliteSystem.QZSS, 1): gpstk.CarrierBand.L1,
            (gpstk.SatelliteSystem.QZSS, 4): gpstk.CarrierBand.L2,
            (gpstk.SatelliteSystem.QZSS, 5): gpstk.CarrierBand.L2,
            (gpstk.SatelliteSystem.Glonass, 0): gpstk.CarrierBand.G1,
            (gpstk.SatelliteSystem.Glonass, 2): gpstk.CarrierBand.G2
            }


def read_lat_lon_height_key(file_path, key=0):
    """
    Read .ubx file and return latitude, longitude of reference solution.
    
    Parameters
    ----------
    file_path : str
        Path to .ubx file.
    key : int, optional
        Index of location to return. The default is 0.

    Returns
    -------
    latitude : float
        Latitude [decimal degrees]
    longitude : float
        Latitude [decimal degrees]
    height : float
        Height above ellipsoid [m]

    Author: Jonas Beuchert
    """
    # Loop counter
    idx = -1   
    # Read geodetic coordinates from file
    with open(file_path, "rb") as stream:
        ubr = UBXReader(stream, ubxonly=False)
        while idx < key:
            try:
                (raw_data, parsed_data) = next(ubr)
                if parsed_data.identity == "NAV-PVT":
                    idx += 1
            except StopIteration:
                raise ValueError(f"Could not find latitude/lomgitude for key {key} in {file_path}.")
            except UBXParseError as msg:
                print(msg)
    # Convert to degrees
    lat = parsed_data.lat * 1e-7
    lon = parsed_data.lon * 1e-7
    height = parsed_data.height * 1e-3
    return lat, lon, height


def read_nav(civil_time, nav_folder="", provider="IGS"):
    """
    Read navigation data from existing RINEX file or download it.

    Parameters
    ----------
    civil_time : gpstk.CivilTime
        Coarse time.
    nav_folder : str, optional
        Path to directory with navigation data files. The default is
        'navigation_data'.
    provider : str, optional
        'IGS' or 'MGEX'. The default is 'IGS'.

    Returns
    -------
    eph_store : gpstk.Rinex3EphemerisStore
        Ephemerides.
    nav_header : gpstk.Rinex3NavHeader
        Navigation data header without ionospheric corrections and time system
        corrections.

    Author: Jonas Beuchert
    """
    import requests
    import gzip
    import re


    if nav_folder == "":
        nav_folder = "navigation_data"

    if provider not in ["IGS", "MGEX"]:
        raise ValueError(f"Provider must be 'IGS' or 'MGEX', but is {provider}.")

    # Determine file name
    day_of_year = gpstk.YDSTime(civil_time).doy
    if provider == "IGS":
        file_name_start = "BRDM00DLR"
    elif provider == "MGEX":
        file_name_start = "BRDC00WRD"
    nav_file = f"{file_name_start}_S_{civil_time.year:04d}{day_of_year:03d}0000_01D_MN.rnx"

    # Determine file path
    nav_path = os.path.join(nav_folder, nav_file)

    try:
        # Read file
        nav_header, nav_data = gpstk.readRinex3Nav(nav_path)
    except OSError:
        # Download link for broadcasted ephemerides
        url = f"https://igs.bkg.bund.de/root_ftp/IGS/BRDC/{civil_time.year:04d}/{day_of_year:03d}/{nav_file}.gz"
        # Download
        response = requests.get(url)
        # Uncompress
        bytes = gzip.decompress(response.content)
        txt = str(bytes.decode())
        # Clean up file to be processable by gpstk.readRinex3Nav()
        for keyword_0_list, keyword_1 in zip([[""], ["BDSA", "BDSB", "QZSA", "QZSB", "IRNA", "IRNB"], [""]], ["MERGED_FILE", "IONOSPHERIC CORR", "TIME SYSTEM CORR"]):
            for keyword_0 in keyword_0_list:
                txt = re.sub(pattern=f"\n{keyword_0}(.*){keyword_1}( *)", repl="", string=txt)
        # Write to file
        with open(nav_path, 'w') as f_out:
            f_out.write(txt)
        # Read file
        nav_header, nav_data = gpstk.readRinex3Nav(nav_path)

    # Store ephemerides
    eph_store = gpstk.gpstk.Rinex3EphemerisStore()
    for nav_data_obj in nav_data:
        try:
            eph_store.addEphemeris(nav_data_obj)
        except gpstk.InvalidParameter:
            print("Could not add ephemeris because of invalid value.")

    return eph_store, nav_header


def plot_map(result, lat_ref, lon_ref, pos_init_ecef, name="temp",
             terrain=False):
    # Create map
    m = folium.Map(
        location=[lat_ref, lon_ref],
        tiles="OpenStreetMap" if not terrain else "http://tile.stamen.com/terrain-background/{z}/{x}/{y}.png",
        attr=' ',
        control_scale=True,
        zoom_control=False,
        zoom_start=18
    )
    # Assemble estimated path
    pos_ecef_list = [gpstk.Position(result.atVector(i)[0] + pos_init_ecef[0],
                                    result.atVector(i)[1] + pos_init_ecef[1],
                                    result.atVector(i)[2] + pos_init_ecef[2])
                    for i in result.keys()]
    path = [(pos_ecef.getGeodeticLatitude(),
                -360.0 + pos_ecef.getLongitude())
            for pos_ecef in pos_ecef_list]
    # Draw path
    folium.PolyLine(path, weight=3, color="red").add_to(m)
    # Draw reference point
    folium.Marker([lat_ref, lon_ref]).add_to(m)
    # Save map
    m.save(f"{name}.html")
    try:
        display(m)
    except Exception as e:
        print("To display the map, run the script with IPython/jupyter.")
    
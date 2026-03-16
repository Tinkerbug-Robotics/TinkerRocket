"""ISA (International Standard Atmosphere) model.

Matches the atmosphere model used in TR_Sensor_Collector_Sim.
"""
import math


# Sea level standard values
P0 = 101325.0   # Pa
T0 = 288.15     # K (15 C)
RHO0 = 1.225    # kg/m^3
LAPSE_RATE = 0.0065  # K/m (troposphere)
G0 = 9.80665    # m/s^2
R_AIR = 287.058  # J/(kg*K)


def air_density(altitude_m: float) -> float:
    """Air density at altitude using ISA exponential model.

    Matches the model in TR_Sensor_Collector_Sim:
        rho = 1.225 * (1 - 2.2558e-5 * alt)^4.2559

    Args:
        altitude_m: Altitude above sea level in meters.

    Returns:
        Air density in kg/m^3.
    """
    x = 1.0 - 2.2558e-5 * altitude_m
    if x <= 0:
        return 0.0
    return RHO0 * x ** 4.2559


def pressure(altitude_m: float, ground_pressure_pa: float = P0) -> float:
    """Atmospheric pressure at altitude.

    Matches TR_Sensor_Collector_Sim:
        P = P0 * (1 - alt/44330)^5.255

    Args:
        altitude_m: Altitude above sea level in meters.
        ground_pressure_pa: Ground-level pressure in Pa.

    Returns:
        Pressure in Pa.
    """
    x = 1.0 - altitude_m / 44330.0
    if x <= 0:
        return 0.0
    return ground_pressure_pa * x ** 5.255


def temperature(altitude_m: float) -> float:
    """Temperature at altitude using ISA lapse rate.

    Args:
        altitude_m: Altitude above sea level in meters.

    Returns:
        Temperature in Kelvin.
    """
    # Troposphere only (up to ~11 km)
    return T0 - LAPSE_RATE * altitude_m


def speed_of_sound(altitude_m: float) -> float:
    """Speed of sound at altitude.

    Args:
        altitude_m: Altitude in meters.

    Returns:
        Speed of sound in m/s.
    """
    T = temperature(altitude_m)
    if T <= 0:
        return 0.0
    return math.sqrt(1.4 * R_AIR * T)


def dynamic_pressure(airspeed: float, altitude_m: float) -> float:
    """Dynamic pressure q = 0.5 * rho * V^2.

    Args:
        airspeed: m/s
        altitude_m: m

    Returns:
        Dynamic pressure in Pa.
    """
    rho = air_density(altitude_m)
    return 0.5 * rho * airspeed ** 2

"""Range and distance equation helpers."""

from __future__ import annotations

import math


def sodano_reduced_latitude(
    geodetic_latitude_radians: float,
    equatorial_radius: float,
    polar_radius: float,
) -> float:
    """Return the Sodano reduced latitude."""

    return math.atan2(
        polar_radius * math.sin(geodetic_latitude_radians),
        equatorial_radius * math.cos(geodetic_latitude_radians),
    )
####


def sodano_central_angle(
    beta1_radians: float,
    beta2_radians: float,
    longitude_difference_radians: float,
) -> float:
    """Return the Sodano auxiliary central angle."""

    cosine_phi = (
        math.sin(beta1_radians) * math.sin(beta2_radians)
        + math.cos(beta1_radians)
        * math.cos(beta2_radians)
        * math.cos(longitude_difference_radians)
    )
    return math.acos(max(-1.0, min(1.0, cosine_phi)))
####


def sodano_inverse_m(
    beta1_radians: float,
    beta2_radians: float,
    phi_radians: float,
    longitude_difference_radians: float,
) -> float:
    """Return the Sodano inverse auxiliary m."""

    sine_phi = math.sin(phi_radians)
    if math.isclose(sine_phi, 0.0, abs_tol=1e-15):
        return 0.0
    ####
    return 1.0 - (
        math.cos(beta1_radians) ** 2
        * math.cos(beta2_radians) ** 2
        * math.sin(longitude_difference_radians) ** 2
        / (sine_phi * sine_phi)
    )
####


def sodano_inverse_distance(
    equatorial_radius: float,
    polar_radius: float,
    flattening: float,
    beta1_radians: float,
    beta2_radians: float,
    phi_radians: float,
    inverse_m: float,
) -> float:
    """Return the Sodano inverse ellipsoidal surface distance."""

    sine_phi = math.sin(phi_radians)
    cosine_phi = math.cos(phi_radians)
    cosecant_phi = 0.0 if math.isclose(sine_phi, 0.0, abs_tol=1e-15) else 1.0 / sine_phi
    cotangent_phi = 0.0 if math.isclose(sine_phi, 0.0, abs_tol=1e-15) else cosine_phi / sine_phi
    sine_beta1 = math.sin(beta1_radians)
    sine_beta2 = math.sin(beta2_radians)
    flattening_squared = flattening * flattening
    term1 = (1.0 + flattening + flattening_squared) * phi_radians
    term2 = sine_beta1 * sine_beta2 * (
        (flattening + flattening_squared) * sine_phi
        - 0.5 * flattening_squared * phi_radians * phi_radians * cosecant_phi
    )
    term3 = -0.5 * inverse_m * (
        (flattening + flattening_squared)
        * (phi_radians + sine_phi * cosine_phi)
        - flattening_squared * phi_radians * phi_radians * cotangent_phi
    )
    term4 = -0.5 * sine_beta1 * sine_beta1 * sine_beta2 * sine_beta2 * flattening_squared * sine_phi * cosine_phi
    term5 = 0.5 * sine_beta1 * sine_beta2 * inverse_m * flattening_squared * (
        sine_phi * cosine_phi * cosine_phi + phi_radians * phi_radians * cosecant_phi
    )
    term6 = 0.0625 * flattening_squared * inverse_m * inverse_m * (
        phi_radians
        + sine_phi * cosine_phi
        - 2.0 * sine_phi * cosine_phi**3
        - 8.0 * phi_radians * phi_radians * cotangent_phi
    )
    _ = equatorial_radius, polar_radius
    return polar_radius * (term1 + term2 + term3 + term4 + term5 + term6)
####


def sodano_forward_azimuth(
    beta1_radians: float,
    beta2_radians: float,
    corrected_longitude_difference_radians: float,
    delta_beta_radians: float,
) -> float:
    """Return the Sodano forward azimuth."""

    numerator = math.cos(beta2_radians) * math.sin(corrected_longitude_difference_radians)
    denominator = math.sin(delta_beta_radians) + 2.0 * math.cos(beta2_radians) * math.sin(beta1_radians) * math.sin(
        corrected_longitude_difference_radians / 2.0
    ) ** 2
    return math.atan2(numerator, denominator)
####


def sodano_backward_azimuth(
    beta1_radians: float,
    beta2_radians: float,
    corrected_longitude_difference_radians: float,
    delta_beta_radians: float,
) -> float:
    """Return the Sodano backward azimuth."""

    numerator = math.cos(beta1_radians) * math.sin(corrected_longitude_difference_radians)
    denominator = math.sin(delta_beta_radians) - 2.0 * math.cos(beta1_radians) * math.sin(beta2_radians) * math.sin(
        corrected_longitude_difference_radians / 2.0
    ) ** 2
    return math.atan2(numerator, denominator)
####


def sodano_corrected_longitude_difference(
    beta1_radians: float,
    beta2_radians: float,
    flattening: float,
    phi_radians: float,
    inverse_m: float,
    longitude_difference_radians: float,
) -> float:
    """Return the corrected longitude difference for Sodano's inverse method."""

    sine_phi = math.sin(phi_radians)
    cosine_phi = math.cos(phi_radians)
    sine_beta1 = math.sin(beta1_radians)
    sine_beta2 = math.sin(beta2_radians)
    cosine_beta1 = math.cos(beta1_radians)
    cosine_beta2 = math.cos(beta2_radians)
    flattening_squared = flattening * flattening
    if math.isclose(sine_phi, 0.0, abs_tol=1e-15):
        return longitude_difference_radians
    ####
    correction = (
        (cosine_beta1 * cosine_beta2 * math.sin(longitude_difference_radians) / sine_phi)
        * (
            (flattening + flattening_squared) * phi_radians
            - 0.5
            * sine_beta1
            * sine_beta2
            * flattening_squared
            * (sine_phi + 2.0 * phi_radians * phi_radians / sine_phi)
            + 0.25
            * inverse_m
            * flattening_squared
            * (sine_phi * cosine_phi - 5.0 * phi_radians + 4.0 * phi_radians * phi_radians * cosine_phi / sine_phi)
        )
    )
    return longitude_difference_radians + correction
####


def sodano_delta_beta_series(
    reference_latitude_radians: float,
    target_latitude_radians: float,
    third_flattening: float,
) -> float:
    """Return the reduced-latitude difference series."""

    return (
        target_latitude_radians
        - reference_latitude_radians
        + third_flattening * (
            math.sin(2.0 * reference_latitude_radians) - math.sin(2.0 * target_latitude_radians)
        )
        - 0.5
        * third_flattening
        * third_flattening
        * (
            math.sin(4.0 * reference_latitude_radians) - math.sin(4.0 * target_latitude_radians)
        )
        + (third_flattening**3 / 3.0)
        * (
            math.sin(6.0 * reference_latitude_radians) - math.sin(6.0 * target_latitude_radians)
        )
    )
####


def sodano_delta_beta_alternative(
    reference_latitude_radians: float,
    target_latitude_radians: float,
    beta1_radians: float,
    beta2_radians: float,
    third_flattening: float,
) -> float:
    """Return the alternative reduced-latitude difference relation."""

    sine_delta = math.sin(target_latitude_radians - reference_latitude_radians)
    return (
        target_latitude_radians
        - reference_latitude_radians
        + 2.0
        * sine_delta
        * (
            math.sin(beta1_radians) * math.sin(beta2_radians) * (third_flattening + third_flattening**2 + third_flattening**3)
            - math.cos(beta1_radians) * math.cos(beta2_radians) * (third_flattening - third_flattening**2 + third_flattening**3)
        )
    )
####


def sodano_third_flattening(equatorial_radius: float, polar_radius: float) -> float:
    """Return the third flattening parameter n."""

    return (equatorial_radius - polar_radius) / (equatorial_radius + polar_radius)
####


def downrange_crossrange_perpendicular_error(
    backward_azimuth_radians: float,
    forward_azimuth_radians: float,
) -> float:
    """Return the perpendicularity error used in the downrange search."""

    return backward_azimuth_radians - forward_azimuth_radians - math.pi / 2.0
####


def downrange_newton_raphson_update(
    current_value: float,
    function_value: float,
    derivative_value: float,
) -> float:
    """Return the generic Newton-Raphson update."""

    return current_value - function_value / derivative_value
####


def downrange_error_function(
    backward_azimuth_radians: float,
    forward_azimuth_radians: float,
) -> float:
    """Return the downrange error function."""

    return math.pi / 2.0 - abs(backward_azimuth_radians - forward_azimuth_radians)
####


def sodano_direct_xi(
    surface_distance: float,
    polar_radius: float,
) -> float:
    """Return the normalized surface distance xi."""

    return surface_distance / polar_radius
####


def sodano_eprime(eccentricity: float) -> float:
    """Return the Sodano eccentricity factor e prime."""

    return 1.0 / math.sqrt(1.0 - eccentricity * eccentricity)
####


def sodano_direct_m(
    beta1_radians: float,
    azimuth_radians: float,
    eccentricity_prime: float,
) -> float:
    """Return the direct-method auxiliary m."""

    sine_beta1 = math.sin(beta1_radians)
    cosine_beta1 = math.cos(beta1_radians)
    sine_azimuth = math.sin(azimuth_radians)
    return 0.5 * (1.0 + 0.5 * eccentricity_prime * eccentricity_prime * sine_beta1 * sine_beta1) * (
        1.0 - cosine_beta1 * cosine_beta1 * sine_azimuth * sine_azimuth
    )
####


def sodano_direct_n(
    beta1_radians: float,
    azimuth_radians: float,
    xi_radians: float,
    eccentricity_prime: float,
) -> float:
    """Return the direct-method auxiliary n."""

    sine_beta1 = math.sin(beta1_radians)
    cosine_beta1 = math.cos(beta1_radians)
    sine_xi = math.sin(xi_radians)
    cosine_xi = math.cos(xi_radians)
    cosine_azimuth = math.cos(azimuth_radians)
    return 0.5 * (1.0 + 0.5 * eccentricity_prime * eccentricity_prime * sine_beta1 * sine_beta1) * (
        sine_beta1 * sine_beta1 * cosine_xi
        + cosine_beta1 * cosine_azimuth * sine_beta1 * sine_xi
    )
####


def sodano_direct_phi(
    xi_radians: float,
    m_value: float,
    n_value: float,
    eccentricity_prime: float,
) -> float:
    """Return the Sodano direct angular-distance correction."""

    sine_xi = math.sin(xi_radians)
    cosine_xi = math.cos(xi_radians)
    eprime_squared = eccentricity_prime * eccentricity_prime
    eprime_fourth = eprime_squared * eprime_squared
    return (
        xi_radians
        - n_value * eprime_squared * sine_xi
        + 0.5 * m_value * eprime_squared * (sine_xi * cosine_xi - xi_radians)
        + 2.5 * n_value * n_value * eprime_fourth * sine_xi * cosine_xi
        + 0.0625
        * m_value
        * m_value
        * eprime_fourth
        * (
            11.0 * xi_radians
            - 13.0 * sine_xi * cosine_xi
            - 8.0 * xi_radians * cosine_xi * cosine_xi
            + 10.0 * sine_xi * cosine_xi**3
        )
        + 0.5
        * m_value
        * n_value
        * eprime_fourth
        * (
            3.0 * sine_xi
            + 2.0 * xi_radians * cosine_xi
            - 5.0 * sine_xi * cosine_xi * cosine_xi
        )
    )
####


def sodano_direct_sin_beta2(
    beta1_radians: float,
    azimuth_radians: float,
    phi_radians: float,
) -> float:
    """Return the Sodano direct-method sine of reduced latitude beta2."""

    sine_beta1 = math.sin(beta1_radians)
    cosine_beta1 = math.cos(beta1_radians)
    return sine_beta1 * math.cos(phi_radians) + cosine_beta1 * math.cos(azimuth_radians) * math.sin(phi_radians)
####


def sodano_direct_cos_beta2(
    beta1_radians: float,
    azimuth_radians: float,
    phi_radians: float,
) -> float:
    """Return the Sodano direct-method cosine of reduced latitude beta2."""

    sine_beta1 = math.sin(beta1_radians)
    cosine_beta1 = math.cos(beta1_radians)
    sine_azimuth = math.sin(azimuth_radians)
    cosine_azimuth = math.cos(azimuth_radians)
    sine_phi = math.sin(phi_radians)
    cosine_phi = math.cos(phi_radians)
    return math.sqrt(
        cosine_beta1 * cosine_beta1 * sine_azimuth * sine_azimuth
        + (sine_beta1 * sine_phi - cosine_beta1 * cosine_azimuth * cosine_phi) ** 2
    )
####


def sodano_direct_latitude(
    beta2_radians: float,
    equatorial_radius: float,
    polar_radius: float,
) -> float:
    """Return the Sodano direct-method geodetic latitude."""

    return math.atan2(equatorial_radius * math.sin(beta2_radians), polar_radius * math.cos(beta2_radians))
####


def sodano_direct_l_correction(
    beta1_radians: float,
    azimuth_radians: float,
    phi_radians: float,
    xi_radians: float,
    flattening: float,
    m_value: float,
    n_value: float,
) -> float:
    """Return the direct-method longitude correction l."""

    sine_xi = math.sin(xi_radians)
    cosine_xi = math.cos(xi_radians)
    cosine_beta1 = math.cos(beta1_radians)
    sine_azimuth = math.sin(azimuth_radians)
    flattening_squared = flattening * flattening
    return cosine_beta1 * sine_azimuth * (
        -flattening * xi_radians
        + 3.0 * flattening_squared * n_value * sine_xi
        + 1.5 * flattening_squared * m_value * (xi_radians - sine_xi * cosine_xi)
    )
####


def sodano_direct_longitude(
    reference_longitude_radians: float,
    beta1_radians: float,
    azimuth_radians: float,
    phi_radians: float,
    l_correction_radians: float,
    beta2_radians: float,
) -> float:
    """Return the Sodano direct-method longitude."""

    numerator = math.sin(phi_radians) * math.sin(azimuth_radians)
    denominator = math.cos(beta1_radians) * math.cos(phi_radians) - math.sin(beta1_radians) * math.sin(phi_radians) * math.cos(azimuth_radians)
    _ = beta2_radians
    return reference_longitude_radians + l_correction_radians + math.atan2(numerator, denominator)
####


def sodano_direct_backward_azimuth(
    beta1_radians: float,
    azimuth_radians: float,
    phi_radians: float,
) -> float:
    """Return the Sodano direct-method backward azimuth."""

    numerator = -math.cos(beta1_radians) * math.sin(azimuth_radians)
    denominator = math.sin(beta1_radians) * math.sin(phi_radians) - math.cos(beta1_radians) * math.cos(azimuth_radians) * math.cos(phi_radians)
    return math.atan2(numerator, denominator)
####

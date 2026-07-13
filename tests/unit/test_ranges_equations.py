from __future__ import annotations

import math

import pytest

from taoryx.equations import (
    downrange_crossrange_perpendicular_error,
    downrange_error_function,
    downrange_newton_raphson_update,
    sodano_backward_azimuth,
    sodano_central_angle,
    sodano_corrected_longitude_difference,
    sodano_delta_beta_alternative,
    sodano_delta_beta_series,
    sodano_direct_backward_azimuth,
    sodano_direct_cos_beta2,
    sodano_direct_l_correction,
    sodano_direct_latitude,
    sodano_direct_longitude,
    sodano_direct_m,
    sodano_direct_n,
    sodano_direct_phi,
    sodano_direct_sin_beta2,
    sodano_direct_xi,
    sodano_eprime,
    sodano_forward_azimuth,
    sodano_inverse_distance,
    sodano_inverse_m,
    sodano_reduced_latitude,
    sodano_third_flattening,
)


def test_sodano_inverse_helpers_follow_the_manual_formulas() -> None:
    equatorial_radius = 6_378_137.0
    polar_radius = 6_356_752.314245179
    flattening = (equatorial_radius - polar_radius) / equatorial_radius
    longitude_1 = math.radians(-73.0)
    longitude_2 = math.radians(-72.5)
    latitude_1 = math.radians(34.0)
    latitude_2 = math.radians(34.5)

    beta1 = sodano_reduced_latitude(latitude_1, equatorial_radius, polar_radius)
    beta2 = sodano_reduced_latitude(latitude_2, equatorial_radius, polar_radius)
    delta_longitude = longitude_2 - longitude_1
    phi = sodano_central_angle(beta1, beta2, delta_longitude)
    inverse_m = sodano_inverse_m(beta1, beta2, phi, delta_longitude)
    delta_beta = sodano_delta_beta_series(latitude_1, latitude_2, sodano_third_flattening(equatorial_radius, polar_radius))
    corrected_longitude = sodano_corrected_longitude_difference(
        beta1,
        beta2,
        flattening,
        phi,
        inverse_m,
        delta_longitude,
    )

    expected_beta1 = math.atan2(polar_radius * math.sin(latitude_1), equatorial_radius * math.cos(latitude_1))
    expected_beta2 = math.atan2(polar_radius * math.sin(latitude_2), equatorial_radius * math.cos(latitude_2))
    expected_phi = math.acos(
        math.sin(expected_beta1) * math.sin(expected_beta2)
        + math.cos(expected_beta1) * math.cos(expected_beta2) * math.cos(delta_longitude)
    )
    expected_inverse_m = 1.0 - (
        math.cos(expected_beta1) ** 2
        * math.cos(expected_beta2) ** 2
        * math.sin(delta_longitude) ** 2
        / math.sin(expected_phi) ** 2
    )
    expected_distance = polar_radius * (
        (1.0 + flattening + flattening * flattening) * expected_phi
        + math.sin(expected_beta1)
        * math.sin(expected_beta2)
        * (
            (flattening + flattening * flattening) * math.sin(expected_phi)
            - 0.5 * flattening * flattening * expected_phi * expected_phi / math.sin(expected_phi)
        )
        - 0.5
        * expected_inverse_m
        * (
            (flattening + flattening * flattening)
            * (expected_phi + math.sin(expected_phi) * math.cos(expected_phi))
            - flattening * flattening * expected_phi * expected_phi * math.cos(expected_phi) / math.sin(expected_phi)
        )
        - 0.5
        * math.sin(expected_beta1) ** 2
        * math.sin(expected_beta2) ** 2
        * flattening
        * flattening
        * math.sin(expected_phi)
        * math.cos(expected_phi)
        + 0.5
        * math.sin(expected_beta1)
        * math.sin(expected_beta2)
        * expected_inverse_m
        * flattening
        * flattening
        * (math.sin(expected_phi) * math.cos(expected_phi) ** 2 + expected_phi * expected_phi / math.sin(expected_phi))
        + 0.0625
        * flattening
        * flattening
        * expected_inverse_m
        * expected_inverse_m
        * (
            expected_phi
            + math.sin(expected_phi) * math.cos(expected_phi)
            - 2.0 * math.sin(expected_phi) * math.cos(expected_phi) ** 3
            - 8.0 * expected_phi * expected_phi * math.cos(expected_phi) / math.sin(expected_phi)
        )
    )

    assert beta1 == pytest.approx(expected_beta1)
    assert beta2 == pytest.approx(expected_beta2)
    assert phi == pytest.approx(expected_phi)
    assert inverse_m == pytest.approx(expected_inverse_m)
    assert sodano_inverse_distance(
        equatorial_radius,
        polar_radius,
        flattening,
        beta1,
        beta2,
        phi,
        inverse_m,
    ) == pytest.approx(expected_distance)

    expected_delta_beta = sodano_delta_beta_alternative(latitude_1, latitude_2, beta1, beta2, sodano_third_flattening(equatorial_radius, polar_radius))
    expected_forward = math.atan2(
        math.cos(beta2) * math.sin(corrected_longitude),
        delta_beta + 2.0 * math.cos(beta2) * math.sin(beta1) * math.sin(corrected_longitude / 2.0) ** 2,
    )
    expected_backward = math.atan2(
        math.cos(beta1) * math.sin(corrected_longitude),
        delta_beta - 2.0 * math.cos(beta1) * math.sin(beta2) * math.sin(corrected_longitude / 2.0) ** 2,
    )

    assert delta_beta == pytest.approx(expected_delta_beta)
    assert corrected_longitude == pytest.approx(
        delta_longitude
        + (
            math.cos(beta1) * math.cos(beta2) * math.sin(delta_longitude) / math.sin(phi)
        )
        * (
            (flattening + flattening * flattening) * phi
            - 0.5 * math.sin(beta1) * math.sin(beta2) * flattening * flattening * (math.sin(phi) + 2.0 * phi * phi / math.sin(phi))
            + 0.25 * inverse_m * flattening * flattening * (math.sin(phi) * math.cos(phi) - 5.0 * phi + 4.0 * phi * phi * math.cos(phi) / math.sin(phi))
        )
    )
    assert sodano_forward_azimuth(beta1, beta2, corrected_longitude, delta_beta) == pytest.approx(
        expected_forward,
        rel=1e-5,
        abs=1e-5,
    )
    assert sodano_backward_azimuth(beta1, beta2, corrected_longitude, delta_beta) == pytest.approx(
        expected_backward,
        rel=1e-5,
        abs=1e-5,
    )
####


def test_sodano_direct_helpers_follow_the_manual_formulas() -> None:
    equatorial_radius = 6_378_137.0
    polar_radius = 6_356_752.314245179
    flattening = (equatorial_radius - polar_radius) / equatorial_radius
    eccentricity = math.sqrt(1.0 - (polar_radius / equatorial_radius) ** 2)
    latitude_1 = math.radians(15.0)
    azimuth = math.radians(40.0)
    surface_distance = 250_000.0

    beta1 = sodano_reduced_latitude(latitude_1, equatorial_radius, polar_radius)
    eprime = sodano_eprime(eccentricity)
    xi = sodano_direct_xi(surface_distance, polar_radius)
    m_value = sodano_direct_m(beta1, azimuth, eprime)
    n_value = sodano_direct_n(beta1, azimuth, xi, eprime)
    phi = sodano_direct_phi(xi, m_value, n_value, eprime)
    beta2_sin = sodano_direct_sin_beta2(beta1, azimuth, phi)
    beta2_cos = sodano_direct_cos_beta2(beta1, azimuth, phi)
    beta2 = math.atan2(beta2_sin, beta2_cos)
    latitude_2 = sodano_direct_latitude(beta2, equatorial_radius, polar_radius)
    l_correction = sodano_direct_l_correction(beta1, azimuth, phi, xi, flattening, m_value, n_value)
    longitude_2 = sodano_direct_longitude(-math.radians(73.0), beta1, azimuth, phi, l_correction, beta2)
    backward = sodano_direct_backward_azimuth(beta1, azimuth, phi)

    expected_beta1 = math.atan2(polar_radius * math.sin(latitude_1), equatorial_radius * math.cos(latitude_1))
    expected_eprime = 1.0 / math.sqrt(1.0 - eccentricity * eccentricity)
    expected_xi = surface_distance / polar_radius
    expected_m = 0.5 * (1.0 + 0.5 * expected_eprime * expected_eprime * math.sin(expected_beta1) ** 2) * (
        1.0 - math.cos(expected_beta1) ** 2 * math.sin(azimuth) ** 2
    )
    expected_n = 0.5 * (1.0 + 0.5 * expected_eprime * expected_eprime * math.sin(expected_beta1) ** 2) * (
        math.sin(expected_beta1) ** 2 * math.cos(expected_xi)
        + math.cos(expected_beta1) * math.cos(azimuth) * math.sin(expected_beta1) * math.sin(expected_xi)
    )
    expected_phi = (
        expected_xi
        - expected_n * expected_eprime * expected_eprime * math.sin(expected_xi)
        + 0.5 * expected_m * expected_eprime * expected_eprime * (math.sin(expected_xi) * math.cos(expected_xi) - expected_xi)
        + 2.5 * expected_n * expected_n * expected_eprime**4 * math.sin(expected_xi) * math.cos(expected_xi)
        + 0.0625
        * expected_m
        * expected_m
        * expected_eprime**4
        * (
            11.0 * expected_xi
            - 13.0 * math.sin(expected_xi) * math.cos(expected_xi)
            - 8.0 * expected_xi * math.cos(expected_xi) ** 2
            + 10.0 * math.sin(expected_xi) * math.cos(expected_xi) ** 3
        )
        + 0.5
        * expected_m
        * expected_n
        * expected_eprime**4
        * (
            3.0 * math.sin(expected_xi)
            + 2.0 * expected_xi * math.cos(expected_xi)
            - 5.0 * math.sin(expected_xi) * math.cos(expected_xi) ** 2
        )
    )
    expected_beta2_sin = math.sin(expected_beta1) * math.cos(expected_phi) + math.cos(expected_beta1) * math.cos(azimuth) * math.sin(expected_phi)
    expected_beta2_cos = math.sqrt(
        math.cos(expected_beta1) ** 2 * math.sin(azimuth) ** 2
        + (math.sin(expected_beta1) * math.sin(expected_phi) - math.cos(expected_beta1) * math.cos(azimuth) * math.cos(expected_phi)) ** 2
    )
    expected_latitude_2 = math.atan2(equatorial_radius * expected_beta2_sin, polar_radius * expected_beta2_cos)
    expected_l = math.cos(expected_beta1) * math.sin(azimuth) * (
        -flattening * expected_xi
        + 3.0 * flattening * flattening * expected_n * math.sin(expected_xi)
        + 1.5 * flattening * flattening * expected_m * (expected_xi - math.sin(expected_xi) * math.cos(expected_xi))
    )
    expected_longitude_2 = -math.radians(73.0) + expected_l + math.atan2(
        math.sin(expected_phi) * math.sin(azimuth),
        math.cos(expected_beta1) * math.cos(expected_phi) - math.sin(expected_beta1) * math.sin(expected_phi) * math.cos(azimuth),
    )
    expected_backward = math.atan2(
        -math.cos(expected_beta1) * math.sin(azimuth),
        math.sin(expected_beta1) * math.sin(expected_phi) - math.cos(expected_beta1) * math.cos(azimuth) * math.cos(expected_phi),
    )

    assert beta1 == pytest.approx(expected_beta1)
    assert eprime == pytest.approx(expected_eprime)
    assert xi == pytest.approx(expected_xi)
    assert m_value == pytest.approx(expected_m)
    assert n_value == pytest.approx(expected_n)
    assert phi == pytest.approx(expected_phi)
    assert beta2_sin == pytest.approx(expected_beta2_sin)
    assert beta2_cos == pytest.approx(expected_beta2_cos)
    assert latitude_2 == pytest.approx(expected_latitude_2)
    assert l_correction == pytest.approx(expected_l)
    assert longitude_2 == pytest.approx(expected_longitude_2)
    assert backward == pytest.approx(expected_backward)
####


def test_downrange_helpers_follow_the_manual_formulas() -> None:
    backward = math.radians(110.0)
    forward = math.radians(18.0)

    assert downrange_crossrange_perpendicular_error(backward, forward) == pytest.approx(
        backward - forward - math.pi / 2.0
    )
    assert downrange_error_function(backward, forward) == pytest.approx(math.pi / 2.0 - abs(backward - forward))
    assert downrange_newton_raphson_update(10.0, -2.0, 4.0) == pytest.approx(10.5)
####

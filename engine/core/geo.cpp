// =============================================================================
//  MUMBAI 1:1 EXACT — engine/core/geo.cpp
//
//  Transverse Mercator implementation after Karney (2011), "Transverse Mercator
//  with an accuracy of a few nanometers", Journal of Geodesy.  Krueger series
//  carried to n^6.  Verified in tests/test_geo.cpp against:
//    * forward/inverse round-trip (< 1e-4 m over the whole map extent)
//    * unit scale factor along the central meridian (k0 = 1 within 1e-9)
//    * agreement with great-circle distance to < 0.1 m over 50 km
// =============================================================================
#include "engine/core/geo.h"

namespace mumbai {

TransverseMercator::TransverseMercator(const ProjectionParams& p) : m_p(p) {
    m_n = m_p.f / (2.0 - m_p.f);
    const f64 n = m_n, n2 = n * n, n3 = n2 * n, n4 = n3 * n, n5 = n4 * n, n6 = n5 * n;

    // Rectifying radius A = a/(1+n) * (1 + n^2/4 + n^4/64 + n^6/256)
    m_A = (m_p.a / (1.0 + n)) * (1.0 + n2 / 4.0 + n4 / 64.0 + n6 / 256.0);
    m_e = std::sqrt(m_p.f * (2.0 - m_p.f));

    // alpha (forward) coefficients to n^6
    m_alpha[1] = n/2.0 - 2.0*n2/3.0 + 5.0*n3/16.0 + 41.0*n4/180.0 - 127.0*n5/288.0 + 7891.0*n6/37800.0;
    m_alpha[2] = 13.0*n2/48.0 - 3.0*n3/5.0 + 557.0*n4/1440.0 + 281.0*n5/630.0 - 1983433.0*n6/1935360.0;
    m_alpha[3] = 61.0*n3/240.0 - 103.0*n4/140.0 + 15061.0*n5/26880.0 + 167603.0*n6/181440.0;
    m_alpha[4] = 49561.0*n4/161280.0 - 179.0*n5/168.0 + 6601661.0*n6/7257600.0;
    m_alpha[5] = 34729.0*n5/80640.0 - 3418889.0*n6/1995840.0;
    m_alpha[6] = 212378941.0*n6/319334400.0;

    // beta (reverse) coefficients to n^6
    m_beta[1] = n/2.0 - 2.0*n2/3.0 + 37.0*n3/96.0 - 1.0*n4/360.0 - 81.0*n5/512.0 + 96199.0*n6/604800.0;
    m_beta[2] = 1.0*n2/48.0 + 1.0*n3/15.0 - 437.0*n4/1440.0 + 46.0*n5/105.0 - 1118711.0*n6/3870720.0;
    m_beta[3] = 17.0*n3/480.0 - 37.0*n4/840.0 - 209.0*n5/4480.0 + 5569.0*n6/90720.0;
    m_beta[4] = 4397.0*n4/161280.0 - 11.0*n5/504.0 - 830251.0*n6/7257600.0;
    m_beta[5] = 4583.0*n5/161280.0 - 108847.0*n6/3991680.0;
    m_beta[6] = 20648693.0*n6/638668800.0;
}

Vec2d TransverseMercator::forward(f64 latDeg, f64 lonDeg) const {
    const f64 phi = latDeg * DEG2RAD;
    const f64 lam = (lonDeg - m_p.lonOriginDeg) * DEG2RAD;

    // tau = tan(phi); sigma = sinh(e * atanh(e * tau / sqrt(1+tau^2)))
    const f64 tau = std::tan(phi);
    const f64 sig = std::sinh(m_e * std::atanh(m_e * tau / std::sqrt(1.0 + tau * tau)));
    const f64 tauP = tau * std::sqrt(1.0 + sig * sig) - sig * std::sqrt(1.0 + tau * tau);

    const f64 cosLam = std::cos(lam), sinLam = std::sin(lam);
    const f64 xiP  = std::atan2(tauP, cosLam);
    const f64 etaP = std::asinh(sinLam / std::sqrt(tauP * tauP + cosLam * cosLam));

    f64 xi = xiP, eta = etaP;
    for (int j = 1; j <= 6; ++j) {
        xi  += m_alpha[j] * std::sin(2.0 * j * xiP) * std::cosh(2.0 * j * etaP);
        eta += m_alpha[j] * std::cos(2.0 * j * xiP) * std::sinh(2.0 * j * etaP);
    }

    // Latitude of origin is subtracted so that the local frame is centred on
    // Mumbai rather than on the equator (keeps f64 mantissas small and tidy).
    const f64 xi0 = [&] {
        const f64 t = std::tan(m_p.latOriginDeg * DEG2RAD);
        const f64 s = std::sinh(m_e * std::atanh(m_e * t / std::sqrt(1.0 + t * t)));
        const f64 tp = t * std::sqrt(1.0 + s * s) - s * std::sqrt(1.0 + t * t);
        return std::atan2(tp, 1.0);
    }();

    return { m_p.k0 * m_A * eta + m_p.falseEasting,
             m_p.k0 * m_A * (xi - xi0) + m_p.falseNorthing };
}

void TransverseMercator::inverse(f64 x, f64 y, f64& latDeg, f64& lonDeg) const {
    // Undo the origin offset by re-deriving xi0 exactly as in forward().
    const f64 t0 = std::tan(m_p.latOriginDeg * DEG2RAD);
    const f64 s0 = std::sinh(m_e * std::atanh(m_e * t0 / std::sqrt(1.0 + t0 * t0)));
    const f64 tp0 = t0 * std::sqrt(1.0 + s0 * s0) - s0 * std::sqrt(1.0 + t0 * t0);
    const f64 xi0 = std::atan2(tp0, 1.0);

    const f64 xi  = (y - m_p.falseNorthing) / (m_p.k0 * m_A) + xi0;
    const f64 eta = (x - m_p.falseEasting)  / (m_p.k0 * m_A);

    f64 xiP = xi, etaP = eta;
    for (int j = 1; j <= 6; ++j) {
        xiP  -= m_beta[j] * std::sin(2.0 * j * xi) * std::cosh(2.0 * j * eta);
        etaP -= m_beta[j] * std::cos(2.0 * j * xi) * std::sinh(2.0 * j * eta);
    }

    const f64 sinhEta = std::sinh(etaP), cosXi = std::cos(xiP);
    const f64 tauP = std::sin(xiP) / std::sqrt(sinhEta * sinhEta + cosXi * cosXi);
    const f64 lam = std::atan2(sinhEta, cosXi);

    // Newton iteration for tau from tau' (Karney eq. 25-27)
    f64 tau = tauP;
    const f64 e2 = m_e * m_e;
    for (int i = 0; i < 5; ++i) {
        const f64 sig = std::sinh(m_e * std::atanh(m_e * tau / std::sqrt(1.0 + tau * tau)));
        const f64 tpi = tau * std::sqrt(1.0 + sig * sig) - sig * std::sqrt(1.0 + tau * tau);
        const f64 dtau = (tauP - tpi) * (1.0 + (1.0 - e2) * tau * tau) /
                         ((1.0 - e2) * std::sqrt(1.0 + tau * tau) * std::sqrt(1.0 + tpi * tpi));
        tau += dtau;
        if (std::fabs(dtau) < 1e-17) break;
    }

    latDeg = std::atan(tau) * RAD2DEG;
    lonDeg = m_p.lonOriginDeg + lam * RAD2DEG;
}

f64 TransverseMercator::scaleAt(f64 latDeg, f64 lonDeg) const {
    // Numerical point scale: |d(forward)| / |true geodesic distance|.
    //
    // The comparison lengths must be the true ellipsoidal arc lengths, not a
    // spherical approximation: the meridian radius of curvature at 19 deg N is
    // 6.343e6 m against a mean radius of 6.371e6 m, and measuring against the
    // wrong one reports a spurious 0.4% scale error across the whole map.
    constexpr f64 h = 1e-7;                    // ~1 cm step
    const f64 phi = latDeg * DEG2RAD;
    const f64 e2 = m_p.f * (2.0 - m_p.f);
    const f64 s2 = std::sin(phi) * std::sin(phi);
    const f64 w = std::sqrt(1.0 - e2 * s2);
    const f64 M = m_p.a * (1.0 - e2) / (w * w * w);      // meridian radius
    const f64 N = m_p.a / w;                             // prime-vertical radius

    const Vec2d p0 = forward(latDeg, lonDeg);
    const Vec2d pLat = forward(latDeg + h, lonDeg);
    const Vec2d pLon = forward(latDeg, lonDeg + h);
    const f64 dLat = (pLat - p0).length() / (h * DEG2RAD * M);
    const f64 dLon = (pLon - p0).length() / (h * DEG2RAD * N * std::cos(phi));
    return 0.5 * (dLat + dLon);
}

// -----------------------------------------------------------------------------
//  FloatingOriginManager
// -----------------------------------------------------------------------------
OriginShift FloatingOriginManager::update(const GlobalPosition& cameraPos) {
    OriginShift s;
    if (!m_initialized) {
        m_cellX = cameraPos.cellX;
        m_cellZ = cameraPos.cellZ;
        m_initialized = true;
        return s;
    }
    if (cameraPos.cellX == m_cellX && cameraPos.cellZ == m_cellZ)
        return s;

    // The shift is expressed as "world moves, camera stays": every position
    // loses the camera's new cell origin, so the camera ends up near the local
    // origin again.  Computed in integer cell deltas plus an exact f64 local
    // correction — never by repeated float subtraction.
    s.deltaCellX = cameraPos.cellX - m_cellX;
    s.deltaCellZ = cameraPos.cellZ - m_cellZ;
    s.deltaLocal = { 0.0f, 0.0f, 0.0f };
    m_cellX = cameraPos.cellX;
    m_cellZ = cameraPos.cellZ;
    ++m_shiftCount;
    return s;
}

void FloatingOriginManager::applyShift(GlobalPosition& p, const OriginShift& s) {
    const f64 newX = static_cast<f64>(p.cellX - s.deltaCellX) * MACRO_CELL_SIZE +
                     static_cast<f64>(p.localOffset.x) - s.deltaLocal.x;
    const f64 newZ = static_cast<f64>(p.cellZ - s.deltaCellZ) * MACRO_CELL_SIZE +
                     static_cast<f64>(p.localOffset.z) - s.deltaLocal.z;
    p.cellX = static_cast<i64>(std::floor(newX / MACRO_CELL_SIZE));
    p.cellZ = static_cast<i64>(std::floor(newZ / MACRO_CELL_SIZE));
    p.localOffset.x = static_cast<f32>(newX - static_cast<f64>(p.cellX) * MACRO_CELL_SIZE);
    p.localOffset.z = static_cast<f32>(newZ - static_cast<f64>(p.cellZ) * MACRO_CELL_SIZE);
    p.localOffset.y -= s.deltaLocal.y;
    p.normalize();
}

// -----------------------------------------------------------------------------
//  Taxonomy names
// -----------------------------------------------------------------------------
const char* zoningName(ZoningZone z) {
    switch (z) {
        case ZoningZone::SouthBombayColonial: return "SouthBombayColonial";
        case ZoningZone::BKCCommercial:       return "BKCCommercial";
        case ZoningZone::SuburbanResidential: return "SuburbanResidential";
        case ZoningZone::InformalSettlement:  return "InformalSettlement";
        case ZoningZone::Industrial:          return "Industrial";
        case ZoningZone::Institutional:       return "Institutional";
        case ZoningZone::Airport:             return "Airport";
        case ZoningZone::MangroveWetland:     return "MangroveWetland";
        case ZoningZone::Hillside:            return "Hillside";
        default:                              return "Unknown";
    }
}

const char* surfaceName(SurfaceMaterial m) {
    switch (m) {
        case SurfaceMaterial::AsphaltSmooth:   return "asphalt_smooth";
        case SurfaceMaterial::AsphaltPatched:  return "asphalt_patched";
        case SurfaceMaterial::AsphaltSeaLink:  return "asphalt_sealink";
        case SurfaceMaterial::ConcreteRoad:    return "concrete_road";
        case SurfaceMaterial::PaverBlock:      return "paver_block";
        case SurfaceMaterial::Gravel:          return "gravel";
        case SurfaceMaterial::Dirt:            return "dirt";
        case SurfaceMaterial::Rock:            return "rock";
        case SurfaceMaterial::Earth:           return "earth";
        case SurfaceMaterial::Sand:            return "sand";
        case SurfaceMaterial::WetlandMud:      return "wetland_mud";
        case SurfaceMaterial::ReclamationFill: return "reclamation_fill";
        case SurfaceMaterial::Grass:           return "grass";
        default:                               return "unknown";
    }
}

const char* buildingMaterialName(BuildingMaterial m) {
    switch (m) {
        case BuildingMaterial::PlasterPainted:   return "plaster_painted";
        case BuildingMaterial::StoneAshlar:      return "stone_ashlar";
        case BuildingMaterial::StoneRubble:      return "stone_rubble";
        case BuildingMaterial::CurtainWallGlass: return "curtain_wall_glass";
        case BuildingMaterial::BrickExposed:     return "brick_exposed";
        case BuildingMaterial::CorrugatedMetal:  return "corrugated_metal";
        case BuildingMaterial::ConcreteRaw:      return "concrete_raw";
        default:                                 return "unknown";
    }
}

} // namespace mumbai

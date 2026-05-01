#pragma once

#include <array>
#include <cmath>

namespace evs
{
namespace cf
{

// ── Input layout ──────────────────────────────────────────────────────────
namespace lee_in
{
enum Idx {
  // Current state
  POS_X = 0, POS_Y, POS_Z,
  VEL_X, VEL_Y, VEL_Z,
  QW, QX, QY, QZ,
  OMEGA_X, OMEGA_Y, OMEGA_Z,
  // Desired state
  DES_POS_X, DES_POS_Y, DES_POS_Z,
  DES_VEL_X, DES_VEL_Y, DES_VEL_Z,
  DES_YAW,
  // Control gains
  KP_POS, KP_VEL, KP_ATT, KP_OMEGA,
  // Physical parameters
  MASS, GRAVITY, IXX, IYY, IZZ,
  SIZE
};
}  // namespace lee_in

// ── Output layout ─────────────────────────────────────────────────────────
namespace lee_out
{
enum Idx {
  THRUST = 0,
  TORQUE_X, TORQUE_Y, TORQUE_Z,
  SIZE
};
}  // namespace lee_out

using LeeInputVec  = std::array<double, lee_in::SIZE>;
using LeeOutputVec = std::array<double, lee_out::SIZE>;

inline std::array<double, 3> vee_map(
  double r12, double r13,
  double r21, double r23,
  double r31, double r32)
{
  return {
    0.5 * (r32 - r23),
    0.5 * (r13 - r31),
    0.5 * (r21 - r12)
  };
}

inline LeeOutputVec compute_lee(const LeeInputVec & u)
{
  using namespace lee_in;
  using namespace lee_out;

  LeeOutputVec y{};

  double ep_x = u[POS_X] - u[DES_POS_X];
  double ep_y = u[POS_Y] - u[DES_POS_Y];
  double ep_z = u[POS_Z] - u[DES_POS_Z];
  double ev_x = u[VEL_X] - u[DES_VEL_X];
  double ev_y = u[VEL_Y] - u[DES_VEL_Y];
  double ev_z = u[VEL_Z] - u[DES_VEL_Z];

  double a_x = -u[KP_POS] * ep_x - u[KP_VEL] * ev_x;
  double a_y = -u[KP_POS] * ep_y - u[KP_VEL] * ev_y;
  double a_z = -u[KP_POS] * ep_z - u[KP_VEL] * ev_z + u[MASS] * u[GRAVITY];

  double a_norm = std::sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
  double zBd_x = 0.0;
  double zBd_y = 0.0;
  double zBd_z = 1.0;
  if (a_norm > 1e-9) {
    zBd_x = a_x / a_norm;
    zBd_y = a_y / a_norm;
    zBd_z = a_z / a_norm;
  }

  // x_C is the desired heading direction projected into the world xy-plane.
  double xC_x = std::cos(u[DES_YAW]);
  double xC_y = std::sin(u[DES_YAW]);
  double xC_z = 0.0;

  // y_Bd is orthogonal to the thrust direction z_Bd and the commanded heading x_C.
  double yBd_x = zBd_y * xC_z - zBd_z * xC_y;
  double yBd_y = zBd_z * xC_x - zBd_x * xC_z;
  double yBd_z = zBd_x * xC_y - zBd_y * xC_x;
  double yBd_norm = std::sqrt(yBd_x * yBd_x + yBd_y * yBd_y + yBd_z * yBd_z);
  if (yBd_norm <= 1e-9) {
    double fallback_x = (std::fabs(zBd_z) < 0.999) ? 0.0 : 1.0;
    double fallback_y = 0.0;
    double fallback_z = (std::fabs(zBd_z) < 0.999) ? 1.0 : 0.0;
    yBd_x = zBd_y * fallback_z - zBd_z * fallback_y;
    yBd_y = zBd_z * fallback_x - zBd_x * fallback_z;
    yBd_z = zBd_x * fallback_y - zBd_y * fallback_x;
    yBd_norm = std::sqrt(yBd_x * yBd_x + yBd_y * yBd_y + yBd_z * yBd_z);
  }
  if (yBd_norm > 1e-9) {
    yBd_x /= yBd_norm;
    yBd_y /= yBd_norm;
    yBd_z /= yBd_norm;
  } else {
    yBd_x = 0.0;
    yBd_y = 1.0;
    yBd_z = 0.0;
  }

  // Complete the desired body frame R_d = [x_Bd y_Bd z_Bd].
  double xBd_x = yBd_y * zBd_z - yBd_z * zBd_y;
  double xBd_y = yBd_z * zBd_x - yBd_x * zBd_z;
  double xBd_z = yBd_x * zBd_y - yBd_y * zBd_x;

  double qw_cur = u[QW];
  double qx_cur = u[QX];
  double qy_cur = u[QY];
  double qz_cur = u[QZ];

  // Current body axes expressed in world coordinates (columns of R).
  double xB_x = 1.0 - 2.0 * (qy_cur * qy_cur + qz_cur * qz_cur);
  double xB_y = 2.0 * (qx_cur * qy_cur + qw_cur * qz_cur);
  double xB_z = 2.0 * (qx_cur * qz_cur - qw_cur * qy_cur);

  double yB_x = 2.0 * (qx_cur * qy_cur - qw_cur * qz_cur);
  double yB_y = 1.0 - 2.0 * (qx_cur * qx_cur + qz_cur * qz_cur);
  double yB_z = 2.0 * (qy_cur * qz_cur + qw_cur * qx_cur);

  double zB_x = 2.0 * (qx_cur * qz_cur + qw_cur * qy_cur);
  double zB_y = 2.0 * (qy_cur * qz_cur - qw_cur * qx_cur);
  double zB_z = 1.0 - 2.0 * (qx_cur * qx_cur + qy_cur * qy_cur);

  // Desired total thrust
  y[THRUST] = a_x * zB_x + a_y * zB_y + a_z * zB_z;
  if (y[THRUST] < 0.0) {
    y[THRUST] = 0.0;
  }

  // Q = R_d^T R is the current attitude expressed in the desired body frame.
  double Q12 = xBd_x * yB_x + xBd_y * yB_y + xBd_z * yB_z;
  double Q13 = xBd_x * zB_x + xBd_y * zB_y + xBd_z * zB_z;
  double Q21 = yBd_x * xB_x + yBd_y * xB_y + yBd_z * xB_z;
  double Q23 = yBd_x * zB_x + yBd_y * zB_y + yBd_z * zB_z;
  double Q31 = zBd_x * xB_x + zBd_y * xB_y + zBd_z * xB_z;
  double Q32 = zBd_x * yB_x + zBd_y * yB_y + zBd_z * yB_z;

  // e_R = 0.5 * (R_d^T R - R^T R_d)^vee
  std::array<double, 3> e_r = vee_map(Q12, Q13, Q21, Q23, Q31, Q32);

  double omega_x = u[OMEGA_X];
  double omega_y = u[OMEGA_Y];
  double omega_z = u[OMEGA_Z];

  double jw_x = u[IXX] * omega_x;
  double jw_y = u[IYY] * omega_y;
  double jw_z = u[IZZ] * omega_z;

  double coriolis_x = omega_y * jw_z - omega_z * jw_y;
  double coriolis_y = omega_z * jw_x - omega_x * jw_z;
  double coriolis_z = omega_x * jw_y - omega_y * jw_x;

  y[TORQUE_X] = -u[KP_ATT] * e_r[0] - u[KP_OMEGA] * omega_x + coriolis_x;
  y[TORQUE_Y] = -u[KP_ATT] * e_r[1] - u[KP_OMEGA] * omega_y + coriolis_y;
  y[TORQUE_Z] = -u[KP_ATT] * e_r[2] - u[KP_OMEGA] * omega_z + coriolis_z;

  return y;
}

}  // namespace cf
}  // namespace evs

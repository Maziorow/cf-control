#pragma once

#include <array>
#include <cmath>

namespace evs
{
namespace cf
{

// ── Input layout ──────────────────────────────────────────────────────────
namespace flat_in
{
enum Idx {
  POS_X = 0, POS_Y, POS_Z,
  VEL_X, VEL_Y, VEL_Z,
  ACC_X, ACC_Y, ACC_Z,
  JERK_X, JERK_Y, JERK_Z,
  SNAP_X, SNAP_Y, SNAP_Z,
  YAW, YAW_VEL, YAW_ACC,
  MASS, GRAVITY,
  IXX, IYY, IZZ,
  SIZE
};
}  // namespace flat_in

// ── Output layout ─────────────────────────────────────────────────────────
namespace flat_out
{
enum Idx {
  POS_X = 0, POS_Y, POS_Z,
  VEL_X, VEL_Y, VEL_Z,
  QW, QX, QY, QZ,
  OMEGA_X, OMEGA_Y, OMEGA_Z,
  THRUST,
  TORQUE_X, TORQUE_Y, TORQUE_Z,
  SIZE
};
}  // namespace flat_out

using FlatInputVec  = std::array<double, flat_in::SIZE>;
using FlatOutputVec = std::array<double, flat_out::SIZE>;

// Differential flatness map for a quadrotor.
//
// Computes orientation (quaternion), angular velocity, collective thrust and
// torques from the flat outputs [x, y, z, ψ] and their time derivatives up to
// snap order.
//
// Reference: Mellinger & Kumar, ICRA 2011 — "Minimum Snap Trajectory
// Generation and Control for Quadrotors".
inline FlatOutputVec compute_flatness(const FlatInputVec & u)
{
  using namespace flat_in;   // index names for the input array
  using namespace flat_out;  // index names for the output array

  FlatOutputVec y{};

  // ── Pass-through: position and velocity ───────────────────────────────
  y[POS_X] = u[flat_in::POS_X];  y[POS_Y] = u[flat_in::POS_Y];  y[POS_Z] = u[flat_in::POS_Z];
  y[VEL_X] = u[flat_in::VEL_X];  y[VEL_Y] = u[flat_in::VEL_Y];  y[VEL_Z] = u[flat_in::VEL_Z];

  // ── Thrust ─────────────────────────────────────────────────────────────
  // Desired thrust vector: t = a + g·e3
  const double tx = u[ACC_X];
  const double ty = u[ACC_Y];
  const double tz = u[ACC_Z] + u[GRAVITY];
  const double t_norm = std::sqrt(tx * tx + ty * ty + tz * tz);
  y[THRUST] = u[MASS] * t_norm;

  // Body z-axis: z_B = t / ‖t‖
  const double zBx = tx / t_norm;
  const double zBy = ty / t_norm;
  const double zBz = tz / t_norm;

  // ── Orientation ────────────────────────────────────────────────────────
  // Heading constraint: x_C = [cos ψ, sin ψ, 0]
  const double xCx = std::cos(u[YAW]);
  const double xCy = std::sin(u[YAW]);
  // xCz = 0

  // y_B = normalize(z_B × x_C)
  // z_B × x_C = [zBy·0 − zBz·xCy,  zBz·xCx − zBx·0,  zBx·xCy − zBy·xCx]
  double yBx = -zBz * xCy;
  double yBy =  zBz * xCx;
  double yBz =  zBx * xCy - zBy * xCx;
  const double yB_norm = std::sqrt(yBx * yBx + yBy * yBy + yBz * yBz);
  yBx /= yB_norm;  yBy /= yB_norm;  yBz /= yB_norm;

  // x_B = y_B × z_B
  const double xBx = yBy * zBz - yBz * zBy;
  const double xBy = yBz * zBx - yBx * zBz;
  const double xBz = yBx * zBy - yBy * zBx;

  // Rotation matrix → quaternion  (columns of R are x_B, y_B, z_B)
  const double trace = xBx + yBy + zBz;
  double qw, qx, qy, qz;
  if (trace > 0.0) {
    const double s = 0.5 / std::sqrt(trace + 1.0);
    qw = 0.25 / s;
    qx = (zBy - yBz) * s;
    qy = (xBz - zBx) * s;
    qz = (yBx - xBy) * s;
  } else if (xBx > yBy && xBx > zBz) {
    const double s = 2.0 * std::sqrt(1.0 + xBx - yBy - zBz);
    qw = (zBy - yBz) / s;  qx = 0.25 * s;
    qy = (xBy + yBx) / s;  qz = (xBz + zBx) / s;
  } else if (yBy > zBz) {
    const double s = 2.0 * std::sqrt(1.0 + yBy - xBx - zBz);
    qw = (xBz - zBx) / s;  qx = (xBy + yBx) / s;
    qy = 0.25 * s;          qz = (yBz + zBy) / s;
  } else {
    const double s = 2.0 * std::sqrt(1.0 + zBz - xBx - yBy);
    qw = (yBx - xBy) / s;  qx = (xBz + zBx) / s;
    qy = (yBz + zBy) / s;  qz = 0.25 * s;
  }
  y[QW] = qw;  y[QX] = qx;  y[QY] = qy;  y[QZ] = qz;

  // ── Angular velocity from jerk ─────────────────────────────────────────
  // h_Ω = dz_B/dt = (m/T)·(j − (z_B·j)·z_B)
  const double zB_dot_j = zBx * u[JERK_X] + zBy * u[JERK_Y] + zBz * u[JERK_Z];
  const double mT = u[MASS] / y[THRUST];
  const double hx = mT * (u[JERK_X] - zB_dot_j * zBx);
  const double hy = mT * (u[JERK_Y] - zB_dot_j * zBy);
  const double hz = mT * (u[JERK_Z] - zB_dot_j * zBz);

  y[OMEGA_X] = -(hx * yBx + hy * yBy + hz * yBz);  // −h·y_B
  y[OMEGA_Y] =   hx * xBx + hy * xBy + hz * xBz;   //  h·x_B
  y[OMEGA_Z] = u[YAW_VEL] * zBz;                    // ψ̇·(z_B·e_z)

  // ── Angular acceleration from snap ─────────────────────────────────────
  // Ṫ = m·(z_B·j)
  // dh/dt = −(Ṫ/T)·h + (m/T)·(s − (h·j + z_B·s)·z_B − (z_B·j)·h)
  const double T_dot    = u[MASS] * zB_dot_j;
  const double zB_dot_s = zBx * u[SNAP_X] + zBy * u[SNAP_Y] + zBz * u[SNAP_Z];
  const double h_dot_j  = hx * u[JERK_X] + hy * u[JERK_Y] + hz * u[JERK_Z];
  const double coeff    = h_dot_j + zB_dot_s;
  const double Tdot_T   = T_dot / y[THRUST];

  const double dhdtx = -Tdot_T * hx + mT * (u[SNAP_X] - coeff * zBx - zB_dot_j * hx);
  const double dhdty = -Tdot_T * hy + mT * (u[SNAP_Y] - coeff * zBy - zB_dot_j * hy);
  const double dhdtz = -Tdot_T * hz + mT * (u[SNAP_Z] - coeff * zBz - zB_dot_j * hz);

  const double alpha_x = -(dhdtx * yBx + dhdty * yBy + dhdtz * yBz);
  const double alpha_y =   dhdtx * xBx + dhdty * xBy + dhdtz * xBz;
  const double alpha_z = u[YAW_ACC] * zBz;

  // ── Torque = I·α + ω × (I·ω) ──────────────────────────────────────────
  y[TORQUE_X] = u[IXX] * alpha_x + (u[IZZ] - u[IYY]) * y[OMEGA_Y] * y[OMEGA_Z];
  y[TORQUE_Y] = u[IYY] * alpha_y + (u[IXX] - u[IZZ]) * y[OMEGA_X] * y[OMEGA_Z];
  y[TORQUE_Z] = u[IZZ] * alpha_z + (u[IYY] - u[IXX]) * y[OMEGA_X] * y[OMEGA_Y];

  return y;
}

}  // namespace cf
}  // namespace evs

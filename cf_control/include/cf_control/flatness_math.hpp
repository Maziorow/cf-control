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

inline FlatOutputVec compute_flatness(const FlatInputVec & u)
{
  using namespace flat_in;
  using namespace flat_out;

  FlatOutputVec y{};

  y[flat_out::POS_X] = u[flat_in::POS_X];  y[flat_out::POS_Y] = u[flat_in::POS_Y];  y[flat_out::POS_Z] = u[flat_in::POS_Z];
  y[flat_out::VEL_X] = u[flat_in::VEL_X];  y[flat_out::VEL_Y] = u[flat_in::VEL_Y];  y[flat_out::VEL_Z] = u[flat_in::VEL_Z];

  double tx = u[ACC_X];
  double ty = u[ACC_Y];
  double tz = u[ACC_Z] + u[GRAVITY];
  double t_norm = std::sqrt(tx * tx + ty * ty + tz * tz);
  y[THRUST] = u[MASS] * t_norm;

  double zBx = tx / t_norm;
  double zBy = ty / t_norm;
  double zBz = tz / t_norm;

  double xCx = std::cos(u[YAW]);
  double xCy = std::sin(u[YAW]);
  // xCz = 0

  double yBx = -zBz * xCy;
  double yBy =  zBz * xCx;
  double yBz =  zBx * xCy - zBy * xCx;
  double yB_norm = std::sqrt(yBx * yBx + yBy * yBy + yBz * yBz);
  yBx /= yB_norm;  yBy /= yB_norm;  yBz /= yB_norm;

  double xBx = yBy * zBz - yBz * zBy;
  double xBy = yBz * zBx - yBx * zBz;
  double xBz = yBx * zBy - yBy * zBx;

  double trace = xBx + yBy + zBz;
  double qw, qx, qy, qz;
  if (trace > 0.0) {
    double s = std::sqrt(trace + 1.0) * 2.0;
    qw = 0.25 * s;
    qx = (yBz - zBy) / s;
    qy = (zBx - xBz) / s;
    qz = (xBy - yBx) / s;
  } else if (xBx > yBy && xBx > zBz) {
    double s = 2.0 * std::sqrt(1.0 + xBx - yBy - zBz);
    qw = (yBz - zBy) / s;  qx = 0.25 * s;
    qy = (yBx + xBy) / s;  qz = (zBx + xBz) / s;
  } else if (yBy > zBz && yBy > xBx) {
    double s = 2.0 * std::sqrt(1.0 + yBy - xBx - zBz);
    qw = (zBx - xBz) / s;  qx = (yBx + xBy) / s;
    qy = 0.25 * s;          qz = (zBy + yBz) / s;
  } else {
    double s = 2.0 * std::sqrt(1.0 + zBz - xBx - yBy);
    qw = (xBy - yBx) / s;  qx = (zBx + xBz) / s;
    qy = (zBy + yBz) / s;  qz = 0.25 * s;
  }
  y[QW] = qw;  y[QX] = qx;  y[QY] = qy;  y[QZ] = qz;

  double zB_dot_j = zBx * u[JERK_X] + zBy * u[JERK_Y] + zBz * u[JERK_Z];
  double mT = u[MASS] / y[THRUST];
  double hx = mT * (u[JERK_X] - zB_dot_j * zBx);
  double hy = mT * (u[JERK_Y] - zB_dot_j * zBy);
  double hz = mT * (u[JERK_Z] - zB_dot_j * zBz);

  y[OMEGA_X] = -(hx * yBx + hy * yBy + hz * yBz);
  y[OMEGA_Y] =   hx * xBx + hy * xBy + hz * xBz;
  y[OMEGA_Z] = u[YAW_VEL] * zBz;

  double T_dot    = u[MASS] * zB_dot_j;
  double zB_dot_s = zBx * u[SNAP_X] + zBy * u[SNAP_Y] + zBz * u[SNAP_Z];
  double h_dot_j  = hx * u[JERK_X] + hy * u[JERK_Y] + hz * u[JERK_Z];
  double coeff    = h_dot_j + zB_dot_s;
  double Tdot_T   = T_dot / y[THRUST];

  double dhdtx = -Tdot_T * hx + mT * (u[SNAP_X] - coeff * zBx - zB_dot_j * hx);
  double dhdty = -Tdot_T * hy + mT * (u[SNAP_Y] - coeff * zBy - zB_dot_j * hy);
  double dhdtz = -Tdot_T * hz + mT * (u[SNAP_Z] - coeff * zBz - zB_dot_j * hz);

  double alpha_x = -(dhdtx * yBx + dhdty * yBy + dhdtz * yBz);
  double alpha_y =   dhdtx * xBx + dhdty * xBy + dhdtz * xBz;
  double alpha_z = u[YAW_ACC] * zBz;

  y[TORQUE_X] = u[IXX] * alpha_x + (u[IZZ] - u[IYY]) * y[OMEGA_Y] * y[OMEGA_Z];
  y[TORQUE_Y] = u[IYY] * alpha_y + (u[IXX] - u[IZZ]) * y[OMEGA_X] * y[OMEGA_Z];
  y[TORQUE_Z] = u[IZZ] * alpha_z + (u[IYY] - u[IXX]) * y[OMEGA_X] * y[OMEGA_Y];

  return y;
}

}  // namespace cf
}  // namespace evs

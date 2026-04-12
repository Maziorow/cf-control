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
  y[flat_out::POS_X] = u[flat_in::POS_X];  y[flat_out::POS_Y] = u[flat_in::POS_Y];  y[flat_out::POS_Z] = u[flat_in::POS_Z];
  y[flat_out::VEL_X] = u[flat_in::VEL_X];  y[flat_out::VEL_Y] = u[flat_in::VEL_Y];  y[flat_out::VEL_Z] = u[flat_in::VEL_Z];

  // ── Thrust ─────────────────────────────────────────────────────────────
  // Desired thrust vector: t = a + g·e3
  double tx = u[ACC_X];
  double ty = u[ACC_Y];
  double tz = u[ACC_Z] + u[GRAVITY];
  double t_norm = std::sqrt(tx * tx + ty * ty + tz * tz);
  y[THRUST] = u[MASS] * t_norm;

  // Body z-axis: z_B = t / ‖t‖
  double zBx = tx / t_norm;
  double zBy = ty / t_norm;
  double zBz = tz / t_norm;

  // ── Orientation ────────────────────────────────────────────────────────
  // Heading constraint: x_C = [cos ψ, sin ψ, 0]
  double xCx = std::cos(u[YAW]);
  double xCy = std::sin(u[YAW]);
  // xCz = 0

  // y_B = normalize(z_B × x_C)
  // z_B × x_C = [zBy·0 − zBz·xCy,  zBz·xCx − zBx·0,  zBx·xCy − zBy·xCx]
  double yBx = -zBz * xCy;
  double yBy =  zBz * xCx;
  double yBz =  zBx * xCy - zBy * xCx;
  double yB_norm = std::sqrt(yBx * yBx + yBy * yBy + yBz * yBz);
  yBx /= yB_norm;  yBy /= yB_norm;  yBz /= yB_norm;

  // x_B = y_B × z_B
  double xBx = yBy * zBz - yBz * zBy;
  double xBy = yBz * zBx - yBx * zBz;
  double xBz = yBx * zBy - yBy * zBx;

  // Rotation matrix → quaternion  (columns of R are x_B, y_B, z_B)
  double trace = xBx + yBy + zBz;
  double qw, qx, qy, qz;
  // Shepperd method: columns of R are x_B, y_B, z_B, so
  //   R[row][col]:  R[i][0]=xB[i]  R[i][1]=yB[i]  R[i][2]=zB[i]
  // Anti-symmetric terms: (R[2][1]-R[1][2]) = yBz-zBy, etc.
  if (trace > 0.0) {
    double s = std::sqrt(trace + 1.0) * 2.0;  // s = 4·qw
    qw = 0.25 * s;
    qx = (yBz - zBy) / s;  // R[2][1] - R[1][2]
    qy = (zBx - xBz) / s;  // R[0][2] - R[2][0]
    qz = (xBy - yBx) / s;  // R[1][0] - R[0][1]
  } else if (xBx > yBy && xBx > zBz) {
    double s = 2.0 * std::sqrt(1.0 + xBx - yBy - zBz);  // s = 4·qx
    qw = (yBz - zBy) / s;  qx = 0.25 * s;
    qy = (yBx + xBy) / s;  qz = (zBx + xBz) / s;
  } else if (yBy > zBz && yBy > xBx) {
    double s = 2.0 * std::sqrt(1.0 + yBy - xBx - zBz);  // s = 4·qy
    qw = (zBx - xBz) / s;  qx = (yBx + xBy) / s;
    qy = 0.25 * s;          qz = (zBy + yBz) / s;
  } else {
    double s = 2.0 * std::sqrt(1.0 + zBz - xBx - yBy);  // s = 4·qz
    qw = (xBy - yBx) / s;  qx = (zBx + xBz) / s;
    qy = (zBy + yBz) / s;  qz = 0.25 * s;
  }
  y[QW] = qw;  y[QX] = qx;  y[QY] = qy;  y[QZ] = qz;

  // ── Angular velocity from jerk ─────────────────────────────────────────
  // h_Ω = dz_B/dt = (m/T)·(j − (z_B·j)·z_B)
  double zB_dot_j = zBx * u[JERK_X] + zBy * u[JERK_Y] + zBz * u[JERK_Z];
  double mT = u[MASS] / y[THRUST];
  double hx = mT * (u[JERK_X] - zB_dot_j * zBx);
  double hy = mT * (u[JERK_Y] - zB_dot_j * zBy);
  double hz = mT * (u[JERK_Z] - zB_dot_j * zBz);

  y[OMEGA_X] = -(hx * yBx + hy * yBy + hz * yBz);  // −h·y_B
  y[OMEGA_Y] =   hx * xBx + hy * xBy + hz * xBz;   //  h·x_B
  y[OMEGA_Z] = u[YAW_VEL] * zBz;                    // ψ̇·(z_B·e_z)

  // ── Angular acceleration from snap ─────────────────────────────────────
  // Ṫ = m·(z_B·j)
  // dh/dt = −(Ṫ/T)·h + (m/T)·(s − (h·j + z_B·s)·z_B − (z_B·j)·h)
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

  // ── Torque = I·α + ω × (I·ω) ──────────────────────────────────────────
  y[TORQUE_X] = u[IXX] * alpha_x + (u[IZZ] - u[IYY]) * y[OMEGA_Y] * y[OMEGA_Z];
  y[TORQUE_Y] = u[IYY] * alpha_y + (u[IXX] - u[IZZ]) * y[OMEGA_X] * y[OMEGA_Z];
  y[TORQUE_Z] = u[IZZ] * alpha_z + (u[IYY] - u[IXX]) * y[OMEGA_X] * y[OMEGA_Y];

  return y;
}

}  // namespace cf
}  // namespace evs

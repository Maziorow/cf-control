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

}  // namespace cf
}  // namespace evs

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "cf_control/flatness_math.hpp"

using namespace evs::cf;

// ── CSV row ────────────────────────────────────────────────────────────────

struct TestRow {
  std::string name;
  FlatInputVec  input;
  FlatOutputVec expected;
};

// CSV columns (0-indexed):
//  0  test_name
//  1-3   in_pos_{x,y,z}
//  4-6   in_vel_{x,y,z}
//  7-9   in_acc_{x,y,z}
// 10-12  in_jerk_{x,y,z}
// 13-15  in_snap_{x,y,z}
// 16     in_yaw
// 17     in_yaw_rate       → flat_in::YAW_VEL
// 18     in_yaw_acceleration
// 19     in_mass
// 20     in_gravity
// 21-23  in_I_{xx,yy,zz}
// 24-26  out_pos_{x,y,z}
// 27-30  out_quat_{w,x,y,z}
// 31-33  out_vel_{x,y,z}
// 34-36  out_omega_{x,y,z}
// 37     out_thrust
// 38-40  out_torque_{x,y,z}

static std::vector<TestRow> load_csv(const std::string & path)
{
  std::ifstream f(path);
  if (!f.is_open()) {
    throw std::runtime_error("Cannot open CSV: " + path);
  }

  std::vector<TestRow> rows;
  std::string line;
  std::getline(f, line);  // discard header

  while (std::getline(f, line)) {
    if (line.empty()) continue;

    std::vector<std::string> tok;
    std::stringstream ss(line);
    std::string t;
    while (std::getline(ss, t, ',')) tok.push_back(t);
    if (tok.size() < 41) continue;

    auto d = [&](int i) { return std::stod(tok[static_cast<size_t>(i)]); };

    TestRow r;
    r.name = tok[0];

    // ── Input vector ──────────────────────────────────────────────────────
    namespace fi = flat_in;
    r.input[fi::POS_X]  = d(1);  r.input[fi::POS_Y]  = d(2);  r.input[fi::POS_Z]  = d(3);
    r.input[fi::VEL_X]  = d(4);  r.input[fi::VEL_Y]  = d(5);  r.input[fi::VEL_Z]  = d(6);
    r.input[fi::ACC_X]  = d(7);  r.input[fi::ACC_Y]  = d(8);  r.input[fi::ACC_Z]  = d(9);
    r.input[fi::JERK_X] = d(10); r.input[fi::JERK_Y] = d(11); r.input[fi::JERK_Z] = d(12);
    r.input[fi::SNAP_X] = d(13); r.input[fi::SNAP_Y] = d(14); r.input[fi::SNAP_Z] = d(15);
    r.input[fi::YAW]     = d(16);
    r.input[fi::YAW_VEL] = d(17);
    r.input[fi::YAW_ACC] = d(18);
    r.input[fi::MASS]    = d(19);
    r.input[fi::GRAVITY] = d(20);
    r.input[fi::IXX] = d(21); r.input[fi::IYY] = d(22); r.input[fi::IZZ] = d(23);

    // ── Expected output vector ────────────────────────────────────────────
    namespace fo = flat_out;
    r.expected[fo::POS_X] = d(24); r.expected[fo::POS_Y] = d(25); r.expected[fo::POS_Z] = d(26);
    r.expected[fo::QW] = d(27); r.expected[fo::QX] = d(28);
    r.expected[fo::QY] = d(29); r.expected[fo::QZ] = d(30);
    r.expected[fo::VEL_X] = d(31); r.expected[fo::VEL_Y] = d(32); r.expected[fo::VEL_Z] = d(33);
    r.expected[fo::OMEGA_X] = d(34); r.expected[fo::OMEGA_Y] = d(35); r.expected[fo::OMEGA_Z] = d(36);
    r.expected[fo::THRUST]   = d(37);
    r.expected[fo::TORQUE_X] = d(38); r.expected[fo::TORQUE_Y] = d(39); r.expected[fo::TORQUE_Z] = d(40);

    rows.push_back(r);
  }
  return rows;
}

// ── Parameterised test ────────────────────────────────────────────────────

class FlatnessTest : public ::testing::TestWithParam<TestRow> {};

TEST_P(FlatnessTest, OutputsMatchExpected)
{
  const TestRow & row = GetParam();
  constexpr double kTol = 1e-6;
  namespace fo = flat_out;

  const FlatOutputVec y = compute_flatness(row.input);

  EXPECT_NEAR(y[fo::POS_X], row.expected[fo::POS_X], kTol);
  EXPECT_NEAR(y[fo::POS_Y], row.expected[fo::POS_Y], kTol);
  EXPECT_NEAR(y[fo::POS_Z], row.expected[fo::POS_Z], kTol);

  EXPECT_NEAR(y[fo::VEL_X], row.expected[fo::VEL_X], kTol);
  EXPECT_NEAR(y[fo::VEL_Y], row.expected[fo::VEL_Y], kTol);
  EXPECT_NEAR(y[fo::VEL_Z], row.expected[fo::VEL_Z], kTol);

  // Quaternion: q and −q encode the same rotation — check |q·q_exp| ≈ 1
  const double q_dot =
    y[fo::QW] * row.expected[fo::QW] + y[fo::QX] * row.expected[fo::QX] +
    y[fo::QY] * row.expected[fo::QY] + y[fo::QZ] * row.expected[fo::QZ];
  EXPECT_NEAR(std::abs(q_dot), 1.0, kTol) << "quaternion represents a different rotation";

  // Unit-norm sanity
  const double q_norm = std::sqrt(
    y[fo::QW]*y[fo::QW] + y[fo::QX]*y[fo::QX] +
    y[fo::QY]*y[fo::QY] + y[fo::QZ]*y[fo::QZ]);
  EXPECT_NEAR(q_norm, 1.0, kTol) << "quaternion is not unit-norm";

  EXPECT_NEAR(y[fo::OMEGA_X], row.expected[fo::OMEGA_X], kTol);
  EXPECT_NEAR(y[fo::OMEGA_Y], row.expected[fo::OMEGA_Y], kTol);
  EXPECT_NEAR(y[fo::OMEGA_Z], row.expected[fo::OMEGA_Z], kTol);

  EXPECT_NEAR(y[fo::THRUST],   row.expected[fo::THRUST],   kTol);

  EXPECT_NEAR(y[fo::TORQUE_X], row.expected[fo::TORQUE_X], kTol);
  EXPECT_NEAR(y[fo::TORQUE_Y], row.expected[fo::TORQUE_Y], kTol);
  EXPECT_NEAR(y[fo::TORQUE_Z], row.expected[fo::TORQUE_Z], kTol);
}

INSTANTIATE_TEST_SUITE_P(
  CsvData, FlatnessTest,
  ::testing::ValuesIn(
    load_csv(TEST_DATA_DIR "/trajectory_from_flat_output_test_data.csv")),
  [](const ::testing::TestParamInfo<TestRow> & info) {
    return info.param.name;
  });

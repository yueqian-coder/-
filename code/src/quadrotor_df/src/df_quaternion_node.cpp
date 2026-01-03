#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char **argv)
{
  std::string output_path =
    "/home/user/MRPC-2025-homework/solutions/df_quaternion.csv";
  if (argc > 1)
  {
    output_path = argv[1];
  }

  std::ofstream out(output_path);
  if (!out.is_open())
  {
    std::cerr << "Failed to open output file: " << output_path << std::endl;
    return 1;
  }

  out << "t,x,y,z,w\n";

  const double kPi = 3.14159265358979323846;
  const double g = 9.81;
  const double dt = 0.02;
  const double t_end = 2.0 * kPi;

  Eigen::Quaterniond q_prev(1.0, 0.0, 0.0, 0.0);
  bool has_prev = false;

  for (double t = 0.0; t <= t_end + 1e-9; t += dt)
  {
    const double s = std::sin(t);
    const double c = std::cos(t);
    const double s2 = s * s;
    const double s4 = s2 * s2;
    const double denom = 1.0 + s2;
    const double denom2 = denom * denom;
    const double denom3 = denom2 * denom;

    const double vx = -10.0 * s * (3.0 - s2) / denom2;
    const double vy = 10.0 * (1.0 - 3.0 * s2) / denom2;

    const double ax = -10.0 * c * (s4 - 12.0 * s2 + 3.0) / denom3;
    const double ay = -20.0 * s * c * (5.0 - 3.0 * s2) / denom3;

    Eigen::Vector3d a(ax, ay, 0.0);
    Eigen::Vector3d b3 = (a + Eigen::Vector3d(0.0, 0.0, g)).normalized();

    double yaw = std::atan2(vy, vx);
    Eigen::Vector3d b1d(std::cos(yaw), std::sin(yaw), 0.0);

    Eigen::Vector3d b2 = b3.cross(b1d);
    if (b2.norm() < 1e-6)
    {
      b1d = Eigen::Vector3d(1.0, 0.0, 0.0);
      b2 = b3.cross(b1d);
    }
    b2.normalize();
    Eigen::Vector3d b1 = b2.cross(b3).normalized();

    Eigen::Matrix3d R;
    R.col(0) = b1;
    R.col(1) = b2;
    R.col(2) = b3;

    Eigen::Quaterniond q(R);
    q.normalize();

    if (has_prev && q.coeffs().dot(q_prev.coeffs()) < 0.0)
    {
      q.coeffs() *= -1.0;
    }
    if (q.w() < 0.0)
    {
      q.coeffs() *= -1.0;
    }
    q_prev = q;
    has_prev = true;

    out << std::fixed << std::setprecision(2) << t << ",";
    out << std::setprecision(7) << q.x() << "," << q.y() << "," << q.z()
        << "," << q.w() << "\n";
  }

  out.close();
  std::cout << "Saved " << output_path << std::endl;
  return 0;
}

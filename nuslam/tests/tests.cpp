#include <gtest/gtest.h>
#include "rigid2d/diff_drive.hpp"
#include <sstream>
#include <cstdlib>
#include <string>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::JacobiSVD;
using Eigen::ComputeThinU;
using Eigen::ComputeThinV;
using Eigen::SelfAdjointEigenSolver;
using rigid2d::Vector2D;
using namespace std;

TEST(TestSuite, testCase1)
{
  double threshold = 1e-4;
  double x_mean, y_mean, z_mean, x_sum = 0.0, y_sum = 0.0, z_sum = 0.0, sigma_min, eigen_min, r1, r2, R_square, radius;
  std::vector<Vector2D> position = {{1,7},{2,6},{5,8},{7,7},{9,5},{3,7}};
  double center_x, center_y;
  int index;

  for (std::size_t p = 0; p < position.size(); p++)
  {
    x_sum += position[p].x;
    y_sum += position[p].y;
  }
  x_mean = x_sum / position.size();
  y_mean = y_sum / position.size();

  for (std::size_t p = 0; p < position.size(); p++)
  {
    position[p].x -= x_mean;
    position[p].y -= y_mean;
    z_sum = z_sum + position[p].x * position[p].x + position[p].y * position[p].y;
  }
  z_mean = z_sum / position.size();

  MatrixXd Z = MatrixXd::Random(position.size(),4);
  for (std::size_t q = 0; q < position.size(); q++)
  {
    Z(q, 0) = position[q].x * position[q].x + position[q].y * position[q].y;
    Z(q, 1) = position[q].x;
    Z(q, 2) = position[q].y;
    Z(q, 3) = 1;
  }

  MatrixXd Z_transpose = Z.transpose();
  MatrixXd M = Z_transpose * Z / position.size();

  MatrixXd H = MatrixXd::Random(4,4);
  MatrixXd H_inverse = MatrixXd::Random(4,4);
  H(0,0) = 8 * z_mean;
  H(0,1) = H(0,2) = H(1,0) = H(1,2) = H(1,3) = H(2,0) = H(2,1) = H(2,3) = H(3,1) = H(3,2) = H(3,3) = 0;
  H(0,3) = H(3,0) = 2;
  H(1,1) = H(2,2) = 1;
  H_inverse(3,3) = (-2) * z_mean;
  H_inverse(0,3) = H_inverse(3,0) = 0.5;
  H_inverse(1,1) = H_inverse(2,2) = 1;
  H_inverse(0,0) = H_inverse(0,1) = H_inverse(0,2) = H_inverse(1,0) = H_inverse(1,2) = H_inverse(1,3) = H_inverse(2,0) = H_inverse(2,1) = H_inverse(2,3) = H_inverse(3,1) = H_inverse(3,2) = 0.0;

  JacobiSVD <MatrixXd> svd(Z, ComputeThinU | ComputeThinV);
  MatrixXd singular_values = svd.singularValues();
  MatrixXd U = svd.matrixU();
  MatrixXd V = svd.matrixV();
  MatrixXd A = MatrixXd::Random(4,1);
  MatrixXd A_star = MatrixXd::Random(4,1);
  MatrixXd Q;
  MatrixXd ADD = MatrixXd::Random(singular_values.size(),singular_values.size());
  MatrixXd Y;
  MatrixXd eigen_values;

  for (std::size_t w = 0; w < singular_values.size(); w++)
  {
    for (std::size_t ww = 0; ww < singular_values.size(); ww++)
    {
      if (w == ww)
      {
        ADD(w,ww) = singular_values(w,0);
      }else
      {
        ADD(w,ww) = 0;
      }
    }

  }

  for (std::size_t r = 0; r < singular_values.size() - 1; r++)
  {
    if (singular_values(r) < singular_values(r + 1))
    {
      sigma_min = singular_values(r);
    }else
    {
      sigma_min = singular_values(r + 1);
    }
  }

  if (sigma_min > 1e-12)
  {
    Y = V * ADD * V.transpose();
    Q = Y * H_inverse * Y;
    SelfAdjointEigenSolver<MatrixXd> es(Q);
    eigen_values = es.eigenvalues();
    for (std::size_t s = 0; s < eigen_values.size();)
    {
      if (eigen_values(s,0) > 0)
      {
        eigen_min = eigen_values(s,0);
        index = s;
        break;
      }else
      {
        s++;
      }
    }
    for (std::size_t s = 0; s < eigen_values.size();)
    {
      if (eigen_min > eigen_values(s,0) && eigen_values(s,0) > 0)
      {
        eigen_min = eigen_values(s,0);
        index = s;
      }else
      {
        s++;
      }
    }

    for (std::size_t s = 0; s < 4; s++)
    {
      A_star(s, 0) = V(s, index);
    }
    A = Y.colPivHouseholderQr().solve(A_star);
  }else
  {
  //   for (std::size_t t = 0; t < 4; t++)
  //   {
  //     A(t,0) = V(t,4);
  //   }
  }
  //
  // r1 = -A(1,0)/2/A(0,0);
  // r2 = -A(2,0)/2/A(0,0);
  // R_square = (A(1,0) * A(1,0) + A(2,0) * A(2,0) - 4 * A(0,0) * A(3,0)) / 4 / A(0,0) / A(0,0);
  // center_x = r1 + x_mean;
  // center_y = r2 + y_mean;
  // radius = std::sqrt(R_square);

  // ASSERT_NEAR(center_x, 10.031622802828981, 1e-4);
  // ASSERT_NEAR(center_y, 41.235311934346704, 1e-4);
  // ASSERT_NEAR(radius, 35.659750288180462, 1e-4);
  ASSERT_NEAR(index, 2, 0.01);
}

TEST(TestSuite, testCase2)
{
  double threshold = 1e-4;
  double x_mean, y_mean, z_mean, x_sum = 0.0, y_sum = 0.0, z_sum = 0.0, sigma_min, eigen_min, r1, r2, R_square, radius;
  std::vector<Vector2D> position = {{-1,0},{-0.3,-0.06},{0.3,0.1},{1,0}};
  double center_x, center_y;

  for (std::size_t p = 0; p < position.size(); p++)
  {
    x_sum += position[p].x;
    y_sum += position[p].y;
  }
  x_mean = x_sum / position.size();
  y_mean = y_sum / position.size();

  for (std::size_t p = 0; p < position.size(); p++)
  {
    position[p].x -= x_mean;
    position[p].y -= y_mean;
    z_sum = z_sum + position[p].x * position[p].x + position[p].y * position[p].y;
  }
  z_mean = z_sum / position.size();

  MatrixXd Z = MatrixXd::Random(position.size(),4);
  for (std::size_t q = 0; q < position.size(); q++)
  {
    Z(q, 0) = position[q].x * position[q].x + position[q].y * position[q].y;
    Z(q, 1) = position[q].x;
    Z(q, 2) = position[q].y;
    Z(q, 3) = 1;
  }

  MatrixXd Z_transpose = Z.transpose();
  MatrixXd M = Z_transpose * Z / position.size();

  MatrixXd H = MatrixXd::Random(4,4);
  MatrixXd H_inverse = MatrixXd::Random(4,4);
  H(0,0) = 8 * z_mean;
  H(0,1) = H(0,2) = H(1,0) = H(1,2) = H(1,3) = H(2,0) = H(2,1) = H(2,3) = H(3,1) = H(3,2) = H(3,3) = 0;
  H(0,3) = H(3,0) = 2;
  H(1,1) = H(2,2) = 1;
  H_inverse(3,3) = -2 * z_mean;
  H_inverse(0,3) = H_inverse(3,0) = 0.5;
  H_inverse(1,1) = H_inverse(2,2) = 1;
  H_inverse(0,0) = H_inverse(0,1) = H_inverse(0,2) = H_inverse(1,0) = H_inverse(1,2) = H_inverse(1,3) = H_inverse(2,0) = H_inverse(2,1) = H_inverse(2,3) = H_inverse(3,1) = H_inverse(3,2);

  JacobiSVD <MatrixXd> svd(Z, ComputeThinU | ComputeThinV);
  MatrixXd singular_values = svd.singularValues();
  MatrixXd U = svd.matrixU();
  MatrixXd V = svd.matrixV();
  MatrixXd A = MatrixXd::Random(4,1);
  MatrixXd A_star = MatrixXd::Random(4,1);
  MatrixXd Q;
  MatrixXd ADD = MatrixXd::Random(singular_values.size(),singular_values.size());
  MatrixXd Y;
  MatrixXd eigen_values;

  for (std::size_t w = 0; w < singular_values.size(); w++)
  {
    for (std::size_t ww = 0; ww < singular_values.size(); ww++)
    {
      if (w == ww)
      {
        ADD(w,ww) = singular_values(w,0);
      }else
      {
        ADD(w,ww) = 0;
      }
    }

  }

  for (std::size_t r = 0; r < singular_values.size() - 1; r++)
  {
    if (singular_values(r) < singular_values(r + 1))
    {
      sigma_min = singular_values(r);
    }else
    {
      sigma_min = singular_values(r + 1);
    }
  }

  if (sigma_min > 0.01)
  {
    Y = V * ADD * V.transpose();
    Q = Y * H.inverse() * Y;
    SelfAdjointEigenSolver<MatrixXd> es(Q);
    eigen_values = es.eigenvalues();
    for (std::size_t s = 0; s < eigen_values.size()-1; s++)
    {
      if (eigen_values(s,0) < eigen_values((s+1),0))
      {
        eigen_min = s;
      }else
      {
        eigen_min = s + 1;
      }
    }
    for (std::size_t s = 0; s < 4; s++)
    {
      A_star(s, 0) = V(s, eigen_min);
    }
    A = Y.colPivHouseholderQr().solve(A_star);
  }else
  {
    for (std::size_t t = 0; t < 4; t++)
    {
      A(t,0) = V(t,4);
    }
  }

  r1 = -A(1,0)/2/A(0,0);
  r2 = -A(2,0)/2/A(0,0);
  R_square = (A(1,0) * A(1,0) + A(2,0) * A(2,0) - 4 * A(0,0) * A(3,0)) / 4 / A(0,0) / A(0,0);
  center_x = r1 + x_mean;
  center_y = r2 + y_mean;
  radius = std::sqrt(R_square);

  ASSERT_NEAR(center_x, -0.00015467658417971322, 1e-4);
  ASSERT_NEAR(center_y, 0.025080462853520565, 1e-4);
  ASSERT_NEAR(radius, 0.80369960858404954, 1e-4);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

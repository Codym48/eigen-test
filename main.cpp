#include <iostream>
#include <Eigen/Dense>
 
using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
 
int main()
{
  /// Ned = North-East-Down ... Body = Nose-RightWing-Down
  Vector3d velocityInNed(100.0, 0.0, 0.0);
  std::cout << "velocityInNed = " << std::endl << velocityInNed << std::endl;

  /// This is a Direction Cosine Matrix that, when premultiplying a
  /// vector, will rotate that vector about the +y axis by 30deg.
  /// This is a _rotation_ matrix, not a _transformation_ matrix.
  Matrix3d dcm;
  dcm = AngleAxisd(30.0*3.14/180.0, Vector3d::UnitY());
  std::cout << "dcm = " << std::endl << dcm << std::endl;

  /// This is invalid!
  Vector3d velocityInBody = dcm * velocityInNed;
  std::cout << "WRONG" << std::endl;
  std::cout << "velocityInBody = " << std::endl << velocityInBody << std::endl;

  /// This is valid.
  velocityInBody = dcm.transpose() * velocityInNed;
  std::cout << "RIGHT" << std::endl;
  std::cout << "velocityInBody = " << std::endl << velocityInBody << std::endl;

  /// This is a _transformation_ matrix, that can be used to transform
  /// a vector expressed in Ned coordinates into one expressed in Body
  /// coordinates, which by some naming conventions would be confusingly
  /// named trueRotBodyFromNed
  Matrix3d transformBodyFromNed = dcm.transpose();
  std::cout << "transformBodyFromNed = " << std::endl << transformBodyFromNed << std::endl;
  velocityInBody = transformBodyFromNed * velocityInNed;
}

#include <iostream>
#include <Eigen/Dense>
 
using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
 
int main()
{
  // Ned = North-East-Down ... Body = Nose-RightWing-Down
  Vector3d velocityInNed(100.0, 0.0, 0.0);
  std::cout << "velocityInNed = " << std::endl << velocityInNed << std::endl;

  // Let's try to model a rotation of the Body w.r.t. Ned.
  // This is a Direction Cosine Matrix that, when premultiplying a
  // vector, will rotate that vector about the +Z axis by 30deg.
  // This is a _rotation_ matrix, not a _transformation_ matrix.
  AngleAxisd aa = AngleAxisd(30.0*EIGEN_PI/180.0, Vector3d::UnitZ());
  Matrix3d dcm = aa.toRotationMatrix();
  std::cout << "dcm = " << std::endl << dcm << std::endl;

  // This is invalid!
  Vector3d velocityInBody = dcm * velocityInNed;
  std::cout << "WRONG" << std::endl;
  std::cout << "velocityInBody = " << std::endl << velocityInBody << std::endl;

  // This is valid.
  velocityInBody = dcm.transpose() * velocityInNed;
  std::cout << "RIGHT" << std::endl;
  std::cout << "velocityInBody = " << std::endl << velocityInBody << std::endl;

  // This is a _transformation_ matrix, that can be used to transform
  // a vector expressed in Ned coordinates into one expressed in Body
  // coordinates, which by some naming conventions would be confusingly
  // named trueRotBodyFromNed
  Matrix3d transformBodyFromNed = dcm.transpose();
  velocityInBody = transformBodyFromNed * velocityInNed;
  std::cout << "transformBodyFromNed = " << std::endl << transformBodyFromNed << std::endl;

  // All Euler Angle sequences store the angles in rotation order
  // ZYX Euler angles representing orientation of Body w.r.t. Ned (matches aa: <30,0,0>deg)
  Vector3d ea1 = dcm.eulerAngles(2, 1, 0);
  // ZYX Euler Angles representing orientation of Ned w.r.t. Body (opposite of aa, and wrapped because of numeric issues: <150,-180,180>deg)
  Vector3d ea2 = transformBodyFromNed.eulerAngles(2, 1, 0);
  // XYZ Euler Angles representing orientation of Body w.r.t. Ned (matches aa: <0,0,30>deg)
  Vector3d ea3 = dcm.eulerAngles(0, 1, 2);
  // XYZ Euler Angles representing orientation of Ned w.r.t. Body (opposite of aa: <0,0,-30>deg)
  Vector3d ea4 = transformBodyFromNed.eulerAngles(0, 1, 2);

  // See if the DCM->Euler angle conversion trips over asin(>1.0)
  aa = AngleAxisd(EIGEN_PI/2.0, Vector3d::UnitY());
  dcm = aa.toRotationMatrix(); // 1.0 in the (0,2) location
  ea1 = dcm.eulerAngles(2, 1, 0);
  dcm(0,2) += 0.0000000002; // >1.0 in the (0,2) location
  dcm(2,0) -= 0.0000000002; // <-1.0 in the (2,0) location
  ea2 = dcm.eulerAngles(2, 1, 0);
  std::cout << "ea2 = " << std::endl << ea2 << std::endl;
}

#include "../include/dq2tfm.h"
// normalized dq!!!
Matrix4d dq2tfm(const DQ &dq){
    // Extract the translation vector from the dual quaternion
    Vector3d dq_t_vec = dq.translation().vec3();
    std::cout<<"dq translation: "<<std::endl<<dq_t_vec<<std::endl;
    // Extract the rotation quaternion from the dual quaternion
    Vector4d dq_r_vec = dq.rotation().vec4();
    Quaterniond rotationQuaternion(dq_r_vec(0), dq_r_vec(1), dq_r_vec(2), dq_r_vec(3));

    // Convert the rotation quaternion into a 3x3 rotation matrix
    Matrix3d rotationMatrix = rotationQuaternion.toRotationMatrix();
    double roll, pitch, yaw;
    roll = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));
    pitch = atan2(-rotationMatrix(2, 0), sqrt(rotationMatrix(2, 1) * rotationMatrix(2, 1) + rotationMatrix(2, 2) * rotationMatrix(2, 2)));
    yaw = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
    double scal = 180/pi;
    std::cout<<"Euler angle in matlab: "<<roll*scal<<" "<<pitch*scal<<" "<<yaw*scal<<std::endl;

    // Assemble the 3x3 rotation matrix and the translation vector into a 4x4 transformation matrix
    Matrix4d transformationMatrix = Matrix4d::Identity();
    transformationMatrix.block<3, 3>(0, 0) = rotationMatrix;
    transformationMatrix.block<3, 1>(0, 3) = dq_t_vec;

    // Print the resulting transformation matrix
    std::cout << "Transformation Matrix:" << std::endl << transformationMatrix << std::endl;
    return transformationMatrix;
}

//
// Created by Megan Finch on 26/01/2022.
//

#include "constants.h"

#include <gtest/gtest.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <cost/simple_reprojection_error.h>
#include <types.h>

TEST(SimpleReprojectionErrorTest, ComputeCorrectError) {
    /*
     * I think these are the expected errors?
     * Error x: [-0.00752335 ; 73.007, 2410.17, 499.382, 664.835, 0, -5.05074, 642.661, -3.04149, -170.319, 0.00759699]
     * Error y: [0.53642 ; -2571.32, 137.755, -318.594, 0, 664.835, 135.218, 37.3859, 664.187, 133.216, -0.203385]
     */

    // Arrange
    cv::Point2d point2d(19, -509);
    SimpleReprojectionError computeError = SimpleReprojectionError(point2d.x, point2d.y);
    PoseVector pv(-0.004077719524502754, -0.251457691192627, 0.005174771882593632, 0.9919545650482178, -0.0116012804210186, 0.1260616481304169);
    cv::Vec3d point3d(-0.033097, -0.765754, 3.75711);
    double focal = 2500;
    double expected [2] = { -0.00752335, 0.53642 };

    // Act
    double actual [2] = {0,0};
    computeError(pv.val, point3d.val, &focal, actual);
    std::cout << actual[0] << std::endl;
    std::cout << actual[1] << std::endl;

    // Assert
    EXPECT_NEAR(actual[0], expected[0], ALLOWED_ERROR);
    EXPECT_NEAR(actual[1], expected[1], ALLOWED_ERROR);
}
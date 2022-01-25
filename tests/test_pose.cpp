//
// Created by Megan Finch on 21/01/2022.
//

#include <opencv2/core/matx.hpp>

#include <gtest/gtest.h>
#include <pose.h>

const double ALLOWABLE_ERROR = 0.01;

TEST(PoseTest, ConvertPoseToPoseVector) {
    // Arrange
    cv::Matx34d mat(0.85861647, -0.00413041, -0.51260191, 0.70863354,
        0.0074001253, 0.99996322, 0.004337891, -0.0027955303,
        0.51256514, -0.0075179031, 0.85861546, 0.26115116);
    Pose pose(mat);

    PoseVector expected(-0.006224129814654589, -0.5381986498832703, 0.006053373217582703, 0.7086335420608521, -0.002795530250295997, 0.2611511647701263);

    // Act
    PoseVector actual = pose.getPoseVector();

    // Assert
    for (size_t i = 0; i < 6; i++) {
        EXPECT_NEAR(actual(i), expected(i), ALLOWABLE_ERROR);
    }
}

TEST(PoseTest, InvertPoseToPoseVector) {
    // Arrange
    // Define pose
    cv::Matx34d mat(0.85861647, -0.00413041, -0.51260191, 0.70863354,
                    0.0074001253, 0.99996322, 0.004337891, -0.0027955303,
                    0.51256514, -0.0075179031, 0.85861546, 0.26115116);
    Pose pose(mat);
    auto expected = pose.getProjectionMatrix();

    // Act
    // Convert to pose vector and back
    auto pv = pose.getPoseVector();
    auto actual = Pose(pv).getProjectionMatrix();

    // Assert
    // If both conversions are correct, actual = expected, give or take rounding errors
    for (size_t r = 0; r < 3; r++) {
        for (size_t c = 0; c < 4; c++) {
            EXPECT_NEAR(actual(r,c), expected(r,c), ALLOWABLE_ERROR);
        }
    }

    // EXPECT_EQ(0,1);
}


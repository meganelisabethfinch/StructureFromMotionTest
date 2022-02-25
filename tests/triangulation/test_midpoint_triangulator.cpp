//
// Created by Megan Finch on 24/02/2022.
//

#include <gtest/gtest.h>
#include <opencv2/core/matx.hpp>
#include <triangulation/triangulator.h>
#include <triangulation/midpoint_triangulator.h>

TEST(MidpointTriangulatorTest, TriangulateCorrectPointWhenExactIntersection) {
    // Arrange
    // L1 = { p = q1 + lambda1 * v1 }
    cv::Vec3d q1 = { -2, -1, 0 };
    cv::Vec3d v1 = { 1, 1, 1 };

    // L2 = { p = q2 + lambda2 * v2 }
    cv::Vec3d q2 = { 8, -6, -11 };
    cv::Vec3d v2 = { -2, 3, 5 };

    cv::Point3d expected = { 2, 3, 4 };

    // Act
    cv::Point3d actual = MidpointTriangulator::triangulatePoint(q1, q2, v1, v2);

    // Assert
    EXPECT_EQ(actual.x, expected.x);
    EXPECT_EQ(actual.y, expected.y);
    EXPECT_EQ(actual.z, expected.z);
};

TEST(MidpointTriangulatorTest, TriangulateCorrectPointWhenApproximateIntersection) {
    // Arrange
    // Two skew lines
    // L1 = { p = q1 + lambda1 * v1 }
    cv::Vec3d q1 = { -1,4,-2 };
    cv::Vec3d v1 = { 3,3,5 };

    // L2 = { p = q2 + lambda2 * v2 }
    cv::Vec3d q2 = { 2,5,-3 };
    cv::Vec3d v2 = { 1,1,2 };

    cv::Point3d expected = { 14.5, 18.5, 23 };

    // Act
    cv::Point3d actual = MidpointTriangulator::triangulatePoint(q1,q2,v1,v2);

    // Assert

    EXPECT_EQ(actual.x, expected.x);
    EXPECT_EQ(actual.y, expected.y);
    EXPECT_EQ(actual.z, expected.z);
};
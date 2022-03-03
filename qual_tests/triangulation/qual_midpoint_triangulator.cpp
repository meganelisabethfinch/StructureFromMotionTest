//
// Created by Megan Finch on 14/01/2022.
//


#include <gtest/gtest.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include "triangulation/triangulator.h"

TEST(MidpointTriangulatorQual, VisualiseImagePoints) {
    // Arrange
    auto fn = "/Users/meganfinch/CLionProjects/StructureFromMotionTest/data/lion/Camera.013.png";
    cv::Mat data = cv::imread(fn, cv::IMREAD_COLOR);

    // cv::imshow("Input image", data);
    // cv::waitKey(0);

    auto midpoint = Triangulator::create(TriangulatorType::MIDPOINT);
    // Act

    EXPECT_EQ(0,1);
}

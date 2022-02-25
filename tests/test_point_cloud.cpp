//
// Created by Megan Finch on 10/02/2022.
//
#include <gtest/gtest.h>
#include <point_cloud.h>

#include "matches.h"
#include "imagecollection.h"


TEST(TestPointCloud, mergePointsProducesCorrectSizePointCloud) {
    // Arrange
    ImageCollection images = ImageCollection("/Users/meganfinch/CLionProjects/StructureFromMotionTest/data/lion-xs");
    PointCloud pc;

    // Act

    // Assert
    EXPECT_EQ(0,1);
}
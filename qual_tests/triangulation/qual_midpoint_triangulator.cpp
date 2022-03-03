//
// Created by Megan Finch on 14/01/2022.
//

#include <gtest/gtest.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <triangulation/midpoint_triangulator.h>
#include <cli_util.h>
#include "triangulation/triangulator.h"

TEST(MidpointTriangulatorQual, VisualiseImagePoints) {
    // Arrange
    auto fn = "/Users/meganfinch/CLionProjects/StructureFromMotionTest/data/lion/Camera.013.png";
    auto data = cv::imread(fn, cv::IMREAD_COLOR);
    auto image = Image(0, "Test", data);

    auto detector = CLIUtilities::CreateDetector(DetectorType::SIFT);
    Features features = Features(detector, image);

    // Default intrinsic matrix
    auto intrinsic = Camera(image);

    // 'Correct' pose taken from reconstruction with linear triangulation
    auto mat = cv::Matx34d(-0.02807987371218846, -0.02554154289262983, -0.9992793154463729, 6.045161720350626,
    0.09578170244602439, 0.9950049844577836, -0.02812376896341345, 0.1861102762690821,
    0.9950062241859842, -0.09650238593335485, -0.02549320184521835, 5.951125005589014);
    auto pose = Pose(mat);

    auto midpoint = cv::makePtr<MidpointTriangulator>();
    PointCloud pc;

    // Act
    for (size_t i = 0; i < features.size(); i++) {
        auto pt = features.getPoint(i);
        // Get image point
        auto pt3d = midpoint->backProjectPointToPoint(pt, intrinsic, pose);
        Point3DInMap pt3dMap;
        pt3dMap.setPoint(pt3d.x, pt3d.y, pt3d.z);
        pt3dMap.originatingViews.emplace(0, i);
        pc.addPoint(pt3dMap);
    }

    pc.toPlyFile("/Users/meganfinch/CLionProjects/StructureFromMotionTest/data/out/qual_tests/visual.ply",
                 { features },
                 { image });

    EXPECT_EQ(0,1);
}

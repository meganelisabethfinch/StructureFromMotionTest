//
// Created by Megan Finch on 14/01/2022.
//

#include <gtest/gtest.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <triangulation/midpoint_triangulator.h>
#include <cli_util.h>
#include "triangulation/triangulator.h"
#include <fstream>

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
    // Add 1st image
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
}

TEST(MidpointTriangulatorQual, VisualiseImagePointsSeveralImages) {
    // Arrange
    std::vector<std::string> fns = {
            "/Users/meganfinch/CLionProjects/StructureFromMotionTest/data/lion/Camera.013.png",
            "/Users/meganfinch/CLionProjects/StructureFromMotionTest/data/lion/Camera.015.png",
            "/Users/meganfinch/CLionProjects/StructureFromMotionTest/data/lion/Camera.004.png"
    };

    auto detector = CLIUtilities::CreateDetector(DetectorType::SIFT);
    std::vector<Image> mImages;
    std::vector<Features> mFeatures;
    std::vector<Camera> mCameras;
    for (size_t id = 0; id < fns.size(); id++) {
        auto data = cv::imread(fns.at(id), cv::IMREAD_COLOR);
        auto image = Image(id, "Test", data);
        mImages.push_back(image);
        mFeatures.emplace_back(detector, image);
        mCameras.emplace_back(image); // Default intrinsic matrix
    }

    std::vector<Pose> mPoses;
    // 'Correct' poses taken from reconstruction with linear triangulation
    mPoses.emplace_back(Pose(cv::Matx34d(-0.02807987371218846, -0.02554154289262983, -0.9992793154463729, 6.045161720350626,
                           0.09578170244602439, 0.9950049844577836, -0.02812376896341345, 0.1861102762690821,
                           0.9950062241859842, -0.09650238593335485, -0.02549320184521835, 5.951125005589014)));
    mPoses.emplace_back(Pose(cv::Matx34d(-0.2162838888991954, -0.01773074871021963, -0.9761695036994423, 5.937455394723456,
                            0.08964194787624172, 0.9952512104238447, -0.03793875763418892, 0.2456000957218092,
                            0.9722065627137032, -0.09571127781012859, -0.2136673833704122, 7.087962927113679)));
    mPoses.emplace_back(Pose(cv::Matx34d(0.9463811837323562, -0.003497948655846861, -0.3230331553146599, 1.936359860306994,
    0.003301824815025525, 0.9999938818054799, -0.001155120988842052, 0.006606728293530107,
    0.3230352194888895, 2.658588048049275e-05, 0.9463869432018576, 0.3184415607894677)));

    auto midpoint = cv::makePtr<MidpointTriangulator>();
    PointCloud pc;

    // Act
    for (size_t i = 0; i < mImages.size(); i++) {
        auto features = mFeatures.at(i);
        for (size_t j = 0; j < features.size(); j++) {
            auto pt = features.getPoint(j);
            // Get image point
            auto pt3d = midpoint->backProjectPointToPoint(pt, mCameras.at(i), mPoses.at(i));
            Point3DInMap pt3dMap;
            pt3dMap.setPoint(pt3d.x, pt3d.y, pt3d.z);
            pt3dMap.originatingViews.emplace(i, j);
            pc.addPoint(pt3dMap);
        }
    }

    pc.toPlyFile("/Users/meganfinch/CLionProjects/StructureFromMotionTest/data/out/qual_tests/visual2.ply",
                 mFeatures,
                 mImages);
}

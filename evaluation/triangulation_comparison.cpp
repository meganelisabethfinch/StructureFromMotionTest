//
// Created by Megan Finch on 28/02/2022.
//

#include <gtest/gtest.h>
#include <triangulation/triangulator.h>
#include <imagecollection.h>
#include <cli_util.h>

/*
class TriangulationEvaluation : public testing::Test {
protected:
    std::vector<Image> images;
    std::vector<Camera> cameras;
    std::map<Image, Pose> poses;
    std::vector<Features> features;
    Matches matches;

    std::map<std::string, cv::Ptr<Triangulator>> triangulators;

    void SetUp() override {
        ImageCollection ic("/Users/meganfinch/CLionProjects/StructureFromMotionTest/data/lion-xs");
        // TODO: move ic -> images vector

        // Detect features
        auto detector = CLIUtilities::CreateDetector(DetectorType::SIFT);
        for (auto & image : images) {
            features.emplace_back(Features(detector, image));
        }

        // Find matches
        auto matcher = CLIUtilities::CreateMatcher(MatcherType::FLANNBASED);
        matches = Matches(matcher, features);

    }

};

TEST_F(TriangulationEvaluation, CompareReprojectionError) {
    // Arrange
    auto midpoint = CLIUtilities::CreateTriangulator(TriangulatorType::MIDPOINT);
    auto linear = CLIUtilities::CreateTriangulator(TriangulatorType::LINEAR);

    // Act
    for (ImageID i = 0; i < images.size() - 1; i++) {
        for (ImageID j = i + 1; j < images.size(); j++) {
            // Choose images i and j

            for (auto match : matches.get({i,j})) {
                // Choose pair of points

                for (const auto& kv : triangulators) {
                    // Choose a triangulation method
                    auto name = kv.first;
                    auto triangulator = kv.second;

                    // TODO: triangulate!
                }
            }
        }
    }

    // Assert
    EXPECT_EQ(0,1);
}
*/
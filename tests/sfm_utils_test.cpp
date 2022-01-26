//
// Created by Megan Finch on 01/02/2022.
//

#include <opencv2/core/matx.hpp>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <gtest/gtest.h>

#include <cli_util.h>
#include <sfm_util.h>

class SFMUtilitiesTest : public testing::Test {
protected:
    // Per test-suite set-up, called before the first test in this test suite.
    // Can be omitted if not needed.
    static void SetUpTestSuite() {
        // TODO
    }
};

TEST(SFMUtilitiesTest, SortHomographyInliersCorrectly) {
    // Arrange
    // TODO: put this all into per-test-fixture set-up
    std::vector<Image> mImages;
    std::vector<Features> mImageFeatures;

    std::string directory = "/Users/meganfinch/CLionProjects/StructureFromMotionTest/data/lion/";
    std::vector<cv::String> filenames;
    cv::glob(directory + "/*.png", filenames, false);

    auto detector = CLIUtilities::CreateDetector(DetectorType::SIFT);

    for (const auto& fn : filenames) {
        cv::Mat data = cv::imread(fn, cv::IMREAD_COLOR);
        Image image = Image(mImages.size(), std::filesystem::path(fn).filename(), data);

        mImages.push_back(image);
        mImageFeatures.emplace_back(Features(detector, image));
    }

    auto matcher = CLIUtilities::CreateMatcher(MatcherType::FLANNBASED);
    auto mFeatureMatchMatrix = Matches(matcher, mImageFeatures);

    // Act
    auto sortedPairs = SFMUtilities::SortViewsForBaseline(mImageFeatures, mFeatureMatchMatrix);

    // Assert
    // Check the first pair is (5,17) with inlier ratio 0.0529801
    auto kv_pair = sortedPairs.begin();
    EXPECT_EQ(kv_pair->second.left, 5);
    EXPECT_EQ(kv_pair->second.right, 17);
    EXPECT_EQ(kv_pair->first, 0.0529801);

    // Check the second pair is (12,26) with inlier ratio 0.0538462
    kv_pair++;
    EXPECT_EQ(kv_pair->second.left, 12);
    EXPECT_EQ(kv_pair->second.right, 26);
    EXPECT_EQ(kv_pair->first, 0.0538462);
}


TEST(SFMUtilitiesTest, CountHomographyInliersCorrectly) {
    // Arrange
    // TODO: put this all into per-test-fixture set-up
    std::vector<Image> mImages;
    std::vector<Features> mImageFeatures;

    std::string directory = "/Users/meganfinch/CLionProjects/StructureFromMotionTest/data/lion/";
    std::vector<cv::String> filenames;
    cv::glob(directory + "/*.png", filenames, false);

    auto detector = CLIUtilities::CreateDetector(DetectorType::SIFT);

    for (const auto& fn : filenames) {
        cv::Mat data = cv::imread(fn, cv::IMREAD_COLOR);
        Image image = Image(mImages.size(), std::filesystem::path(fn).filename(), data);

        mImages.push_back(image);
        mImageFeatures.emplace_back(Features(detector, image));
    }

    auto matcher = CLIUtilities::CreateMatcher(MatcherType::FLANNBASED);
    auto mFeatureMatchMatrix = Matches(matcher, mImageFeatures);

    // Act
    auto ip = ImagePair(5,17);
    auto numInliers = SFMUtilities::CountHomographyInliers(mImageFeatures[ip.left],
                                                           mImageFeatures[ip.right],
                                                           mFeatureMatchMatrix.get(ip));

    // Assert
    // Expect (5,17) to have (about) 8 inliers
    EXPECT_EQ(numInliers, 8);
}

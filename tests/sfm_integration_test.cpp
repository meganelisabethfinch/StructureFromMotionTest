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

TEST(SFMUtilitiesTest, CorrectNumberOfMatches) {
    // TODO: put this all into per-test-fixture set-up
    std::vector<Image> mImages;
    std::vector<Features> mImageFeatures;

    std::string directory = "/Users/meganfinch/CLionProjects/StructureFromMotionTest/data/lion-s/";
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
    std::vector<ImagePair> imagePairs = {ImagePair(0,1), ImagePair(1,2), ImagePair(2,6)};
    std::vector<size_t> expected = { 2120, 165, 101 };
    std::vector<size_t> actual;
    for (auto& ip : imagePairs) {
        actual.emplace_back(mFeatureMatchMatrix.get(ip).size());
    }

    // Assert
    for (size_t i = 0; i < imagePairs.size(); i++) {
        EXPECT_NEAR(actual.at(i), expected.at(i), 5);
    }
}

TEST(SFMUtilitiesTest, SortHomographyInliersCorrectly) {
    // Arrange
    // TODO: put this all into per-test-fixture set-up
    std::vector<Image> mImages;
    std::vector<Features> mImageFeatures;

    std::string directory = "/Users/meganfinch/CLionProjects/StructureFromMotionTest/data/lion-s/";
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
    // Check the first pair is (1,2) with inlier ratio 0.0787879
    auto kv_pair = sortedPairs.begin();
    EXPECT_EQ(kv_pair->second.left, 1);
    EXPECT_EQ(kv_pair->second.right, 2);
    EXPECT_EQ(kv_pair->first, 0.0787879);

    // Check the second pair is (2,6) with inlier ratio 0.0792079
    kv_pair++;
    EXPECT_EQ(kv_pair->second.left, 2);
    EXPECT_EQ(kv_pair->second.right, 6);
    EXPECT_EQ(kv_pair->first, 0.0792079);
}

TEST(SFMUtilitiesTest, CountHomographyInliersCorrectly) {
    // Arrange
    // TODO: put this all into per-test-fixture set-up
    std::vector<Image> mImages;
    std::vector<Features> mImageFeatures;

    std::string directory = "/Users/meganfinch/CLionProjects/StructureFromMotionTest/data/lion-s/";
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
    std::vector<ImagePair> imagePairs = {ImagePair(0,1), ImagePair(1,2), ImagePair(2,6)};
    std::vector<double> expected = {1474, 13, 8};
    std::vector<double> actual;
    for (auto& ip : imagePairs) {
        auto numInliers = SFMUtilities::CountHomographyInliers(mImageFeatures[ip.left],
                                                                mImageFeatures[ip.right],
                                                                mFeatureMatchMatrix.get(ip));
        actual.push_back(numInliers);
    }
    // Assert
    for (size_t i = 0; i < imagePairs.size(); i++) {
        EXPECT_EQ(actual.at(i), expected.at(i));
    }
}
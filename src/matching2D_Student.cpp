#include <numeric>
#include "matching2D.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource,
                      std::vector<cv::KeyPoint> &kPtsRef,
                      cv::Mat &descSource,
                      cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches,
                      std::string descriptorType,
                      std::string matcherType,
                      std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0) {
        int normType = descriptorType == "SIFT" ? cv::NORM_L1 : cv::NORM_HAMMING;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0) {
        matcher = cv::FlannBasedMatcher::create();
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
        const int k = 2;
        const float ratio_threshold = 0.8f;
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descSource, descRef, knn_matches, k);

        //cout << "Found " << knn_matches.size() << " pairs of matches." << std::endl;

        for (size_t i = 0; i < knn_matches.size(); i++) {
            if (knn_matches[i][0].distance < ratio_threshold * knn_matches[i][1].distance) {
                matches.push_back(knn_matches[i][0]);
            }
        }        
    }
}

cv::Ptr<cv::DescriptorExtractor> createDescriptorExtractor(const std::string& descriptorType) {
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    } else if (descriptorType == "BRIEF") {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    } else if (descriptorType == "ORB") {
        extractor = cv::ORB::create();
    } else if (descriptorType == "FREAK") {
        extractor = cv::xfeatures2d::FREAK::create();
    } else if (descriptorType == "AKAZE") {
        extractor = cv::AKAZE::create();
    } else if (descriptorType == "SIFT") {
        extractor = cv::xfeatures2d::SIFT::create();
    } else {
        std::cerr << "Unknown descriptor type: " << descriptorType << std::endl;
        throw std::runtime_error("Unknown descriptor type");
    }

    return extractor;
}


// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, cv::Ptr<cv::DescriptorExtractor> extractor) {
    // perform feature description
    extractor->compute(img, keypoints, descriptors);
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img) {
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    int apertureSize = 5;
    double k = 0.04;
    int minResponse = 100;

    cv::Mat dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::Mat dst_norm, dst_norm_scaled;

    cv::cornerHarris(img, dst, blockSize, apertureSize, k);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());

    // Look for prominent corners and instantiate keypoints
    double maxOverlap = 0.0; // max. permissible overlap between two features in %, used during non-maxima suppression
    for (size_t j = 0; j < dst_norm.rows; j++) {
        for (size_t i = 0; i < dst_norm.cols; i++) {
            int response = (int)dst_norm.at<float>(j, i);
            if (response > minResponse) { // only store points above a threshold

                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i, j);
                newKeyPoint.size = 2 * apertureSize;
                newKeyPoint.response = response;

                // perform non-maximum suppression (NMS) in local neighbourhood around new key point
                bool bOverlap = false;
                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                    if (kptOverlap > maxOverlap)
                    {
                        bOverlap = true;
                        if (newKeyPoint.response > (*it).response)
                        {                      // if overlap is >t AND response is higher for new kpt
                            *it = newKeyPoint; // replace old key point with new one
                            break;             // quit loop over keypoints
                        }
                    }
                }
                if (!bOverlap)
                {                                     // only add new key point if no overlap has been found in previous NMS
                    keypoints.push_back(newKeyPoint); // store new keypoint in dynamic list
                }
            }
        } // eof loop over cols
    }     // eof loop over rows

}

cv::Ptr<cv::Feature2D> createKeypointDetector(const std::string& detectorType) {
    cv::Ptr<cv::Feature2D> det;

    if (detectorType == "FAST") {
        det = cv::FastFeatureDetector::create();
    } else if (detectorType == "BRISK") {
        det = cv::BRISK::create();
    } else if (detectorType == "ORB") {
        det = cv::ORB::create();
    } else if (detectorType == "AKAZE") {
        det = cv::AKAZE::create();
    } else if (detectorType == "SIFT") {
        det = cv::xfeatures2d::SIFT::create();
    } else {
        std::cerr << "Non-modern detector type: " << detectorType << std::endl;
    }
    return det;
}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Ptr<cv::Feature2D> det) {
    det->detect(img, keypoints);
}

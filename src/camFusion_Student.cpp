
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <set>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;

static std::vector<double> extractX(const std::vector<LidarPoint>& points) {
    std::vector<double> xs;
    for (const auto& p : points) {
        xs.push_back(p.x);
    }
    return xs;
}

static double median(std::vector<double> ds) {
    if (ds.empty()) {
        return NAN;
    }

    std::sort(ds.begin(), ds.end());

    if (ds.size() % 2 == 0) {
        return (ds.at(ds.size() / 2) + ds.at(ds.size() / 2 - 1)) / 2.0;
    } else {
        return ds.at(ds.size() / 2);
    }
}

static double min(const std::vector<double>& ds) {
    if (ds.empty()) {
        return NAN;
    }

    double min = 100000;
    for (const auto& d : ds) {
        min = std::min(min, d);
    }

    return min;
}

static double percentile(double p, std::vector<double> ds) {
    int index = std::min(static_cast<size_t>(std::min(1.0, p / 100.0) * ds.size()), ds.size() - 1);
    std::nth_element(ds.begin(), ds.begin() + index, ds.end());
    return ds.at(index);
}

static double width(const std::vector<double>& ds) {
    if (ds.empty()) {
        return 0.0;
    }

    double min = 1000000;
    double max = -10000000;
    for (const auto& d : ds) {
        min = std::min(min, d);
        max = std::max(max, d);
    }

    return max - min;
}

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, xmedian = %2.2f m, yw=%2.2f m",
                xwmin, median(extractX(it1->lidarPoints)), ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}

static cv::Point2f centerOf(const std::vector<cv::KeyPoint>& points) {
    float x = 0;
    float y = 0;
    for (const auto& p : points) {
        x += p.pt.x;
        y += p.pt.y;
    }

    return cv::Point2f(x / points.size(), y / points.size());
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox,
                              std::vector<cv::KeyPoint> &kptsPrev,
                              std::vector<cv::KeyPoint> &kptsCurr,
                              std::vector<cv::DMatch> &kptMatches)
{
    std::vector<cv::DMatch> inboxMatches;
    std::vector<cv::KeyPoint> inboxKps;
    for (const auto& match : kptMatches) {
        const auto kpCurr = kptsCurr.at(match.trainIdx);
        if (boundingBox.roi.contains(kpCurr.pt)) {
            inboxMatches.push_back(match);
            inboxKps.push_back(kpCurr);
        }
    }

    cv::Point2f center = centerOf(inboxKps);
    double var = std::accumulate(inboxKps.begin(), inboxKps.end(), 0.0,
        [center](double d, const cv::KeyPoint& kp) {
            cv::Point2f error = kp.pt - center;
            return d + error.dot(error);
        }) / inboxKps.size();
    double stdev = std::sqrt(var);
    double sigma = 1.0;

    boundingBox.kptMatches.clear();
    std::copy_if(inboxMatches.begin(), inboxMatches.end(), std::back_inserter(boundingBox.kptMatches),
        [kptsCurr, center, stdev, sigma](const cv::DMatch& match) {
            const auto kp = kptsCurr.at(match.trainIdx);
            return cv::norm(kp.pt - center) < stdev * sigma;
        });

#if 0
    std::cout << "clusterKpMatchesWithROI: inbox = " << inboxMatches.size()
            << ", filtered = " << boundingBox.kptMatches.size() << std::endl;
    std::cout << "bbox: " << boundingBox << std::endl;
    for (const auto& kp : boundingBox.keypoints) {
        std::cout << "  kp: " << kp.pt << std::endl;
    }
#endif
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = it1 + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute camera-based TTC from distance ratios
    double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();
    double medianDistRatio = distRatios.size() % 2 == 0
        ? (distRatios.at(distRatios.size() / 2 - 1) + distRatios.at(distRatios.size() / 2)) / 2.0
        : distRatios.at(distRatios.size() / 2);

    double dT = 1 / frameRate;
    //    TTC = -dT / (1 - meanDistRatio);
    TTC = -dT / (1 - medianDistRatio);
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC) {
    std::vector<double> xPrev = extractX(lidarPointsPrev);
    std::vector<double> xCurr = extractX(lidarPointsCurr);
    double prev = percentile(10.0, xPrev);
    double curr = percentile(10.0, xCurr);

    if (curr >= prev) {
        // do not collide
        TTC = 0.0;
        return;
    }

    double speed = (prev - curr) / (1.0 / frameRate);
    TTC = curr / speed;

#if 0
    std::cout << "TTCLidar = " << TTC
    << ", prev/curr = " << prev << "/" << curr << " (" << prev - curr << ")"
    << ", min = " << min(xPrev) << "/" << min(xCurr) << " (" << min(xPrev) - min(xCurr) << ")"
    << ", height " << width(xPrev) << "/" << width(xCurr)
    << std::endl;
#endif

    return;
}

static bool compare(const std::tuple<int, int, int>& a, const std::tuple<int, int, int>& b) {
    return std::get<2>(a) > std::get<2>(b);
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches,
                        std::map<int, int> &bbBestMatches,
                        DataFrame &prevFrame,
                        DataFrame &currFrame)
{
    std::map<std::pair<int, int>, int> matchCandidates;
    for (const auto& match : matches) {
        const auto& prevKp = prevFrame.keypoints.at(match.queryIdx);
        const auto& currKp = currFrame.keypoints.at(match.trainIdx);

        for (const auto& prevBox : prevFrame.boundingBoxes) {
            if (not prevBox.roi.contains(prevKp.pt)) {
                continue;
            }

            for (auto& currBox : currFrame.boundingBoxes) {
                if (not currBox.roi.contains(currKp.pt)) {
                    continue;
                }

                currBox.kptMatches.push_back(match);

                auto key = std::make_pair(prevBox.boxID, currBox.boxID);
                if (matchCandidates.find(key) != matchCandidates.end()) {
                    matchCandidates[key]++;
                } else {
                    matchCandidates[key] = 1;
                }
            }
        }
    }

    std::vector<std::tuple<int, int, int>> candidates;
    for (const auto& c : matchCandidates) {
        candidates.push_back(std::make_tuple(c.first.first, c.first.second, c.second));
    }
    std::sort(candidates.begin(), candidates.end(), compare);

#if 0
    for (const auto& c : candidates) {
        std::cout << "prev: " << std::get<0>(c) << ", curr: " << std::get<1>(c) << ", count: " << std::get<2>(c) << std::endl;
    }
#endif

    std::set<int> matchFoundPrev;
    std::set<int> matchFoundCurr;
    for (const auto& c : candidates) {
        int prevId = std::get<0>(c);
        int currId = std::get<1>(c);
        if (matchFoundPrev.find(prevId) == matchFoundPrev.end() and
            matchFoundCurr.find(currId) == matchFoundCurr.end()) {
            bbBestMatches[prevId] = currId;
        }

        matchFoundPrev.insert(prevId);
        matchFoundCurr.insert(currId);
    }
}

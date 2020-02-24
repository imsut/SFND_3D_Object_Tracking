# 3D Object Tracking

## FP.1 Match 3D Objects

`matchBoundingBoxes` is implemented that generates bounding box matches by
1. assign each keypoint match (`cv::DMatch`) to possibly more than one pairs of "bbox in previous frame" and "bbox in current frame" that enclose the keypoints in the keypoint match.
2. sort the pairs of bounding boxes by the number of enclosing keypoint matches.
3. pick a bounding box pair that has the maximum number of keypoint matches, and the bounding boxes are not picked yet.
4. repeat step 3 until all bounding boxes in both previous frame and current frame are covered.


## FP.2 Compute Lidar-based TTC

`computeTTCLidar` is implemented to compute Lidar-based TTC.
To avoid estimation error by outliers, it uses p10 distances from the ego car to the lidar points in the preceding car's bounding box.
p10 was chosen to remove outliers from calculation and also to keep it close enough to the ego car.
(If I use p50 or median, an dthe lidar points are spared in x-axis, the estimated TTC may be too big/conservative.)


## FP.3 Associate Keypoint Correspondences with Bounding Boxes

`clusterKptMatchesWithROI` is implemented that finds keypoint matches associating to a given bounding box by
1. simply selecting all keypoint matches that geographycally fall in the given bounding box.
2. computing standard deviation over a distance from each point to the center of keypoints.
3. retaining only keypoints whose distance is within one sigma from the center point.

## FP.4 Compute Camera-based TTC

`computeTTCCamera` is implemented that
1. computes the ratios of distance from the two frames by computing distance of every pair of keypoints in the frame.
2. takes median of the ratios. Median is more robust than mean when there exist outliers.


## FP.5 Performance Evaluation 1

Below is the series of TTC numbers based on Lidar using p10 value of X-axis distances from ego-car to each Lidar points.

|                  |distance p10 [m]    | distance delta [m] | TTC [s] |
|------------------|--------------------|---------|---------|
| Image 0          | 7.991              |         |         |
| Image 1          | 7.935              | -0.056  | 14.1696 |
| Image 2          | 7.87               | -0.065  | 12.1077 |
| Image 3          | 7.837              | -0.033  | 23.7485 |
| Image 4          | 7.768              | -0.069  | 11.258  |
| Image 5          | 7.703              | -0.065  | 11.8508 |
| Image 6          | 7.638              | -0.065  | 11.7508 |
| Image 7          | 7.584              | -0.054  | 14.0445 |
| Image 8          | 7.548              | -0.036  | 20.9665 |
| Image 9          | 7.486              | -0.062  | 12.0742 |


The TTC jumps at frame 3 and frame 8, when the distance change from the previous frame drops to roughly -0.03 (from -0.05 ~ -0.06). Smaller distance change leads to smalller estimated velocity hence larger TTC.

Because such jumps may be false negatives, it's better not to produce big increase in TTC. Falsy big TTC estimate may result in safety system (such as automatic braking) not kicking in. (whereas big decrease in TTC, which may be false positive, is ok from safety perspective).

In order to address this, we can
* cap TTC delta from the previous TTC estimate.
* lookback longer and use multiple past frames to estimate velocity. (currently, velocity is calculated based only on the current frame and the previous frame).


## FP.6 Performance Evaluation 2

Attached below is the table showing TTC estimate in each frame by detector/descriptor combination. Some conbinations contain nan/inf values because they find too few keypoints.


| Detector/Descriptor | 1               | 2       | 3        | 4         | 5        | 6         | 7         | 8         | 9        | 10       | 11        | 12        | 13        | 14        | 15       | 16       | 17        | 18       |
|-----------------|---------|----------|-----------|----------|-----------|-----------|-----------|----------|----------|-----------|-----------|-----------|-----------|----------|----------|-----------|----------|----------| 
| SHITOMASI/BRISK | nan     | 15.4247  | 30.9699   | nan      | nan       | nan       | nan       | 27.2632  | 25.0548  | -inf      | 10.8444   | 12.1911   | 8.36241   | 7.22097  | 10.5483  | 5.61555   | -51.978  | 17.3019  | 
| SHITOMASI/BRIEF | 8.52258 | 10.5453  | 22.1928   | 17.3892  | 10.8792   | -10.5961  | -2.60208  | 5.34232  | 14.2207  | -inf      | 9.20651   | 5.09694   | 5.10212   | 5.02252  | 9.17412  | 5.89068   | 10.6233  | 5.80265  | 
| SHITOMASI/ORB   | 7.96017 | 8.37943  | 22.1928   | 7.06804  | 0.29154   | -5.29804  | 4.43596   | 8.62008  | 36.1288  | 10.0216   | 22.0242   | 5.09694   | 15.6491   | 5.02252  | 9.17412  | 8.81594   | 38.9084  | 13.7308  | 
| SHITOMASI/FREAK | 21.2    | 10.6507  | -0.123115 | 10.6589  | -0.266211 | -0.234386 | -inf      | 13.0531  | 22.5306  | -inf      | 9.11505   | -0.266657 | -0.283687 | 9.84532  | 8.43324  | 5.61555   | -148.521 | 7.06691  | 
| SHITOMASI/SIFT  | 8.52258 | 8.37943  | 10.6061   | 11.2667  | 11.1196   | -10.5961  | 6.89769   | 5.071    | 5.78155  | 61.0256   | -inf      | 5.20497   | 5.9783    | 5.02252  | 9.17412  | 36.2243   | -inf     | 5.80265  | 
| HARRIS/BRISK    | nan     | nan      | nan       | nan      | nan       | nan       | nan       | nan      | nan      | nan       | nan       | nan       | nan       | nan      | nan      | nan       | nan      | nan      | 
| HARRIS/BRIEF    | nan     | nan      | nan       | nan      | nan       | nan       | nan       | nan      | nan      | nan       | nan       | nan       | nan       | nan      | nan      | nan       | nan      | nan      | 
| HARRIS/ORB      | nan     | nan      | nan       | nan      | nan       | nan       | nan       | nan      | nan      | nan       | nan       | nan       | nan       | nan      | nan      | nan       | nan      | nan      | 
| HARRIS/FREAK    | nan     | nan      | nan       | nan      | nan       | nan       | nan       | nan      | nan      | nan       | nan       | nan       | nan       | nan      | nan      | nan       | nan      | nan      | 
| HARRIS/SIFT     | nan     | nan      | nan       | nan      | nan       | nan       | nan       | nan      | nan      | nan       | nan       | nan       | nan       | nan      | nan      | nan       | nan      | nan      | 
| FAST/BRISK      | 21.7608 | 23.8581  | 7.69818   | -inf     | -40.6338  | 7.47429   | 10.727    | -inf     | 20.943   | 4.52672   | 8.11962   | 10.0614   | 7.48609   | 14.7793  | 7.31204  | 5.86137   | 10.6212  | -10.0507 | 
| FAST/BRIEF      | -inf    | 20.5786  | 34.3842   | 10.0216  | 16.6037   | 7.47429   | 8.5451    | -15.3737 | -12.1327 | 11.5308   | 28.6816   | 30.2046   | 12.0086   | 15.821   | 6.33555  | 16.4402   | 4.99521  | 30.6549  | 
| FAST/ORB        | 3.29118 | 24.2097  | 28.3908   | 5.72346  | -inf      | -21.9191  | 12.9165   | 1049.44  | 7.59845  | 5.03573   | 7.13497   | 4.91541   | 25.7406   | 19.6396  | 59.9928  | 18.7444   | 5.77271  | 16.8246  | 
| FAST/FREAK      | 9.14883 | 36.1513  | 10.7503   | 10.8689  | 13.4207   | -inf      | 11.2733   | 6.10209  | 10.5723  | 30.0238   | -0.289122 | 11.2317   | 9.02315   | 9.91005  | 12.3542  | 6.3529    | 7.55793  | 32.731   | 
| FAST/SIFT       | 3.08701 | 9.73932  | 11.0141   | -16.4544 | 22.3104   | -22.0562  | 7.16239   | 5.98384  | 14.6096  | 9.21472   | 18.9655   | 52.1371   | -47.3202  | 5.60707  | 19.1382  | 8.12292   | 10.7137  | 46.5734  | 
| BRISK/BRISK     | 19.3964 | 6.8052   | 5.74869   | 4.01581  | -48.6215  | -12.2583  | -0.119993 | -40.6668 | 12.5293  | 10.7198   | 70.2607   | -112.891  | 7.02282   | 9.1755   | 4.83981  | 10.3943   | -9.61189 | 10.4692  | 
| BRISK/BRIEF     | 16.4997 | 4.28807  | 10.7555   | -21.1882 | 14.035    | 11.7973   | 18.2201   | 10.4335  | 7.80166  | -9.56416  | 4.05765   | -50.5316  | 7.2679    | 11.5876  | 8.96193  | 8.68955   | 7.73015  | 10.8141  | 
| BRISK/ORB       | 11.7464 | 3.60523  | 11.4007   | 26.7388  | -15.5879  | 5.08952   | 5.75055   | 15.7781  | nan      | 12.5143   | 11.3553   | 133.746   | 8.28677   | 9.52836  | 19.7368  | 9.42101   | 6.04011  | 12.2553  | 
| BRISK/FREAK     | -32.114 | 8.51134  | 67.6617   | 10.2968  | -32.1989  | nan       | nan       | nan      | 7.18864  | 6.87837   | 12.3976   | 14.3672   | 2.97927   | -561.338 | 36.917   | 7.895     | 6.63265  | 10.1543  | 
| BRISK/SIFT      | 11.5241 | 3.04214  | 8.0559    | 13.2642  | -10.1125  | 45.4818   | -10.3703  | 80.3801  | 10.9518  | 33.1565   | -2.83156  | 7.0754    | 35.6843   | 4.2556   | -2.02435 | 10.7916   | 0.875784 | 38.3007  | 
| ORB/BRISK       | nan     | -213.481 | -inf      | 7.84734  | nan       | nan       | nan       | nan      | nan      | nan       | nan       | nan       | 1.97583   | 209.681  | nan      | -inf      | nan      | nan      | 
| ORB/BRIEF       | nan     | nan      | nan       | nan      | nan       | nan       | nan       | 9.26948  | nan      | nan       | nan       | -inf      | nan       | 8.29883  | nan      | nan       | nan      | nan      | 
| ORB/ORB         | nan     | 3.8905   | 16.9653   | nan      | nan       | nan       | nan       | nan      | nan      | nan       | nan       | nan       | nan       | -6.01313 | nan      | 4.06784   | 2.71379  | nan      | 
| ORB/FREAK       | nan     | nan      | -40.1571  | nan      | nan       | nan       | nan       | nan      | nan      | nan       | nan       | nan       | nan       | nan      | nan      | nan       | nan      | nan      | 
| ORB/SIFT        | nan     | 24.6348  | 6.80196   | 0.140353 | nan       | nan       | nan       | nan      | nan      | nan       | nan       | nan       | 2.82887   | nan      | nan      | nan       | nan      | nan      | 
| AKAZE/BRISK     | 18.1727 | 69.6437  | 13.6947   | 17.7501  | 10.6043   | 16.2368   | 32.6542   | 19.4843  | 15.1649  | 14.4561   | 11.443    | 8.84314   | 10.3764   | 7.04384  | 27.1813  | 7.78998   | 12.0762  | 8.21914  | 
| AKAZE/BRIEF     | 10.9985 | 22.9642  | 14.7343   | 13.1986  | 11.0711   | 12.7747   | -212.42   | 9.99882  | 16.833   | 9.16014   | 42.8799   | 13.7624   | 9.35469   | 4.17237  | 17.4348  | 7.6574    | 18.2001  | 183.278  | 
| AKAZE/ORB       | 7.89459 | 15.4089  | 13.6015   | 159.982  | 8.92929   | 19.4422   | 25.4931   | 9.44875  | 12.8005  | 10.9695   | 5.06779   | 10.0827   | 9.36532   | 8.76476  | 9.60412  | -0.125617 | 8.23411  | 29.6526  | 
| AKAZE/FREAK     | 13.6128 | 17.1131  | -117.9    | 16.4049  | 13.2474   | 17.7484   | 12.5645   | 36.2875  | 18.5174  | 0.0815352 | 7.13646   | 8.57627   | 25.4447   | 8.33602  | 25.0196  | 8.65583   | 10.0485  | 8.45773  | 
| AKAZE/SIFT      | 23.402  | 18.2563  | 11.947    | 16.8777  | 20.2299   | 20.7258   | 31.2624   | 14.6379  | 37.0478  | 15.5952   | 12.0187   | 9.5285    | 7.21257   | -46.5776 | 3.78166  | 18.0197   | 24.8037  | 18.0394  | 
| SIFT/BRISK      | 3.51904 | 12.444   | 12.4708   | 28.8706  | nan       | 17.3198   | 11.9857   | 36.9936  | 18.1953  | 8.20005   | 12.6022   | 10.2777   | 21.2892   | nan      | 39.2796  | nan       | 9.39992  | 7.00674  | 
| SIFT/BRIEF      | 16.0203 | -17.7085 | -15.4458  | 14.081   | 21.4108   | 12.656    | 3.90791   | 17.0895  | 11.459   | 7.59838   | 21.8837   | 13.2082   | 8.21611   | 2.09682  | 13.2466  | -10.3064  | 11.5755  | 6.97569  | 
| SIFT/FREAK      | 3.06271 | 15.2683  | -16.0399  | 18.3163  | 42.6595   | 17.3198   | 56.7192   | 33.3719  | 11.7315  | 7.59838   | 20.5468   | 8.51164   | 6.7742    | 5.61075  | 8.23683  | 12.7659   | 9.05558  | 6.90725  | 
| SIFT/SIFT       | 3.33996 | 11.8101  | -15.4458  | 10.7685  | -26.6114  | 11.6249   | 14.1793   | 28.4273  | 11.7166  | 10.7065   | 18.6388   | 8.51164   | 9.77171   | 7.19901  | 19.0322  | 8.65332   | 8.9451   | 8.30575  | 
| AKAZE/AKAZE     | 26.5675 | 18.2563  | 12.6864   | 23.3536  | 15.1454   | 36.3833   | 9.72355   | 42.0462  | 15.9971  | 19.6731   | 12.3084   | 12.8952   | 10.3176   | 4.29533  | 27.1813  | 8.48611   | 8.51457  | 10.7913  | 


This graph shows TTC estimate difference by frame between each detector/descriptor combination and Lidar based estimation.

![TTC difference between camera-based and Lidar based][file://task6.png]

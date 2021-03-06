# image_mosaic

In this project, I investigate and apply a Harris corner detector on a set of images and subsequently ﬁnd correspondences between the images in order to create a stitched mosaic. We start by reading in a pair of images and converting them to gray scale to simplify the matrix dimensions. A Harris corner detector is applied to the images, using an algorithm that gives weighting values to points dependent on their gradients with which we can determine which points contain corners or edges. After ﬁnding these points in our images we attempt to ﬁnd correspondences by computing the normalized cross correlation (NCC) between the images which gives us the statistical likelihood of point correlation. In order to do this we must determine the optimal windowing and threshold values to apply to the correlation function. After using the NCC we should have a set of corner correspondences that match with a level of statistical certainty. We then are able to estimate the homography between the images through the use of internal MATLAB functions that use random sample consensus (RANSAC) to ﬁlter out noisy correspondences. Using this homography we are then able to determine the proper transformation necessary to cleanly warp both images together to create a seamless mosaic.

Example output of a homography estimation and resulting stitched image:

![result1](hallway_inliers.png?raw=false "inliers")
![result2](hallway_stitched.png?raw=false "stitched")

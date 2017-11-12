# Kernel

-   A kernel is a square matrix that an image is multipled by
-   Different kernels do different things
-   can be used for edge detection, reducing noise, feature extraction, etc


## Identity

-   Returns the original image

| 0 | 0 | 0 |
| 0 | 1 | 0 |
| 0 | 0 | 0 |

![img](https://i.imgur.com/YWH6NPC.png)


## Blur

-   used to reduce noise

| 1/9 | 1/9 | 1/9 |
| 1/9 | 1/9 | 1/9 |
| 1/9 | 1/9 | 1/9 |

![img](https://i.imgur.com/ogsHVT9.png)


### Gaussian Distribution

-   Called a normal distribution, bell curve
-   Defined by the mean and variance
-   Infinite expanse so we can integrate at any point
    -   Calculus is hard with edges

![img](https://upload.wikimedia.org/wikipedia/commons/7/74/Normal_Distribution_PDF.svg)


### Gaussian blur

| 1/16 | 1/8 | 1/16 |
| 1/8  | 1/4 | 1/8  |
| 1/16 | 1/8 | 1/16 |

![img](https://i.imgur.com/l3lahuH.png)


### Gaussian blur larger

| 1/256 | 1/64 | 3/128 | 1/64 | 1/256 |
| 1/64  | 1/16 | 3/32  | 1/16 | 1/64  |
| 3/128 | 3/32 | 9/64  | 3/32 | 3/128 |
| 1/64  | 1/16 | 3/32  | 1/16 | 1/64  |
| 1/256 | 1/64 | 3/128 | 1/64 | 1/256 |


## Edge Detection

-   Gradients in images
    -   Derivatives
-   Good to blur before using edge detection


### Sobel

-   Gradient calculation
    -   First derivative
-   Edge detection
-   Uses to kernels and combines the results


### Right Sobel

| -1 | 0 | 1 |
| -2 | 0 | 2 |
| -1 | 0 | 1 |

![img](https://i.imgur.com/n70YDco.png)


### Top Sobel

| 1  | 2  | 1  |
| 0  | 0  | 0  |
| -1 | -2 | -1 |

![img](https://i.imgur.com/0ag5YRp.png)


### Combination

![img](https://i.imgur.com/zOUwHgY.png)


### Blurred Sobel

![img](https://i.imgur.com/LapsYpb.png)


### Laplacian

| 0 | 1  | 0 |
| 1 | -4 | 1 |
| 0 | 1  | 0 |

![img](https://i.imgur.com/3nHT9Uz.png)


### Blurred Laplacian

![img](https://i.imgur.com/1lRPNTa.png)


# Thresholding

-   Used before projection to get pixel we care about
-   Used by both teams
    -   Was in find blue from last week


## Binary

-   set to max if threshold is reached

![img](https://docs.opencv.org/3.2.0/Threshold_Tutorial_Theory_Base_Figure.png) ![img](https://docs.opencv.org/3.2.0/Threshold_Tutorial_Theory_Binary.png)


## Truncate

-   set anything above threshold to the threshold

![img](https://docs.opencv.org/3.2.0/Threshold_Tutorial_Theory_Base_Figure.png) ![img](https://docs.opencv.org/3.2.0/Threshold_Tutorial_Theory_Truncate.png)


# Hough

-   Express all points in polar coordinates
-   Looks for overlapping lines to see what is a feature we care about


## Lines

[Hough Lines](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_houghlines/py_houghlines.html)


## Circles

[Hough Circles](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_houghcircles/py_houghcircles.html)


# Canny Edge

[Canny Edge](http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_canny/py_canny.html)


# OpenCV

-   Has good documentation
-   Use google to search


# Work

-   Use the template to publish images
    -   use whatever technique you want


## Commands

```shell
roscore
rosbag play -l NAME
```

-   launch file also
# What are we doing today

-   Cameras
-   OpenCV
-   Tensorflow
-   Deep Learning for Semantic Segmentation


# Cameras

-   An image is a collection of pixels


## Stereo

-   Can calculate the distances to things
    -   Finds the same features on the frames
    -   known distance in between cameras
-   Sensitive to amount of features

![img](https://i2.wp.com/scorpion.tordivel.no/images/3D-Lens-Calculator-Sketch.png)


# Computer Vision

-   We have the knowledge in C++ to describe the logic we might want a robot to have. But we need to be able to make sense of what the robot sees and classify it before we can act on this logic.
-   Cue OpenCV, an open source computer vision library with bindings for C++ (and a few other languages)
-   I guess our ability to see has been ++'d


# OpenCV

-   Industrial standard for image processing

![img](https://upload.wikimedia.org/wikipedia/commons/thumb/3/32/OpenCV_Logo_with_text_svg_version.svg/1200px-OpenCV_Logo_with_text_svg_version.svg.png)


# What does an Image look like to your computer?

-   OpenCV stores images in an object called a *Mat*
-   A Mat is an array with rows and columns. Each element of the Mat is a pixel in the image and its location in the Mat corresponds to its location in the image
-   Computers have no concept of "2d", so Images in memory are *continuous*. This means each row of the image is appended onto the end of the last. To iterate through a Mat you just get a pointer to the beginning of the first row and keep track of your row number by how far you've traversed.


# Color Types

-   There are many different formats for an image
    -   Grey scale
    -   RGB
    -   HSV


## Grey scale

-   An image where each pixel is only white to black
-   Range [0-255]
    -   255 is white
    -   0 is black


## Color Images

-   Color images don't embed the color of a pixel in one element. Often, you'll find each pixel represented in BGR (Blue component, Green Component, Red Component) form. So now, each row of a color image is 3 times as long as a row of a black and white image.
-   ![img](https://i.imgur.com/QlokNTv.png)
-   Images don't have to be stored in just BGR format!


## HSV Images

-   Each Pixel in a color image has a hue, a saturation, and a luminosity.
-   Even though our cameras read in images with RGB, converting them to HSV is easy with OpenCV

![img](https://image.slidesharecdn.com/01presentationhuehistograms-150707215651-lva1-app6892/95/about-perception-and-hue-histograms-in-hsv-space-5-638.jpg)


### HSV explained

-   Hue
    -   The actual color
-   Saturation
    -   Indicates the amount of grey
-   Luminosity
    -   How dark the color is

![img](https://www.nmt.edu/tcc/help/pubs/colortheory/img/cone.png)


### Why do we use HSV

-   HSV encodes image data in a way that is resistant to changes in color
-   To put it another way, on a sunny day an image will contain more red, more blue, and more green than on a cloudy day. All three channels are affected.
-   On a sunny day, the saturation channel will be largely effected, but we can expect hue to remain mainly stable. This makes it easier to do searches for colors in the HSV space.


# Finding the blue in an image

<https://github.com/RoboJackets/ros-training/blob/master/code/week6/src/findBlue.cpp>


# Tensorflow

-   Sometimes, CV problems present themselves in such a way that Deep Learning becomes the best solution.
-   Tensorflow is a library containing machine learning algorithms.
-   We use it to provide an implementation of a neural network to perform semantic segmentation for line detection


# Semantic Segmentation

-   Semantic Segmentation is the art of grouping pixels in an image into clusters that share a meaning.
-   In our use case, we classify pixels as "Not a line" or "Is a line"
-   To do this, we utilize something called a Convolutional Neural Network, or CNN


# What is Convolving?

-   Convolving refers to the procedural application of a kernel to an image.


# Kernels

-   A kernel is a square matrix
-   Each pixel value in a region of the image is multiplied by its corresponding element in the matrix, then summed for a result.
-   can be used for edge detection, reducing noise, feature extraction, etc


## Identity

-   Returns the original image

| 0 | 0 | 0 |
| 0 | 1 | 0 |
| 0 | 0 | 0 |

![img](https://i.imgur.com/YWH6NPC.png)


## Blur

-   used to reduce noise
-   replaces the center pixel with the average of all of it's neighbors.

| 1/9 | 1/9 | 1/9 |
| 1/9 | 1/9 | 1/9 |
| 1/9 | 1/9 | 1/9 |

![img](https://i.imgur.com/ogsHVT9.png)i


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


# Back to CNN

-   These kernels are used to generate feature maps, which are then subjected to certain preprocessing steps and fed into a neural network as input, which then classifies each pixel as line or not-line.
-   To train our network, we manually labelled many images from past competitions and testing runs, which allows our network to preform well for our use cases.
-   For the more curious amoung you, here is a beginner's guide to CNNs that goes into more detail: [CNN Guide](https://adeshpande3.github.io/A-Beginner%27s-Guide-To-Understanding-Convolutional-Neural-Networks/)


# I bet you all have wondered why I have gathered you here today.

-   As many of you might have noticed, we are in the shop rather than on campus.
-   In addition, many of you might have noticed our poor documentation habits.
-   We are going to be walking you (and us) through different parts of the codebase, explaining and documenting things as we go to help you get onboarded and clean up some shoddy code.
-   Welcome to the Shop!
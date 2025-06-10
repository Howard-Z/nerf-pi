# Camera Implementation

The primary objective of the camera module is to determine the angles in which to rotate the gun in order to point it at the person (if someone is present).  To do this, first, the world coordinate position of the person is needed, and from there, assuming the cameras are located at the origin, the angles can be calculated.  The angle calculation given the camera is present at the origin and the person is located at $(X, Y, Z)$, where $X$ is left/right position, $Y$ is the height, and $Z$ is the depth, is simply

$$
\begin{align*}
    \theta &= \tan^{-1}(\frac{X}{Z}) \\
    \phi &= \tan^{-1}(\frac{Y}{Z})
\end{align*}
$$

Here, $\theta$ represents the horizontal rotation of the gun to position the person at the center of the camera frame, and $\phi$ represents the vertical rotation.

To find the $(X, Y, Z)$ world coordinates, we used the point coordinates of the center of the chest $(x_1, y_1)$ and $(x_2, y_2)$, represented in the spaces of the left and right cameras, respectively, multiplied by the intrinsic matrix.  The intrinsic matrix is calculated by the camera calibration file.  The calibration matrices, $K_L$ and $K_R$ are approximately:

$$
\begin{align*}
    K_R &= \begin{bmatrix} 1401 & 0 & 9514 \\ 0 & 1400 & 481 \\ 0 & 0 & 1\end{bmatrix}\\
    K_L &= \begin{bmatrix} 1393 & 0 & 9222 \\ 0 & 1387 & 530 \\ 0 & 0 & 1\end{bmatrix}
\end{align*}
$$

From these, the focal lengths for each camera can be extracted.  This, with the distance between the two cameras, are used to calculate the depth (distance away from the camera) the person is standing.

```
def calculate_depth(K, point_L, point_R, dist_between_cams):
    x_disparity = point_L[0] - point_R[0]
    f_x = K[0, 0]

    depth = (f_x * dist_between_cams) / x_disparity
    return abs(depth)
```

$$
    Z = \frac{F_x \cdot d}{x_1 - x_2}
$$

$F_x$ is the horizontal focal length of the camera.  This method was used in the formula to calculate the world coordinates:

```
def calculate_world(K_L, K_R, point_L, point_R, dist_between_cams):
    _ZL = calculate_depth(K_L, point_L, point_R, dist_between_cams)
    _XL = _ZL * (point_L[0] - K_L[0, 2]) / K_L[0, 0] - (dist_between_cams / 2)
    _YL = _ZL * (point_L[1] - K_L[1, 2]) / K_L[1, 1]

    _ZR = calculate_depth(K_R, point_L, point_R, dist_between_cams)
    _XR = _ZR * (point_R[0] - K_R[0, 2]) / K_R[0, 0] - (dist_between_cams / 2)
    _YR = _ZR * (point_R[1] - K_R[1, 2]) / K_R[1, 1]

    return [(_XL + _XR) / 2, (_YL + _YR) / 2, (_ZL + _ZR) / 2]
```

Although only one camera may be used for this step, the averages of both cameras was used instead in case of one of the cameras being inaccurate.  Using the depth estimate, the $X$ and $Y$ coordinates can be estimated too.

$$
\begin{align*}
    X &= Z \cdot \frac{x - C_x}{F_x}\\
    Y &= Z \cdot \frac{y - C_y}{F_y}
\end{align*}
$$

$C_x$ and $C_y$ are the $x$ and $y$ of the principal point, and $F_x$ and $F_y$ are the horizontal and vertical focal lengths, respectively.

Now, the only part that is missing is the coordinates of the point of interest in the left and right cameras.  These are sourced from the media pipe module which gives a point estimate for 33 landmarks on the human body.  See [here](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker).  The average of the points 11, 12, 23, and 24 gives a good estimate for the center of a person's chest, and was used for this project.

With this pipeline, a 2D point provided from each of the cameras can be translated into rotation angles for the turret to point at the correct location.

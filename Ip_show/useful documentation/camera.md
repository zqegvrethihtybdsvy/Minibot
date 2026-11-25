# Minibot Computer Vision & ArUco 6-DoF Pose Estimation Reference

## 1. System Architecture Overview

The minibot vision pipeline runs on a **Raspberry Pi** operating in a **headless environment** (accessed via SSH at `10.3.141.1`) coupled to a **Raspberry Pi Camera Module v2 (Sony IMX219)** over the CSI interface. Because Raspberry Pi OS uses the modern `libcamera` stack (`rpi/vc4` pipeline), standard OpenCV V4L2 video capture (`cv2.VideoCapture(0)`) is bypassed in favor of `Picamera2` for zero-copy frame acquisition.

- Do `rpicam-hello --list-cameras` for having the parameters

### End-to-End Processing Pipeline
```text
+-------------------+     CSI / libcamera      +----------------------+
|  Sony IMX219      | -----------------------> |  Picamera2           |
|  Camera Sensor    |   640x480 RGB888 (BGR)   |  Frame Buffer (x2)   |
+-------------------+                          +----------+-----------+
                                                          |
                                                          v
+-------------------+   calibration_chessboard +----------------------+
|  K & D Matrices   | -----------------------> |  cv2.aruco           |
|  (YAML File)      |                          |  DICT_4X4_100 +      |
+-------------------+                          |  Sub-Pixel Refine    |
                                               +----------+-----------+
                                                          |
                                                          v
+-------------------+     IPPE_SQUARE PnP      +----------------------+
|  Per-ID Physical  | -----------------------> |  cv2.solvePnP        |
|  Tag Sizes (m)    |                          |  (rvec, tvec)        |
+-------------------+                          +----------+-----------+
                                                          |
                         +--------------------------------+--------------------------------+
                         |                                                                 |
                         v                                                                 v
          +-----------------------------+                                   +-----------------------------+
          |  Minibot Navigation / Logic |                                   |  On-Demand MJPEG Server     |
          |  (X, Y, Z, Roll, Pitch, Yaw)|                                   |  http://10.3.141.1:8000     |
          +-----------------------------+                                   +-----------------------------+
```

### Software Stack
* **OS / Camera Backend:** Raspberry Pi OS with `libcamera v0.5.2` (`/usr/share/libcamera/ipa/rpi/vc4/imx219.json`)
* **Python Interface:** `Picamera2` (direct ISP access and hardware sensor mode selection)
* **Computer Vision:** `OpenCV 4.6.0` (`cv2.aruco`, `cv2.solvePnP`, `cv2.Rodrigues`, `cv2.projectPoints`)
* **Math & Orientation:** `NumPy` and `SciPy` (`scipy.spatial.transform.Rotation`)

---

## 2. Camera Hardware & Sensor Optics (Sony IMX219)

### Physical Specifications
* **Sensor Model:** Sony IMX219PQ (8.08 Megapixels, rolling shutter)
* **Full Pixel Array:** $3280 \times 2464\text{ pixels}$ ($4:3$ aspect ratio)
* **Physical Pixel Pitch:** $1.12\,\mu\text{m} \times 1.12\,\mu\text{m}$
* **Full Active Sensor Dimensions:** $3.674\,\text{mm} \times 2.760\,\text{mm}$
* **Stock Lens Focal Length:** $f = 3.04\,\text{mm}$ (fixed-focus lens tuned from $\approx 1\,\text{m}$ to $\infty$, aperture $f/2.0$)

### Hardware Sensor Modes (`picam2.sensor_modes`)
The IMX219 ISP exposes four primary hardware readout modes. Understanding `crop_limits` is essential because changing sensor modes changes the effective focal length and field of view (FOV):

| Sensor Mode (`output_size`) | Hardware `crop_limits` `(x, y, w, h)` | Binning | Max FPS (10-bit / 8-bit) | Field of View (FOV) | Optical Behavior |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`(640, 480)`** *(Active)* | `(1000, 752, 1280, 960)` | $2 \times 2$ | $206.65 / 206.65$ | Narrow ($2.56\times$ crop) | Reads only the center $1280 \times 960$ sensor window |
| **`(1640, 1232)`** | `(0, 0, 3280, 2464)` | $2 \times 2$ | $41.85 / 83.70$ | **100% Full Wide-Angle** | Reads entire chip; best wide FOV + high FPS |
| **`(1920, 1080)`** | `(680, 692, 1920, 1080)` | $1 \times 1$ | $47.57 / 47.57$ | Medium ($16:9$ crop) | Center crop for HD widescreen |
| **`(3280, 2464)`** | `(0, 0, 3280, 2464)` | $1 \times 1$ | $21.19 / 21.19$ | **100% Full Wide-Angle** | Maximum raw resolution; slowest CPU processing |

### Optical Math for the Active `640x480` Cropped Mode
When `Picamera2` configures `main={"size": (640, 480)}` without overriding `sensor={"output_size": ...}`, `libcamera` selects the `(640, 480)` cropped sensor mode:
1. **Active Physical Sensor Area:**
   $$W_{\text{sensor}} = 1280 \times 1.12\,\mu\text{m} = 1.434\,\text{mm}, \qquad H_{\text{sensor}} = 960 \times 1.12\,\mu\text{m} = 1.075\,\text{mm}$$
2. **Effective Pixel Pitch (after $2 \times 2$ binning):**
   $$p_{\text{eff}} = 2 \times 1.12\,\mu\text{m} = 2.24\,\mu\text{m}$$
3. **Theoretical Focal Length in Pixels:**
   $$f_{\text{px}} = \frac{f}{p_{\text{eff}}} = \frac{3.04\,\text{mm}}{0.00224\,\text{mm}} \approx 1357.1\,\text{px}$$
4. **Visible Scene Dimensions at Distance $Z = 50\,\text{cm}$ ($0.50\,\text{m}$):**
   $$W_{\text{scene}} = Z \cdot \frac{W_{\text{sensor}}}{f} = 50\,\text{cm} \times \frac{1.434}{3.04} \approx 23.6\,\text{cm}$$
   $$H_{\text{scene}} = Z \cdot \frac{H_{\text{sensor}}}{f} = 50\,\text{cm} \times \frac{1.075}{3.04} \approx 17.7\,\text{cm}$$

> **Switching to Wide-Angle Mode Later:** If the minibot needs a wider field of view to track multiple stones or nearby robots simultaneously, configure `sensor={"output_size": (1640, 1232)}` alongside `main={"size": (640, 480)}`. That expands the visible scene at $Z = 50\,\text{cm}$ to $60.4\,\text{cm} \times 45.4\,\text{cm}$ ($f_{\text{px}} \approx 530\,\text{px}$), **provided you re-run camera calibration once in that mode**.

### Color Channel Order (`RGB888` in `Picamera2`)
In `Picamera2`, requesting `format="RGB888"` allocates a 3-byte-per-pixel buffer whose byte order in memory is **Blue-Green-Red (`BGR`)** (matching DRM/V4L2 naming conventions). Consequently, `picam2.capture_array()` can be passed directly into `cv2.aruco.detectMarkers()` and `cv2.imencode(".jpg", ...)` without color conversion.

---

## 3. Camera Calibration & Intrinsic Parameters

### Pinhole & Distortion Model
OpenCV models the projection of a 3D world point $(X_w, Y_w, Z_w)$ onto a 2D image pixel $(u, v)$ using the intrinsic camera matrix $K$ and distortion vector $D$:

$$s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = K \begin{bmatrix} R & t \end{bmatrix} \begin{bmatrix} X_w \\ Y_w \\ Z_w \\ 1 \end{bmatrix}, \qquad K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}$$

Lens radial and tangential distortions are corrected using the Brown-Conrady model:

$$\begin{aligned}
x_{\text{dist}} &= x\left(1 + k_1 r^2 + k_2 r^4 + k_3 r^6\right) + 2p_1 xy + p_2\left(r^2 + 2x^2\right) \\
y_{\text{dist}} &= y\left(1 + k_1 r^2 + k_2 r^4 + k_3 r^6\right) + p_1\left(r^2 + 2y^2\right) + 2p_2 xy
\end{aligned}$$

where $r^2 = x^2 + y^2$ in normalized camera coordinates, and $D = \begin{bmatrix} k_1 & k_2 & p_1 & p_2 & k_3 \end{bmatrix}$.

### Calibration Parameters (`calibration_chessboard.yaml`)
Calibrated using a $10 \times 7$ square chessboard ($9 \times 6 = 54$ interior corners, $25\,\text{mm}$ square width) in `640x480` mode:

| Parameter | Symbol | Measured Value | Ideal / Theoretical Target | Physical Interpretation |
| :--- | :--- | :--- | :--- | :--- |
| **Horizontal Focal Length** | $f_x$ | $1336.12\,\text{px}$ | $\approx 1357\,\text{px}$ | Determines horizontal scale and depth $Z$ accuracy |
| **Vertical Focal Length** | $f_y$ | $1335.65\,\text{px}$ | $\approx 1357\,\text{px}$ | $0.03\%$ difference from $f_x$ confirms square pixels |
| **Principal Point X** | $c_x$ | $403.49\,\text{px}$ | $320.0\,\text{px}$ | Optical center horizontal pixel coordinate |
| **Principal Point Y** | $c_y$ | $227.75\,\text{px}$ | $240.0\,\text{px}$ | Optical center vertical pixel coordinate |
| **Radial Distortion** | $k_1, k_2, k_3$ | $[0.486,\; -7.576,\; 48.412]$ | $|k_i| < 1.0$ | High-order $k_2, k_3$ oscillation due to narrow center crop |
| **Tangential Distortion** | $p_1, p_2$ | $[-0.0017,\; 0.0268]$ | $\approx 0$ | Minor sensor-to-lens tilt |

### Notes on $k_3$ Overfitting & `CALIB_FIX_K3`
Because the `640x480` cropped readout uses only the central $39\%$ of the IMX219 lens radius ($r \le 0.25$), actual barrel distortion across the image is minimal. Fitting a 6th-order radial term ($k_3 r^6$) without wide-angle corner data allows $k_2$ and $k_3$ to oscillate with opposite signs while canceling near the center.
* When re-running `camera_calib.py` on existing images, passing `flags=cv2.CALIB_FIX_K3` locks $k_3 = 0$, yielding cleaner corner geometry while preserving $f_x, f_y \approx 1336\,\text{px}$.
* **Strict Rule:** `calibration_chessboard.yaml` is tied to the exact `(sensor_mode, main_resolution)` pair used during capture. Never reuse a `640x480` cropped YAML file with `1640x1232` wide-angle capture.

---

## 4. Eurobot ArUco Specifications & ID Allocation

### Dictionary Selection: `DICT_4X4_100`
* **Grid Structure:** $4 \times 4$ internal binary data matrix ($16\text{ bits}$) surrounded by a $1\text{-bit}$ solid black border (`markerBorderBits = 1`), creating a $6 \times 6$ cell grid.
* **Why `DICT_4X4_100` over `DICT_ARUCO_ORIGINAL`:**
  * Covers IDs `0` through `99`, encompassing every Eurobot robot, field, and team-reserved tag ID.
  * `DICT_ARUCO_ORIGINAL` uses a $5 \times 5$ grid with weak minimum Hamming distance, causing frequent false positives on background room clutter. `DICT_4X4_100` enforces strict inter-marker Hamming distance and rejects non-marker rectangles reliably.

### Official Eurobot ID Allocation Table
| ID Range | Category | Team / Owner | Usage & Competition Rules |
| :--- | :--- | :--- | :--- |
| **`0`** | Reserved / Forbidden | Organizers | Forbidden on team robots/mechanisms |
| **`1` – `5`** | **Blue Robots** | Blue Team | Mandatory identification markers mounted on Blue robots |
| **`6` – `10`** | **Yellow Robots** | Yellow Team | Mandatory identification markers mounted on Yellow robots |
| **`11` – `50`** | **Game Field & Elements** | Arena / Organizers | Includes **ID `13` (Stones)**; strictly forbidden on team hardware |
| **`51` – `70`** | **Blue Custom Tags** | Blue Team | Available for Blue team beacons, mechanisms, or localization |
| **`71` – `90`** | **Yellow Custom Tags** | Yellow Team | Available for Yellow team beacons, mechanisms, or localization |
| **`> 90` or non-4x4** | **Unrestricted Custom** | Either Team | Permitted for custom team vision targets |

### Physical Dimensions & Critical Measurement Rule

#### 1. Robot Identification Plates (IDs `1` – `10`)
* **Outer Plate Dimensions:** $100\,\text{mm} \times 100\,\text{mm}$ square, thickness $20\,\text{mm} \pm 2\,\text{mm}$, mass $\le 100\,\text{g}$.
* **Visual Composition (from center outward):**
  1. **Black ArUco `4x4` Tag:** **$70\,\text{mm} \times 70\,\text{mm}$ (`0.070 m`)** centered on the top face.
  2. **White Quiet Zone:** $10\,\text{mm}$ wide white margin on all four sides (brings span to $90\,\text{mm}$).
  3. **Team Color Rim:** $5\,\text{mm}$ wide blue or yellow outer border + colored plate edges (brings total span to $100\,\text{mm}$).

> **CRITICAL FOR POSE ACCURACY:** OpenCV's `detectMarkers()` locates the **four outer corners of the black ArUco border**, ignoring the white quiet zone and colored rim. Therefore, `MARKER_SIDE_LENGTH` in `solvePnP` **must be set to `0.070` m ($70\,\text{mm}$)**—never `0.100` m. Setting `0.100` m would scale all estimated distances $Z$ by $\frac{100}{70} \approx +42.9\%$.

#### 2. The Stones (ID `13`)
* **Physical Dimensions:** Cardboard boxes measuring $320\,\text{mm} \times 110\,\text{mm} \times 110\,\text{mm}$ ($30$ stones placed in the quarries).
* **Tag Placement:** ArUco `4x4` **ID `13`** printed on each of the four large ($320\,\text{mm} \times 110\,\text{mm}$) faces.
* **Per-ID Scaling:** Because game elements can have a different black-square side length than the $70\,\text{mm}$ robot plates, the pose estimator uses a `MARKER_SIZES` lookup dictionary (`{id: side_length_in_meters}`) so IDs `1..10` use `0.070` m while ID `13` uses its exact measured black-square width.

---

## 5. 6-DoF Pose Estimation Pipeline

### Step 1: Marker Detection & Sub-Pixel Refinement
1. `cv2.aruco.detectMarkers()` thresholds the grayscale image adaptively, extracts quadrilateral contours, and decodes the $4 \times 4$ bit grid against `DICT_4X4_100`.
2. Setting `aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX` refines each of the 4 detected corner coordinates to sub-pixel precision ($\approx 0.05\text{–}0.1\,\text{px}$) using local image gradients, significantly reducing angular jitter in `roll`, `pitch`, and `yaw`.

### Step 2: Planar Pose Estimation (`SOLVEPNP_IPPE_SQUARE`)
Rather than using the deprecated `estimatePoseSingleMarkers()`, the script defines the 3D object corners of a marker of side length $L$ in its local coordinate frame ($Z_m = 0$) in OpenCV's clockwise corner order (`Top-Left`, `Top-Right`, `Bottom-Right`, `Bottom-Left`):

$$P_{\text{obj}} = \begin{bmatrix}
-L/2 & +L/2 & 0 \\
+L/2 & +L/2 & 0 \\
+L/2 & -L/2 & 0 \\
-L/2 & -L/2 & 0
\end{bmatrix}$$

It then solves Perspective-n-Point using **`cv2.SOLVEPNP_IPPE_SQUARE`** (Infinitesimal Plane-Based Pose Estimation specialized for square markers):
* **Why `IPPE_SQUARE` is superior:** Generic PnP solvers can suffer from planar pose ambiguity (suddenly flipping the $Z$-axis sign or mirror-inverting tilt angles when a tag is viewed nearly head-on). `IPPE_SQUARE` analytically computes both planar solutions and selects the physically consistent pose with lower reprojection error.

### Step 3: Rotation & Translation Outputs
`cv2.solvePnP()` returns:
* **Translation vector $t = \begin{bmatrix} t_x & t_y & t_z \end{bmatrix}^T$ (in meters):** Position of the marker center relative to the camera optical center.
* **Rodrigues rotation vector $r_{\text{vec}} \in \mathbb{R}^3$:** Axis-angle representation converted into a $3 \times 3$ rotation matrix $R = \text{Rodrigues}(r_{\text{vec}})$ and decomposed into intrinsic Euler angles $(\text{roll}, \text{pitch}, \text{yaw})$ in degrees via `scipy.spatial.transform.Rotation`.

### Step 4: Outlier Filtering
Any detection outside $0.02\,\text{m} < t_z < \text{MAX\_VALID\_DISTANCE}$ is discarded immediately before logging or publishing to robot navigation, eliminating impossible far-field noise.

---

## 6. Coordinate Frames & Minibot Navigation Integration

### 1. Marker Coordinate Frame (Right-Hand Rule)
Defined by `obj_points` and rendered on the live stream via `draw_labeled_axes()`:
* **Origin $(0, 0, 0)$:** Geometric center of the black ArUco square.
* **$+X_m$ (Red Arrow):** Points to the **right** of the upright marker.
* **$+Y_m$ (Green Arrow):** Points **up** toward the top edge of the upright marker.
* **$+Z_m$ (Blue Arrow):** Points **straight out** perpendicular to the marker face (toward the camera).

### 2. Camera Optical Frame
* **Origin $(0, 0, 0)$:** Optical center of the lens (projected onto the image at principal point $(c_x, c_y)$).
* **$+X_c$:** Points **right** across the image sensor ($t_x > 0$ means marker is to the right of camera center).
* **$+Y_c$:** Points **down** toward the ground ($t_y > 0$ means marker is below camera center).
* **$+Z_c$:** Points **straight forward** along the optical axis ($t_z > 0$ is forward depth).

### 3. Transforming Camera Pose to Minibot Chassis Frame
To use $(t_x, t_y, t_z)$ for path planning or stone alignment on the Eurobot table, convert the camera-frame point $p_c = \begin{bmatrix} t_x & t_y & t_z \end{bmatrix}^T$ into the **Minibot Base Frame** (where $+X_b$ is forward, $+Y_b$ is left, and $+Z_b$ is upward from the floor):

1. **Axis Alignment (for a level forward-facing camera):**
   $$\begin{bmatrix} X_b \\ Y_b \\ Z_b \end{bmatrix}_{\text{level}} = \begin{bmatrix} t_z \\ -t_x \\ -t_y \end{bmatrix} + \begin{bmatrix} x_{\text{cam}} \\ y_{\text{cam}} \\ z_{\text{cam}} \end{bmatrix}$$
   where $(x_{\text{cam}}, y_{\text{cam}}, z_{\text{cam}})$ is the physical mounting offset of the camera lens relative to the minibot's center of rotation.

2. **Accounting for Camera Downward Pitch Angle $\theta$ ($\theta > 0$ tilted downward toward the table):**
   $$\begin{bmatrix} X_b \\ Y_b \\ Z_b \end{bmatrix} = \begin{bmatrix} t_z \cos\theta + t_y \sin\theta \\ -t_x \\ t_z \sin\theta - t_y \cos\theta \end{bmatrix} + \begin{bmatrix} x_{\text{cam}} \\ y_{\text{cam}} \\ z_{\text{cam}} \end{bmatrix}$$

3. **2D Ground-Plane Bearing & Planar Distance to Target (Stone or Enemy Robot):**
   $$d_{\text{planar}} = \sqrt{X_b^2 + Y_b^2}, \qquad \psi_{\text{target}} = \text{atan2}(Y_b, X_b)$$

---

## 7. Headless Web Streaming & Performance Optimization

Because the Raspberry Pi operates without an attached HDMI display, calling `cv2.imshow()` aborts with `qt.qpa.xcb: could not connect to display`. Instead, `marker_pose_estimation.py` embeds a lightweight, multi-threaded HTTP server (`ThreadingHTTPServer` on port `8000`).

### Available HTTP Endpoints
* **`http://10.3.141.1:8000/`**: Minimal HTML viewer displaying the live stream, axis color key (`Red = +X`, `Green = +Y`, `Blue = +Z`), and a snapshot link.
* **`http://10.3.141.1:8000/stream`**: Continuous `multipart/x-mixed-replace` MJPEG video feed with bounding polygons, labeled 3D axes, optical-center crosshair, FPS counter, and per-tag `(X, Y, Z, R, P, Y)` telemetry.
* **`http://10.3.141.1:8000/snapshot.jpg`**: Returns the latest annotated JPEG frame on demand for quick verification.

### Zero-Overhead Match Execution
JPEG encoding (`cv2.imencode`) is gated behind `if ENABLE_WEB_STREAM and stream_clients > 0:`. During an actual Eurobot match when no debugging browser tab is connected (`stream_clients == 0`), the script skips frame copying, overlay drawing, and JPEG compression completely—dedicating $100\%$ of CPU cycles to real-time ArUco detection and pose estimation.

---
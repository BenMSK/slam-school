# SLAM School, Korean mode: translation rules

The reader is a Korean speaker learning SLAM who reads the English *SLAM Handbook* side by side with this course and
will go on to read English papers. So the prose is Korean and the terms of art stay in English, exactly as the book
writes them. "SLAM" is "SLAM", not "동시적 위치 추정 및 지도 작성"; "localization" is "localization", not "위치 추정";
"factor graph" is "factor graph", not "팩터 그래프"; "Jacobian" is "Jacobian", not "자코비안". This is the same rule as
LLM School ("LLM", not "언어모델") and Robotics School.

## 1. What stays in English (Latin script, same spelling and capitalisation as the English lesson)

The test: would a Korean SLAM or robotics researcher say this word in English in a lab meeting? Then keep it in
English. **When unsure, keep it in English.** The lists below are fixed decisions so that all 24 weeks agree.

- **The problem and the system.** SLAM, localization, mapping, map, odometry, visual odometry (VO), VIO, LIO, drift,
  loop closure, place recognition, relocalization, re-localization, front-end, back-end, pose, pose graph, landmark,
  keyframe, trajectory, state, sensor fusion, calibration, intrinsics, extrinsics, synchronization (time sync),
  ground truth, ATE, RPE, benchmark, dataset, pipeline, real-time, online / offline, global / local (as in "global
  map", "local map", "global optimum").
- **Probability and estimation.** prior, posterior, likelihood, MAP, MLE, maximum likelihood, Bayes rule, Bayes net,
  Bayes tree, Gaussian, covariance, information matrix, information form, information vector, precision (matrix),
  Fisher information, Cramér-Rao lower bound (CRLB), Mahalanobis distance, marginal, marginalization, conditioning,
  conditional, joint distribution, Schur complement, estimator, unbiased, consistent / consistency, Kalman filter,
  EKF, MSCKF, particle filter, filtering, smoothing, sliding window, factor, factor graph, variable, clique,
  elimination, variable ordering, fill-in, sparsity, sparse, dense, white noise, random walk, measurement model,
  motion model, noise model, whitening.
- **Optimization.** least squares, nonlinear least squares, residual, cost, loss, Jacobian, Hessian, gradient,
  gradient descent, steepest descent, Gauss-Newton, Levenberg-Marquardt, dogleg, trust region, damping, linearization,
  normal equations, Cholesky, QR, SVD, eigenvalue, eigenvector, condition number, positive definite, rank, nullspace,
  null space, convex, non-convex, convex relaxation, SDP, QCQP, Shor's relaxation, Lagrangian, duality, dual,
  certificate, certifiable, local minimum, global minimum, initial guess, initialization, iteration, step, step size,
  line search, preconditioning, Burer-Monteiro, Riemannian staircase.
- **Rotations and Lie theory.** Lie group, Lie algebra, manifold, tangent space, SO(2), SO(3), SE(2), SE(3), rotation
  matrix, quaternion, Euler angles, axis-angle, roll / pitch / yaw, frame (a coordinate frame), Exp, Log, exponential
  map, logarithm map, wedge, vee, skew-symmetric, adjoint, perturbation, retraction, left Jacobian, right Jacobian,
  BCH, Rodrigues' formula, homogeneous coordinates, banana distribution.
- **Continuous time.** continuous-time, spline, B-spline, knot, control point, basis function, Gaussian process (GP),
  kernel, hyperparameter, SDE, constant-velocity prior, interpolation.
- **Outliers.** outlier, inlier, data association, correspondence, RANSAC, minimal solver, consensus set, maximum
  clique, pairwise consistency, PCM, M-estimator, robust loss, robust cost, Huber, Cauchy, Geman-McClure, truncated
  least squares (TLS), IRLS, Black-Rangarajan duality, GNC, breakdown point, influence function.
- **Differentiable optimization and learning.** autodiff, automatic differentiation, forward mode, reverse mode,
  backpropagation, unrolling, unrolled differentiation, implicit differentiation, implicit function theorem, bilevel
  optimization, end-to-end, neural network, CNN, MLP, transformer, attention, feature map, encoder, decoder,
  embedding, training, inference, self-supervised, supervised, fine-tuning, foundation model, LLM, VLM, CLIP, DINO,
  SAM, zero-shot, open-vocabulary, open-world, closed-set, pointmap, confidence, correlation volume, optical flow.
- **Dense maps and rendering.** point cloud, occupancy grid, occupancy map, log-odds, voxel, voxel grid, voxel hashing,
  octree, TSDF, ESDF, SDF, signed distance field, surfel, mesh, marching cubes, ray casting, raycast, free space,
  Hilbert map, NeRF, radiance field, neural field, volume rendering, density, transmittance, alpha, alpha compositing,
  positional encoding, Gaussian splatting, 3DGS, splat, rasterization, densification, pruning, novel view synthesis.
- **Cameras and vision.** pinhole camera model, focal length, principal point, distortion, rolling shutter, global
  shutter, keypoint, feature, descriptor, matching, ratio test, epipolar line, epipolar constraint, essential matrix,
  fundamental matrix, homography, triangulation, PnP, reprojection error, photometric error, bundle adjustment (BA),
  direct method, indirect method, feature-based, structure from motion (SfM), photogrammetry, baseline, parallax,
  depth, inverse depth, scale, scale ambiguity, gauge freedom, bag of words, image pyramid, coarse-to-fine, RGB-D,
  stereo, monocular.
- **LiDAR, radar, events.** LiDAR, scan, scan matching, registration, ICP, point-to-point, point-to-plane, GICP, NDT,
  deskewing, motion distortion, submap, Scan Context; radar, FMCW, chirp, Doppler, beat frequency, range, azimuth,
  elevation, angle of arrival, CFAR, multipath, speckle, clutter, ego-velocity, SoC radar, spinning radar; event camera,
  event, polarity, contrast threshold, time surface, event frame, voxel grid, contrast maximization (CMax).
- **Inertial and legged.** IMU, gyroscope, accelerometer, specific force, bias, bias random walk, preintegration,
  dead reckoning, strapdown, gravity, gravity alignment, Allan variance, observability, observable, unobservable,
  observability matrix, degenerate motion, FEJ, time offset; leg odometry, legged robot, contact, stance, swing, slip,
  forward kinematics, kinematics, dynamics, encoder, joint, link, floating base, foot.
- **Dynamic, semantic, spatial AI.** dynamic object, static, lifelong SLAM, long-term, short-term, change detection,
  deformable, non-rigid, NRSfM, ARAP, deformation graph, metric-semantic, semantic segmentation, instance, object
  (as in "object landmark", "object-level map"; a physical thing in plain prose is 물체), quadric, ellipsoid, label
  fusion, scene graph, 3D scene graph, hierarchy, query, Spatial AI, world model, belief propagation, Gaussian belief
  propagation (GBP), message passing, message, node, edge, graph (a factor graph, pose graph, scene graph; a plain
  "graph" may be 그래프), GPU, CPU, FPGA, ASIC, edge computing.
- **Every proper noun.** People, labs, companies, robots, products, systems, algorithms, datasets, libraries, paper
  and book titles, venues: Kalman, Durrant-Whyte, ORB-SLAM, DSO, LSD-SLAM, KinectFusion, iSAM, iSAM2, GTSAM, Ceres,
  SE-Sync, DROID-SLAM, DPVO, DUSt3R, MASt3R, RAFT, SuperPoint, SuperGlue, LIO-SAM, FAST-LIO, KITTI, EuRoC, TUM RGB-D,
  Theseus, PyPose, ANYmal, *SLAM Handbook*, *Probabilistic Robotics*. Never write names in Hangul.
- Symbols, formulas, variable names, code, units, file names. Page references: "p. 30", "pp. 29–32", "§1.3.2",
  "eq. (1.12)" → "식 (1.12)", "Figure 1.4" → "Figure 1.4", "Table 11.1" → "Table 11.1", "Box 6.1" → "Box 6.1".
- Day labels stay as in §4 ("Day 3"), and so does the app name "SLAM School".

Attach Korean particles straight to the English word, chosen by how the word is pronounced:
SLAM은, pose를, landmark가, factor graph에서, Jacobian은, covariance가, loop closure를, Gauss-Newton으로, IMU의,
odometry는, outlier를, map에, keyframe이, residual이.

**Gloss once.** Where the English lesson defines a term (it is in `<strong>` at its first use), keep the term in English
and, if it is a common noun with a standard Korean word, add that word once in parentheses right after the closing tag:
`<strong>localization</strong>(위치 추정)`, `<strong>mapping</strong>(지도 작성)`, `<strong>odometry</strong>(주행 거리 측정)`,
`<strong>drift</strong>(누적 오차)`, `<strong>covariance</strong>(공분산)`, `<strong>least squares</strong>(최소제곱)`,
`<strong>manifold</strong>(다양체)`, `<strong>outlier</strong>(이상치)`. After that use the English term alone. No gloss for
names and acronyms (SLAM, Jacobian, Gauss-Newton, RANSAC, ICP, IMU, NeRF), and no gloss in quizzes, tables, figures or
widgets.

## 2. What becomes Korean

- All ordinary prose, and ordinary words that are not SLAM jargon: 로봇, 센서, 카메라, 이미지, 픽셀, 행렬, 벡터, 스칼라, 원소,
  열, 행, 대각, 역행렬, 전치, 축, 원점, 좌표, 각도, 회전, 이동 (translation), 위치, 방향, 거리, 속도, 가속도, 시간, 오차,
  확률, 분포, 평균, 분산, 표준편차, 함수, 미분, 적분, 도함수, 근사, 최적화, 수렴, 발산, 해 (solution), 변수 (in plain prose),
  차원, 모델, 데이터, 알고리즘, 계산, 메모리, 실험, 논문, 연구, 장, 절, 쪽, 문제, 예제, 증명, 정리 (theorem), 가정, 조건,
  측정 (a measurement in plain prose may be 측정값, but "measurement model" stays), 추정 (estimation as an activity),
  추정값 (an estimate), 불확실성, 정확도, 비용 (expense, not the cost function), 물체, 장애물, 환경, 건물, 방, 바닥, 벽.
- Field names: robotics → 로보틱스, computer vision → 컴퓨터 비전, machine learning → 머신러닝, deep learning → 딥러닝,
  estimation theory → 추정 이론, linear algebra → 선형대수, probability → 확률론 (as a field).
- A verb used casually may be natural Korean ("the robot localizes itself" → "로봇이 자기 위치를 찾습니다"), but when the sentence
  is about the concept, use the term ("localization은 …", "loop closure가 drift를 줄입니다").

## 3. Voice

- Lessons and reviews: 합니다체 (…합니다, …입니다). Clear textbook Korean, short sentences. Translate the meaning, not the
  word order; no translationese (no "~에 의해", no stacked "~의 ~의", no "그것은", no "~하는 것이 가능하다").
- The "In plain words" box (`.box.kid`): 해요체 (…예요, …해요), warm and simple, and still no jargon beyond what it already uses.
- Headings, table headers, labels, buttons: noun phrases without a sentence ending.
- Quiz questions: 합니다체 questions ("…는 무엇입니까?" or "…일까요?" — pick one per file and keep it). Options: the same form
  as the English (a noun phrase stays a noun phrase).
- Quotation marks around a passage quoted from the book: translate the passage and keep the marks and the page chip.

## 4. Fixed wording (use exactly these)

| English | Korean |
|---|---|
| Week 3 · Day 2 | 3주차 · Day 2 |
| Week 3 · Review | 3주차 · 복습 |
| Week 3 (anywhere), Weeks 1–10 | 3주차, 1–10주차 |
| Day 2 (anywhere), Review (the day) | Day 2, 복습 |
| Part I / Part II / Part III | Part I / Part II / Part III |
| Chapter 7, Ch 7 | 7장 |
| Read first: §1.3, pp. 29–32 | 먼저 읽기: §1.3, pp. 29–32 |
| While you read, look for: | 읽으면서 찾아볼 것: |
| About 60 min · Read §1.3, pp. 29–32 (4 pages) · Needs Week 2 Day 2 | 약 60분 · §1.3, pp. 29–32 읽기 (4쪽) · 2주차 Day 2 필요 |
| About 60 min · No new pages | 약 60분 · 새로 읽을 쪽 없음 |
| No prerequisites | 선수 지식 없음 |
| After today you can … | 오늘이 끝나면 …할 수 있습니다. |
| Today's big idea | 오늘의 핵심 |
| In plain words | 쉽게 말하면 |
| Intuition | 직관 |
| Filling the gap: … | 빈틈 채우기: … |
| Going deeper (optional): … | 더 깊이 (선택): … |
| Common mistakes | 흔한 실수 |
| Recap | 정리 |
| Check yourself | 스스로 확인하기 |
| Sources | 출처 |
| Try it: | 직접 해 보기: |
| Notice … (caption under a widget) | 눈여겨볼 점: … |
| The week in one paragraph | 한 문단으로 보는 이번 주 |
| Formula sheet / Mixed quiz / Pen-and-paper problems | 공식 정리 / 종합 퀴즈 / 손으로 푸는 문제 |
| Solution | 풀이 |
| You are ready for next week if you can | 다음 주로 넘어가도 되는 기준 |
| Q1. … | Q1. … |
| the book, the handbook | 책, 핸드북 (the title stays *SLAM Handbook*) |
| lesson / widget / quiz / review day | 레슨 / 위젯 / 퀴즈 / 복습일 |
| this course | 이 코스 |
| Reset / Step / Run / Play / Pause / New noise / New start (buttons) | 초기화 / 한 step / 실행 / 재생 / 일시정지 / 새 noise / 새 시작점 |
| Next → / ← Back | 다음 → / ← 이전 |
| start / goal (figure labels) | 시작 / 목표 |
| true / estimate / truth (legend labels) | 실제 / 추정 / 실제값 |
| yes / no (table cells) | 예 / 아니오 |
| time (s), position (m), iteration (axis labels) | 시간 (s), 위치 (m), iteration |

## 5. Mechanics

- **Formulas are untouchable.** Copy every `\( … \)` and `\[ … \]` character for character. The only thing you may change
  inside a formula is the English words inside `\text{…}` (for example `\text{error after } n \text{ steps}` →
  `\text{오차 (} n \text{ step 후)}`); if in doubt, leave the formula exactly as it is. Korean particles go after the
  closing `\)`: `\(\mathbf{x}\)를`, `\(n\)개`.
- Numbers: keep every digit exactly as written (10,000 stays 10,000; 0.9 stays 0.9; 3% stays 3%). Number words may
  become digits or Korean (two → 두, three poses → 세 pose, one million → 100만).
- Tags: every tag of the English unit appears in the Korean unit, character for character (same attributes, same
  href). You may move a tag with its words to fit Korean word order. Never add, drop or edit a tag. Keep `&amp;`,
  `&lt;`, `&gt;` as entities.
- Page chips (`<span class="src">§1.3.2 · p. 30</span>`) stay exactly as they are and sit after the sentence they support.
- SVG labels (short units, often with `<tspan>`): room is tight. Hangul is about twice as wide as a Latin letter, so a
  Korean label should have at most about half as many characters as the English one. Most labels are terms and stay
  as they are (`=`).
- A unit that should stay exactly as in English: write `=` alone.
- `attr` units are plain text for screen readers (aria-label, alt, title): no tags and no double quotes. Translate them
  (they describe the picture), keeping terms in English.
- Widget strings (in `js.wNN.txt`): the same rules. Labels on sliders, buttons, legends, stat tiles, axis titles,
  captions and messages become Korean; terms, names, symbols and units stay. Never change code, only text inside
  string literals and template literals.

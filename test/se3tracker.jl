include("groundtruth.jl")


tracker = LSDSLAM.SE3Tracker()
K = [517.3 0 318.6; 0 516.5 244.3; 0 0 1]
d = [0.2624, -0.9531, -0.0054, 0.0026, 1.1633];
factory = LSDSLAM.TUMFrameFactory(DATASET_DIR, 640, 480, K, d)

baseframe = LSDSLAM.read!(factory, 1)
relframe = LSDSLAM.read!(factory, 2)

groundtruth = TUMGroundTruth()
groundtruth(baseframe, relframe)

@test_broken LSDSLAM.track!(tracker, baseframe, frame) == groundtruth(baseframe, relframe)

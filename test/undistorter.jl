

@testset "Simple Pinhole Undistorter" begin
    K = [517.3 0 318.6; 0 516.5 244.3; 0 0 1]
    d = [0.2624, -0.9531, -0.0054, 0.0026, 1.1633];

    image = load("test/tumtestset/rgb/1305031102.175304.png")
    image = Gray.(image)
    undistorter = LSDSLAM.SimplePinholeUndistorter(K, d)
    undistorted_image = undistorter(image)
    @test true
end

@testset "TUM Factory is undistorting" begin
    K = [517.3 0 318.6; 0 516.5 244.3; 0 0 1]
    d = [0.2624, -0.9531, -0.0054, 0.0026, 1.1633];

    image = load("test/tumtestset/rgb/1305031102.175304.png")
    image = Gray.(image)
    undistorter = LSDSLAM.SimplePinholeUndistorter(K, d)
    undistorted_image = undistorter(image)

    factory = LSDSLAM.TUMFrameFactory("test//tumtestset", 640, 480, K, d)
    frame = LSDSLAM.read!(factory, 1)

    @test LSDSLAM.𝙄!(frame, level=1) == undistorted_image
end

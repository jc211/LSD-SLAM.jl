@testset "TUM" begin
    K = [517.3 0 318.6; 0 516.5 244.3; 0 0 1]
    d = [0.2624, -0.9531, -0.0054, 0.0026, 1.1633];
    factory = LSDSLAM.TUMFrameFactory("test//tumtestset", 640, 480, K, d)
    frame = LSDSLAM.read!(factory, 1)
    @test isa(factory, LSDSLAM.TUMFrameFactory)
    @test isa(frame, LSDSLAM.TUMFrame)
    @test factory._counter == 1
    LSDSLAM.read!(factory)
    @test factory._counter == 2
end


@testset "TUMFrame" begin
    K = [517.3 0 318.6; 0 516.5 244.3; 0 0 1]
    d = [0.2624, -0.9531, -0.0054, 0.0026, 1.1633];
    factory = LSDSLAM.TUMFrameFactory("test//tumtestset", 640, 480, K, d)
    frame = LSDSLAM.read!(factory, 1)

    @test LSDSLAM.width(frame, level=1) == 640
    @test LSDSLAM.width(frame, level=2) == 320
    @test LSDSLAM.width(frame, level=3) == 160

    @test LSDSLAM.height(frame, level=1) == 480
    @test LSDSLAM.height(frame, level=2) == 240
    @test LSDSLAM.height(frame, level=3) == 120


    for i = 1:LSDSLAM.NUM_PYRAMID_LEVELS
        @test LSDSLAM.𝙄!(frame, level=i) != nothing
    end

    for i = 1:LSDSLAM.NUM_PYRAMID_LEVELS
        @test LSDSLAM.∇x!(frame, level=i) != nothing
        @test LSDSLAM.∇y!(frame, level=i) != nothing
        @test LSDSLAM.∇max!(frame, level=i) != nothing
    end

    LSDSLAM.loaddepth!(frame)

    for i = 1:LSDSLAM.NUM_PYRAMID_LEVELS
        @test LSDSLAM.d⁻¹!(frame, level=i) != nothing
        @test LSDSLAM.σ²!(frame, level=i) != nothing
    end

    frame = LSDSLAM.read!(factory, 2)
    @test LSDSLAM.∇max!(frame, level=LSDSLAM.NUM_PYRAMID_LEVELS) != nothing
    LSDSLAM.loaddepth!(frame)
    @test LSDSLAM.d⁻¹!(frame, level=LSDSLAM.NUM_PYRAMID_LEVELS) != nothing
end

@testset "Camera" begin
    K = LSDSLAM.Camera(5, 10, 3, 4)
    @test isa(K, LSDSLAM.Camera)
    @test K.fx == 5 && K.fy == 10 && K.cx == 3 && K.cy == 4
    @test K.ifx ≈ 1/5 && K.ify ≈ 1/10
end

@testset "Projecting" begin
    K = LSDSLAM.Camera(5, 10, 3, 4)
    x = 8
    y = 12
    z = 13
    @test LSDSLAM.project(K, x, y, z) == (6.076923076923077, 13.23076923076923)
end


@testset "Inverting 3D point" begin
    K = LSDSLAM.Camera(5, 10, 3, 4)
    x = 8
    y = 12
    z = 13

    u, v = LSDSLAM.project(K, x, y, z)
    @test inv(K, u=u, v=v, z=z) == (x, y, z)
end

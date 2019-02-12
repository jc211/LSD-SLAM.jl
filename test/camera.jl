include("../src/lsdslam.jl")
using Test
using LinearAlgebra

@testset "Camera" begin
    K = LSDSLAM.CameraIntrinsics(5, 10, 3, 4)
    @test K.fx == 5 && K.fy == 10 && K.cx == 3 && K.cy == 4
    @test K.ifx ≈ 1/5 && K.ify ≈ 1/10
end

using Test
using Images
include("../src/lsdslam.jl")

@testset "LSD-SLAM" begin
    include("undistorter.jl")
    include("camera.jl")
    include("frame.jl")
    include("tum.jl")
    include("se3tracker.jl")
end

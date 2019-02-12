using Test
include("../src/lsdslam.jl")

@testset "LSD-SLAM" begin
    include("camera.jl")
    include("frame.jl")
    include("tum.jl")
end

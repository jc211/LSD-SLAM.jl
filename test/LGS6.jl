include("../src/lsdslam.jl")
using StaticArrays
using ForwardDiff

function Base.rand(::Type{AffineMap})
    AffineMap(rand(Quat), rand(3))
end

function tohomogeneous(T::AffineMap)
    R = T.linear
    return @SMatrix [R[1,1] R[1,2] R[1,3] T.translation[1];
                    R[2,1] R[2,2] R[2,3] T.translation[2];
                    R[3,1] R[3,2] R[3,3] T.translation[3];
                    0       0      0                    1]
end

function toaffinemap(T::SMatrix{4,4})
    R = RotMatrix{3}(T[1:3,1:3])
    t = T[1:3, 4]
    return AffineMap(R, t)
end

function SE3_exp(w::SVector{6})
    tohomogeneous(LSDSLAM.SE3_exp(w))
end

function SE3_log(T::AbstractMatrix)
    return LSDSLAM.SE3_log(toaffinemap(map))
end


@testset "utility functions" begin
    T = rand(AffineMap)
    @test toaffinemap(tohomogeneous(T)) ≈ T
end

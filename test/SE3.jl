function Base.rand(::Type{AffineMap})
    AffineMap(rand(Quat), rand(3))
end

@testset "SE3 function" begin
    T = rand(AffineMap)
    tangent = LSDSLAM.SE3_log(T)
    @test LSDSLAM.SE3_exp(tangent) ≈ T atol=0.001
end

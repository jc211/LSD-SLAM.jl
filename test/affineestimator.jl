#using Plots
@testset "Affine Correction" begin
    x = [i for i=0:0.001:1]
    slope = 1.1
    bias = 0.4
    y = slope.*(x) .+ bias
    variance = 0.02^2
    y_n = y + randn(size(y))*sqrt(variance)
    estimator = LSDSLAM.AffineEstimator()
    for i in eachindex(x)
        LSDSLAM.update!(estimator, x[i], y_n[i])
    end
    affinecorrection = LSDSLAM.estimate!(estimator)
    @test affinecorrection.slope ≈ slope atol=0.01
    @test affinecorrection.bias ≈ bias atol=0.01

    #scatter(x, y_n, label="noisy line")
    #plot!(x, y, label="real line")
    #plot!(x, y_c, label="estimated line ($(affinecorrection.slope), $(affinecorrection.bias))")
end

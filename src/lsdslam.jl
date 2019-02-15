module LSDSLAM
    using Parameters
    using CSV
    using DataFrames
    using FileIO
    using Images
    using GeometryTypes
    using ColorTypes
    using StaticArrays
    using CoordinateTransformations
    using UUIDs: UUID, uuid1
    using LinearAlgebra
    using Rotations
    using ImageDistortion

    include("constants.jl")
    include("types.jl")
    include("camera.jl")
    include("SE3.jl")
    include("distortion.jl")
    include("framefactory.jl")
    include("frame.jl")
    include("tumframe.jl")
    include("affineestimator.jl")
    include("LGS6.jl")
    include("se3tracker.jl")

    #include("debuggers/se3tracker.jl")
end

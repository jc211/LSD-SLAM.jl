abstract type AbstractUndistorter end

struct SimplePinholeUndistorter <: AbstractUndistorter
    _cameraintrinsics::Matrix{Float64}
    _distcoeffs::NothingOr{Vector{Float64}}
    _xmap::Matrix{Float64}
    _ymap::Matrix{Float64}
    _K::Matrix{Float64}
end

function SimplePinholeUndistorter(cameraintrinsics::AbstractMatrix{<:Real}, distcoeffs::NothingOr{AbstractVector{<:Real}})
    newcameramatrix = get_optimalnewcameramatrix(cameramatrix=cameraintrinsics, distcoeffs=distcoeffs, imgsize=(640,480), alpha=0, newimgsize=(640,480))
    xmap, ymap = init_undistortrectifymap(cameramatrix=cameraintrinsics, distcoeffs=distcoeffs, newcameramatrix=newcameramatrix, imgsize=(640,480))
    x = SimplePinholeUndistorter(cameraintrinsics, distcoeffs, xmap, ymap, newcameramatrix)
end

function (undistorter::SimplePinholeUndistorter)(image::Matrix{Pixel})
    return remap(image, undistorter._xmap, undistorter._ymap) # undistort
end

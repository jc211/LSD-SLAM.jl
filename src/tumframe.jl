struct TUMFrameHeader
    imagetimestamp::Float64
    imagepath::String
    depthtimestamp::Float64
    depthpath::String
end

struct FrameSE3TrackingData
    _𝓕_trackedon::AbstractFrame # The 𝓕₁ frame that was used in SE3 tracker to track this frame
    _pixelsused::PyramidOf{Matrix{Bool}} # Pixels that were used from _𝓕_trackedon
end

function FrameSE3TrackingData(𝓕::AbstractFrame)
    FrameSE3TrackingData(𝓕, [zeros(Bool, height(𝓕, level=i), width(𝓕, level=i)) for i=1:NUM_PYRAMID_LEVELS])
end

mutable struct TUMFrame <: AbstractFrame
    _id::Int64
    _header::TUMFrameHeader
    _data::FrameData
    _undistorter::AbstractUndistorter
    _se3trackingdata::NothingOr{FrameSE3TrackingData}
end

function TUMFrame(;
    id::Integer,
    header::TUMFrameHeader,
    width::Integer,
    height::Integer,
    camera::Camera,
    undistorter::AbstractUndistorter
    )

    framedata = FrameData(width, height, camera)
    TUMFrame(id, header, framedata, undistorter, nothing)
end

function loaddepth!(f::TUMFrame)
    variance = 0.01
    depth_img = Gray.(load("$(f._header.depthpath)"))
    depth_img = rawview(real.(depth_img))/5000
    d = _getframedata(f)

    d.d⁻¹[1] = fill(-1.0, (height(f), width(f)))
    d.σ²[1] = fill(-1.0, (height(f), width(f)))
    for i in eachindex(depth_img)
        if depth_img[i] != 0
            d.d⁻¹[1][i] = 1/depth_img[i]
            d.σ²[1][i] = variance
        end
    end


end

function _require_base_image!(f::TUMFrame)
    d = _getframedata(f)
    image = load("$(f._header.imagepath)")
    d.𝙄[1] = f._undistorter(Gray.(image))
end

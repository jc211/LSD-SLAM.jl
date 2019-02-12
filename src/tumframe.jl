struct TUMFrameHeader
    imagetimestamp::Float64
    imagepath::String
    depthtimestamp::Float64
    depthpath::String
end

mutable struct TUMFrame <: AbstractFrame
    _id::Int64
    _header::TUMFrameHeader
    _data::FrameData
end

function TUMFrame(;id::Integer, header::TUMFrameHeader, width::Integer, height::Integer, cameraintrinsics::CameraIntrinsics)
    framedata = FrameData(width, height, cameraintrinsics)
    TUMFrame(id, header, framedata)
end

function loaddepth!(f::TUMFrame)
    variance = 0.01
    depth_img = Gray.(load("$(f._header.depthpath)"))
    depth_img = rawview(real.(depth_img))/5000
    d = _getframedata(f)
    d.d⁻¹[1] = depth_img
    d.σ²[1] = fill(variance, (height(f), width(f)))
end

function _require_base_image!(f::TUMFrame)
    d = _getframedata(f)
    image = load("$(f._header.imagepath)")
    d.𝙄[1] = Gray.(image)
end

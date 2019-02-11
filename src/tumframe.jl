struct TUMFrameHeader
    imagetimestamp::Float64
    imagepath::String
    depthtimestamp::Float64
    depthpath::String
end

mutable struct TUMFrame <: AbstractFrame
    _id::Int64
    _header::TUMFrameHeader
    _pose::AffineMap
    _data::FrameData  
    _tracking_cache::TrackingCache
    _camera::Camera
end

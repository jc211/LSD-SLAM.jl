struct CameraIntrinsics
    fx::Float64
    fy::Float64
    cx::Float64
    cy::Float64
    ifx::Float64
    ify::Float64
    inv::SMatrix{3,3, Float64}
end

function CameraIntrinsics(K::AbstractMatrix{<:Real})
    return CameraIntrinsics(K[1,1], K[2,2], K[1, 3], K[2,3], 1/K[1,1], 1/K[2,2], inv(K))
end

function CameraIntrinsics(fx::Real, fy::Real, cx::Real, cy::Real)
    m = SMatrix{3,3, Float64}(fx, 0, 0, 0, fy, 0, cx, cy, 1)
    return CameraIntrinsics(m)
end

Base.inv(c::CameraIntrinsics) = c.inv
Base.show(io::IO, c::CameraIntrinsics) = print(io, "$(c())")
Base.convert(::Type{CameraIntrinsics}, x::StaticMatrix{3,3,<:Real}) = CameraIntrinsics(x)

function (c::CameraIntrinsics)()
    return SMatrix{3,3, Float64}(c.fx, 0, 0, 0, c.fy, 0, c.cx, c.cy, 1)
end

function scale(c::CameraIntrinsics, scale::Real)
    return CameraIntrinsics(c.fx*scale, c.fy*scale, c.cx*scale, c.cy*scale)
end

function project(point::Point{3,T}; camera::CameraIntrinsics) :: Point{2, T} where T
    u = camera.fx*point[1]/point[3] + camera.cx
    v = camera.fy*point[2]/point[3] + camera.cy
    return Point{2, T}(u, v)
end

struct Camera
    fx::Float64
    fy::Float64
    cx::Float64
    cy::Float64
    ifx::Float64
    ify::Float64
    inv::SMatrix{3,3, Float64}
end

function Camera(K::AbstractMatrix{<:Real})
    return Camera(K[1,1], K[2,2], K[1, 3], K[2,3], 1/K[1,1], 1/K[2,2], inv(K))
end

function Camera(fx::Real, fy::Real, cx::Real, cy::Real)
    m = SMatrix{3,3, Float64}(fx, 0, 0, 0, fy, 0, cx, cy, 1)
    return Camera(m)
end

Base.inv(c::Camera) = c.inv
Base.show(io::IO, c::Camera) = print(io, "$(c())")
Base.convert(::Type{Camera}, x::StaticMatrix{3,3,<:Real}) = Camera(x)

function (c::Camera)()
    return SMatrix{3,3, Float64}(c.fx, 0, 0, 0, c.fy, 0, c.cx, c.cy, 1)
end

function scale(c::Camera, scale::Real)
    return Camera(c.fx*scale, c.fy*scale, c.cx*scale, c.cy*scale)
end

"Project a 3D point to a camera plane"
function project(camera::Camera, x::Real, y::Real, z::Real)
    u = camera.fx*x/z + camera.cx
    v = camera.fy*y/z + camera.cy
    return (u, v)
end

"Invert a point in an image to the 3D world"
function Base.inv(c::Camera; u::Real, v::Real, z::Real)
    x = (u-c.cx)*z*c.ifx
    y = (v-c.cy)*z*c.ify
    return x, y, z
end

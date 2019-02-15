# 𝓕₁ Pointcloud
# =================================

struct 𝓕₁_Point
    𝙄::Pixel # Intensity of point
    ∇x::Float64 # Derivative of intensity in x direction
    ∇y::Float64 # Derivative of intensity in y direction
    x::Float64 # X Position of 3D point
    y::Float64 # Y Position of 3D point
    z::Float64 # Z Position of 3D point
    σ²::Float64 # Variance of 1/z
    index::CartesianIndex # Index of point in original image
end

const 𝓕₁_Points = Vector{NothingOr{𝓕₁_Point}}

struct 𝓕₁_Pointcloud
    p::PyramidOf{𝓕₁_Points} # Pyramid of point clouds for 𝓕₁
    ∑p::PyramidOf{Int64} # Number of points actually used in each level of _points_𝓕₁
end

function 𝓕₁_Pointcloud(width::Integer, height::Integer)
    _points_𝓕₁ = 𝓕₁_Points[NothingOr{𝓕₁_Point}[nothing for j=1:(width*height÷i)] for i=1:NUM_PYRAMID_LEVELS]
    ∑_points_𝓕₁ = [0 for  i=1:NUM_PYRAMID_LEVELS]
    𝓕₁_Pointcloud(_points_𝓕₁, ∑_points_𝓕₁)
end


# 𝓕₂ Pointcloud
# =================================

struct 𝓕₂_Point
    𝙄::Pixel # Intensity of point
    ∇x::Float64 # Derivative of intensity in x direction
    ∇y::Float64 # Derivative of intensity in y direction
    x::Float64 # X Position of 3D point
    y::Float64 # Y Position of 3D point
    z::Float64 # Z Position of 3D point
    𝓕₁_point::Ref{𝓕₁_Point} ## Corresponding point in 𝓕₁
end

const 𝓕₂_Points = Vector{NothingOr{𝓕₂_Point}}

struct 𝓕₂_Pointcloud
    p::PyramidOf{𝓕₂_Points} # Pyramid of point clouds for 𝓕₁
    ∑p::PyramidOf{Int64} # Number of points actually used in each level of _points_𝓕₁
end

function 𝓕₂_Pointcloud(width::Integer, height::Integer)
    _points_𝓕₂ = 𝓕₂_Points[NothingOr{𝓕₂_Point}[nothing for j=1:(width*height÷i)] for i=1:NUM_PYRAMID_LEVELS]
    ∑_points_𝓕₂ = [0 for  i=1:NUM_PYRAMID_LEVELS]
    𝓕₂_Pointcloud(_points_𝓕₂, ∑_points_𝓕₂)
end

# Statistics
# ==============================
@with_kw mutable struct SE3TrackerStatistics
    pointsskipped::PyramidOf{Int64} = [0 for i=1:NUM_PYRAMID_LEVELS]
end

function reset!(s::SE3TrackerStatistics)
    pointsskipped = [0 for i=1:NUM_PYRAMID_LEVELS]
end



# Error Point
# ==============================
struct SE3Error
    photometricerror::Float64
    _𝓕₁_point::Ref{NothingOr{𝓕₁_Point}}
    _𝓕₂_point::Ref{NothingOr{𝓕₂_Point}}
end

# SE3 Tracker
# =================================
@with_kw mutable struct SE3Tracker
    _𝓕₁::Ref{NothingOr{AbstractFrame}} = nothing # Last frame used as a base for tracking
    _𝓕₁_pointcloud::𝓕₁_Pointcloud # 𝓕₁ converted to pointcloud
    _𝓕₂_pointcloud::𝓕₂_Pointcloud # 𝓕₁ pointcloud converted to 𝓕₂ frame
    _affinecorrector::AffineCorrection = AffineCorrection()
    _proposedaffinecorrector::AffineCorrection = AffineCorrection()
    _statistics::SE3TrackerStatistics = SE3TrackerStatistics()
    _solver::LGS6 = LGS6()
end

function SE3Tracker(width::Integer, height::Integer)
    SE3Tracker(
        _𝓕₁_pointcloud = 𝓕₁_Pointcloud(width, height),
        _𝓕₂_pointcloud = 𝓕₂_Pointcloud(width, height))
end


"Get the relative transform between 𝓕₁ and 𝓕₂ by minimizing photometric error"
function (tracker::SE3Tracker)(𝓕₁::AbstractFrame, 𝓕₂::AbstractFrame, ξ₀::AffineMap)

    #                                        ξ0
    #                                         |   Transform to 𝓕₂
    #           Depth        𝓕₁ Pointcloud   |     𝓕₂ Pointcloud
    #         +------+                        |
    #       +------+ |           +------+     |      +------+             +------+
    #       |      | |           |      |     v      |      |             |      |
    #       |  𝓕₁  | | +-------> |      | +-------> |      +----->+<-----+  𝓕₂  |
    #       |       -+           |      |     ^      |      |      |      |      |
    #       +------+             +------+     |      +------+      |      +------+
    #                                         |                    |
    #                                         |                    v
    #                                         |             Photometric Error
    #                                         |                    +
    #                                         |                    |
    #                                         |                    |
    #                                         +--------------------ξ
    #

    tracker._𝓕₁ != 𝓕₁ && _make_𝓕₁_pointcloud!(tracker, 𝓕₁)
    𝓕₂._se3trackingdata = FrameSE3TrackingData(𝓕₁)
    ξ = ξ₀

    for level = SE3TRACKING_MAX_LEVEL:-1:SE3TRACKING_MIN_LEVEL

        _transform_𝓕₁_to_𝓕₂!(tracker, 𝓕₂, ξ, level)
        # check if enough points were found
        if(tracker._𝓕₂_pointcloud.∑p[level] < 0.01*width(𝓕₂, level=level)*height(𝓕₂, level=level))
            return nothing
        end

        prev_error = _calculate_residuals!(tracker, ξ, level)

        λ::Float64 = 1.0*λ_INITIAL
        for  i=1:MAX_ITERATIONS[level]
            incTry = 0
            while true
                incTry += 1
                ξ, inc = _propose_new_ξ!(tracker, ξ, λ=λ)
                _transform_𝓕₁_to_𝓕₂!(tracker, 𝓕₂, ξ, level)
                error = _calculate_residuals!(tracker, ξ, level)
                status = _evaluate_error(error, prev_error, inc)

                @show level, incTry, status, error
                if status == OK
                    λ = λ <= 0.2 ? 0 : λ*λ_SUCCESS # scale λ down
                    prev_error = error
                    break
                elseif status == NOTOK
                    λ = λ == 0 ? 0.2 : λ*(λ_FAIL^incTry) # scale λ up
                elseif status == CONVERGED
                    @goto next_level
                elseif status == STEPTOOSMALL
                    @goto next_level
                end
            end
        end
        @label next_level

    end

    return ξ
end

"Transforms the pointcloud in 𝓕₁ to 𝓕₂"
function _transform_𝓕₁_to_𝓕₂!(tracker::SE3Tracker, 𝓕₂::AbstractFrame, ξ::AbstractAffineMap, level::Integer)
    _𝓕₁_pointcloud = tracker._𝓕₁_pointcloud
    _𝓕₂_pointcloud = tracker._𝓕₂_pointcloud

    𝓕₁_points = @view _𝓕₁_pointcloud.p[level][1:(_𝓕₁_pointcloud.∑p[level])]

    𝙄 = 𝙄!(𝓕₂, level=level) # 𝓕₂ Image at pyramid level
    ∇x = ∇x!(𝓕₂, level=level) # 𝓕₂ x gradients at pyramid level
    ∇y = ∇y!(𝓕₂, level=level) # 𝓕₂ y gradients at pyramid level
    K = camera(𝓕₂, level=level) # Camera object for pyramid level
    w = width(𝓕₂, level=level) # 𝓕₂ width
    h = height(𝓕₂, level=level) # 𝓕₂ height

    ∑p = 0 # number of points transformed from 𝓕₁ to 𝓕₂

    for p_𝓕₁ in 𝓕₁_points
        x, y, z = ξ(SVector(p_𝓕₁.x, p_𝓕₁.y, p_𝓕₁.z))
        u_𝓕₂, v_𝓕₂ = project(K, x, y, z)

        # check if projected point is in image 𝓕₂
        if !(u_𝓕₂>2 && u_𝓕₂<w-1 && v_𝓕₂ >2 && v_𝓕₂ < h-1)
            tracker._statistics.pointsskipped[level] += 1
            continue
        end

        # Remember which pixels we used from frame 1
        𝓕₂._se3trackingdata._pixelsused[level][p_𝓕₁.index] = true

        ∑p += 1
        _𝓕₂_pointcloud.p[level][∑p] = 𝓕₂_Point(
            bilinear_interpolation(𝙄, v_𝓕₂, u_𝓕₂),
            bilinear_interpolation(∇x, v_𝓕₂, u_𝓕₂),
            bilinear_interpolation(∇y, v_𝓕₂, u_𝓕₂),
            x,
            y,
            z,
            p_𝓕₁)
    end
    _𝓕₂_pointcloud.∑p[level] = ∑p
end

"Construct points cloud from the frame data and stores it in the given cache"
function _make_𝓕₁_pointcloud!(tracker::SE3Tracker, 𝓕::AbstractFrame)
    tracker._𝓕₁ = 𝓕
    for level=SE3TRACKING_MIN_LEVEL:SE3TRACKING_MAX_LEVEL
        _make_𝓕₁_pointcloud!(tracker, 𝓕, level)
    end
end

"Construct points cloud from the frame data and stores it in the given cache"
function _make_𝓕₁_pointcloud!(tracker::SE3Tracker, 𝓕::AbstractFrame, level::Integer)
    _𝓕₁_pointcloud = tracker._𝓕₁_pointcloud
    𝙄 = 𝙄!(𝓕, level=level) # 𝓕 Image at pyramid level
    ∇x = ∇x!(𝓕, level=level) # 𝓕 x gradients at pyramid level
    ∇y = ∇y!(𝓕, level=level) # 𝓕 y gradients at pyramid level
    d⁻¹ = d⁻¹!(𝓕, level=level) # 𝓕 inverse depth at pyramid level
    σ² = σ²!(𝓕, level=level) # 𝓕 variance at pyramid level
    K = camera(𝓕, level=level) # Camera object for pyramid level
    ∑p = 0 # number of points transformed to 3d points

    for p in CartesianIndices(𝙄)
        if d⁻¹[p] == -1
            continue
        end
        x, y, z = inv(K, u=p[2], v=p[1], z=1/d⁻¹[p]) # invert 2d point into 3d world using camera
        ∑p += 1 # track added point
        _𝓕₁_pointcloud.p[level][∑p] = 𝓕₁_Point(𝙄[p], ∇x[p], ∇y[p], x, y, z, σ²[p], p)
    end
    _𝓕₁_pointcloud.∑p[level] = ∑p
end

@enum SE3ErrorStatus begin
    OK
    NOTOK
    STEPTOOSMALL
    CONVERGED
end

function _evaluate_error(error::Float64, previouserror::Float64, increment::SVector{6})::SE3ErrorStatus
    if error < previouserror
        convergence = error/previouserror
        if convergence > CONVERGENCEEPS
            return CONVERGED
        else
            return OK
        end
    else
        if norm(increment) <= MINSTEPSIZE
            return STEPTOOSMALL
        else
            return NOTOK
        end
    end
end


"Solve the Guass Newton system and propose a new ξ to reduce the error"
function _propose_new_ξ!(tracker::SE3Tracker, ξ::AffineMap; λ::Float64 = 0)
    inc = solve!(tracker._solver, λ=λ)
    ξ_new = SE3_exp(inc)
    reset!(tracker._solver)
    return ξ_new, inc
end

"Compute the current photometric error for a specific level"
function _calculate_residuals!(tracker::SE3Tracker, ξ::AffineMap, level::Integer)
    # Photometric Error is given by
    # Ep(ξ) = ∑|rp²/σ²_rp|
    # rp = I₁ - I₂(w(p,ξ,d))
    # σ_rp = 2σ²_I + (δrδd)²*σ²_d

    # Huber Error is applied for robustness
    # huber(Ep) = Ep² if |Ep| < k
    # huber(Ep) = Ep²*k/|Ep| if |Ep| > k
    _𝓕₂_points = @view tracker._𝓕₂_pointcloud.p[level][1:(tracker._𝓕₂_pointcloud.∑p[level])]

    _K = camera(tracker._𝓕₁.x, level=level)

    affineestimator = AffineEstimator()
    tracker._solver = LGS6()

    ∑residuals = 0
    ∑p = 0
    for p_𝓕₂ in _𝓕₂_points
        p_𝓕₁ = p_𝓕₂.𝓕₁_point.x

        # Calculate Photometric Error
        𝙄_𝓕₁ = tracker._affinecorrector(p_𝓕₁.𝙄)
        rp = 𝙄_𝓕₁ - p_𝓕₂.𝙄

        # Calculate error variance
        wp = 1/_calculate_error_variance(p_𝓕₂, ξ, _K) # photometric weight

        # M Estimator - Calculate final weight
        weighted_rp = abs(rp)*√(wp)
        wh = weighted_rp<HUBER_THRESHOLD/2 ? 1 : HUBER_THRESHOLD/(2*weighted_rp) # Huber weight
        weight = wh*wp

        # Calculate Jacobian
        J = _calculate_jacobian(p_𝓕₂, _K)

        # Update affine estimator
        update!(affineestimator, 𝙄_𝓕₁, p_𝓕₂.𝙄)

        # Update solver
        update!(tracker._solver, J, rp, weight)

        # Update final error
        ∑p += 1
        ∑residuals += weight*rp*rp
    end

    # Propose new affine parameters
    tracker._proposedaffinecorrector = estimate!(affineestimator)

    return ∑residuals/∑p
end

"Calculate jacobian of photometric error relative to ξ tangent space"
function _calculate_jacobian(p::𝓕₂_Point, K::Camera)
    ∇x = K.fx*p.∇x
    ∇y = K.fy*p.∇y
    d⁻¹ = 1/p.z
    d⁻² = 1/(p.z * p.z)

    J =  SVector(
        d⁻¹                     * ∇x + 0,
        0                            + d⁻¹                        * ∇y,
        (-p.x * d⁻²)            * ∇x + (-p.y * d⁻²)               * ∇y,
        (-p.x * p.y * d⁻²)      * ∇x + (-(1.0 + p.y * p.y * d⁻²)) * ∇y,
        (1.0 + p.x * p.x * d⁻²) * ∇x + (p.x * p.y * d⁻²)          * ∇y,
        (-p.y * d⁻¹)            * ∇x + (p.x * d⁻¹)                * ∇y
        )
    J
end

"Calculate the propagated variance of the photometric error"
function _calculate_error_variance(p::𝓕₂_Point, ξ::AffineMap, camera::Camera)
    # Calculate the derivative of the residual relative to the inverse depth
    𝓕₁_point = p.𝓕₁_point.x
    tx, ty, tz = ξ.translation
    d = 1/𝓕₁_point.z
    σ² = 𝓕₁_point.σ² * σ²_WEIGHT

    # Deriving the below equation requires the intuition that the 3d homogeneous
    # point [u, v, 1, d] is the same as [x, y, z, 1] where u and v are the pixels
    # coordinate and d is the inverse depth.

    # g = [R, T]∗[u, v, 1, d]
    # wx = fx*gx/gz = fx*(r11*u + r12*v + r13 + d*tx)/(r31*u + r32*v + r33 + d*tz)
    # deriving gets us the equations below

    δwxδd = camera.fx*(tx*p.z - tz*p.x) / (p.z*p.z*d)
    δwyδd = camera.fy*(ty*p.z - tz*p.y) / (p.z*p.z*d)
    δrδd = p.∇x*δwxδd + p.∇y*δwyδd

    # Apply propagation of error: Σr = J'*Σ*J. Intrestingly they are not
    # accounting for the slope of the affine estimator
    return 2*σ²_CAMERA + σ² * δrδd * δrδd
end

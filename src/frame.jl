
abstract type AbstractFrame end;

@enum FrameDataFlag begin
    IMAGE = 1
    GRADIENT = 2
    DEPTH = 3
end

@with_kw struct FrameData

    #            +-----------+
    #            | Level   4 |
    #         +--+-----------+--+
    #         |    Level   3    |
    #      +--+-----------------+--+
    #      |       Level   2       |
    #   +--+-----------------------+--+
    #   |          Level   1          |
    #   +-----------------------------+
    #
    # Datastructure to hold image pyramids, gradient pyramids and depth pyramids
    #

    w::PyramidOf{Int64}
    h::PyramidOf{Int64}
    K::PyramidOf{CameraIntrinsics}
    𝙄::PyramidOfNothingOr{Matrix{Pixel}} = NothingOr{Matrix{Pixel}}[nothing for i=1:NUM_PYRAMID_LEVELS]
    ∇x::PyramidOfNothingOr{Matrix{Float64}} = NothingOr{Matrix{Float64}}[nothing for i=1:NUM_PYRAMID_LEVELS]
    ∇y::PyramidOfNothingOr{Matrix{Float64}} = NothingOr{Matrix{Float64}}[nothing for i=1:NUM_PYRAMID_LEVELS]
    ∇max::PyramidOfNothingOr{Matrix{Float64}} = NothingOr{Matrix{Float64}}[nothing for i=1:NUM_PYRAMID_LEVELS]
    d⁻¹::PyramidOfNothingOr{Matrix{Float64}} = NothingOr{Matrix{Float64}}[nothing for i=1:NUM_PYRAMID_LEVELS]
    σ²::PyramidOfNothingOr{Matrix{Float64}} = NothingOr{Matrix{Float64}}[nothing for i=1:NUM_PYRAMID_LEVELS]
end

function FrameData(width::Integer, height::Integer, cameraintrinsics::CameraIntrinsics)
    width =  Int64[width÷(1<<i) for i=0:NUM_PYRAMID_LEVELS-1]
    height =  Int64[height÷(1<<i) for i=0:NUM_PYRAMID_LEVELS-1]
    k =  CameraIntrinsics[scale(cameraintrinsics, 1/(1<<i)) for i=0:NUM_PYRAMID_LEVELS-1]
    return FrameData(w=width, h= height, K=k)
end

function width(f::AbstractFrame; level::Integer = 1)
    d = _getframedata(f)
    d.w[level]
end

function height(f::AbstractFrame; level::Integer = 1)
    d = _getframedata(f)
    d.h[level]
end

function K(f::AbstractFrame; level::Integer = 1)
    d = _getframedata(f)
    d.K[level]
end

function 𝙄!(f::AbstractFrame; level::Integer = 1)
    _require!(f, IMAGE, level=level)
    d = _getframedata(f)
    d.𝙄[level]
end

function ∇x!(f::AbstractFrame; level::Integer = 1)
    _require!(f, GRADIENT, level=level)
    d = _getframedata(f)
    d.∇x[level]
end

function ∇y!(f::AbstractFrame; level::Integer = 1)
    _require!(f, GRADIENT, level=level)
    d = _getframedata(f)
    d.∇y[level]
end

function ∇max!(f::AbstractFrame; level::Integer = 1)
    _require!(f, GRADIENT, level=level)
    d = _getframedata(f)
    d.∇max[level]
end

function d⁻¹!(f::AbstractFrame; level::Integer = 1)
    _require!(f, DEPTH, level=level)
    d = _getframedata(f)
    d.d⁻¹[level]
end

function σ²!(f::AbstractFrame; level::Integer = 1)
    _require!(f, DEPTH, level=level)
    d = _getframedata(f)
    d.σ²[level]
end


_getframedata(f::AbstractFrame)::FrameData = f._data

function _require!(f::AbstractFrame, flag::FrameDataFlag; level::Integer=1)
    if flag == IMAGE
        return _require_image!(f, level=level)
    elseif flag == GRADIENT
        return _require_gradient!(f, level=level)
    elseif flag == DEPTH
        return _require_depth!(f, level=level)
    end
end

function _require_image!(f::AbstractFrame; level::Integer=1)
    #    +--+   +--+   +--+   +--+   +--+
    #    |I1+-->+I2+-->+I3+-->+I4+-->+I5|
    #    +--+   +--+   +--+   +--+   +--+
    d = _getframedata(f)
    if !isnothing(d.𝙄[level])
        return
    end
    # Start Nodes
    if level == 1
        return _require_base_image!(f)
    end
    _require_image!(f, level=level-1)
    _build_image!(f, level=level)
end

function _require_gradient!(f::AbstractFrame; level::Integer=1)
    #    +--+   +--+   +--+   +--+   +--+
    #    |I1+-->+I2+-->+I3+-->+I4+-->+I5|
    #    +--+   +--+   +--+   +--+   +--+
    #       ^      ^      ^      ^      ^
    #       |      |      |      |      |
    #    +--+   +--+   +--+   +--+   +--+
    #    |G1+-->+G2+-->+G3+-->+G4+-->+G5|
    #    +--+   +--+   +--+   +--+   +--+
    d = _getframedata(f)
    if !isnothing(d.∇x[level])
        return
    end
    _require_image!(f, level=level)
    _build_gradient!(f, level=level)
end

function _require_depth!(f::AbstractFrame; level::Integer=1)
    #    +--+   +--+   +--+   +--+   +--+
    #    |D1+-->+D2+-->+D3+-->+D4+-->+D5|
    #    +--+   +--+   +--+   +--+   +--+
    d = _getframedata(f)
    if !isnothing(d.d⁻¹[level])
        return
    end
    # Start Nodes
    if level == 1
        return _require_depth!(f)
    end
    _require_depth!(f, level=level-1)
    _build_depth!(f, level=level)
end


function _require_base_image!(f::AbstractFrame)
    throw("Loading image from disk is not supported")
end

function _require_base_depth!(f::AbstractFrame)
    throw("Loading depth from disk is not supported")
end

function _build_image!(f::AbstractFrame; level::Integer=1)
    d = _getframedata(f)
    s = size(d.𝙄[level-1]) .÷2
    d.𝙄[level] = imresize(d.𝙄[level-1], s)
end

function _build_gradient!(f::AbstractFrame; level::Integer=1)
    d = _getframedata(f)
    d.∇y[level], d.∇x[level], d.∇max[level], theta = imedge(d.𝙄[level], KernelFactors.ando3, "replicate")
    d.∇max[level] = mapwindow(maximum, d.∇max[level], (3,3))
end

function _build_depth!(f::AbstractFrame; level::Integer=1)
    data = _getframedata(f)

    w = width(f, level=level)
    h = height(f, level=level)

    # preallocate memory
    data.d⁻¹[level] = Matrix{Float64}(undef, h, w)
    data.σ²[level] = Matrix{Float64}(undef, h, w)

    w = width(f, level=level-1)
    h = height(f, level=level-1)

    for i=1:2:h-1, j=1:2:w-1

        d⁻¹_window = view(data.d⁻¹[level-1], i:i+1, j:j+1)
        σ²_window = view(data.σ²[level-1], i:i+1, j:j+1)

        μ, σ², ∑n =  _depth_weightedaverage_onwindow(d⁻¹_window, σ²_window)

        ii = (i-1)÷2 + 1
        jj = (j-1)÷2 + 1
        if ∑n == 0
            data.d⁻¹[level][ii, jj] = -1
            data.σ²[level][ii, jj] = -1 # Multiply variance to account for correlation
        else
            data.d⁻¹[level][ii, jj] = μ
            data.σ²[level][ii, jj] = ∑n * σ² # Multiply variance to account for correlation
        end

        # Remember the calculation up to this point assumes A,B,C, and D are independant
        # which they are likely not. So the variance is likely higher than this because
        # we are not getting as much information as we think we are.
    end

end

function _depth_weightedaverage_onwindow(window::T, weightswindow::T) where T<:SubArray{Float64,2,Array{Float64,2},Tuple{UnitRange{Int64},UnitRange{Int64}},false}

    #       Source                   Reduced
    #   +------+------+
    #   |      |      |
    #   |  A   |  B   |             +------+
    #   |      |      |             |      |
    #   +-------------+ +---------> |  E   |
    #   |      |      |             |      |
    #   |  C   |  D   |             +-------
    #   |      |      |
    #   +------+------+

    # E is the weighted average. We want the weight to be bigger if the variance is lower so we use
    # the inverse of the variance as a weight.
    #
    #        σ⁻²(A)μ(A) + σ⁻²(B)μ(B) + σ⁻²(C)μ(C) + σ⁻²(D)μ(D)
    # E = --------------------------------------------------
    #               σ⁻²(A)+ σ⁻²(B) + σ⁻²(C) + σ⁻²(D)
    #
    # The rules for calculating the the mean and the variance of the above assuming that A,B,C, and D
    # are independant are given by:
    #
    # ∑aᵢXᵢ ~ N(∑aᵢμᵢ, ∑(aᵢσᵢ)²)
    #

    ∑d⁻¹ = 0
    ∑σ⁻² = 0
    ∑n = 0

    for k in eachindex(window)
        d⁻¹ = window[k]
        σ² = weightswindow[k]
        if d⁻¹ != -1
            σ⁻² = 1/σ²
            ∑σ⁻² += σ⁻²
            ∑d⁻¹ += σ⁻²*d⁻¹
            ∑n += 1
        end
    end

    σ² = 1/∑σ⁻²
    μ = ∑d⁻¹*σ²

    return μ, σ², ∑n
end

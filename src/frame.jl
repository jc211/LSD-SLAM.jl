
abstract type AbstractFrame end;

@enum FrameDataFlag begin
    IMAGE = 1
    GRADIENT = 2
    DEPTH = 3
end

@with_kw struct FrameData
    timestamp::Float64 = -1
    𝙄::PyramidOfNothingOr{Matrix{Gray{Normed{UInt8,8}}}} = [nothing for i=1:NUM_PYRAMID_LEVELS]
    ∇x::PyramidOfNothingOr{Matrix{Float64}} = [nothing for i=1:NUM_PYRAMID_LEVELS]
    ∇y::PyramidOfNothingOr{Matrix{Float64}} = [nothing for i=1:NUM_PYRAMID_LEVELS]
    ∇max::PyramidOfNothingOr{Matrix{Float64}} = [nothing for i=1:NUM_PYRAMID_LEVELS]
    d⁻¹::PyramidOfNothingOr{Matrix{Float64}} = [nothing for i=1:NUM_PYRAMID_LEVELS]
    σ²::PyramidOfNothingOr{Matrix{Float64}} = [nothing for i=1:NUM_PYRAMID_LEVELS]
end 


function timestamp(f::AbstractFrame) 
    d = _getframedata(f)
    d._timestamp 
end

function width(f::AbstractFrame; level::Integer = 1) end
function height(f::AbstractFrame; level::Integer = 1) end
function K(f::AbstractFrame; level::Integer = 1) end
function K⁻¹(f::AbstractFrame; level::Integer = 1) end

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

function d⁻¹(f::AbstractFrame; level::Integer = 1)
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




function _require(f::AbstractFrame, flag::FrameDataFlag; level::Integer=1)
    if flag == IMAGE
        return _require_image!(f, level=level)
    elseif flag == GRADIENT
        return _require_gradient!(f, level=level)
    elseif flag == DEPTH
        return _require_base_depth!(f, level=level)
    end
end

function _require_image!(f::AbstractFrame; level::Integer=1)
    #    +--+   +--+   +--+   +--+   +--+
    #    |I1+-->+I2+-->+I3+-->+I4+-->+I5|
    #    +--+   +--+   +--+   +--+   +--+

    if !isnothing(f.𝙄[level])
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


function _require_base_image(f::AbstractFrame) 
    throw("Loading image from disk is not supported")
end

function _require_base_depth(f::AbstractFrame) 
    throw("Loading depth from disk is not supported")
end

function _build_image!(f::AbstractFrame; level::Integer=1)
    d = _getframedata(f)
    size = size(d.𝙄[level-1]) .÷2
    d.𝙄[level] = imresize(d.𝙄[level-1], size)
end

function _build_gradient!(f::AbstractFrame; level::Integer=1)
    d = _getframedata(f)
    d.∇y[level], d.∇x[level], d.∇max[level], theta = imedge(d.𝙄[level], KernelFactors.ando3, "replicate")
    mapwindow!(maximum, f.∇max[level], (3,3))
end

function _build_depth!(f::AbstractFrame; level::Integer=1)

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

    data = _getframedata(f)

    w = width(f, level=level)
    h = height(f, level=level)

    # preallocate memory 
    data.d⁻¹[level] = Array{Float64}(undef, h, w)
    data.σ²[level] = Array{Float64}(undef, h, w)

    w = width(f, level=level-1)
    h = height(f, level=level-1)
    
    for i=1:2:h-1, j=1:2:w-1
        ∑d⁻¹ = 0
        ∑σ⁻² = 0
        ∑n = 0

        d⁻¹_window = view(data.d⁻¹[level-1], i:i+1, j:j+1)
        σ²_window = view(data.σ²[level-1], i:i+1, j:j+1)
        for k in eachindex(σ²_window)
            d⁻¹ = d⁻¹_window[k]
            σ² = σ²_window[k]
            if d⁻¹ != -1
                σ⁻² = 1/σ²
                ∑σ⁻² += σ⁻²
                ∑d⁻¹ += σ⁻²*d⁻¹
                ∑n += 1
            end
        end

        if Σn == 0
            data.d⁻¹[level] = -1
            data.σ²[level] = -1
        end

        σ² = 1/∑σ⁻²
        μ = ∑d⁻¹*σ² 

        ii = (i-1)÷2 + 1
        jj = (j-1)÷2 + 1
        data.d⁻¹[level][ii, jj] = μ
        data.σ²[level][ii, jj] = ∑n * σ² # Multiply variance to account for correlation 

        # Remember the calculation up to this point assumes A,B,C, and D are independant 
        # which they are likely not. So the variance is likely higher than this because 
        # we are not getting as much information as we think we are. 
    end
    
end


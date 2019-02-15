# Affine Correction
# =================================
"Affine correction parameters to deal with changing camera response functions"
@with_kw struct AffineCorrection
    slope::Float64 = 1
    bias::Float64 = 0
end

(a::AffineCorrection)(v) = a.slope*v + a.bias

@with_kw mutable struct AffineEstimator
    sxx::Float64 = 0
    syy::Float64 = 0
    sx::Float64 = 0
    sy::Float64 = 0
    sw::Float64 = 0
end

function update!(e::AffineEstimator, x, y)
    error = abs(y-x)
    w = error < 0.02 ? 1 : 0.02/error
    e.sxx += x*x*w
    e.syy += y*y*w
    e.sx += x*w
    e.sy += y*w
    e.sw += w
end

function estimate!(e::AffineEstimator)
    slope = √((e.syy - e.sy*e.sy/e.sw)/ (e.sxx-e.sx*e.sx/e.sw))
    bias = (e.sy - slope*e.sx)/e.sw
    return AffineCorrection(slope=slope, bias=bias)
end

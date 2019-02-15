@with_kw mutable struct LGS6
    ∑p::Int64 = 0
    _A::MMatrix{6,6, Float64} = @SMatrix zeros(6,6) # Gauss Newton System Ax = b
    _b::SVector{6, Float64} = SVector(0,0,0,0,0,0) # Guass Newton System Ax = b
end

function update!(ls::LGS6, jacobian, error, weight)
    ls.∑p += 1
    ls._A += jacobian*jacobian'*weight
    ls._b -= jacobian*error*weight
end

function reset!(ls::LGS6)
    ls.∑p += 0
    ls._A = @SMatrix zeros(6,6)
    ls._b = SVector(0,0,0,0,0,0)
end

function solve!(ls::LGS6; λ::Float64 = 0)
    ls._A = ls._A / ls.∑p
    ls._b = ls._b / ls.∑p
    ls._A[diagind(ls._A)] *= 1+λ # LM alteration to the jacabian
    return ls._A \ (-ls._b)
end

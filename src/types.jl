const NothingOr{T} = Union{T, Nothing} where T
const PyramidOf{T} = Vector{<:T} where T
const PyramidOfNothingOr{T} = Vector{<:NothingOr{T}} where T
const Pixel = Gray{Normed{UInt8,8}}

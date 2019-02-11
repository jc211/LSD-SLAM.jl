const NothingOr{T} = Union{T, Nothing} where T
const PyramidOf{T} = Array{<:T} where T
const PyramidOfNothingOr{T} = Array{<:NothingOr{T}} where T
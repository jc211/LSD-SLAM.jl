mutable struct PreallocatedVector{T}
    _length::Int64
    _size::Int64
    _data::Vector{T}
end

function PreallocatedVector{T}(size) where T
    PreallocatedVector(0, size, Vector{T}(undef, size))
end

function Base.iterate(S::PreallocatedVector{T}) where T
    S._length == 0 ? nothing : (S._data[1], 1)
end

function Base.iterate(S::PreallocatedVector{T}, state) where T
    if state == S._length
        return nothing
    else
        nextstate = state + 1
        return S._data[nextstate], nextstate
    end
end

function Base.eltype(::Type{PreallocatedVector{T}}) where T
    return T
end

function Base.length(S::PreallocatedVector{T}) where T
    return S._length
end

function Base.getindex(S::PreallocatedVector{T}, i) where T
    return S._data[i]
end

function Base.setindex!(S::PreallocatedVector{T}, v, i) where T
    if i > S._length
        S._length = i
    end
    S._data[i] = v
end

function Base.firstindex(S::PreallocatedVector{T}) where T
    return S._data[1]
end

function Base.lastindex(S::PreallocatedVector{T}) where T
    return S._data[S._length]
end

function reset!(S::PreallocatedVector{T}) where T
    S._length = 0
end

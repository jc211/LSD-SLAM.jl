function SE3_exp(w::SVector{6})::AffineMap
    t = SVector{3}(w[1:3])
    w = SVector{3}(w[4:6])

    rod = RodriguesVec(w...)
    θ = rotation_angle(rod)
    ω = SVector{3}(rod.sx, rod.sy, rod.sz)
    ŵ = skewsymmetric(w)

    θ² = θ*θ
    θ³ = θ²*θ

    if θ ≈ 0
        V = I
    else
        V = I + ŵ*(1-cos(θ))/θ²  + ŵ*ŵ*(θ - sin(θ))/θ³
    end

    return AffineMap(rod, V*t)
end

function skewsymmetric(w)
    return @SMatrix [0 -w[3] w[2]; w[3] 0 -w[1]; -w[2] w[1] 0]
 end

function SE3_log(tform::AffineMap)
    # make sure tform.linear is actually a rotation

    rot = RodriguesVec(tform.linear)
    θ = rotation_angle(rot)
    θ² = θ*θ

    ω = SVector{3, Float64}(rot.sx, rot.sy, rot.sz)
    ŵ = skewsymmetric(ω)

    if abs(θ) ≈ 0
        V⁻¹= I - 0.5*ŵ + 1/12*ŵ*ŵ
    else
        V⁻¹= I - 0.5*ŵ + 1/θ² * (1-((θ*cos(θ/2)/(2sin(θ/2)))))*ŵ*ŵ
    end

    t = V⁻¹*tform.translation

    return [t; ω]


end

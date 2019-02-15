include("../src/lsdslam.jl")
include("groundtruth.jl")

using ProfileView
K = [517.3 0 318.6; 0 516.5 244.3; 0 0 1]
d = [0.2624, -0.9531, -0.0054, 0.0026, 1.1633];
factory = LSDSLAM.TUMFrameFactory(DATASET_DIR, 640, 480, K, d)
ξ₀ = AffineMap(one(RotMatrix{3}), SVector(0,0,0))
groundtruth = TUMGroundTruth()

baseframe = LSDSLAM.read!(factory, 1)
LSDSLAM.loaddepth!(baseframe)
relframe = LSDSLAM.read!(factory, 2)
tracker = LSDSLAM.SE3Tracker(640,480)

@test tracker(baseframe, relframe, ξ₀) ≈ groundtruth(baseframe, relframe) atol=0.01

tracker._statistics

tracker._𝓕₁_pointcloud
tracker._𝓕₂_pointcloud

tracker._𝓕₁

function profile_test(n)
    for i = 1:n
        A = randn(100,100,20)
        m = maximum(A)
        Am = mapslices(sum, A; dims=2)
        B = A[:,:,5]
        Bsort = mapslices(sort, B; dims=1)
        b = rand(100)
        C = B.*b
    end
end
Base.Sys.KERNEL
using MeshCat
using GeometryTypes
function extract_𝓕₁_pointcloud(tracker::LSDSLAM.SE3Tracker; level::Integer = 1)
    pointcloud = tracker._𝓕₁_pointcloud
    points = @view pointcloud.p[level][1:(pointcloud.∑p[level])]
    verts = Vector{Point3f0}()
    colors = Vector{RGB{Float32}}()
    for p in points
        push!(verts, Point3f0(p.x, p.y, p.z))
        push!(colors, RGB(p.𝙄))
    end
    return PointCloud(verts, colors)
end

function extract_𝓕₂_pointcloud(tracker::LSDSLAM.SE3Tracker; level::Integer = 1)
    pointcloud = tracker._𝓕₂_pointcloud
    points = @view pointcloud.p[level][1:(pointcloud.∑p[level])]
    verts = Vector{Point3f0}()
    colors = Vector{RGB{Float32}}()
    for p in points
        push!(verts, Point3f0(p.x, p.y, p.z))
        push!(colors, RGB(p.𝙄))
    end
    return PointCloud(verts, colors)
end

𝓕₁_pointcloud = extract_𝓕₁_pointcloud(tracker, level=3)
𝓕₂_pointcloud = extract_𝓕₁_pointcloud(tracker, level=3)


verts = Vector{Point3f0}

baseframe = LSDSLAM.read!(factory, 1)
LSDSLAM.loaddepth!(baseframe)
relframe = LSDSLAM.read!(factory, 2)
tracker = LSDSLAM.SE3Tracker(640,480)
LSDSLAM._make_𝓕₁_pointcloud!(tracker, baseframe)
LSDSLAM._transform_𝓕₁_to_𝓕₂!(tracker, relframe, ξ₀, 5)

vis = Visualizer()
open(vis)
setobject!(vis["F1"], 𝓕₁_pointcloud)
setobject!(vis["F2"], 𝓕₂_pointcloud)

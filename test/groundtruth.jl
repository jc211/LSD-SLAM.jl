struct TUMGroundTruth
    _data::DataFrame
end

function TUMGroundTruth(directory::String = DATASET_DIR)
    groundtruth = CSV.File(directory*"//groundtruth.txt", header=["timestamp", "tx", "ty", "tz", "qx", "qy", "qz", "qw"], skipto=4, delim=" ") |> DataFrame
    TUMGroundTruth(groundtruth)
end

function (groundtruth::TUMGroundTruth)(frame::LSDSLAM.AbstractFrame)
    index = searchsortedfirst(groundtruth._data.timestamp, frame._header.imagetimestamp)
    tx = groundtruth._data.tx[index]
    ty = groundtruth._data.ty[index]
    tz = groundtruth._data.tz[index]
    qx = groundtruth._data.qx[index]
    qy = groundtruth._data.qy[index]
    qz = groundtruth._data.qz[index]
    qw = groundtruth._data.qw[index]
    translation = [tx, ty, tz]
    quat = Quat(qx, qy, qz, qw)
    return AffineMap(quat, translation)
end

function (groundtruth::TUMGroundTruth)(baseframe::LSDSLAM.AbstractFrame, relframe::LSDSLAM.AbstractFrame)
    T1 = groundtruth(baseframe)
    T2 = groundtruth(relframe)
    return inv(T1)∘(T2)
end

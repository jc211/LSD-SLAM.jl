abstract type FrameFactory end

mutable struct TUMFrameFactory <: FrameFactory
    directory::String
    _counter::Int32
    _df::DataFrame
    _width::Int64
    _height::Int64
    _cameraintrinsics::CameraIntrinsics
    _undistorter::AbstractUndistorter

    function TUMFrameFactory(
        directory::String,
        width::Integer,
        height::Integer,
        cameraintrinsics::AbstractMatrix{<:Real},
        distcoeffs::Union{AbstractVector{<:Real},Nothing}
        )

        x = new()
        x.directory = directory

        df = DataFrame(color_time=Float64[], color_file=String[], depth_time=Float64[], depth_file=String[])
        dcolor = CSV.File(directory*"//rgb.txt", header=["timestamp", "url"], skipto=4, delim=" ") |> DataFrame
        ddepth = CSV.File(directory*"//depth.txt", header=["timestamp", "url"], skipto=4, delim=" ") |> DataFrame

        dt_i = 0

        for (i, row) in enumerate(eachrow(dcolor))
            t = dcolor.timestamp[i]
            dt_i = searchsortedfirst(ddepth.timestamp[1:end], t) # search for first timestamp that is greater or equal to t
            if dt_i < size(ddepth.timestamp)[1]
                dt = ddepth.timestamp[dt_i]
            else
                dt = 0
            end

            # check the timestamp directly previous to this one in case it is closer
            if dt_i != 1
                dt2_i = dt_i - 1
                dt2 = ddepth.timestamp[dt2_i]
                if abs(dt - t) > abs(dt2 - t) # replace with closer timestamp
                    dt_i = dt2_i
                    dt = dt2
                end
            end
            color_path =  "$directory/$(dcolor.url[i])"
            depth_path =  "$directory/$(ddepth.url[dt_i])"
            push!(df, [dcolor.timestamp[i], color_path, ddepth.timestamp[dt_i], depth_path])

        end

        x._undistorter = SimplePinholeUndistorter(cameraintrinsics,distcoeffs)
        x._width = width
        x._height = height
        x._cameraintrinsics = CameraIntrinsics(x._undistorter._cameraintrinsics)
        x._df = df
        x._counter = 1
        x._undistorter
        return x
    end
end


function read!(source::TUMFrameFactory, ind = Nothing)
    if ind == Nothing
        ind = source._counter
        source._counter += 1
    end
    dataset = source._df
    imagepath = dataset[ind, :color_file]
    depthpath = dataset[ind, :depth_file]
    imagetimestamp = dataset[ind, :color_time] - dataset[1, :color_time]
    depthtimestamp = dataset[ind, :depth_time] - dataset[1, :depth_time]

    header = TUMFrameHeader(imagetimestamp, imagepath, depthtimestamp, depthpath)
    return TUMFrame(
        id=ind,
        header=header,
        width=source._width,
        height=source._height,
        cameraintrinsics= source._cameraintrinsics,
        undistorter=source._undistorter )
end

abstract type DataSource end

mutable struct TUMFileSource <: DataSource
    directory::String
    _counter::Int32
    _df::DataFrame
    
    function TUMFileSource(directory::String)
        x = new()
        x.directory = directory
        
        df = DataFrame(color_time=Float64[], color_file=String[], depth_time=Float64[], depth_file=String[])
        dcolor = CSV.File(directory*"//rgb.txt", header=["timestamp", "url"], skipto=4, delim=" ") |> DataFrame
        ddepth = CSV.File(directory*"//depth.txt", header=["timestamp", "url"], skipto=4, delim=" ") |> DataFrame
        
        dt_i = 0
        
        for (i, row) in enumerate(eachrow(dcolor))
            t = dcolor.timestamp[i]
            dt_i = searchsortedfirst(ddepth.timestamp[1:end], t) # search for first timestamp that is greater or equal to t
            dt = ddepth.timestamp[dt_i]

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
        x._df = df
        x._counter = 1
        return x
    end
end


function read!(source::TUMFileSource, ind = Nothing)
    if ind == Nothing
        ind = source._counter
        source._counter += 1
    end
    
    dataset = source._df
    color_fname = dataset[ind, :color_file]
    depth_fname = dataset[ind, :depth_file]
    color_img = load("$color_fname")
    depth_img = Gray.(load("$depth_fname"))
    depth_img = rawview(real.(depth_img))/5000
    timestamp = dataset[ind, :color_time] - dataset[1, :color_time]
    depth_timestamp = dataset[ind, :depth_time] - dataset[1, :depth_time]
    
    return timestamp, color_img, depth_timestamp, depth_img
end

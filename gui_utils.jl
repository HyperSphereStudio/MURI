function urlify_settings(base_url, d)
    buf = IOBuffer()
    print(buf, base_url)
    foreach(p -> print(buf, "&$(p[1])=$(p[2])"), d)
    return String(take!(buf))
end

function download_map(center, area; width=600, height=600, zoom=10.5, pitch=0, style="osm-carto", filename="image")
    api_key = "e4e60768d8f84606a14ddc290716a6b8"

    settings = Set(["width"=>width, "height"=>height, "zoom"=>zoom, "pitch"=>pitch, "area"=> "rect:$(join(area, ","))", "apiKey"=>api_key])

    add_map_marker(lon, lat) = push!(settings, "marker"=>"lonlat:$lon,$lat;color:%23ff0000;size:small")
    add_map_marker(center...)

    filename = "$filename-$(hash(settings)).png"
    if !isfile(filename)
        Downloads.download(urlify_settings("https://maps.geoapify.com/v1/staticmap?style=$style", settings), filename)
    end
    return load(filename)
end
using JuliaSAILGUI
using Dates, Downloads
using JuliaSAILGUI: DataFrames, Observables, CSV, GLMakie, GeometryBasics, Gtk4, FileIO

include("gui_utils.jl")

const TotalPacketLength = 27
const ThermResistence = 200
const ThermCoefficients = [1.07245334681955e-07, 1.61138531293258e-06, 0.000247173445828048, 0.00277515898838713]
const ColorMap = [:blue, :red, :green, :cyan, :magenta, :yellow]
const EarthRadius = 6378100
const Location = (-81.042343, 29.198796)
const LocationWidth = .1

const LocationArea = (Location[1] - LocationWidth, Location[2] - LocationWidth, Location[1] + LocationWidth, Location[2] + LocationWidth) 

RunningTime = now()
runningtime() = Dates.value(now() - RunningTime) * 1E-3

mutable struct Information
    name::String
    plot
    plot_pos
    displaynum

    displayf32num(n) = round(n, digits=2)
    Information(name, plot_pos::Tuple, displaynum = displayf32num) = new(name, nothing, plot_pos, displaynum)
    Information(name, displaynum = displayf32num) = new(name, nothing, nothing, displaynum)
end

mutable struct Payload
    name
    color
    observable          #Holds the altitude just for ease of use
    packet

    Payload(name, color) = new(name, color, Observable(0.0), zeros(Float64, length(information)))
    Observables.notify(p::Payload) = notify(p.observable)
    Observables.on(f, p::Payload) = on(f, p.observable)
end

const payloads = GtkJuliaList(Payload[])     
const ids2payloads = Dict{Int, Payload}() 
const information = [Information("Time", t -> Dates.format(Time(0) + Second(t), "HH::MM::SS")),  Information("Longitude"), Information("Latitude"), 
                     Information("Altitude [km]"),                Information("Temp [∘C]", (1, 1)),         Information("Pressure [atm]", (1, 2)),      
                     Information("Ascent V [m/s]", (2, 1)),     Information("North V [m/s]", (2, 2)),   Information("Horiz V [m/s]", (3, 1)),
                     Information("EastVel [m/s]", (3, 2)),        Information("RSSI"),                      Information("Sat", Int)]

function init_plot(gui)
    mesh!(gui[:SAx3], Rect(Vec(LocationArea[1:2]..., 0), Vec(2LocationWidth, 2LocationWidth, 0)), color = download_map(Location, LocationArea; pitch=45, filename="imgcache/streetimage45"))
    mesh!(gui[:SAx2], Rect(Vec(LocationArea[1:2]...), Vec(2LocationWidth, 2LocationWidth)), color = reverse!(download_map(Location, LocationArea; pitch=0, filename="imgcache/streetimage"), dims=1))
end

function create_plot(spatialplots, infoplots, gui)
    set_theme!(theme_hypersphere())
    infoFig = Figure()
    spatialFig = Figure()

    for info in information
        info.plot_pos === nothing || (info.plot = Axis(infoFig[info.plot_pos...], xlabel=info.name, ylabel="Altitude [km]"))
    end

    ax3 = Axis3(spatialFig[1, 1], xlabel="", ylabel="", zlabel="")
    ax2 = Axis(spatialFig[1, 2], xlabel="", ylabel="")
    gui[:SAx3] = ax3
    gui[:SAx2] = ax2
    init_plot(gui)

    limits!(ax3, Rect(Vec(LocationArea[1:2]..., 0), Vec(2LocationWidth, 2LocationWidth, 100)))
    limits!(ax2, Rect(Vec(LocationArea[1:2]...), Vec(2LocationWidth, 2LocationWidth)))
    
    display(GtkGLScreen(spatialplots), infoFig)
    display(GtkGLScreen(infoplots), spatialFig)

    gui[:connect_payload_to_plots] = function connect_payload_to_plots(p::Payload)
        for idx in eachindex(information)
            info = information[idx]
            if info.plot_pos !== nothing
                plot_data = Observable(Point2f[])
                lines!(info.plot, plot_data, color=p.color, label=p.name)
                on(p) do a
                    push!(plot_data[], Point2f((p.packet[idx], a)))
                    notify(plot_data)
                    autolimits!(info.plot)
                end
            end
        end
        
        pos3 = Observable(Point3f(0))
        pos2 = Observable(Point2f(0))
        apos = Observable(Point3f[])
        scatter!(ax3, pos3, color=p.color, markersize=25, label=p.name)
        scatter!(ax2, pos2, color=p.color, markersize=25, label=p.name)
        lines!(ax3, apos; color=p.color)                                                               #Trajectory
        on(p) do a
            pos3[] = Point3f(p.packet[2], p.packet[3], p.packet[4])
            pos2[] = Point2f(p.packet[2], p.packet[3])
            autolimits!(ax3)
            push!(apos[], pos3[])
            notify(apos)
        end
    end
end

function create_column_view()
    payload_cv = GtkColumnView(GtkSelectionModel(GtkSingleSelection(Gtk4.GListModel(payloads))), reorderable=true, hexpand=true, vexpand=true)
    cv = GtkJuliaColumnViewColumn(payloads, "ID", () -> GtkLabel(""), (l, p) -> Gtk4.markup(l, "<span foreground=\"$(p.color)\">$(p.name)</span>"), expand=true, resizable=true)     #Set Text Color
    append!(payload_cv, cv)
    
    for n in eachindex(information)
        info = information[n]
        cv = GtkJuliaColumnViewColumn(payloads, info.name, () -> GtkLabel(""), (l, p) -> on(t -> l[] = info.displaynum(p.packet[n]), p), expand=true, resizable=true)
        append!(payload_cv, cv)
    end

    sw = GtkScrolledWindow(hscrollbar_policy=Gtk4.PolicyType_AUTOMATIC, vscrollbar_policy=Gtk4.PolicyType_AUTOMATIC)
    sw[] = payload_cv
    return sw
end

function create_control_panel(gui, replaySpeed, streamMode)
    control_box = GtkGrid(column_homogeneous=true)
    
    streamButton = GtkCheckButton("Stream")
  #  stopButton = buttonwithimage("Stop", GtkImage(icon_name="media-playback-stop"))
    replayButton = buttonwithimage("Play", GtkImage(icon_name="media-playback-start"))
    replaySpeedSlider = GtkScale(:h, .25:.5:100)
    Observables.ObservablePair(replaySpeedSlider, replaySpeed)
    Observables.ObservablePair(streamButton, streamMode)

    gui[:Stream] = streamButton
    gui[:Replay] = replayButton
   # gui[:Stop] = stopButton

    control_box[1, 1] = replayButton
   # control_box[2, 1] = stopButton
    control_box[1, 2] = streamButton
    control_box[2:3, 2] = makewidgetwithtitle(replaySpeedSlider, lift(s -> "Play Speed (x$(round(s, digits=2)))", replaySpeed))

    return control_box
end

function create_gui(replaySpeed, streamMode)
    gui = Dict{Symbol, Any}()

    spatialplots = GtkGLArea(hexpand=true, vexpand=true)
    infoplots = GtkGLArea(hexpand=true, vexpand=true)

    win = GtkWindow("GPS Sonde GUI")
    gui[:Window] = win

    grid = GtkGrid(column_homogeneous=true)
    win[] = grid

    grid[1, 1:3] = infoplots
    grid[2, 1] = create_column_view()
    grid[2, 2] = create_control_panel(gui, replaySpeed, streamMode)
    grid[2, 3] = spatialplots

    create_plot(infoplots, spatialplots, gui)

    return gui
end

function reset(gui)
    empty!(payloads)
    empty!(ids2payloads)
    Makie.empty!(gui[:SAx3])
    foreach(i -> i.plot === nothing || empty!(i.plot), information)
    init_plot(gui)
end

function gui_main()
    file = nothing
    replaySpeed = Observable(1.0)
    streamMode = Observable(false)
    gui = create_gui(replaySpeed, streamMode)

    signal_connect(gui[:Window], :close_request) do _
        file === nothing || close(file)
        exit(0)
    end

    on(gui[:Replay]) do r
        open_dialog("Open Replay File", nothing, ["*.bin"]; start_folder="data") do f
            @async begin 
                reset(gui)
                open(f, "r", lock=false) do fp
                    data = zeros(UInt8, TotalPacketLength)
					offset = 1
                    lastTime = nothing
                    try
                        @label top
						#Read from data file and try to read line from it or wait
						offset += readbytes!(fp, @view(data[offset:end]), TotalPacketLength - offset + 1)
                        while offset == TotalPacketLength + 1
                            p = ondata(data, gui)
                            if lastTime === nothing
                                lastTime = p.packet[1]
                            else
                                newTime = max(p.packet[1], lastTime)
                                streamMode[] || sleep((newTime - lastTime) / replaySpeed[])
                                lastTime = newTime
                            end     
							offset = 1
                        end
                        streamMode[] && (sleep(.1); @goto top)
                    catch e
                        showerror(stdout, e)
                        rethrow(e)
                    end   
                end
            end
        end
    end

    maximize(gui[:Window])
    display_gui(gui[:Window])
end

readl(io::IO, ::Type{T}) where T <: Number = ltoh(read(io, T))
readl(io::IO, T::Type) = read(io, T)
readl(io::IO, Types::Type...) = [readl(io, T) for T in Types]
readl(io::IO, T::Type, count::Integer) = [readl(io, T) for i in 1:count]
polyval(coeffs, x) = sum(i -> coeffs[i] * x ^ (i - 1), length(coeffs):-1:1)
Base.getindex(g::GtkCheckButton, ::Type{Bool}) = g.active
Base.setindex!(g::GtkCheckButton, v) = g.active = v
JuliaSAILGUI.on_update_signal_name(::GtkCheckButton) = "toggled"

function ondata(data, gui)
    io = IOBuffer(data)
    header = readl(io, UInt16)
    id = (header & 0xFFF0 - 43664) >> 4 + 1                              #Clear last 4 bytes. (Number of Sats)
    sat = header & 0x000F

    alt, lat, lon = readl(io, Float32, 3)
    temp, press = Float32.(readl(io, UInt16, 2))
    packNum, utcSec, utcMin, utcHour, numSatsGS = readl(io, UInt8, 5)

    rssi = Float32(readl(io, Int32))                            

    alt /= 1000                                                          #km

    temp /= 65535                                                        #Normalized Voltage
    println("n:", temp)
    temp = temp * ThermResistence / (1 - temp)                           #Convert to Thermistor Resistance
    println("r:", temp)
    temp = 1 / polyval(ThermCoefficients, log(temp)) - 273.15            #Temperature

    press /= 0.537392*101625

    time = utcSec + 60*utcMin + 3600*utcHour
    shouldObserve = true

    if !haskey(ids2payloads, id)
        p = Payload(string(id), ColorMap[id])
        ids2payloads[id] = p
        push!(payloads, p)
        gui[:connect_payload_to_plots](p)
        nVel, eVel, hVel, aVel = 0, 0, 0, 0  
        shouldObserve = false
    else
        p = ids2payloads[id]
        ΔTime, ΔLon, ΔLat, ΔAlt = [time, lon, lat, alt] .- p.packet[1:4]
        nVel = deg2rad(ΔLat) * (alt + EarthRadius) / ΔTime
        eVel = deg2rad(ΔLon) * (alt + EarthRadius) / ΔTime
        hVel = √(nVel^2 + eVel^2)
        aVel = ΔAlt / ΔTime

        #Dont trigger plot for a couple cycles to allow for better calc
        #Filter out small delta T's (could have rapid sent packet)
        shouldObserve = packNum >= 4 && ΔTime >= 1
    end

    p.packet = Float32[time, lon, lat, alt, temp, press, aVel, nVel, hVel, eVel, rssi, sat]
    shouldObserve && (p.observable[] = alt)

    return p
end

gui_main()
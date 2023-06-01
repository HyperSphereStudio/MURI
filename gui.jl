using JuliaSAILGUI
using Dates, Downloads
using JuliaSAILGUI: DataFrames, Observables, LibSerialPort, CSV, GLMakie, GeometryBasics, Gtk4, FileIO

include("gui_utils.jl")

const SerialBaudRate = 230400
const TotalPacketLength = 27
const ThermResistence = 200
const ThermCoefficients = [1.07245334681955e-07, 1.61138531293258e-06, 0.000247173445828048, 0.00277515898838713]
const ColorMap = [:blue, :red, :green, :cyan, :magenta, :yellow]
const EarthRadius = 6378100
const MaxAltitude = 100

RunningTime = now()
runningtime() = Dates.value(now() - RunningTime) * 1E-3
resettime() = global RunningTime = now()

@enum PlotType begin
    NoPlot
    LinePlot
end

struct FixedLengthReader
    length::Integer
end

function Base.take!(r::FixedLengthReader, data, len)
    if length(data) >= r.length
        len[] = r.length
        return data[1:r.length]
    end
    return nothing
end

mutable struct Information
    name::String
    plotType::PlotType
    plot
    plot_pos

    Information(name, plotType, plos_pos) = new(name, plotType, nothing, plos_pos)
end

mutable struct Payload
    name
    color
    packet_time     #The Actual Observable
    packet
    prevpacket

    Payload(name, color) = new(name, color, Observable(0.0), zeros(Float64, length(information)), zeros(Float64, length(information)))
    Observables.notify(p::Payload) = notify(p.packet_time)
    Observables.on(f, p::Payload) = on(f, p.packet_time)
end

information = [Information("Time", NoPlot, -1),                         Information("Longitude", LinePlot, (1, 3)),                 Information("Latitude", LinePlot, (1, 2)), 
               Information("Altitude [km]", LinePlot, (1, 1)),          Information("Temp [∘C]", LinePlot, (2, 2)),                 Information("Pressure [atm]", LinePlot, (2, 3)),      
               Information("Ascent Vel [m/s]", LinePlot, (2, 1)),       Information("North Vel [m/s]", LinePlot, (3, 2)),           Information("Horiz Vel [m/s]", LinePlot, (3, 3)),
               Information("EastVel", LinePlot, (3, 1)),                Information("RSSI", NoPlot, -1),                            Information("Sat", NoPlot, -1)]

payloads = GtkJuliaList(Payload[])     
ids2payloads = Dict{Int, Payload}()  
center = (-81.022833, 29.210815)
w = .025
area = (center[1] - w, center[2] - w, center[1] + w, center[2] + w)
zoom = 10.5

function init_plot(gui)
    mesh!(gui[:Ax3], Rect(Vec(center[1] - w, center[2] - w, 0), Vec(2w, 2w, 0)), color=load("test.png"))
end

function create_plot(plots, plot3d, gui)
    set_theme!(theme_hypersphere())
    fig = Figure()
    fig3D = Figure()

    for info in information
        if info.plotType == LinePlot
            info.plot = Axis(fig[info.plot_pos...], xlabel = "Time [s]", ylabel = info.name)
        end
    end

    ax3 = Axis3(fig3D[1, 1], xlabel="", ylabel="", zlabel="")
    gui[:Ax3] = ax3
    init_plot(gui)
    limits!(ax3, Rect(Vec(center[1] - w, center[2] - w, 0), Vec(2w, 2w, MaxAltitude)))
    
    screen = GtkGLScreen(plots)
    screen3d = GtkGLScreen(plot3d)
    display(screen, fig)
    display(screen3d, fig3D)

    gui[:connect_payload_to_plots] = function connect_payload_to_plots(p::Payload)
        for idx in eachindex(information)
            info = information[idx]
            if info.plotType == LinePlot
                plot_data = Observable(Point2f[])
                lines!(info.plot, plot_data, color=p.color, label=p.name)
                on(p) do t
                    push!(plot_data[], Point2f((t, p.packet[idx])))
                    autolimits!(info.plot)
                    notify(plot_data)
                end
            end
        end
        
        pos = Observable(Point3f(0))
        apos = Observable(Point3f[])
        scatter!(ax3, pos, color=p.color, markersize=25, label=p.name)                        #Sphere    
        lines!(ax3, apos; color=p.color)                                                      #Trajectory
        on(p) do t
            pos[] = Point3f(p.packet[2], p.packet[3], p.packet[4])
            push!(apos[], pos[])
            notify(apos)
        end
    end
end

function create_column_view()
    payload_cv = GtkColumnView(GtkSelectionModel(GtkSingleSelection(Gtk4.GListModel(payloads))), reorderable=true, hexpand=true, vexpand=true)

    cv = GtkJuliaColumnViewColumn(payloads, "ID", () -> GtkLabel(""), (l, p) -> Gtk4.markup(l, "<span foreground=\"$(p.color)\">$(p.name)</span>"))     #Set Text Color
    Gtk4.setproperties!(cv, expand=true, resizable=true)
    append!(payload_cv, cv)
    
    for n in eachindex(information)
        cv = GtkJuliaColumnViewColumn(payloads, information[n].name, () -> GtkLabel(""), (l, p) -> on(t -> l[] = round(p.packet[n], digits=2), p))
        Gtk4.setproperties!(cv, expand=true, resizable=true)
        append!(payload_cv, cv)
    end

    sw = GtkScrolledWindow(hscrollbar_policy=Gtk4.PolicyType_AUTOMATIC, vscrollbar_policy=Gtk4.PolicyType_AUTOMATIC)
    sw[] = payload_cv
    return sw
end

function create_control_panel(gui)
    control_box = GtkBox(:h)
    
    resetButton = buttonwithimage("Reset", GtkImage(icon_name = "system-reboot"))
    replayButton = buttonwithimage("Replay", GtkImage(icon_name = "media-record"))
    serialSelect = GtkComboBoxText(halign=Gtk4.Align_CENTER)

    gui[:SerialPorts] = serialSelect
    gui[:Replay] = replayButton
    gui[:Reset] = resetButton

    append!(control_box, makewidgetwithtitle(serialSelect, "Serial Port"), replayButton, resetButton)
    return control_box
end

function create_gui()
    gui = Dict{Symbol, Any}()

    plot3d = GtkGLArea(hexpand=true, vexpand=true)
    plots = GtkGLArea(hexpand=true, vexpand=true)

    win = GtkWindow("MURI")
    gui[:Window] = win
    maximize(win)

    grid = GtkGrid(column_homogeneous=true)
    win[] = grid

    grid[1, 1:3] = plots
    grid[2, 1] = create_column_view()
    grid[2, 2] = create_control_panel(gui)
    grid[2, 3] = plot3d

    create_plot(plots, plot3d, gui)
    display_gui(win; blocking=false)

    return gui
end

polyval(coeffs, x) = sum(i -> coeffs[i] * x ^ (i - 1), length(coeffs):-1:1)

readn(io, t::Type) = ntoh(read(io, t))
readn(io, t::Type, count::Integer) = ntoh.(reinterpret(t, read(io, sizeof(t) * count)))

function reset(gui)
    empty!(payloads)
    empty!(ids2payloads)
    empty!(gui[:Ax3])
    foreach(i -> i.plotType == LinePlot && empty!(i.plot), information)
    init_plot(gui)
end

function gui_main()
    file = nothing
    gui = create_gui()
    serial = MicroControllerPort(:Serial, SerialBaudRate, FixedLengthReader(TotalPacketLength); nstopbits=1)

    signal_connect(gui[:Window], :close_request) do _
        file === nothing || close(file)
        exit(0)
    end

    on(serial) do c
        c || (gui[:SerialPorts].active = -1)
        println("Device Connected: $c")
    end

    on(gui[:Replay]) do r
        open_dialog("Open Replay File", nothing, ["*.bin"]; start_folder="data") do f
            @async begin 
                reset(gui)
                open(f, "r") do fp
                    data = zeros(UInt8, TotalPacketLength)
                    while !eof(f)
                        readbytes!(s, data, TotalPacketLength)
                        ondata(data)
                    end
                end
            end
        end
    end

    on(r -> reset(gui), gui[:Reset])

    on(PortsObservable; update=true) do p
        isopen(serial) && return
        empty!(gui[:SerialPorts])
        append!(gui[:SerialPorts], p)
    end

    function create_payload(name, color)
        p = Payload(name, color)
        gui[:connect_payload_to_plots](p)
        return p
    end

    while true
        try
            isopen(serial) && readport(serial) do data
                file === nothing || (file = open(Dates.format(now(), "data/data_mm-dd-yyyy_HH-MM-SS.bin"), "w"))
                write(file, data)
                ondata(data)
            end
        catch e
           # showerror(stdout, e)
            close(serial)
            rethrow(e)
        end
        sleep(1E-1)
    end
end

function ondata(data)
    io = IOBuffer(data)

    header = readn(io, UInt16)
    id = (header & 0xFFF0 - 43664) << 4                                 #Clear last 4 bytes. (Number of Sats)
    sat = header & 0x000F

    alt, lat, lon = readn(io, Float32, 3)
    temp, press = Float32.(readn(io, UInt16, 2))
    packNum, utcSec, utcMin, utcHour = readn(io, UInt8, 4)
    rssi = readn(io, Int32)
    tmpheader & 0x000F

    alt /= 1000                                                          #km

    temp = temp/65535                                                    #Normalized Voltage
    temp = temp * ThermResistence / (1 - temp)                           #Convert to Thermistor Resistance
    temp = 1 / polyval(ThermCoefficients, log(temp)) - 273.15            #Temperature

    press /= 0.537392*101625

    time = utcSec + 60 * utcMin + 3600 * utcHour

    if !haskey(ids2payloads, id)
        p = Payload(string(id), ColorMap[id])
        ids2payloads[id] = p
        push!(payloads, p)
        gui[:connect_payload_to_plots](p)
        nVel, eVel, hVel, aVel = 0, 0, 0, 0  
    else
        p = ids2payloads[id]
        p.prevpacket = p.packet
        ΔTime, ΔLon, ΔLat, ΔAlt = [time, lon, lat, alt] .- p.packet[1:4]
        nVel = deg2rad(ΔLat) * (alt + EarthRadius) / ΔTime
        eVel = deg2rad(ΔLon) * (alt + EarthRadius) / ΔTime
        hVel = √(nVel^2 + eVel^2)
        aVel = ΔAlt / ΔTime
    end

    p.packet = Float32[time, lon, lat, alt, temp, press, aVel, nVel, hVel, eVel, rssi, sat]
    p.packet_time[] = time
end

gui_main()
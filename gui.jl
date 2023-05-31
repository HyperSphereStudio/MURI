using JuliaSAILGUI
using Dates, Downloads
using JuliaSAILGUI: DataFrames, Observables, LibSerialPort, CSV, GLMakie, GeometryBasics, Gtk4, FileIO

include("gui_utils.jl")

function theme_hypersphere()
    Theme(
        backgroundcolor = :grey10,
        textcolor = :white,
        linecolor = :gray60,

        fonts = (
            regular = "Noto Sans",
            bold = "Noto Sans Bold",
            italic = "Noto Sans Italic",
            bold_italic = "Noto Sans Bold Italic",
        ),

        Axis = (
            backgroundcolor = :grey,
            xgridcolor = (:white, 0.2),
            ygridcolor = (:white, 0.2),
            leftspinevisible = false,
            rightspinevisible = false,
            bottomspinevisible = false,
            topspinevisible = false,
            xminorticksvisible = false,
            yminorticksvisible = false,
            xticksvisible = false,
            yticksvisible = false,
            xlabelpadding = 3,
            ylabelpadding = 3,
            xgridstyle = :dash,
            ygridstyle = :dash,
        ),

        Legend = (
            framevisible = false,
            padding = (0, 0, 0, 0),
        ),

        Axis3 = (
            xgridcolor = (:white, 0.35),
            ygridcolor = (:white, 0.35),
            zgridcolor = (:white, 0.35),
            xspinesvisible = false,
            yspinesvisible = false,
            zspinesvisible = false,
            xticksvisible = false,
            yticksvisible = false,
            zticksvisible = false,
        ),

        Colorbar = (
            ticksvisible = false,
            spinewidth = 0,
            ticklabelpad = 5,
        )
    )
end

@enum PlotType begin
    NoPlot
    LinePlot
end

struct Information
    name::String
    plotType::PlotType
end

mutable struct Payload
    name
    color
    packet_time     #The Actual Observable
    packet

    Payload(name, color) = new(name, color, Observable(0), Vector{Float64}())
    Observables.notify(p::Payload) = notify(p.packet_time)
    Observables.on(f, p::Payload) = on(f, p.packet_time)
end

information = [Information("Time", NoPlot),             Information("Longitude", LinePlot),     Information("Latitude", LinePlot), 
               Information("Altitude [km]", LinePlot),  Information("Temp [∘C]", LinePlot),     Information("North Vel [m/s]", LinePlot), 
               Information("South Vel [m/s]", LinePlot)]

payloads = GtkJuliaStore(Payload[])         
center = (-81.022833, 29.210815)
w = .025
area = (center[1] - w, center[2] - w, center[1] + w, center[2] + w)
zoom = 10.5

function create_plot(plot, gui)
    set_theme!(theme_hypersphere())
    fig = Figure()

    line_plots = []
    c = 1
    r = 1

    for idx in eachindex(information)
        info = information[idx]
        if info.plotType == LinePlot
            c += 1
            c == (cols + 1) && (c = 1; r += 1)
            push!(line_plots, Axis(fig[r, c], xlabel = "Time [s]", ylabel = information[n]))
        end
    end

    #3D Geospatial Plot
    ax = Axis3(fig[1, 1])
    zlims!(ax, (0, 2w))
    mesh!(ax, Rect(Vec(center[1] - w, center[2] - w, 0), Vec(2w, 2w, 0)), color=load("test.png"))
    screen = GtkGLScreen(plot)
    display(screen, fig)

    gui[:connect_payload_to_plots] = function connect_payload_to_plots(p::Payload)
        for idx in eachindex(line_plots)
            lp = line_plots[idx]
            accumulated_data = Observable(Point2f[])
            lines!(accumulated_data, color=p.color, label=p.name)
            on(p; update=true) do t
                push!(accumulated_data, Point2((t, p.packet[idx])))
                autolimits!(lp)
            end
        end
    end
end

function create_column_view()
    payload_cv = GtkColumnView(GtkSelectionModel(GtkSingleSelection(Gtk4.GListModel(payloads))))
    payload_cv.hexpand = true

    append!(payload_cv, GtkJuliaColumnViewColumn(payloads, "Payload", () -> GtkLabel(""), (l, p) -> l[] = p.name))

    for n in eachindex(information)
        append!(payload_cv, GtkJuliaColumnViewColumn(payloads, information[n], () -> GtkLabel(""), (l, p) -> on(t -> l[] = p.packet[n], p; update=true)))
    end

    return payload_cv
end

function create_gui()
    gui = Dict{Symbol, Any}()
   
    plot = GtkGLArea(hexpand=true, vexpand=true)

    win = GtkWindow("MURI")
    back_box = GtkBox(:h)
    win[] = back_box

    append!(back_box, plot, create_column_view())

    create_plot(plot, gui)

    display_gui(win)

    return gui
end

function gui_main()
    #Put Runtime Vars Here

    gui = create_gui()

    #Put Callbacks here

    #Put Program Functions Here

    #Main Loop Here
end

gui_main()
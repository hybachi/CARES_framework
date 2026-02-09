from nicegui import ui
import plotly.graph_objects as go
from cares_dashboard.layout import theme_wrapper
import cares_dashboard.state as state

@ui.page('/status')
@theme_wrapper
def status_page():

    # ROBOT CARDS
    with ui.column().classes('w-full flex-grow p-6 overflow-hidden'):
        ui.label('FLEET STATUS').classes('text-xl font-bold text-emerald-500 tracking-widest mr-auto')

        chart_refs = {} 

        with ui.row().classes('w-full gap-6 flex-wrap content-start overflow-y-auto h-full'):

            # Robot details modal
            def open_details(rid):
                r = state.get_robot(rid)
                with ui.dialog() as dialog, ui.card().classes('bg-slate-800 border border-slate-600 min-w-[400px]'):
                    with ui.row().classes('w-full justify-between items-center'):
                        ui.label(f"DETAILS: {rid}").classes('text-xl font-bold text-emerald-400')
                        ui.icon('close').classes('cursor-pointer').on('click', dialog.close)
                    
                    ui.separator().classes('bg-slate-600 my-2')
                    ui.label(f"Type: {r['type']}")
                    ui.label(f"Battery: {int(r['battery'])}%")
                    ui.label("Tasks Executed: 0 (Simulated)")
                    ui.label("Sensors: Lidar, IMU, RGB Camera")
                dialog.open()

            # Render robot cards
            for rid, r in sorted(state.robots.items()):
                with ui.card().classes('w-72 bg-slate-800 border border-slate-700 p-3 hover:border-emerald-500 transition-colors cursor-pointer') \
                            .on('click', lambda _, i=rid: open_details(i)):
                    
                    with ui.row().classes('w-full justify-between items-center'):
                        ui.label(rid).classes('font-bold text-lg')
                        ui.icon('battery_full', color='green').tooltip(f"{int(r['battery'])}%")
                    
                    status_label = ui.label(r['status']).classes('text-xs font-mono text-slate-400 mb-2')
                    
                    fig = go.Figure(go.Scatterpolar(
                        r=[0, 0, 0, 0],
                        theta=['MOB', 'SEN', 'NET', 'MOB'],
                        fill='toself', line_color='#10b981', hoverinfo='skip'
                    ))
                    fig.update_layout(
                        paper_bgcolor='rgba(0,0,0,0)', plot_bgcolor='rgba(0,0,0,0)',
                        margin=dict(l=25, r=25, t=5, b=5), height=150,
                        polar=dict(radialaxis=dict(visible=False, range=[0, 1]), 
                                angularaxis=dict(visible=True, color='#64748b')),
                        showlegend=False
                    )
                    
                    plot = ui.plotly(fig).classes('w-full h-32')
                    chart_refs[rid] = (plot, status_label)

                    ui.label('Click for details...').classes('text-[10px] text-slate-600 w-full text-center mt-2')

        def update_charts():
            for rid, (plot, lbl) in chart_refs.items():
                if rid in state.robots:
                    r = state.robots[rid]

                    lbl.set_text(f"Status: {r['status']}")
                    
                    new_r = [r['caps']['MOBILITY'], r['caps']['SENSING'], r['caps']['NETWORK'], r['caps']['MOBILITY']]
                    plot.figure.data[0].r = new_r
                    plot.update()

        ui.timer(0.5, update_charts)
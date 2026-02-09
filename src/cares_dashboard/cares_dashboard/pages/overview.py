from nicegui import ui
from cares_dashboard.layout import theme_wrapper
import cares_dashboard.state as state

@ui.page('/')
@theme_wrapper
def overview_page():
    # TOP BAR
    with ui.row().classes('w-full h-16 bg-slate-900 border-b border-slate-700 items-center px-4 gap-8'):
        ui.label('SWARM OVERVIEW').classes('text-xl font-bold text-emerald-500 tracking-widest mr-auto')

        def stat(label, key, icon, color):
            with ui.row().classes('items-center gap-2'):
                ui.icon(icon, color=color).classes('text-xl')
                with ui.column().classes('gap-0'):
                    ui.label(label).classes('text-[10px] text-slate-400 uppercase font-bold')
                    ui.label().bind_text_from(state.metrics, key).classes('text-lg font-mono font-bold leading-none')

        stat('Active Units', 'active_units', 'memory', 'blue')
        stat('Victims Found', 'victims_found', 'person_search', 'red')
        stat('Tasks Done', 'tasks_completed', 'done_all', 'green')
        stat('Avg Capability', 'swarm_mean_cap', 'speed', 'purple')

    # CENTER CONTENT
    with ui.row().classes('w-full flex-grow overflow-hidden gap-0'):

        # LEFT PANEL
        with ui.card().classes('flex-grow h-full bg-black rounded-none border-r border-slate-700 p-0 relative'):
            with ui.scene(width='100%', height='100%').classes('bg-black') as scene:
                scene.move_camera(-5, -5, 8, 0, 0, 0)

        # RIGHT PANEL
        with ui.column().classes('w-72 h-full bg-slate-800 p-2 gap-2 overflow-y-auto'):
            ui.label('FLEET HEALTH').classes('text-xs font-bold text-slate-400 mb-2')
            snapshot_container = ui.column().classes('w-full gap-2')

            def update_snapshot():
                snapshot_container.clear()
                with snapshot_container:
                    for rid, r in sorted(state.robots.items()):
                        health = sum(r['caps'].values()) / 3
                        color = 'bg-emerald-500' if health > 0.7 else ('bg-yellow-500' if health > 0.4 else 'bg-red-500')
                        with ui.card().classes('w-full bg-slate-900 border border-slate-700 p-2'):
                            with ui.row().classes('w-full justify-between items-center mb-1'):
                                ui.label(rid).classes('font-bold text-sm')
                                ui.label(f"{int(health*100)}%").classes('text-xs font-mono')
                            ui.element('div').classes(f'h-1.5 rounded-full {color}').style(f'width: {health*100}%')

            ui.timer(1.0, update_snapshot)

    # BOTTOM LOG
    with ui.element('div').classes('w-full h-48 shrink-0'):
        with ui.card().classes('w-full h-full bg-slate-950 border-t border-slate-700 rounded-none p-0'):
            ui.label('MISSION LOG').classes('text-xs font-bold text-slate-500 px-2 py-1 bg-slate-900 w-full border-b border-slate-800')
            
            log_area = ui.scroll_area().classes('w-full h-40 p-1 font-mono text-xs text-slate-300')

            def update_logs():
                log_area.clear()
                with log_area:
                    for log in reversed(state.logs):
                        color = 'text-slate-300'
                        if 'VICTIM' in log: color = 'text-red-400 font-bold'
                        elif 'Mission' in log: color = 'text-emerald-400'
                        ui.label(log).classes(color)
                log_area.scroll_to(percent=1.0)

            ui.timer(0.5, update_logs)
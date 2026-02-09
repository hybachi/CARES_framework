from nicegui import ui
from cares_dashboard.layout import theme_wrapper
import cares_dashboard.state as state
import random

ros_driver = None

@ui.page('/missions')
@theme_wrapper
def missions_page():
    
    with ui.row().classes('w-full h-full gap-0'):

        # LEFT PANEL
        with ui.column().classes('w-1/3 h-full bg-slate-800 border-r border-slate-700 p-6 gap-6'):
            ui.label('MISSIONS').classes('text-xl font-bold text-emerald-400')
            
            with ui.column().classes('w-full gap-2'):
                ui.label('Mission Type').classes('text-xs text-slate-400')
                m_type = ui.select(['SEARCH', 'SCAN', 'RESCUE'], value='SEARCH').classes('w-full bg-slate-900')
                
                ui.label('Priority').classes('text-xs text-slate-400 mt-2')
                m_prio = ui.slider(min=1, max=10, value=5).props('label-always')

                ui.label('Target Coordinates').classes('text-xs text-slate-400 mt-2')
                with ui.row().classes('w-full'):
                    mx = ui.number(label='X', value=0).classes('w-1/2')
                    my = ui.number(label='Y', value=0).classes('w-1/2')

            def submit():
                if ros_driver:
                    ros_driver.dispatch_mission(m_type.value, mx.value, my.value, m_prio.value)
                    ui.notify('Order Transmitted', type='positive')
            
            ui.space()
            ui.button('DISPATCH', on_click=submit).classes('w-full bg-emerald-600 py-4 text-lg font-bold')

        # RIGHT PANEL
        with ui.column().classes('flex-grow h-full bg-slate-900 gap-0'):

            ui.add_css('''
                .ag-theme-balham-dark .ag-root-wrapper,
                .ag-theme-balham-dark .ag-header {
                    background-color: #1e293b; /* slate-800 */
                    border-color: #334155;     /* slate-700 */
                }
                .ag-theme-balham-dark .ag-row {
                    background-color: #0f172a; /* slate-900 */
                    border-color: #1e293b;
                    color: #cbd5e1;            /* slate-300 */
                }
                .ag-theme-balham-dark .ag-header-cell-label {
                    color: #94a3b8;            /* slate-400 */
                }
                .ag-theme-balham-dark .ag-row:hover {
                    background-color: #1e293b;
                }
            ''')
            
            # TASK ALLOCATION
            with ui.column().classes('w-full h-1/2 p-6 pb-0'):
                ui.label('TASK ALLOCATION').classes('text-xl font-bold text-slate-500 mb-2')
                
                with ui.card().classes('w-full h-full bg-slate-800 p-0 border border-slate-700'):
                    grid = ui.aggrid({
                        'columnDefs': [
                            {'headerName': 'Task ID', 'field': 'id', 'sortable': True},
                            {'headerName': 'Type', 'field': 'type'},
                            {'headerName': 'Status', 'field': 'status', 'cellStyle': {'color': '#10b981'}},
                            {'headerName': 'Winner', 'field': 'winner'},
                            {'headerName': 'Cost', 'field': 'cost'},
                        ],
                        'rowData': [],
                        'rowSelection': 'single',
                    }).classes('w-full h-full ag-theme-balham-dark')

            # AUCTION LOG
            with ui.column().classes('w-full h-1/2 p-6 pt-4'):
                ui.label('AUCTION LOG').classes('text-xl font-bold text-slate-500 mb-2')
                with ui.card().classes('w-full h-full bg-black border border-slate-700 p-0'):
                    log_area = ui.scroll_area().classes('w-full h-full p-2 font-mono text-xs text-slate-300')

            last_grid_snapshot = None
            last_log_count = 0

            def update_data():
                nonlocal last_grid_snapshot, last_log_count

                # Update Grid
                current_snapshot = str(state.allocations)
                
                if current_snapshot != last_grid_snapshot:
                    grid.options['rowData'] = state.allocations
                    grid.update()
                    last_grid_snapshot = current_snapshot

                # Update Logs
                if len(state.logs) != last_log_count:
                    log_area.clear()
                    with log_area:
                        for log in state.logs:
                            color = 'text-slate-300'
                            if 'Auction' in log: color = 'text-blue-400'
                            elif 'ASSIGNED' in log: color = 'text-emerald-400'
                            elif 'Mission' in log: color = 'text-purple-400'
                            
                            ui.label(log).classes(f'ml-1 {color}')
                    last_log_count = len(state.logs)

            ui.timer(1.0, update_data)
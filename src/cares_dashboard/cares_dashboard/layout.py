from nicegui import ui

THEME = {
    'bg': '#0f172a',      # Slate 900
    'panel': '#1e293b',   # Slate 800
    'border': '#334155',  # Slate 700
    'primary': '#10b981', # Emerald
    'text': '#f8fafc'     # Slate 50
}

def nav_icon(icon, target, tooltip):
    ui.button(icon=icon, on_click=lambda: ui.navigate.to(target)) \
        .props('flat round color=white size=lg') \
        .tooltip(tooltip)

def theme_wrapper(page_func):
    def wrapper():
        ui.query('.nicegui-content').classes('p-0')
        ui.query('body').style('background-color: #0f172a;')
        
        ui.colors(primary=THEME['primary'], dark=THEME['bg'])
        
        with ui.row().classes('w-full h-screen gap-0 m-0 p-0 no-wrap'):
            
            # SIDEBAR
            with ui.column().classes('w-16 h-full bg-slate-950 border-r border-slate-700 items-center gap-6 m-0'):
                nav_icon('dashboard', '/', 'Overview')
                nav_icon('bar_chart', '/status', 'Fleet')
                nav_icon('gavel', '/missions', 'Missions')
                ui.space()
                # nav_icon('settings', '#', 'Settings')

            # MAIN CONTENT AREA
            with ui.column().classes('flex-grow h-full p-0 relative m-0'):
                page_func()
                
    return wrapper
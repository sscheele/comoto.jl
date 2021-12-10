import PySimpleGUI as sg

layout = [[sg.Text("Hello from PySimpleGUI")], [sg.Button("OK")],[sg.Button("Change")]]

# Create the window
window = sg.Window("Demo", layout)

# Create an event loop
while True:
    event, values = window.read()
    # End program if user closes window or
    # presses the OK button
    if event == "Change":
        layout[0] = [sg.Text("Bye from PySimpleGUI")]
        window = sg.Window("Demo", layout)
    
    if event == "OK" or event == sg.WIN_CLOSED:
        break

window.close()
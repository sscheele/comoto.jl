import tkinter as tk
from pygame import mixer


mixer.init()

def play_music():
    mixer.music.load("beep.mp3")
    mixer.music.play()
    
root = tk.Tk()
tk.Button(root, text="Play music", command=play_music).pack()
root.mainloop()
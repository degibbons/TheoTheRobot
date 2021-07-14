import tkinter as tk
from tkinter import ttk


def open_win():
    new = tk.Toplevel(root)
    new.title("New Window")
    # tk.Label(master=new,text="Hello!").pack(pady=30)

root = tk.Tk()
root.title("Theo Robot Interface")

frame_1 = tk.Frame(master=root, relief=tk.RIDGE, borderwidth=1)
gui_title = tk.Label(master=frame_1, text='Theo Graphical User Interface',
                     relief=tk.SUNKEN, font="Helvetica 18 bold")
frame_1.grid(column=0, row=0, columnspan=2)
gui_title.pack(side=tk.TOP)

frame_2 = tk.Frame(master=root)
button_2 = tk.Button(master=frame_2, text='Move Servo', command=open_win)
frame_3 = tk.Frame(master=root)
button_3 = tk.Button(master=frame_3, text='Move Limb', command=open_win)
frame_4 = tk.Frame(master=root)
button_4 = tk.Button(master=frame_4, text='Move Body', command=open_win)
frame_5 = tk.Frame(master=root)
button_5 = tk.Button(master=frame_5, text='Other Functions', command=open_win)
frame_6 = tk.Frame(master=root)
canvas_1 = tk.Canvas(master=frame_6, width=187, height=200, relief=tk.SUNKEN, borderwidth=5)

frame_2.grid(column=1, row=1)
button_2.pack(ipadx=5, ipady=5)
frame_3.grid(column=1, row=2)
button_3.pack(ipadx=5, ipady=5)
frame_4.grid(column=1, row=3)
button_4.pack(ipadx=5, ipady=5)
frame_5.grid(column=1, row=4)
button_5.pack(ipadx=5, ipady=5)
frame_6.grid(column=0, row=1, rowspan=4)
canvas_1.pack(pady=5)

Theo_Head = canvas_1.create_rectangle(80, 20, 120, 50, fill="red")
Theo_Body = canvas_1.create_rectangle(80, 60, 120, 140, fill="red")
Theo_Tail = canvas_1.create_rectangle(80, 150, 120, 195, fill="red")
Theo_FL_Limb = canvas_1.create_rectangle(10, 60, 70, 95, fill="red")
Theo_BL_Limb = canvas_1.create_rectangle(10, 105, 70, 140, fill="red")
Theo_FR_Limb = canvas_1.create_rectangle(130, 60, 190, 95, fill="red")
Theo_BR_Limb = canvas_1.create_rectangle(130, 105, 190, 140, fill="red")
Head_Label = canvas_1.create_text(100, 35, text="Head")
Body_Label = canvas_1.create_text(100, 100, text="Body")
Tail_Label = canvas_1.create_text(100, 172.5, text="Tail")
FL_Label = canvas_1.create_text(40, 77.5, text="F L Limb")
BL_Label = canvas_1.create_text(40, 122.5, text="B L Limb")
FR_Label = canvas_1.create_text(160, 77.5, text="F R Limb")
BR_Label = canvas_1.create_text(160, 122.5, text="B R Limb")

frame_7 = tk.Frame(master=root, borderwidth=5)
frame_7.grid(column=0, row=5)
button_7 = tk.Button(master=frame_7, text="Connect Servos")
button_7.pack()

frame_8 = tk.Frame(master=root, relief=tk.RIDGE, borderwidth=5)
frame_8.grid(column=1, row=6)
button_8a = tk.Button(master=frame_8, text='Check/Change Variables', command=open_win)
button_8b = tk.Button(master=frame_8, text='Graph Variables', command=open_win)
button_8a.pack(side=tk.LEFT, padx=5, pady=5)
button_8b.pack(side=tk.LEFT, padx=5, pady=5)

frame_9 = tk.Frame(master=root, relief=tk.RIDGE, borderwidth=5)
frame_9.grid(column=0, row=6)
button_9a = tk.Button(master=frame_9, text='Placeholder 1')
button_9b = tk.Button(master=frame_9, text='Placeholder 2')
button_9c = tk.Button(master=frame_9, text='Placeholder 3')
button_9a.pack(side=tk.LEFT, padx=5, pady=5)
button_9b.pack(side=tk.LEFT, padx=5, pady=5)
button_9c.pack(side=tk.LEFT, padx=5, pady=5)


root.mainloop()

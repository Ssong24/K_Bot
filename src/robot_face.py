from tkinter import *
from time import sleep


class RobotFrame():
    def __init__(self):
        self.root = Tk()
        self.root.wm_title("Main page")
        self.root.geometry("800x480")
        self.status_label = None
        self.face_label = None
        self.status = ''
        self.face = ''
        self.default_face()

    # This method closes the recourses
    def cleanup(self):
        self.root.destroy()

    # This method shows the argument as a text on screen
    def show_label(self, text):
        self.status = text
        self.update_screen()

    # This method shows the argument as a face on screen
    def show_face(self, face):
        self.face = face
        self.update_screen()

    # This method updates the labels on the screen
    def update_screen(self):
        if self.status_label is not None:
            self.status_label.pack_forget()
        if self.face_label is not None:    
            self.face_label.pack_forget()
        self.status_label = Label(self.root,
                text= ('\n\n' + self.status), 
                font="Coutrier 32",             # text font
                anchor=W)
        self.face_label = Label(self.root,
                text= self.face,
                font="Coutrier 180",            # face font
                anchor=N)
        self.face_label.pack()
        self.status_label.pack()
        self.root.update()
        
    # This method blinks once
    def blink(self):
        sleep(0.2)
        self.face = '-_-'
        self.update_screen()
        sleep(0.2)
        self.face = '0_0'
        self.update_screen()

    # this method applies the excited face
    def excited_face(self):
        self.face = '>_<'
        self.update_screen()

    # This method applies the default face
    def default_face(self):
        self.face = '0_0'
        self.update_screen()





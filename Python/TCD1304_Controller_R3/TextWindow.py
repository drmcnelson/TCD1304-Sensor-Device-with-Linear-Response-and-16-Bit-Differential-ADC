#!/usr/bin/python

import signal
import atexit

import tkinter
import tkinter as Tk

from tkinter import filedialog
from tkinter import simpledialog
from tkinter import messagebox
from tkinter import font as tkfont
from tkinter import IntVar
from tkinter import ttk
from tkinter import scrolledtext

class TextWindow:

    def __init__( self, title, geometry='300x200', maxlines=100 ):

        self.title = title
        self.geometry = geometry

        self.parent = Tk.Tk()
        if title is not None:
            self.parent.title( title )
        self.textContainer = Tk.Frame(self.parent, borderwidth=1, relief="sunken")
        self.txt = Tk.Text(self.textContainer, width=80, height=20, wrap="none", borderwidth=0)
        self.textVsb = Tk.Scrollbar(self.textContainer, orient="vertical", command=self.txt.yview)
        self.textHsb = Tk.Scrollbar(self.textContainer, orient="horizontal", command=self.txt.xview)
        self.txt.configure(yscrollcommand=self.textVsb.set, xscrollcommand=self.textHsb.set)


        self.txt.grid(row=0, column=0, sticky="nsew")
        self.textVsb.grid(row=0, column=1, sticky="ns")
        self.textHsb.grid(row=1, column=0, sticky="ew")

        self.textContainer.grid_rowconfigure(0, weight=1)
        self.textContainer.grid_columnconfigure(0, weight=1)
        self.textContainer.pack(side="top", fill="both", expand=True)

        self.nlines = 0
        self.maxlines = maxlines

        self.parent.update_idletasks()
        self.parent.update()
       
    def addline_( self, line ):
        self.txt.configure(state ='normal')        
        if self.nlines >= self.maxlines:
            self.txt.delete("1.0")
        self.txt.insert(Tk.END, line )
        self.txt.see("end")
        self.nlines += 1
        self.txt.configure(state ='disabled')
        
    def addline( self, line ):
        self.addline_( line )
        self.parent.update_idletasks()
        self.parent.update()

    def update( self ):
        self.parent.update_idletasks()
        self.parent.update()

    def close(self):
        self.parent.destroy()

    def loop(self, flag, queue ):
        while flag.value:
            while not queue.empty:
                line = queue.get()
                self.addline(line)
            sleep(0.1)
            self.update()
        self.close()
        print( 'TextWindow loop ', self.title, 'exit' )

        


if __name__ == "__main__":

    import sys
    from time import sleep
    import signal

    def SignalHandler(signal, frame):
        print('Ctrl-C')
        logwin.close()

        print('Exit')
        sys.exit(0)

    signal.signal(signal.SIGINT, SignalHandler)
    
    logwin = TextWindow( 'logging' )

    while True:
        line = input( "line ? " )
        if line.lower() in [ 'q', 'quit', 'e', 'exit' ]:
            logwin.close()
            quit()
        if line.lower() in [ 'c', 'close' ]:
            print( 'closing' )
            logwin.close()
            sleep(1)
            print( 'closed' )
            logwin = TextWindow( 'logging' )
        else:
            logwin.addline( line+'\n' )
        

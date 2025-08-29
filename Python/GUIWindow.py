#!/usr/bin/python

import os

from datetime import datetime

import numpy as np

from threading import Lock, Semaphore, Thread
from queue import SimpleQueue, Empty
from multiprocessing import Process, Queue, Value
from time import sleep, time, process_time, thread_time

import matplotlib as mpl
mpl.use("TkAgg")
#mpl.use("TkCairo" )

import matplotlib.pyplot as plt

plt.rcParams['toolbar'] = 'toolmanager'
from matplotlib.backend_tools import ToolBase, ToolToggleBase

from matplotlib.figure import Figure
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk
#from matplotlib.backends.backend_tkagg import NavigationToolbar2TkAgg as NavigationToolbar2Tk

from tkinter import ttk

from tkinter import *
from tkinter.ttk import *

from tkinter import filedialog
from tkinter import simpledialog
from tkinter import messagebox


#from matplotlib.widgets import TextBox

def transmission_( y, r):
    T = np.zeros_like( y )
    idx = np.where( (r>0.) & (y > 0.) )
    T[idx] = y[idx]/r[idx]
    return T
        
def absorption_( y, r ):
    A = np.zeros_like( y )
    idx = np.where( (r>0.) & (y > 0.) )
    A[idx] = np.log10( r[idx]/y[idx] )
    return A
    
# ========================================================================================================
def addtextwidget( fig, box, label, initial, action ):
    wax = fig.add_axes( box )
    tbox = TextBox( wax, label, initial=initial, textalignment = 'left' )
    tbox.label.set_color( 'blue' )
    tbox.text_disp.set_color( 'green' )
    tbox.on_submit( action )
    return wax, tbox
# ========================================================================================================

def showError( text ):
    root = Tk()
    root.withdraw()
    messagebox.showerror("Error", text)
    root.destroy()

def showWarning( text ):
    root = Tk()
    root.withdraw()
    messagebox.showwarning("Warning", text)
    root.destroy()

def showInfo( text ):
    root = Tk()
    root.withdraw()
    messagebox.showinfo("Info", text)
    root.destroy()

def getfloat( title, prompt, vdef, vmin, vmax ):
    root = Tk()
    root.withdraw()
    result = simpledialog.askfloat( title, prompt, initialvalue=vdef, minvalue=vmin, maxvalue=vmax )
    root.destroy()
    return result

def getinteger( title, prompt, vdef, vmin, vmax ):
    root = Tk()
    root.withdraw()
    result = simpledialog.askinteger( title, prompt, initialvalue=vdef, minvalue=vmin, maxvalue=vmax )
    root.destroy()
    return result

# --------------------------------------------------------------------------
# Custom tkinter dialog for radiobuttons
def getchoice( title, labels, values, default ):
    
    root = Tk()
    root.title( title )
    root.withdraw()

    idx = values.index(default)
    var = StringVar()
    var.set(default)
    
    class ChoiceDialog(simpledialog.Dialog):   
        def body(self, master):
            Label( master, text=title ).pack(anchor='w')
            for l,v in zip(labels,values):
                print( 'label', l, 'value', v )
                Radiobutton(master, text=l, variable=var, value=v ).pack(anchor='w')
                
        def apply(self):
            self.result = var.get()
            print( 'apply', self.result )
            
    mode = ChoiceDialog(root)
    result = mode.result
    print( 'result', result )
    
    root.destroy()

    return result
            
# --------------------------------------------------------------------------
# Custom tkinter dialog for settings

#def getsettings( shutterinterval, frameinterval, framecount, mode, latchmode, processingmode, averaging, parentinstance ):
def getsettings( parent ):

    class SettingsDialog(simpledialog.Dialog):
        
        def body(self, master ):
            
            self.clocktrigger = StringVar(master, parent.mode)
            self.latchvalue = StringVar(master, parent.latchmode)
            self.processingmode = StringVar(master, parent.processingmode)
            self.averaging = StringVar(master,parent.averaging)
            self.verbosevalue = StringVar(master, str(parent.verbose) )

            width = 8
            row = 0

            self.lclock = Radiobutton(master, text = "Clock", variable = self.clocktrigger, value="clocked" )
            self.lclock.grid(row=row,column=0)
            
            self.ltrigger = Radiobutton(master, text = "Trigger", variable = self.clocktrigger, value="triggered" )
            self.ltrigger.grid(row=row,column=1)

            self.lgate = Radiobutton(master, text = "Gate", variable = self.clocktrigger, value="gated" )
            self.lgate.grid(row=row,column=2)

            self.clocktrigger.set( parent.mode )   
            
            row += 1
            
            self.vsep1 = Separator( master, orient="horizontal" )
            self.vsep1.grid( row=row, column=0,columnspan=3, sticky='ew' )
                        
            row += 1
            
            Label( master, text='Shutter', anchor='w' ).grid(row=row,column=0)
            Label( master, text=' (usecs)', anchor='w' ).grid(row=row,column=2)

            self.lshutter = Entry(master, width=width )
            self.lshutter.insert( 0, '%d'%parent.shutterinterval )
            self.lshutter.grid(row=row,column=1)
            
            row += 1
            
            Label( master, text='Frame', anchor='w'  ).grid(row=row,column=0)
            Label( master, text=' (usecs)', anchor='w' ).grid(row=row,column=2)

            self.lframe = Entry(master, width=width)
            self.lframe.insert( 0, '%d'%parent.frameinterval )
            self.lframe.grid(row=row,column=1)
            
            row += 1
            
            Label( master, text='Count', anchor='w' ).grid(row=row,column=0)
            Label( master, text=' (frames)', anchor='w' ).grid(row=row,column=2)
            
            self.lcount = Entry(master, width=width)
            self.lcount.insert( 0, '%d'%parent.framecount )
            self.lcount.grid(row=row,column=1)

            row += 1
            
            self.vsep1 = Separator( master, orient="horizontal" )
            self.vsep1.grid( row=row, column=0,columnspan=3, sticky='ew' )
            
            row += 1
            
            Label( master, text='Averaging', anchor='w' ).grid(row=row,column=0)
            Label( master, text=' (framesets)', anchor='w' ).grid(row=row,column=2)
            
            self.laveraging = Entry(master, width=width)
            self.laveraging.insert( 0, '%d'%parent.averaging )
            self.laveraging.grid(row=row,column=1)

            row += 1
            
            self.vsep1 = Separator( master, orient="horizontal" )
            self.vsep1.grid( row=row, column=0,columnspan=3, sticky='ew' )
            
            row += 1
            
            self.llumin = Radiobutton(master, text = "Raw", variable = self.processingmode, value="emission" )
            self.llumin.grid(row=row,column=0)
            
            self.labsorp = Radiobutton(master, text = "Absorpt", variable = self.processingmode, value="absorption" )
            self.labsorp.grid(row=row,column=1)
            
            self.ltransm = Radiobutton(master, text = "Transm", variable = self.processingmode, value="transmission" )
            self.ltransm.grid(row=row,column=2)
            
            self.processingmode.set( parent.processingmode )
            
            row += 1
            
            self.vsep1 = Separator( master, orient="horizontal" )
            self.vsep1.grid( row=row, column=0,columnspan=3, sticky='ew' )

            row += 1
            
            self.lverbose = Checkbutton(master, text = "Verbose", variable = self.verbosevalue, onvalue=str(True), offvalue=str(False) )
            self.lverbose.grid(row=row,column=1)

            self.verbosevalue.set( str(parent.verbose) )
            
        def apply(self):
            if self.processingmode.get() != "emission" and parent.reference is None:
                showError( "set reference first" )
                self.result = False

            else:
                try:
                    self.mode = self.clocktrigger.get()
                    self.shutterinterval = int( self.lshutter.get() )
                    self.frameinterval = int( self.lframe.get() )
                    self.framecount = int( self.lcount.get() )
                    #self.latchmode = self.latchvalue.get()
                    self.processingmode = self.processingmode.get()
                    self.averaging = int(self.laveraging.get())
                    self.verbose = eval(self.verbosevalue.get())
                    print( 'verbose value', self.verbosevalue.get() )
                    self.result = True
                except Exception as e:
                    print(e)
                    self.result = False

    root = Tk()
    root.title( "Settings" )
    root.withdraw()

    dialog = SettingsDialog(root)
    result = dialog.result
    
    if result:

        if dialog.processingmode != parent.processingmode:
            print( "processing mode change", dialog.processingmode )
            parent.processingmodechange = True
            
        parent.mode = dialog.mode
        parent.shutterinterval =  dialog.shutterinterval
        parent.frameinterval =  dialog.frameinterval
        parent.framecount =  dialog.framecount
        #parent.latchmode =  dialog.latchmode
        parent.processingmode =  dialog.processingmode
        parent.averaging =  dialog.averaging
        parent.verbose =  dialog.verbose
    
    root.destroy()

    return result
                
                
# ========================================================================================================

class GUIWindow:

    def __init__( self, name, xdata=None, xrange=None, xlabel=None,
                  ycols=None, yrange=None, ylabels=None,
                  geometry='1000x600', queue=None, flag=None, nlength=0, ncolumns=0, filespec=None, parentinstance=None, debug = False ):
        
        # -----------------------
        if parentinstance is None:
            raise ValueError('need parent instance')

        if ycols is None and yrange is None:
            raise ValueError('need ycols or yrange')

        if ycols is None and (nlength == 0 or ncolumns == 0):
            raise ValueError('need ycols or, nlength and ncolumns')
        
        if filespec is None:
            raise ValueError('need initial filespec' )
        
        # -----------------------
        # default the initial data
        if name is None:
            name = 'Data'
        self.name = name

        # -----------------------
        # file directory and extension
        self.filespec = filespec
        self.filedir = os.path.dirname( self.filespec )
        self.fileext = os.path.splitext( self.filespec )

        # -----------------------
        # parent instrument class instance
        self.parentinstance = parentinstance

        self.debug = debug
        
        if xdata is None:
            xdata = np.linspace( 0, nlength, nlength )
        self.xdata = xdata
        
        if xlabel is None:
            xlabel = 'index'
        self.xlabel = xlabel

        self.xrange = xrange
        
        if ycols is None:
            ycols = [np.zeros(nlength)] * ncolumns
        self.ycols = ycols
                
        if ylabels is None:
            ylabels = [ 'Chan %d' for d in range(ncolumns) ]
        self.ylabels = ylabels

        self.yrange = yrange

        self.geometry = geometry

        if queue is None:
            self.queue = Queue()
        else:
            self.queue = queue
            
        if flag is None:
            self.flag = Value('i',1)
        else:
            self.flag = flag
            self.flag.value = 1
            
        self.thread = None
            
        self.history = []
        self.historypointer = 0
        
        self.reference = None

        # gui controls
        self.shutterinterval = 10000
        self.frameinterval = 10000
        self.framecount =  1
        self.fastshutter = False
        self.mode = 'clocked'
        self.latchmode = 'nolatch'
        self.processingmode = 'emission'
        self.averaging = 0

        self.processingmodechange = False
        self.autorun = False

        self.verbose = False
        
    def animation( self, interval = 200, blit = True ):
        
        self.fig = plt.figure(self.name)
        if self.geometry is not None:
            dpi = self.fig.get_dpi()
            width,height = self.geometry.lower().split('x',maxsplit=1)
            self.fig.set_size_inches( int(width)/dpi, int(height)/dpi )
            
        self.fig.subplots_adjust(top=0.9)
        
        # This gives us scrolling through the history record
        self.fig.canvas.manager.toolmanager.add_tool('RunStart', self.RunStart, graphobj=self )
        self.fig.canvas.manager.toolmanager.add_tool('STOP', self.RunStop, graphobj=self )
        self.fig.canvas.manager.toolmanager.add_tool('Settings', self.Settings, graphobj=self )
        self.fig.canvas.manager.toolmanager.add_tool('Prev', self.PreviousGraph, graphobj=self )
        self.fig.canvas.manager.toolmanager.add_tool('Next', self.NextGraph, graphobj=self )
        self.fig.canvas.manager.toolmanager.add_tool('Last', self.LastGraph, graphobj=self )
        self.fig.canvas.manager.toolmanager.add_tool('Set', self.StoreReference, graphobj=self )
        self.fig.canvas.manager.toolmanager.add_tool('Clr', self.ClearReference, graphobj=self )
        self.fig.canvas.manager.toolmanager.add_tool('Save', self.SaveData, graphobj=self )
        self.fig.canvas.manager.toolbar.add_tool('RunStart', 'toolgroup0',-1)
        self.fig.canvas.manager.toolbar.add_tool('STOP', 'toolgroup0',-1)
        self.fig.canvas.manager.toolbar.add_tool('Settings', 'toolgroup1',-1)
        self.fig.canvas.manager.toolbar.add_tool('Prev', 'toolgroup2',-1)
        self.fig.canvas.manager.toolbar.add_tool('Next', 'toolgroup2',-1)
        self.fig.canvas.manager.toolbar.add_tool('Last', 'toolgroup2',-1)
        self.fig.canvas.manager.toolbar.add_tool('Set', 'toolgroup3',-1)
        self.fig.canvas.manager.toolbar.add_tool('Clr', 'toolgroup3',-1)
        self.fig.canvas.manager.toolbar.add_tool('Save', 'toolgroup4',-1)
                
        self.ax = self.fig.add_subplot(1, 1, 1)

        self.lns = []
        for y, label in zip( self.ycols, self.ylabels ):
            ln = self.ax.plot( self.xdata, y,  label=label )
            ln = ln[0]
            #print( 'ax.plot returned', ln )
            self.lns.append(ln)
            self.ylabels.append(label)

        self.txt = self.ax.text( 0.99, 0.99, '0', horizontalalignment='right', verticalalignment='top', transform=self.ax.transAxes )

        self.ax.legend( loc='upper left' )

        if self.xrange:
            self.ax.set_xlim( self.xrange )

        if self.yrange:
            self.ax.set_ylim( self.yrange )

        # still need plt.show() to launch it.
        if self.queue is not None:
            self.ani = FuncAnimation(self.fig, self.animation_update, interval=200, blit=blit )

        self.fig.canvas.mpl_connect('close_event', self.close )
        
        plt.show()

        plt.close()
            
    def animation_update( self, i ):

        if self.flag.value:

            gotstuff = False
            
            while True:
                try:
                    record = self.queue.get(block=False)

                    self.graphrecord_( record )

                    self.history.append(record)
                    if len(self.history) > 100:
                        self.history.pop(0)
                    self.historypointer = len(self.history)-1

                    gotstuff = True
                    #print( 'got record' )
                except Empty:
                    break

                if self.autorun and gotstuff:
                    self.autorun = self.autocommand()
                

        else:
            self.ani.event_source.stop()
            plt.close()

        #return tuple( self.lns) + ( self.txt, )
        return (self.txt, *self.lns )

    def start( self, interval=200, blit=True ):

        self.thread = Process( target=self.animation,args=(interval,blit) )
        self.thread.start()
        
    def close( self, ignored=None ):

        self.flag.value = 0
            
        if self.thread:
            try:
                self.thread.terminate()        
                self.thread.join()
            except Exception as e:
                pass

    def autocommand( self ):

        if self.mode == "triggered":
            s = "trigger %d %d"%(max(self.framecount,1),self.shutterinterval)
            
        elif self.mode == "gated":
            s = "gate 1"
            
        else:
            s = "read %d"%(self.shutterinterval)

        return self.command( s )
        
    def command( self, s, clear=True, accumulate=False ):

        if clear:
            self.parentinstance.clear()

        if accumulate:
            self.parentinstance.accumulatorflag.value = 1
        else:
            self.parentinstance.accumulatorflag.value = 0
        
        retv = True

        if self.verbose:
            print( s )

        self.parentinstance.write( s + '\n' )
        
        response = self.parentinstance.read()
        
        if response is not None:
            for r in response:
                if r.startswith( "Error" ):
                    print( s )
                    print( r )
                    showError( r.strip() )
                    retv = False

        return retv
        
    # ---------------------------------------------------------        
    def graphupdate( self, xdata=None, ycols=None, text=None ):
        
        if ycols is not None:
            for ln, y in zip(self.lns, ycols):
                ln.set_ydata( y )
            
        if xdata is not None:
            for ln in self.lns:
                ln.set_xdata( xdata )
            self.ax.set_xlim( left=min(xdata), right=max(xdata) )
            self.ax.relim()
            self.ax.autoscale_view()

        if text is not None:
            self.txt.set_text( text )
            
        return ( self.txt, *self.lns )

    # ---------------------------------------------------------
    # ---------------------------------------------------------
    def graphrecord_( self, record ):

        ycols, text = record
        
        if len(ycols) > 1:

            x = ycols[0]

            if self.processingmode == "absorption":
                if self.verbose:
                    print( "converting to absorption", "new mode", self.processingmodechange )
                yrefs, textrefs = self.reference
                for n, (y, r) in enumerate( ycols[1:], yrefs[1:] ):
                    y = absorption_( y, r )
                    self.lns[n].set_data( x, y )

                if self.processingmodechange:
                    print( 'setting y axis' )
                    self.processingmodechange = False
                    self.ax.set_ylim( -0.01, 2.0 )
                    
            elif self.processingmode == "transmission" and self.reference is not None:
                if self.verbose:
                    print( "converting to transmission", "new mode", self.processingmodechange )
                yrefs, textrefs = self.reference
                for n, (y, r) in enumerate( ycols[1:], yrefs[1:] ):
                    y = transmission_( y, r )
                    self.lns[n].set_data( x, y )

                if self.processingmodechange:
                    print( 'setting y axis' )
                    self.processingmodechange = False
                    self.ax.set_ylim( 0, 1.0 )
                    
            else:
                if self.verbose:
                    print( "raw emission", "new mode", self.processingmodechange )
                    
                for n, y in enumerate( ycols[1:] ):
                    self.lns[n].set_data( x, y )

                if self.processingmodechange:
                    print( 'setting y axis' )
                    self.processingmodechange = False
                    self.ax.set_ylim( self.yrange )
                    
            self.ax.set_xlim( left=min(x), right=max(x) )
            self.ax.relim()
            self.ax.autoscale_view()

        elif len(ycols) == 1:

            if self.processingmode == "absorption":
                if self.verbose:
                    print( "converting to absorption" )
                    
                yrefs, textrefs = self.reference
                y = absorption_( ycols[0], yrefs[0] )
                self.lns[0].set_ydata( y )
                
                if self.processingmodechange:
                    print( 'setting y axis absorption' )
                    self.processingmodechange = False
                    self.ax.set_ylim( -0.01, 2.0 )
                    self.ax.relim()
                    self.ax.autoscale_view()
                    self.fig.canvas.draw()
                    
            elif self.processingmode == "transmission":
                if self.verbose:
                    print( "converting to transmission" )
                    
                yrefs, textrefs = self.reference
                y = transmission_( ycols[0], yrefs[0] )
                self.lns[0].set_ydata( y )

                if self.processingmodechange:
                    print( 'setting y axis transmission' )
                    self.processingmodechange = False
                    self.ax.set_ylim( 0, 1.0 )
                    self.ax.relim()
                    self.ax.autoscale_view()
                    self.fig.canvas.draw()
                    
            else:
                self.lns[0].set_ydata( ycols[0] )

                if self.processingmodechange:
                    print( 'setting y axis, raw' )
                    self.processingmodechange = False
                    self.ax.set_ylim( self.yrange )
                    self.ax.relim()
                    self.ax.autoscale_view()
                    self.fig.canvas.draw()

        self.txt.set_text( text )
    
    class LastGraph(ToolBase):
        default_keymap = 'up'
        description = 'last data vector'

        def __init__(self, *args, **kwargs):
            self.graph = kwargs.pop('graphobj')
            super().__init__(*args, **kwargs)
            #ToolBase.__init__(self, *args, **kwargs)
            
        def trigger(self, *args, **kwargs):
            #print('last graph key pressed')

            try:
                record = self.graph.history[-1]
                self.graph.historypointer = len(self.graph.history)-1

                self.graph.graphrecord_( record )
                self.graph.fig.canvas.draw()
            except Exception as e:
                print("LastGraph", e )
            
    class NextGraph(ToolBase):
        default_keymap = 'right'
        description = 'next data vector'

        def __init__(self, *args, **kwargs):
            self.graph = kwargs.pop('graphobj')
            super().__init__(*args, **kwargs)
            #ToolBase.__init__(self, *args, **kwargs)
            
        def trigger(self, *args, **kwargs):
            #print('next graph key pressed')

            if self.graph.historypointer < len(self.graph.history) - 1:

                self.graph.historypointer += 1        
                record = self.graph.history[self.graph.historypointer]
                
                self.graph.graphrecord_( record )
                self.graph.fig.canvas.draw()
            else:
                print( 'already at last graph' )

    class PreviousGraph(ToolBase):
        default_keymap = 'left'
        description = 'prev data vector'

        def __init__(self, *args, **kwargs):
            self.graph = kwargs.pop('graphobj')
            super().__init__(*args, **kwargs)
            #ToolBase.__init__(self, *args, **kwargs)
                    
        def trigger(self, *args, **kwargs):
            #print('previous graph key pressed')
            
            if self.graph.historypointer > 0:

                self.graph.historypointer -= 1        
                record = self.graph.history[self.graph.historypointer]
                
                self.graph.graphrecord_( record )
                self.graph.fig.canvas.draw()
            else:
                print( 'already at first graph' )

    class StoreReference(ToolBase):
        description = 'store last graphed spectrum as reference'

        def __init__(self, *args, **kwargs):
            self.graph = kwargs.pop('graphobj')
            super().__init__(*args, **kwargs)
            #ToolBase.__init__(self, *args, **kwargs)
                    
        def trigger(self, *args, **kwargs):
            #print('previous graph key pressed')
            
            if self.graph.historypointer > 0:

                record = self.graph.history[self.graph.historypointer]
                self.graph.reference = record
                
            else:
                print( 'no data available' )

    class ClearReference(ToolBase):
        description = 'clear reference spectrum'

        def __init__(self, *args, **kwargs):
            self.graph = kwargs.pop('graphobj')
            super().__init__(*args, **kwargs)
            #ToolBase.__init__(self, *args, **kwargs)
                    
        def trigger(self, *args, **kwargs):
            #print('previous graph key pressed')

            self.graph.reference = None

    class SaveData(ToolBase):
        default_keymap = 'enter'
        description = 'save data to disk'

        def __init__(self, *args, **kwargs):
            self.graph = kwargs.pop('graphobj')
            super().__init__(*args, **kwargs)
            #ToolBase.__init__(self, *args, **kwargs)
            
        def trigger(self, *args, **kwargs):
            #print('previous graph key pressed')

            #initialfile = "tdaqdata." + timestamp.strftime('%Y%m%d.%H%M%S.%f') + self.graph.parentinstance.filesuffix
            timestamp = datetime.now()
            fext = '.' + timestamp.strftime('%Y%m%d.%H%M%S.%f') + self.graph.parentinstance.filesuffix

            root = Tk()
            root.withdraw()
            fname = filedialog.asksaveasfilename(
                title='Save the data',
                defaultextension=fext,
                initialdir='.'
            )
            root.destroy()

            if fname in ["", ()]:
                return

            fname = str(fname)
            print( fname )

            try:
                self.graph.parentinstance.savetofile( fname,
                                                      timestamp=None,
                                                      comments = None,
                                                      write_ascii=True,
                                                      framesetindex = None,
                                                      records = None )
            
            except Exception as e:
                messagebox.showerror("Error saving file", fname + " " + str(e))
            
    # ---------------------------------------------------------
    class Settings(ToolBase):

        description = 'set shutter, frame interval and count'

        def __init__(self, *args, **kwargs):
            self.graph = kwargs.pop('graphobj')
            super().__init__(*args, **kwargs)
            #ToolBase.__init__(self, *args, **kwargs)
            
        def trigger(self, *args, **kwargs):

            '''
            result = getsettings(self.graph.shutter,self.graph.frameinterval,self.graph.framecount,
                                 self.graph.mode, self.graph.latchmode, self.graph.processingmode, self.graph.averaging, self.graph )
            '''
            result = getsettings( self.graph )
            print( result )


    class RunStart(ToolBase):
        description = 'start data acquisition'

        def __init__(self, *args, **kwargs):
            self.graph = kwargs.pop('graphobj')
            super().__init__(*args, **kwargs)
            #ToolBase.__init__(self, *args, **kwargs)
                    
        def trigger(self, *args, **kwargs):

            if self.graph.framecount <= 0:

                if not self.graph.autorun:
                    self.graph.autorun = self.graph.autocommand()
                else:
                    self.graph.autorun = False

            elif self.graph.mode == "triggered":

                if self.graph.averaging > 1:
                    s = "trigger %d %d %d"%( self.graph.averaging,
                                             max(self.graph.framecount,1),
                                             self.graph.shutterinterval)

                    self.graph.command( s, accumulate=True )
                    
                else:
                    s = "trigger %d %d"%( max(self.graph.framecount,1),
                                          self.graph.shutterinterval)

                    self.graph.command( s )
                
            elif self.graph.mode == "gated":

                if self.graph.averaging > 1:
                    s = "gate %d"%(self.graph.averaging)

                    self.graph.command( s, accumulate=True )
                    
                else:
                    s = "gate %d"%( max(self.graph.framecount,1) )

                    self.graph.command( s )
                
            elif self.graph.shutterinterval == self.graph.frameinterval:
                
                if self.graph.averaging > 1:
                    s = "read %d %d"%(self.graph.averaging,
                                      self.graph.shutterinterval)
                    
                    self.graph.command( s, accumulate=True )
                    
                else:
                    s = "read %d %d"%( max(self.graph.framecount,1),
                                      self.graph.shutterinterval)

                    self.graph.command( s )
                
            elif self.graph.averaging > 1:
                
                s = "read %d %d %d"%(self.graph.averaging,
                                     self.graph.shutterinterval,
                                     self.graph.frameinterval)

                self.graph.command( s, accumulate=True )
                    
            else:
                s = "read %d %d %d"%( max(self.graph.framecount,1),
                                      self.graph.shutterinterval,
                                      self.graph.frameinterval)
                
                self.graph.command( s )

    class RunStop(ToolBase):
        
        description = 'stop data acquisition'

        def __init__(self, *args, **kwargs):
            self.graph = kwargs.pop('graphobj')
            super().__init__(*args, **kwargs)
            #ToolBase.__init__(self, *args, **kwargs)
                    
        def trigger(self, *args, **kwargs):

            self.graph.autorun = False

            self.graph.command( 'stop' )

#!/usr/bin/python
""" GraphicsWindow.py: realtime graphics window for data acqusition """

__author__    = "Mitchell C. Nelson, PhD"
__copyright__ = "Copyright 2022, Mitchell C, Nelson"
__version__   = "0.2"
__email__     = "drmcnelson@gmail.com"
__status__    = "alpha testing"

versionstring = 'GraphicsWindow.py - version %s %s M C Nelson, PhD, (c) 2021'%(__version__,__status__)



import numpy as np

from threading import Lock, Semaphore, Thread
#from queue import SimpleQueue, Empty
from multiprocessing import Process, Queue, Value
import queue

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


class GraphicsWindow:

    def __init__( self, name, xdata=None, xrange=None, xlabel=None,
                  ycols=None, yrange=None, ylabels=None,
                  geometry='1000x600', queue=None, flag=None, nlength=0, ncolumns=0, debug = False ):

        # -----------------------
        # default the initial data
        if name is None:
            name = 'Data'
        self.name = name

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
        
    def animation( self, interval = 200, blit = True ):
    
        self.fig = plt.figure(self.name)
        if self.geometry is not None:
            dpi = self.fig.get_dpi()
            width,height = self.geometry.lower().split('x',maxsplit=1)
            self.fig.set_size_inches( int(width)/dpi, int(height)/dpi )
            
        self.fig.subplots_adjust(top=0.9)
        
        # This gives us scrolling through the history record
        self.fig.canvas.manager.toolmanager.add_tool('Prev', self.PreviousGraph, graphobj=self )
        self.fig.canvas.manager.toolmanager.add_tool('Next', self.NextGraph, graphobj=self )
        self.fig.canvas.manager.toolmanager.add_tool('Last', self.LastGraph, graphobj=self )
        self.fig.canvas.manager.toolbar.add_tool('Prev', 'toolgroup', -1)
        self.fig.canvas.manager.toolbar.add_tool('Next', 'toolgroup', -1)
        self.fig.canvas.manager.toolbar.add_tool('Last', 'toolgroup', -1)
                
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
            while True:
                try:
                    record = self.queue.get(block=False)

                    self.graphrecord_( record )

                    self.history.append(record)
                    if len(self.history) > 100:
                        self.history.pop(0)
                    self.historypointer = len(self.history)-1
                        
                    #print( 'got record' )
                except queue.Empty:
                    break

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
        
    # ---------------------------------------------------------
    # graph (verb) the passed record
    def graphrecord_( self, record ):

        ycols, text = record
        
        if len(ycols) > 1:

            x = ycols[0]
            
            for n, y in enumerate( ycols[1:] ):
                self.lns[n].set_data( x, y )

            self.ax.set_xlim( left=min(x), right=max(x) )
            self.ax.relim()
            self.ax.autoscale_view()

        elif len(ycols) == 1:

            self.lns[0].set_ydata( ycols[0] )


        self.txt.set_text( text )


    # History button graph functions
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


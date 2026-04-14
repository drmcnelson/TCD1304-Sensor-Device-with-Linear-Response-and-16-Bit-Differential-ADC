#!/usr/bin/python3

"""
TCD1304DeviceMP.py

Mitchell C. Nelson (c) 2023, 2024, 2025

December 5, 2025

Derived from LLCDController.py Copyright 2022 by M C Nelson
Derived from TDAQSerial.py, TCD1304Serial.py, Copyrigh 2021 by M C Nelson

"""

__author__    = "Mitchell C. Nelson, PhD"
__copyright__ = "Copyright 2025, Mitchell C, Nelson"
__version__   = "0.4"
__email__     = "drmcnelson@gmail.com"
__status__    = "alpha testing"

__all__ = [ 'TCD1304CONTROLLER' ]

versionstring = 'TCD1304Device.py - version %s %s M C Nelson, PhD, (c) 2025'%(__version__,__status__)

import sys
import time
import select

import traceback

import os
import operator

import signal
import atexit

#from timeit import default_timer as timer

import platform

import serial

from datetime import datetime
from time import sleep, time, process_time, thread_time

# from threading import Lock, Semaphore, Thread
import multiprocessing as mp

from queue import SimpleQueue, Empty
from multiprocessing import Process, Queue, Value, Lock
from threading import Thread

import struct

import inspect
from itertools import count

import regex
from pyparsing import nestedExpr

import numpy as np
from scipy.signal import savgol_filter as savgol

import matplotlib.pyplot as plt

import faulthandler
faulthandler.enable()

try:
    from TextWindow import TextWindow
    has_TextWindow = True
except Exception as e:
    has_TextWindow = False
    
try:
    from GUIWindow import GUIWindow
    has_GUIWindow = True
except Exception as e:
    has_GUIWindow = False
    print(e)

try:
    from GraphicsWindow import GraphicsWindow
    has_GraphicsWindow = True
except:
    has_GraphicsWindow = False
    
# ----------------------------------------------------------
try:
    from PulseTimingDevice import PULSETIMINGDEVICE
    has_PulseTimingDevice = True
except:
    has_PulseTimingDevice = False

# ----------------------------------------------------------

def lineno():
    return inspect.currentframe().f_back.f_lineno

def errorprint(*s): 
    print(*s, file = sys.stderr) 

def input_ready():
    return (sys.stdin in select.select([sys.stdin], [], [], 0)[0])

def key_in_list( ls, key, mapf=str ):
    try:
        idx = ls.index(key)
        return mapf( ls[idx+1] )
    except:
        return None

def generate_x_vector( npoints, coefficients = None ):

    x = np.linspace( 0, npoints, npoints )
    
    if coefficients is None:
        return x

    else:
        return np.polynomial.polynomial.polyval( x, coefficients )

# --------------------------------------------------------------------------------------------
def split_nested( s ):
    result = regex.search(r'''
    (?<rec> #capturing group rec
    \( #open parenthesis
    (?: #non-capturing group
    [^()]++ #anyting but parenthesis one or more times without backtracking
    | #or
    (?&rec) #recursive substitute of group rec
    )*
    \) #close parenthesis
    )
    ''',s)
    return result

#matches = [match.group() for match in regex.finditer(r"(?:(\((?>[^()]+|(?1))*\))[\S)+",s)]
#return matches

def split_bracketed(string, delimiter=' ', strip_brackets=False):
    """ Split a string by the delimiter unless it is inside brackets.
    e.g.
        list(bracketed_split('abc,(def,ghi),jkl', delimiter=',')) == ['abc', '(def,ghi)', 'jkl']

    stackoverflow question 21662474, answer by Peter, 10/5/21
    """

    openers = '[{(<'
    closers = ']})>'
    opener_to_closer = dict(zip(openers, closers))
    opening_bracket = dict()
    current_string = ''
    depth = 0
    for c in string:
        if c in openers:
            depth += 1
            opening_bracket[depth] = c
            if strip_brackets and depth == 1:
                continue
        elif c in closers:
            assert depth > 0, f"You exited more brackets than we have entered in string {string}"
            assert c == opener_to_closer[opening_bracket[depth]], \
                f"Closing bracket {c} did not match opening bracket {opening_bracket[depth]} in string {string}"
            depth -= 1
            if strip_brackets and depth == 0:
                continue
        if depth == 0 and c == delimiter:
            yield current_string
            current_string = ''
        else:
            current_string += c
    assert depth == 0, f'You did not close all brackets in string {string}'
    yield current_string    

# --------------------------------------------------------------------------------------------
def parseFlt(s):
    try:
        return float(s)
    except Exception as e:
        print( e, s )
        self.error += 1
        return None

def parseInt(s):
    try:
        return int(s)
    except Exception as e:
        print( e, s )
        self.error += 1
        return None

# -----------------------------------
def isint( s):
    try:
        val = int(s)
        return True
    except:
        return False

def isfloat( s):
    try:
        val = float(s)
        return True
    except:
        return False


    
# ======================================================================================================================
class TCD1304DATAFRAME:

    def __init__(self,old=None):

        self.frame_mode = None
        self.frame_every = 0
        self.timestamp = None

        self.data = None
        self.offset = 0.
        self.rawoffset = 0

        self.frame_counter = 0
        self.frame_counts = 0
        self.frame_elapsed = 0.
        self.frame_exposure = 0.
        
        self.frameset_counter = 0
        self.frameset_counts = 0

        self.trigger_counter = 0
        self.trigger_counts = 0
        self.trigger_elapsed = 0.
        self.trigger_difference = 0.

        self.timer_elapsed = 0
        self.timer_difference = 0
        self.timer_period  = 0
        self.timer_subperiod = 0        

        self.sh_period  = 0
        
        self.accumulator_counter = 0
        self.is_accumulating = False;

        self.adc = None
        self.chip_temperature = None

        self.error = None
            
        self.frameset_complete = False
            
        self.complete = False

        if old is not None:
            self.frame_mode = old.frame_mode
            self.frame_counts = old.frame_counts
            self.frameset_counts = old.frameset_counts
            self.sh_period = old.sh_period
            self.timer_period = old.timer_period
            self.timer_subperiod = old.timer_subperiod
            self.trigger_counts = old.trigger_counts
            
def TCD1304port(portspec, writequeue, textqueue, dataqueue, graphicsqueue, runflag, busyflag, errorflag, debug=False ):

    # ----------------------------------------------
    def sighandler( a, b ):
        print( "readerwriter sighandler" )
        runflag.value = 0
            
    signal.signal(signal.SIGINT, sighandler)    
    
    # ----------------------------------------------
    write_get   = writequeue.get        
    write_empty = writequeue.empty
    
    text_put     = textqueue.put
    data_put     = dataqueue.put
    graphics_put = graphicsqueue.put
    
    runflag_value = runflag.value
    busyflag_value = busyflag.value
    errorflag_value = errorflag.value

    #local parameters
    datalength = 0
    pixelpitch = 0
    darkstart = 0    
    darklength = 0
    darkstop = darkstart + darklength
    invert = False
    sensor = None
    scale_offset = 0
    scale_range = 0
    bits = 16
    vfs = 3.3
    vperbit = vfs/(2**16-1)

    # ----------------------------------------------
    serialport = serial.Serial( portspec, timeout=1., write_timeout=1. )
    if serialport is None:
        runflag = 1
        errorflag = 1
        sleep(0.1)
        sys.exit()
        
    read_until = serialport.read_until
    readline = serialport.readline

    # ----------------------------------------------
    dataframe = TCD1304DATAFRAME()
    
    while runflag:

        # write whatever is waiting to be written
        while  not write_empty():

            line = write_get().strip() + '\n'
            if debug:
                print( "TCD1304port sending", line)

            serialport.write(line.encode())
        
        # Check bytes waiting
        waiting = serialport.in_waiting
        if waiting > 0:

            try:
                line  = readline()
                line = line.decode('utf-8', errors='ignore').strip()

                p = line.split()
                
                if debug:
                    print( "TCD1304port read", line)

                if len(p) == 0:
                    continue

                # --------------------------------------------------------
                # We process data here
                if p[0] == "BINARY16" :
                    ndata = int(p[1])
                    nbytes = 2*ndata

                    # Read the data until we have all of the bytes
                    data    = serialport.read(nbytes)
                    nbytes -= len(data)
                    while nbytes > 0:
                        nextdata = serialport.read(nbytes)
                        nbytes  -= len(nextdata)
                        data.append(nextdata)

                    timestamp = datetime.now()

                    # Read the expected end of data message
                    line  = readline()
                    line = line.decode('utf-8', errors='ignore').strip()

                    if  line.startswith( "END" ):

                        data = struct.unpack( '<%dH'%(len(data)/2), data )
                        # data = np.array(data, dtype=np.int64)
                        data = np.array(data)

                        dataframe.data = [data]
                        dataframe.timestamp = timestamp

                        #print("raw data type", type(dataframe.data[0]), np.dtype(dataframe.data[0][0]))
                        
                        # We put the raw data on the queue
                        data_put(dataframe)

                        # Prepare a record for the graphical display
                        text = ""
                        if dataframe.frame_mode is not None:
                            text += dataframe.frame_mode
                        text += '\ncounters: ' + str(dataframe.frame_counter) + '/' + str(dataframe.frame_counts)
                        text += ' ' + str(dataframe.frameset_counter) + str(dataframe.frameset_counts)
                        text += '\n' + dataframe.timestamp.strftime('%Y-%m-%d.%H%M%S.%f')

                        if debug:
                            print("putting graphics", dataframe.data[0].shape, text )

                        data = np.copy(dataframe.data)
                            
                        if vperbit:
                            data = data * vperbit

                        if scale_offset:
                            data = data - scale_offset

                        graphics_put( [[data], text] )

                        
                    else:
                        print("binary16 transfer missing end")

                    # New data frame (the old one is referenced from the queue!!!)
                    dataframe = TCD1304DATAFRAME(dataframe)
                    
                # ---------------------------------
                elif p[0] == "BINARY32":
                    ndata = int(p[1])
                    nbytes = 4*ndata

                    # Read the data until we have all of the bytes
                    data    = serialport.read(nbytes)
                    nbytes -= len(data)
                    while nbytes > 0:
                        nextdata = serialport.read(nbytes)
                        nbytes  -= len(nextdata)
                        data.append(nextdata)

                    timestamp = datetime.now()

                    # Read the expected end of data message
                    line  = readline()
                    line = line.decode('utf-8', errors='ignore').strip()

                    if  line.startswith( "END" ):

                        data = struct.unpack( '<%dI'%(len(data)/4), data )
                        # data = np.array(data, dtype=np.int64)
                        data = np.array(data)

                        dataframe.data = data
                        dataframe.timestamp = timestamp

                        # We put the raw data on the queue
                        print( "putting frame counter", dataframe.frame_counter, dataframe.frame_elapsed)
                        data_put(dataframe)

                        # Prepare a record for the graphical display
                        text = ""
                        if dataframe.frame_mode is not None:
                            text += dataframe.frame_mode
                        text += '\ncounters: ' + str(dataframe.frame_counter) + '/' + str(dataframe.frame_counts)
                        text += ' ' + str(dataframe.frameset_counter) + str(dataframe.frameset_counts)
                        text += '\n' + dataframe.timestamp.strftime('%Y-%m-%d.%H%M%S.%f')

                        if vperbit:
                            data = data * vperbit

                        if scale_offset:
                            data = data - scale_offset
                        
                        graphics_put( [dataframe.data, text] )

                    else:
                        print("binary32 transfer missing end")
                        
                    # New data frame (the old one is referenced from the queue!!!)
                    dataframe = TCD1304DATAFRAME(dataframe)
                    
                # ---------------------------------
                elif p[0] == "DATA":
                    ndata = int(p[1])

                    # Read the actual text format data buffer(s)
                    data_buffers = []
                    while len(data_buffers) < ndata:
                        data_buffers.append( ser.readline( ) )

                    timestamp = datetime.now()

                    # Read the expected end of data message
                    endbuffer = serialport.read_until( )

                    line  = readline()
                    line = line.decode('utf-8', errors='ignore').strip()

                    if line.startswith( "END" ):

                        data = []
                        for buffer in data_buffers:
                            buffer = buffer.decode('utf-8', errors='ignore').strip()
                            data.append( int(buffer) )

                        dataframe.data = np.array(data)
                        dataframe.timestamp = timestamp

                        # We put the raw data onto the queue
                        print( "putting frame counter", dataframe.frame_counter, dataframe.frame_elapsed)
                        data_put(dataframe)

                        # Prepare a record for the graphical display
                        text = ""
                        if dataframe.frame_mode is not None:
                            text += dataframe.frame_mode
                        text += '\ncounters: ' + str(dataframe.frame_counter) + '/' + str(dataframe.frame_counts)
                        text += ' ' + str(dataframe.frameset_counter) + str(dataframe.frameset_counts)
                        text += '\n' + dataframe.timestamp.strftime('%Y-%m-%d.%H%M%S.%f')
                        
                        graphics_put( [dataframe.data, text] )

                    else:
                        print("ascii data transfer missing end")

                    # New data frame (the old one is referenced from the queue!!!)
                    dataframe = TCD1304DATAFRAME(dataframe)
                    
                # ---------------------------------------                
                # Mode keywords
                elif p[0] == "START":

                    # start a new one
                    #dataframe = TCD1304DATAFRAME()
                
                    dataframe.frame_mode = p[1]

                    busyflag.value = 1

                # --------------------------------
                elif p[0] == "FRAMESET":

                    if p[0] == "START":

                        dataframe.frameset_complete = False

                    elif p[0] == "END":

                        dataframe.frameset_complete = True

                    elif p[1] == "COUNTER":

                        dataframe.frameset_counter = parseInt(p[2])

                    elif p[1] == "COUNTS":

                        dataframe.frameset_counts = parseInt(p[2])

                # --------------------------------
                elif p[0] == "FRAME":

                    if p[1] == "COUNTER":

                        dataframe.frame_counter = parseInt(p[2])

                    elif p[1] == "COUNTS":

                        dataframe.frame_counts = parseInt(p[2])

                    elif p[1] == "ELAPSED":

                        dataframe.frame_elapsed = parseFlt(p[2])

                    elif p[1] == "EXPOSURE":

                        dataframe.frame_exposure = parseFlt(p[2])

                # --------------------------------
                elif p[0] == "SH":
                    
                    if p[1] == "PERIOD":

                        dataframe.sh_period = parseFlt(p[2])
                        
                # --------------------------------
                elif p[0] == "TIMER":

                    if p[1] == "PERIOD":

                        dataframe.timer_period = parseFlt(p[2])

                    elif p[1] == "SUBPERIOD":

                        dataframe.timer_subperiod = parseFlt(p[2])

                    elif p[1] == "ELAPSED":

                        dataframe.timer_elapsed = parseFlt(p[2])

                    elif p[1] == "DIFFERENCE":

                        dataframe.timer_difference = parseFlt(p[2])

                # --------------------------------
                elif p[0] == "TRIGGER":

                    if p[1] == "COUNTER":

                        dataframe.trigger_counter = parseInt(p[2])

                    elif p[1] == "COUNTS":

                        dataframe.trigger_counts = parseInt(p[2])

                    elif p[1] == "ELAPSED":

                        dataframe.trigger_elapsed = parseFlt(p[2])

                    elif p[1] == "DIFFERENCE":

                        dataframe.trigger_difference = parseFlt(p[2])

                # --------------------------------
                elif p[0] == "ICG":

                    if p[1] == "ELAPSED":

                        dataframe.icg_elapsed = parseFlt(p[2])

                # --------------------------------
                elif p[0] == "OFFSET":

                    dataframe.offset = parseFlt(p[1])
                    
                elif p[0] == "RAWOFFSET":

                    dataframe.rawoffset = parseFlt(p[1])
                    
                # --------------------------------
                elif p[0] == "ACCUMULATING":
                    dataframe.is_accumulating = True

                elif p[0] == "ACCUMULATOR":

                    dataframe.accumulator_counter = sef.partInt(p[1])

                # ---------------------------------------
                elif p[0] == "ADC":

                    dataframe.adc = line

                    text_put(line)

                elif p[0] == "CHIPTEMPERATURE":

                    dataframe.chip_temperature = line

                    text_put(line)

                elif p[0] == "COMPLETE":

                    # start a new dataframe
                    dataframe = TCD1304DATAFRAME()
                
                    busyflag.value = 0

                elif p[0] == "Error":

                    dataframe.error = line
                    
                    errorflag.value += 1

                    text_put( line )


                elif p[0] == "PIXELS":

                    # local copy of the configurawtion

                    datalength = key_in_list( p, "PIXELS", int )
                    pixelpitch = 8.0E-3
                    darkstart = 0    
                    darklength = key_in_list( p, "DARK", int )
                    invert = key_in_list( p, "INVERT", float )
                    sensor = key_in_list( p, "SENSOR", str )
                    scale_offset = key_in_list( p, "OFFSET", float )
                    scale_range = key_in_list( p, "RANGE", float )
                    bits = key_in_list( p, "BITS", int )
                    vfs = key_in_list( p, "VFS", float )
                    vperbit = key_in_list( p, "VPERBIT", float )

                    if vfs and bits:
                        vperbit = vfs/(2**bits - 1)

                    text_put(line)

                else:

                    if debug:
                        print( "TCD1304port put", line)

                    text_put(line)

            except KeyboardInterrupt:
                break

            except Exception as e:
                print(f"SerialPort Read error: {e}")
                print(traceback.format_exc())
                line = None



    sys.exit()
            
                
# ======================================================================================================================
# This is the class for the instrument

class TCD1304CONTROLLER:

    _ids = count(0)
    
    def __init__( self, portspec, readtimeout=1., writetimeout=1., monitor=True, graphics=True,
                  graph_by_pixels=False, graph_by_mm=False, xrange=None, yrange=None, graph_ylabel='spectrum',
                  coefficients=None, gui=False,
                  graph_xlabel='wavelength',
                  debug=False ):

        
        if gui and not has_GUIWindow:
            raise ValueError( "GUI requested, but GUIWindow.py not loaded" )
            
        if graphics and not has_GraphicsWindow:
            raise ValueError( "Graphics requested, but GraphicsWindow.py not loaded" )
            
        if monitor and not has_TextWindow:
            raise ValueError( "Monitor requested, but TextWindow.py not loaded." )
            
        # ------------------------------------------------------------------
        self.ser = serial.Serial( portspec, timeout=readtimeout, write_timeout=writetimeout )

        self.instance = next( self._ids )
        
        self.name= portspec

        # used by the reader thread
        self.readqueue = Queue()
        self.writequeue = Queue()
        self.graphicsqueue = Queue()

        self.textqueue = Queue()
        self.dataqueue = Queue()
        
        self.monitorthread = None
        self.monitorWindow = None
        self.monitorqueue = Queue()
        
        self.GrapichsWindow = None
        self.xdata = None

        self.flag = Value( 'i', 1 )
        self.busyflag = Value( 'i', 0 )
                
        self.pulsetimingdevice = None
        
        # Control data processing in reader
        self.baselineflag = Value( 'i', 1 )

        # Report errors back
        self.errorflag = Value( 'i', 0 )

        self.bits = 12
        self.vfs = 3.3
        self.vperbit = self.vfs/(2**self.bits - 1)
        
        # --------------------------------
        self.filesuffix = ".tcd1304"

        self.identifier = None
        self.coefficients = []
        self.units = None

        self.graph_by_pixels = graph_by_pixels

        # ---------------------------------

        self.debug = debug
        
        # ---------------------------------
        self.readerthread = Process( target = TCD1304port,
                                     args=(
                                         portspec,
                                         self.writequeue,
                                         self.textqueue,
                                         self.dataqueue,
                                         self.graphicsqueue,
                                         self.flag,
                                         self.busyflag,
                                         self.errorflag,
                                         self.debug)
                                    )
        self.readerthread.start()

        sleep(0.1)
        print("readerthread started")
        
        # ---------------------------------
        # First, stop
        buffer = self.command('stop')
        print('stop command', buffer)
        
        # ---------------------------------
        # Query for Identifier        
        buffer = self.command('identifier')
        print(buffer)
        
        # Query for Version
        buffer = self.command( 'version' )
        print(buffer)
        
        # ---------------------------------
        # Query for the configuration
        buffer = self.command( 'configuration' )
        print(buffer)

        # ----------------------------------------
        # Query for coefficients        
        if coefficients is not None:
            print( 'using specified coefficients', coefficients )
            self.coefficients = coefficients
            self.xdata = generate_x_vector( self.datalength, self.coefficients )            
            self.xlabel = graph_xlabel
            
        else:
            buffer = self.command('coefficients')
            print(buffer)

            buffer = self.command('units')[0]
            print(buffer)

            if self.units == "nm":
                self.xlabel = "Wavelength"
            else:
                self.xlabel = "Position"

        # ---------------------------------
        if xrange is None:
            xrange = (self.xdata[0],self.xdata[-1])
        if yrange is None:
            yrange = (-self.scale_range/20,self.scale_range)
            
        if gui:
            # We will want to query these from the device
            self.GraphicsWindow = GUIWindow( "TCD1304 Controller Data " + portspec,
                                             xdata = self.xdata,
                                             xlabel = self.xlabel,
                                             ycols = [ np.zeros(self.datalength) ],
                                             yrange = yrange,
                                             ylabels = [graph_ylabel],
                                             queue = self.graphicsqueue,
                                             flag = self.flag,
                                             parentinstance = self,
                                             filespec = os.path.join( 'datafile', self.filesuffix ),
                                             debug = self.debug )
            self.GraphicsWindow.start( )

        elif graphics:
            # We will want to query these from the device
            self.GraphicsWindow = GraphicsWindow( "TCD1304 Controller Data " + portspec,
                                                  xdata = self.xdata,
                                                  xlabel = self.xlabel,
                                                  ycols = [ np.zeros(self.datalength) ],
                                                  yrange = yrange,
                                                  ylabels = [graph_ylabel],
                                                  queue = self.graphicsqueue,
                                                  flag = self.flag,
                                                  debug = self.debug )
            self.GraphicsWindow.start( )
        
        if monitor:
            self.monitorthread = Process( target = self.textmonitor, args=(portspec, self.flag ) )
            self.monitorthread.start()


        atexit.register(self.exit, None )

    # ===========================================
    def close( self, ignored=None ):

        self.flag.value = 0
        sleep( 0.1 )
        #self.ser.reset_input_buffer()
        #self.ser.close()

        self.readerthread.terminate()        
        self.readerthread.join( )

        if self.monitorthread:
            self.monitorthread.terminate()
            self.monitorthread.join()

        if self.GraphicsWindow:
            self.GraphicsWindow.close()

    def exit( self, ignored=None ):
        print( self.name + ' exit()' )
        self.write( 'stop' )
        self.close()

    # ======================================================================================
    def parseparameters(self,line):

        if not line or line is None:
            return False
        
        line=line.strip()
        if len(line) == 0:
            return False

        if line.startswith("Identifier"):
            self.__dict__["identifier"] = line.split(maxsplit=1)[1]
            print("Identifier", self.identifier)
            return True

        if line.startswith("TCD1304Device vers"):
            self.__dict__["version"] = line
            print("Version", self.identifier)
            return True

        if line.startswith("PIXELS"):

            parts = line.split()

            self.datalength = key_in_list( parts, "PIXELS", int )
            self.pixelpitch = 8.0E-3
            self.darkstart = 0    
            self.darklength = key_in_list( parts, "DARK", int )
            self.invert = key_in_list( parts, "INVERT", float )
            self.sensor = key_in_list( parts, "SENSOR", str )
            self.scale_offset = key_in_list( parts, "OFFSET", float )
            self.scale_range = key_in_list( parts, "RANGE", float )
            self.bits = key_in_list( parts, "BITS", int )
            self.vfs = key_in_list( parts, "VFS", float )
            self.vperbit = key_in_list( parts, "VPERBIT", float )

            if self.vfs and self.bits:
                self.vperbit = self.vfs/(2**self.bits - 1)
                print( "BITS", self.bits, "VFS", self.vfs, "Vperbit", self.vperbit )

            if self.version == "T4LCD vers 0.3":
                print( "has early version datastart 12 instead of 16")
                self.darkstart = 4

            if self.scale_offset is None:
                self.scale_offset = 0.

            if self.scale_range is None:
                self.scale_range = self.vfs

            print( "pixels", self.datalength,
                   "dark", self.darklength,
                   "invert", self.invert,
                   "vperbit", self.vperbit,
                   "offset", self.scale_offset,
                   "range", self.scale_range,
                   "sensor", self.sensor )

            return True

        elif line.startswith("units"):
            parts = line.split(maxsplit=1)
            if len(parts) > 1:
                self.units = parts[1]
            print("units", self.units)
            return True

        elif line.startswith("coefficients"):

            print( "parsing coefficients", line)

            parts = line.split()
            if len(parts) < 3:
                print( "coefficients line is blank")
            
            elif 'nan' in line:
                self.coefficients = [ 0., 1., 0., 0. ]
                print( "coefficients", self.coefficients)
                self.xdata = generate_x_vector( self.datalength, None )
                return True

            else:
                self.coefficients = [ a for a in map( float, parts[1:] ) ]
                print( "coefficients", self.coefficients)
                self.xdata = generate_x_vector( self.datalength, self.coefficients )
                return True

        elif line.startswith('flexpwm:'):

            pars=line.split(maxsplit=3)
            if len(pars)==4:
                parname="flexpwm_"+pars[1]+"_"+pars[2]
                self.__dict__[parname]=pars[3]
                return True

            elif len(pars)==3:
                parname="flexpwm_"+pars[1]
                try:
                    self.__dict__[parname]=int(pars[2])
                except:
                    self.__dict__[parname]=pars[2]
                return True
            else:
                print("flexpwm line not processed: ", line)
                return False

        return False

    # --------------------------------------------------------------------------------------
    def write(self,line):
        print("sending:", line)
        self.writequeue.put(line)

    def read(self, untildone=True, timeout=1,parsepars=True):
        if untildone:
            responses = []
            while True:
                try:
                    buffer = self.textqueue.get(timeout=timeout)
                    if buffer.startswith("DONE"):
                        return responses
                    self.parseparameters(buffer)
                    responses.append(buffer)
                except Empty:
                    #print("read timeout")
                    return [None]
        else:
            try:
                buffer = self.textqueue.get(timeout=timeout)
                self.parseparameters(buffer)
                return buffer
            except Empty:
                #print("read timeout")
                return [None]

        return [None]

    def command(self,line,untildone=True,timeout=1,parsepars=True):
        self.cleartextqueue()
        
        self.writequeue.put(line)
        sleep(0.1)
        return self.read(untildone,timeout)
        

    # --------------------------------------------
    def busy(self):
        return self.busyflag.value


    # Wait for completion of data frames
    def wait(self, timeout=None, interruptible=False ):

        if timeout is None:
            while self.busyflag.value:
                if interruptible and input_ready():
                    return False
                sleep( 0.2 )
            return True
        
        try:
            timeout = float(timeout)
        except:
            print( "not valid timeout value", timeout )
            return False
            
        while self.busyflag.value:
            if interruptible and input_ready():
                return False
            elif timeout > 0.:
                sleep( 0.2 )
                timeout -= 0.2
            else:
                return False

        return True
        
    def checkerrors( self ):

        counter = self.errorflag.value
        self.errorflag.value = 0

        return counter
    
    # --------------------------------------------
    def cleartextqueue(self):
        lines = []
        n = 0
        while not self.textqueue.empty():
            line = self.textqueue.get()
            print("clear got line", line)
            self.parseparameters(line)
            lines.append(line)
            n += 1
            while not self.textqueue.empty():
                line = self.textqueue.get()
                print("clear got line", line)
                self.parseparameters(line)
                lines.append(line)
                n += 1
            sleep(0.1)
        #print("cleared ", n, " textqueue entries")

        return lines
    
    def cleardataqueue(self):
        records = []
        n = 0
        while not self.dataqueue.empty():
            record = self.dataqueue.get()
            records.append(record)
            n += 1
            while not self.dataqueue.empty():
                record = self.dataqueue.get()
                records.append(record)
                n += 1
            sleep(0.1)
        #print("cleared ", n, " dataqueue entries")
        return records
        
    def clear( self ):

        print("clearing text and data queues")

        n = 0
        while not self.textqueue.empty():
            self.textqueue.get()
            n += 1
            while not self.textqueue.empty():
                self.textqueue.get()
                n += 1
            sleep(0.1)
        print("cleared ", n, " textqueue entries")

        n = 0
        while not self.dataqueue.empty():
            self.dataqueue.get()
            n += 1
            while not self.dataqueue.empty():
                self.dataqueue.get()
                n += 1
            sleep(0.1)
        print("cleared ", n, " dataqueue entries")

        self.busyflag.value = 0

        #self.writeread("clear accumulator")

        return True
            
    # ===========================================
    def textmonitor( self, name, flag ):

        print( "text monitor started ", name )

        self.monitorWindow = TextWindow("LCCDSpectrometer Log " + name)
        print( self.monitorWindow )

        def on_closing():
            print( 'closed' )
            flag.value = 0
        #self.monitorWindow.parent.protocol("WM_DELETE_WINDOW", on_closing)
        self.monitorWindow.parent.protocol("WM_DELETE_WINDOW", self.monitorWindow.parent.iconify)            
        
        def sighandler( a, b ):
            flag.value = 0
        signal.signal(signal.SIGINT, sighandler)

        while flag.value:
            try:
                line = self.monitorqueue.get(block=False)
                self.monitorWindow.addline_( line )
            except Empty:
                pass
            self.monitorWindow.update()
            sleep(0.1)
        self.monitorWindow.close()

    # =============================================================================================
    # enqueue frame to graphics
    def enqueue_dataframes(self, dataframe):

        if type(dataframe) is list:
            print("is list")
            for f in dataframe:
                self.enqueue_dataframes(f)
            return True

        else:
            text = ""
            text += dataframe.frame_mode
            text += '\ncounters: ' + str(dataframe.frame_counter) + '/' + str(dataframe.frame_counts)
            text += ' ' + str(dataframe.frameset_counter) + str(dataframe.frameset_counts)
            text += '\n' + dataframe.timestamp.strftime('%Y-%m-%d.%H%M%S.%f')

            data = np.copy(dataframe.data)
            
            if self.debug:
                print("putting graphics", dataframe.data[0].shape, text )

            if self.vperbit:
                data = data * self.vperbit

            if self.scale_offset:
                data = data - self.scale_offset

            if dataframe.accumulator_counter > 1:
                data /= dataframe.accumulator_counter
                text += '\n accumulator counter ' + str(dataframe.accumulator_counter)

            self.graphicsqueue.put([[data], text])

            return True
            
    # =============================================================================================
    # Condense all of the acquired frames into a single frame
    def addall(self,framerecords=None, after=1):

        # Fetch everything that is on the queue
        if framerecords is None:
            framerecords = self.cleardataqueue()

        # no records, return empty list
        if framerecords is None:
            return None

        # only one record, put it back
        if len(framerecords) == 1 :
            self.dataqueue.put(framerecords[0])
            return framerecords[0]

        # keep only frames counters after the "after"
        frame_counters = set([ f.frame_counter for f in framerecords ])
        frame_counters = list(frame_counters)
        if len(frame_counters) > 1:
            temp = []
            for f in framerecords:
                if f.frame_counter >= after:
                    temp.append(f)                    
            framerecords = temp

        # if there are no such frames, do nothing
        if len(framerecords) < 1 :
            return []
        
        # if there is only frame, don't add
        if len(framerecords) == 1 :
            self.dataqueue.put(framerecords[0])
            return framerecords[0]

        # Here is the loop where we add frames, accumulate into the new frame
        oldframe = framerecords[0]
        for newframe in framerecords[1:]:
            # for the new frame, make sure we have at least one for accumulator counts
            newframe.accumulator_counter = max(newframe.accumulator_counter,1)

            # Add  the previous data
            newdata = [ np.add(newcol,oldcol) for newcol,oldcol in zip(newframe.data,oldframe.data) ]
            newframe.data = newdata

            # Add from previous offsets, exposure, accumulator counts
            newframe.offset += oldframe.offset
            newframe.rawoffset += oldframe.rawoffset
            newframe.frame_exposure += oldframe.frame_exposure
            newframe.accumulator_counter += oldframe.accumulator_counter

            oldframe = newframe

            print("accumulator_counter", newframe.accumulator_counter)

        # Put the result back onto the queue
        print("final accumulator_counter", newframe.accumulator_counter)
        self.dataqueue.put(newframe)
        
        return newframe
        
    # =============================================================================================
    # Condense all of the acquired framesets into a single frameset
    def addframesets(self,framerecords=None):

        # Fetch everything that is on the queue
        if framerecords is None:
            framerecords = self.cleardataqueue()

        if not len(framerecords) :
            return []

        if self.debug:
            for record in framerecords:
                print("frame_counter", record.frame_counter, record.frame_counts,
                      "frameset_counter", record.frameset_counter, record.frameset_counts)
        
        # Verify that it all has the same setting for frame counts
        frame_counts = set([ f.frame_counts for f in framerecords ])
        frame_counts = list(frame_counts)
        if len(frame_counts) > 1:
            if self.debug:
                print("Error: cannot add framesets with different lengths")
            for r in framerecords:
                self.dataqueue.put(r)
            return []
        print( "framecounts", frame_counts)
        frame_counts = frame_counts[0]

        # Verify that all of the frame counters are within the setting for frame counts
        frame_counters = set([ f.frame_counter for f in framerecords ])
        frame_counters = list(frame_counters)
        if max(frame_counters) > frame_counts:
            if self.debug:
                print("Error: cannot add framesets with frame counters greater than length of the frameset")
            for r in framerecords:
                self.dataqueue.put(r)
            return []
                
        # Now we do the accumulation, continue in the same basis wether binary or volts
        #  Reading BINARY16 seems to produce np.int64 by default.   Keep alert to this and
        #  consider whether to edit TCD1304PORT to enforce 64 bits, data = np.array(data,dtype=np.int64)

        working_frames = [None] * frame_counts

        for newframe in framerecords:
            #index by frame_counter
            n = newframe.frame_counter
            oldframe = working_frames[n]
            if self.debug:
                print( "adding to working_frames at", n)

            # for the new frame, make sure we have at least one for accumulator counts
            newframe.accumulator_counter = max(newframe.accumulator_counter,1)

            # If this is not the first frame in this slot, then
            #   add the previous data, offsets, exposure and accumulation counter
            if oldframe is not None:
                # Add  the previous data
                newdata = [ np.add(newcol,oldcol) for newcol,oldcol in zip(newframe.data,oldframe.data) ]
                newframe.data      = newdata
                # Add from previous offsets, exposure, accumulator counts
                newframe.offset   += oldframe.offset
                newframe.rawoffset += oldframe.rawoffset
                newframe.frame_exposure  += oldframe.frame_exposure
                newframe.accumulator_counter += oldframe.accumulator_counter

            # And store as the new working freame
            working_frames[n] = newframe

        # Put the reduced set back onto the queue, no further processing
        # The new total represents the toral photon number of the accumulated exposure
        for frame in working_frames:
            if self.debug:
                print("putting frame_counter",
                      frame.frame_counter, frame.frame_counts,
                      "frameset_counter", frame.frameset_counter, frame.frameset_counts,
                      "accumulator counter", frame.accumulator_counter)
            self.dataqueue.put(frame)

        return working_frames
            
    # =============================================================================================
    def savetofile( self, file, timestamp=None, comments = None, write_ascii=True, framesetindex = None, records = None ):

        def formattedwrites( file, keys, vals, exclude=None ):
            for key,val in zip( keys,vals ):

                if exclude is not None:
                    if key in exclude:
                        continue

                key = key.strip()

                if key == "data":
                    continue
                    
                if type(val) in [ float, int ] :
                    file.write( '# ' + key + ' = ' + str(val) + '\n' )

                elif type(val) in [ str ] :
                    file.write( '# ' + key + ' = "' + str(val.strip()) + '"\n' )

                elif type(val) in [ list ] :
                    if not len(val):
                        file.write( '# ' + key + ' = []\n' )
                    else:
                        values = val
                        if type(values[0]) in [ float, int ] :
                            file.write( '# ' + key + ' = [ ' + ', '.join( [ str(v) for v in values ] ) + ' ]\n'  )
                        elif type(values[0]) in [ str ] :
                            file.write( '# ' + key + ' = [ "' + '", "'.join( [ str(v) for v in values ] ) + '" ]\n'  )
                            

        def writedictionary( file, d, exclude=None):
            keys, vals = zip(*d.items())
            formattedwrites( file, keys, vals, exclude )

        def writeclass( file, c, exclude=None):
            keys, vals = zip(*c.__dict__.items())
            formattedwrites( file, keys, vals, exclude )
                            
        # -------------------------
        
        newfile = False

        if timestamp is None:
            timestamp = datetime.now()
        
        if type(file) is str:

            if '%' in file:
                try:
                    exec( "file="+file, self.__dict__, globals() )
                    print(file)
                except Exception as e:
                    print( e )
                    return False

            if not file.endswith( self.filesuffix ):
                file += "." + timestamp.strftime('%Y%m%d.%H%M%S.%f') + self.filesuffix

            dirspec = os.path.dirname(file)
            if len(dirspec):
                if os.path.isdir(dirspec):
                    print( 'saving to directory', dirspec )
                else:
                    print( 'creating directory', dirspec )
                    try:
                        os.makedirs(dirspec)
                    except Exception as e:
                        print( e )
                        return False
                
            try:
                file = open(file,"x")
                print( 'saveto', file )
                newfile = True
            except Exception as e:
                print( "open " + file )
                print( e )
                return False
            
        # ------------------------------------------------
            
        file.write( '# TCD1304DeviceMP.py version %s\n'% __version__ )

        file.write( '# ' + timestamp.strftime('%Y-%m-%d %H:%M:%S.%f') + '\n' )

        if self.identifier is not None:
            s = self.identifier
            s = s.replace('\r', '')
            s = s.strip()
            file.write( '# identifier = "' + s + '"\n' )

        # Save all of the numerical and string values in the header portion of the class
        writeclass(file,self,exclude=['identifier','filesuffix'])
        
        if comments is not None:
            if type(comments) not in [list,tuple]:
                comments = [comments]

            for c in comments:
                file.write( '# comment = "' + c + '"\n' )

    
        file.write( '# header end\n' )

        if records is None:
            records = self.cleardataqueue()

        print( 'writing', len(records), 'records' )

        for record in records:
            print("record frame_counter", record.frame_counter, record.frame_elapsed)
            
        wroterecord = False
        for record in records:

            # If starting the next record, start with two blank lines
            if wroterecord:
                file.write( "\n" )
                file.write( "\n" )
                wroterecord = False

            # ----------------------------
            if type(record) is str:
                file.write( '# ' + record.strip() + '\n' )

            elif isinstance(record,TCD1304DATAFRAME):
                
                writeclass(file, record, exclude=['data'] )

                try:
                    ycols = record.data
                    if len(ycols):
                        datatypestring = str(ycols[0].dtype)
                        file.write( "# DATA %s %d COLS %d\n"%(datatypestring,len(ycols[0]),len(ycols)) )
                        if len(ycols) > 1:
                            yrows = np.array(ycols).T
                            for row in yrows:
                                file.write( ' '.join([str(r) for r in row]) )
                        else:
                            for y in ycols[0]:
                                file.write( str(y) + '\n' )
                        file.write( "# END DATA\n" )
                        wroterecord = True
                except Exception as e:
                    print("error writing data", e)

        if newfile:
            file.close()
                  
    # =====================================================================
    def commandlineprocessor( self, line, fileprefix=None ):

        if not line or not len(line):
            return True
        
        # String ubstitutions
        if not line.startswith('for '):
            
            if line[0] == '"' and line[-1] ==')' and line.count('"') == 2:
                if self.debug:
                    print("string subst for command line:", line)
                try:
                    exec( "res="+line, self.__dict__, globals() )
                    line=res
                except Exception as e:
                    print("failed string substitution", e)
                    print(line)
                    return False
        
            
            elif '%(' in line:
                if self.debug:
                    print("command line:", line)
                parts = list( split_bracketed( line ) )
                for n,p in enumerate(parts):
                    if p.startswith('"') and '"%(' in p:
                        try:
                            exec( "res="+p, self.__dict__, globals() )
                            parts[n] =res                    
                            line = ' '.join( parts )
                            #print( "command with string substitution:", line )
                        except Exception as e:
                            print("cli string subst:", p, e)
                            return
                if self.debug:
                    print("command line =>", line)
        
        if self.monitorqueue:
            self.monitorqueue.put( 'command: ' + line + '\n')

            
        
        if line.startswith("help"):

            response = self.command( line, parsepars=False )
            
            for r in response:
                r = r.rstrip()
                if r[0] == '#':
                    r = r[1:]
                print(r)

            if len(line.split()) == 1:
                print( "  " )

                print( "Commands implemented in the CLI/host computer:" )
                print( "   h|help                        - produces this help text" )
                print( "" )
                print( "   Process frames from the data queue" )
                print( "     add all [after n]           - leaves one frame for save, (option skip first n frames)"  )
                print( "     add frameset                - leaves one frameset for save"  )
                print( "" )
                print( "     clear                       - empty the data and text queues" )
                print( "" )
                print( "     save fileprefix comments... - save contents of data queue to diskfile" )
                print( "" )
                print( "  Wait for data collection" )
                print( "    wait                         - wait for \"complete\" message from sensor" )
                print( "    wait read | trigger          - for the tcd1304 firmware wait functions" )
                print( "    wait                         - wait for \"complete\" message from sensor" )
                print( "" )
                print( "   baseline on | off             - turn baseline correction on/off" )
                print( "" )
                print( "   tcd1304 ....                  - pass command to the tcd1304" )
                print( "" )
                print( "   Pass command to operation system shell" )
                print( "     !command                  - execute shell command" )
                print( "" )
                print( "   Pass instruction to python intepretor" )
                print( "     a = 3                     - '=' causes evaluation as python" )
                print( "     = python statement        -  pass to python interprator" )
                print( "      these commands have access to local() and class name spaces" )
                print( "" )
                print( "   Execute commands from text file, parameters appear as a list, batchpars" )
                print( "" )
                print( "     @filespec [parameter list] - read and execute commands from a file" )
                print( "" )
                print( "     Example" )
                print( "" )
                print( "       @testscrpt 1 2" )
                print( "" )
                print( "       testscript:" )
                print( "         =print(batchpars)" )
                print( "" )
                print( "       output:")
                print("           [\'testscript\', \'1\', \'2\']" )
                print( "" )
                print( "   Loops and string substitution, by example" )
                print( "" )
                print( "     for a_ in [0,.1,.2]: @testscript \"%.2f\"%(a_)")
                print( "     for a_ in [0,.1,.2]: for b_ in [ .3,.4,.5]: @testscript \"%.2f\"%(a_) \"%.2f\"%(b_)")
                print( "" )
                print( "      testscript:" )
                print( "         =print(batchpars)" )
                print( "" )
                print( "       output:")
                print("           [\'testscript\', \'0.00\', \'0.30\']" )
                print("           [\'testscript\', \'0.00\', \'0.40\']" )
                print( "          etc" )
                print( "" )
                print( "   q[uit]                      - exit the cli program" )
                print( "" )
            
        elif line.startswith( '#' ):
            #print( "rcvd comment line" )
            print( line )

        elif line.startswith('wait') and \
             not line.startswith("wait read") and \
             not line.startswith("wait trigger"):

            pars = line.split()
            if len(pars) > 3:
                try:
                    if not self.wait( float(pars[2] ), bool(pars[3]) ):
                        return False
                except Exception as e:
                    print( e )
                    return False
            elif len(pars) > 2:
                try:
                    if not self.wait( float(pars[2]), True ):
                        return False
                except Exception as e:
                    print( e )
                    return False
            else:
                if not self.wait( interruptible=True ):
                    return False

        elif line.startswith( 'clear busy'):
            self.busyflag.value = 0
            
        elif line.startswith('clear'):
            self.clear()

        elif self.busyflag.value and (
                line.startswith("read") or
                line.startswith("trigger") or
                line.startswith("flexpwm") or
                line.startswith("start") or
                line.startswith("setup")):
            print( "device is busy, try stop and clear busy")
            return False


        elif line.startswith('add framesets'):
            frames = self.addframesets()
            if frames and len(frames):
                self.enqueue_dataframes(frames)
        
        elif line.startswith('add all after'):
            
            try:
                pars = line.split()            
                zero = int(pars[3])
            except Exception as e:
                print(e)
                return False
            
            print( "adding from", zero)
                
            frame = self.addall(None, zero)
            if frame is not None:
                self.enqueue_dataframes(frame)
            
        elif line.startswith('add all'):
                
            frame = self.addall()
            if frame is not None:
                self.enqueue_dataframes(frame)
            
        elif line.startswith('save'):

            pars = line.split( maxsplit = 2 )

            print( pars )

            if len( pars ) == 1:
                print( 'need filespec [comments]' )

            elif len(pars) == 2:
            
                self.savetofile( pars[1], write_ascii=True )

            elif len(pars) == 3:
            
                self.savetofile( pars[1], comments = pars[2], write_ascii=True )

        elif line.startswith( 'baseline' ):
            if 'off' in line:
                self.baselineflag.value = 0
            elif 'on' in line:
                self.baselineflag.value = 1
            print("baseline processing", self.baselineflag.value)

        elif line.startswith('@'):

            print("")
            print(line)
            
            exec( "batchpars = "+str(line[1:].strip().split()), globals())
            
            batchfile = line[1:].strip().split()[0]

            try:
                with open( batchfile, 'r' ) as f:
                    script = f.readlines()
            except Exception as e:
                print(e)
                return False

            for line in script:
                line = line.strip()
                status = True

                print(batchfile, "=>", line)
                
                for line_ in line.split(';'):

                    if not len(line):
                        continue
                    
                    if line_.startswith("nowait"):
                        try:
                            line_ = line_.split(maxsplit=1)[1]
                        except Exception as e:
                            print("empty line after nowait")
                        
                    elif not line_.startswith("wait") and \
                       not line_.startswith("stop") and \
                       not line_.startswith("clear") and \
                       not line_.startswith("toggle") and \
                       not line_.startswith("set ") and \
                       self.busyflag.value:
                        print("device is busy, need to wait, clear or stop first")
                        status = False
                        return False
                    
                    if not self.commandlineprocessor( line_, fileprefix ):
                        status = False
                        print( "command failed:", line_ )
                        return False
                    
                    if self.debug:
                        print( "command finished:", line_ )
                    
                if not status:
                    return False
                
            if self.debug:
                print("completed",batchfile)

        elif line.startswith('!'):
            result = os.popen( line[1:] ).read()
            if result:
                print( result )

        elif '=' in line:

            if line == '=':
                for key, val in globals().items():
                    print( "global ", key, val )
                for key, val in locals().items():
                    print( "local ",key, val )

            elif line.startswith( '=' ):
                try:
                    exec( line[1:].strip(), self.__dict__, globals() )
                except Exception as e:
                    print( e )
                    return False
                
            else:
                try:
                    exec( line, self.__dict__, globals() )
                except Exception as e:
                    print( e )
                    return False

        elif line.startswith('for') and not line.startswith("format"):

            print("")
            print(line)
            
            try:
                loopspec, line_ = line.split(':',maxsplit=1)
            except Exception as e:
                print( "lccdcontroller cli line:", line )                
                print( "lccdcontroller cli: parse error", e )
                return False
            
            parts = list( split_bracketed( loopspec ) )
            if len( parts ) != 4 or parts[2] != 'in':
                print(  'loopspec not valid', loopspec )
                return False

            exec( "loopvalues = list(" + parts[3] + ")", self.__dict__, globals() )
            print( loopvalues )

            for v in loopvalues:

                exec( parts[1] + " = " + str(v), self.__dict__, globals() )

                for line__ in line_.split(';'):

                    line__ = line__.strip()
                    
                    if line__.startswith( '"' ):
                        try:
                            exec( "line___ = " + line__, self.__dict__, globals() )
                        except Exception as e:
                            print( "command:", line__ )
                            print(e)
                            return False
                        
                        if self.debug:
                            print( "command:", line___ )
                        if not self.commandlineprocessor( line___, fileprefix ):
                            return False                   
                    else:
                        if self.debug:
                            print( "command:", line__ )
                        if not self.commandlineprocessor( line__, fileprefix ):
                            return False

            
        elif line.startswith( 'parse' ):
            try:
                print( list( split_bracketed( line ) ) )
            except Exception as e:
                print( e )

        elif line.startswith('pulsetimingdevice'):
            if self.pulsetimingdevice is None:
                print( "no timing device loaded" )
                return False

            try:
                return self.pulsetimingdevice.commandlineprocessor(line.split(maxsplit=1)[1])
            except:
                print(e)
                return False

        elif line is not None and len(line) > 0:

            if line.startswith("tcd1304") and len(line.split()) > 1:
                line = line.split(maxsplit=1)[1]

            self.command(line)
            
        else:
            # Nothing was sent, read any pending responses
            response = self.cleartextqueue()
            if len(response) :
                for line in response:
                    print(">response:", line )

        if self.checkerrors():
            print( 'checkerrors found errors' )
            return False
        
        return True
        
        
    def commandloop( self, name="CLI", fileprefix=None ):
        
        while self.flag.value:

            line = input( name + ': ' )

            if line.lower() in ['exit', 'quit', 'q' ]:
                break

            count = self.checkerrors()
            if count:
                print( "rcvd %d errors previous to this command"%(count) )
                        
            self.commandlineprocessor( line, fileprefix )

            count = self.checkerrors()
            if count:
                print( "Error detected", count )            

        return
                    
# =========================================================================================================

if __name__ == "__main__":

    mp.set_start_method('spawn')
    
    import argparse

    try:
        import readline
    except:
        pass

    import atexit
    import signal
    
    if platform.system() == 'Linux':
        ser0_default = '/dev/ttyACM0'
        ser1_default = '/dev/ttyACM1'
    elif platform.system() == 'Windows':
        ser0_default = 'COM1:'
        ser1_default = 'COM2:'

    # ---------------------------------------------------------
    def SignalHandler(signal, frame):
        print('Ctrl-C')

        serialdevice.close()
        if dataport is not None:
            dataport.close()
            
        print('Exit')
        sys.exit(0)
    
    # ---------------------------------------------------------
    class ExplicitDefaultsHelpFormatter(argparse.ArgumentDefaultsHelpFormatter):
        def _get_help_string(self, action):
            if action.default in (None, False):
                return action.help
            return super()._get_help_string(action)
    
    parser = argparse.ArgumentParser( description='LCCD Controler Monitor/Cli',
                                      formatter_class=ExplicitDefaultsHelpFormatter )

    parser.add_argument( 'ports', default=[ser0_default,ser1_default], nargs='*',
                         help = 'one or more serial or com ports,' +
                         ' the first is the control port, others are readonly' )

    parser.add_argument( '--historyfile', default = 'lccdcontroller.history', help='history file for the command line interface' )
    parser.add_argument( '--nohistoryfile' )
    
    parser.add_argument( '--pixels', action = 'store_true', help='graph pixel indices on x-axis' )
    
    parser.add_argument( '--guiwindow', action = 'store_true', help='gui version of the graphical display' )
    
    parser.add_argument( '--loggingwindow', action = 'store_true', help='display transactions in a scrolling text window' )

    parser.add_argument( '--raw', action = 'store_true', help='graph raw binary values' )

    parser.add_argument( '--pulsetimingdevice', '--timing', action = 'store_true' )
    
    parser.add_argument( '--debug', action = 'store_true' )

    # for datafile reading
    parser.add_argument( '--xrange', nargs=2, type=float )
    parser.add_argument( '--yrange', nargs=2, type=float )    
    parser.add_argument( '--dump', action = 'store_true' )
    parser.add_argument( '--graph', action = 'store_true' )
    parser.add_argument( '--frame', type=int )
    parser.add_argument( '--x', help = 'xdata or python rexpression' )
    parser.add_argument( '--y', help = 'ydata or python expression' )
    parser.add_argument( '--output' )
    
    args = parser.parse_args()

    # ----------------------------------------------------------
    if os.path.isfile( args.ports[0] ):
        print( 'reading file', args.ports[0] )
        dataobject = LCCDDATA( args.ports[0] )
        if args.dump:
            dataobject.dump()
        if args.graph:
            xdata = dataobject.xdata
            if args.x is not None:
                exec( 'xdata = '+args.x )
            if args.frame is not None:
                frame = dataobject.frames[args.frame]
                ydata = frame.data
                if args.y is not None:
                    exec( 'ydata = '+args.y )
                plt.plot( xdata, ydata )
            else:
                for n, frame in enumerate(dataobject.frames):
                    ydata = frame.data
                    if args.y is not None:
                        exec( 'ydata = '+args.y )
                    plt.plot( xdata, ydata,label=str(frame.offset))
                plt.legend(title=dataobject.mode+', t(secs)')

            if args.output is not None:
                plt.savefig( args.output )
            else:
                plt.show()
        
        quit()
        
    # ---------------------------------------------------------
    serialdevice = TCD1304CONTROLLER( args.ports[0],
                                      monitor=args.loggingwindow,
                                      graph_by_pixels=args.pixels,
                                      gui=args.guiwindow,
                                      debug=args.debug )
    
    dataport = None

    '''
    if len(args.ports) > 1:
        dataport = TCD1304CONTROLLER( args.ports[1] )
    '''

    if args.pulsetimingdevice:
        serialdevice.pulsetimingdevice = PULSETIMINGDEVICE( args.ports[1] )
        
    # ---------------------------------------------------------
    if not args.nohistoryfile:
        try:
            readline.read_history_file(args.historyfile)
        except Exception as e:
            print('historyfile: ', e)
            print('continuing')
            
        atexit.register(readline.write_history_file, args.historyfile)

    signal.signal(signal.SIGINT, SignalHandler)
    
    # ---------------------------------------------------------

    serialdevice.commandloop( name="TCD1304 CLI", fileprefix=None )

    serialdevice.close()

#!/usr/bin/python

"""
LCCDController.py

Mitchell C. Nelson (c) 2023

November 26, 2023

Derived from LLCDController.py Copyright 2022 by M C Nelson
Derived from TDAQSerial.py, TCD1304Serial.py, Copyrigh 2021 by M C Nelson

"""

__author__    = "Mitchell C. Nelson, PhD"
__copyright__ = "Copyright 2023, Mitchell C, Nelson"
__version__   = "0.3"
__email__     = "drmcnelson@gmail.com"
__status__    = "alpha testing"

__all__ = [ 'LCCDFRAME', 'LCCDDATA', 'LCCDDATASET' ]

versionstring = 'LCCDController.py - version %s %s M C Nelson, PhD, (c) 2023'%(__version__,__status__)

import sys
import time
import select

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
from queue import SimpleQueue, Empty
from multiprocessing import Process, Queue, Value, Lock

import struct

import inspect
from itertools import count

import regex
from pyparsing import nestedExpr

import numpy as np
from scipy.signal import savgol_filter as savgol

import matplotlib.pyplot as plt

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
        
        self.textqueue = Queue()
        self.dataqueue = Queue()
        
        self.monitorthread = None
        self.monitorWindow = None
        self.monitorqueue = Queue()
        
        self.GrapichsWindow = None
        self.xdata = None

        self.flag = Value( 'i', 1 )
        self.busyflag = Value( 'i', 0 )

        # Control data processing in reader
        self.baselineflag = Value( 'i', 1 )

        # Control local accumulator in reader
        self.localaccumulatorflag = Value( 'i', 0 )

        # Report errors back
        self.errorflag = Value( 'i', 0 )

        self.bits = 12
        self.vfs = 3.3
        self.vperbit = self.vfs/(2**self.bits - 1)
        
        # --------------------------------
        self.filesuffix = ".lccd"

        self.identifier = None
        self.coefficients = []

        self.graph_by_pixels = graph_by_pixels

        # ---------------------------------

        self.debug = debug
        
        # ---------------------------------
        # First, stop
        buffer = self.rawcommand( 'stop' )
        if buffer is not None:
            print( buffer )
        
        # ---------------------------------
        # Query for Identifier        
        buffer = self.rawcommand( 'identifier', 'Identifier' )
        if buffer is not None:
            print( 'lccd identifier: ', buffer )
            self.identifier = buffer.split(maxsplit=1)[1]
        
        # Query for Version
        print( "querying for version" )
        buffer = self.rawcommand( 'version' )
        if buffer is not None:
            if type(buffer) is list:
                print("buffer is list")
                for b in buffer:
                    b = b.strip()
                    if b != 'version':
                        self.version = b
                        print( 'version found', self.version )
                        break
                    print(b)
            print( 'lccd version: ', buffer )
            #self.version = buffer.split(maxsplit=1)[1]
        
        # ---------------------------------
        # Query for the configuration
        buffer = self.rawcommand( 'configuration' )
        if buffer is None:
            raise ValueError( "configuration, not found in response" )
        print( buffer )
        
        if type(buffer) is list:
            if buffer[-1].startswith("Ack:"):
                buffer = buffer[1]
            else:
                buffer = buffer[-1]
            buffer = buffer.strip()
        print( 'configuration buffer:', buffer )
            
        parts = buffer.split()
        self.datalength = key_in_list( parts, "PIXELS", int )
        self.darkstart = 0
        if self.version == "T4LCD vers 0.3":
            print( "has early version datastart 12 instead of 16")
            self.darkstart = 4

        self.pixelpitch = 8.0E-3

        self.darklength = key_in_list( parts, "DARK", int )
        self.invert = key_in_list( parts, "INVERT", float )
        self.sensor = key_in_list( parts, "SENSOR", str )
        self.scale_offset = key_in_list( parts, "OFFSET", float )
        self.scale_range = key_in_list( parts, "RANGE", float )
        
        self.bits = key_in_list( parts, "BITS", int )
        self.vfs = key_in_list( parts, "VFS", float )

        if  "VPERBIT" in parts:
            self.vperbit = key_in_list( parts, "VPERBIT", float )
            print( "Vperbit", self.vperbit )
            
        if "BITS" in parts and "VFS" in parts :
            self.vperbit = self.vfs/(2**self.bits - 1)
            print( "BITS", self.bits, "VFS", self.vfs, "Vperbit", self.vperbit )

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

        # ----------------------------------------
        # Query for coefficients
        
        self.xdata = np.linspace( 0, self.datalength, self.datalength )
        self.xlabel = 'Pixels'

        if coefficients is not None:
            print( 'using specified coefficients', coefficients )
            self.coefficients = coefficients
            self.xdata = generate_x_vector( self.datalength, self.coefficients )            
            self.xlabel = graph_xlabel
            
        else:
            buffer = self.rawcommand( 'coefficients', 'coefficients' )
            if buffer is not None:
                if self.debug:
                    print( 'coefficients buffer', buffer )
                # Also sets self.xdata
                if self.parsecoefficients( buffer ):
                    self.xlabel = 'Wavelength'
                print( 'coefficients', self.coefficients )
                print( self.xdata )
            

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
                                                  flag = self.flag,
                                                  debug = self.debug )
            self.GraphicsWindow.start( )
        
        if monitor:
            self.monitorthread = Process( target = self.textmonitor, args=(portspec, self.flag ) )
            self.monitorthread.start()
            
        self.readerthread = Process( target = self.reader,
                                     args=(portspec, self.flag, self.busyflag,
                                           self.baselineflag,
                                           self.localaccumulatorflag,
                                           self.errorflag, self.debug ) )
        self.readerthread.start()

        atexit.register(self.exit, None )

    # ------------------------------------------
    def parsecoefficients( self, buffer ):

        print( buffer )
        if 'nan' in buffer:
            self.coefficients = [ 0., 1., 0., 0. ]
            self.xdata = generate_x_vector( self.datalength, None )
            return False

        try:
            self.coefficients = [ a for a in map( float, buffer.split()[1:] ) ]            
            self.xdata = generate_x_vector( self.datalength, self.coefficients )
        except Exception as e:
            print( e )
            return False

        return True
    
    # ===========================================
    def rawcommand( self, command, key=None ):

        print( "sending", command )
        self.write( command + '\n' )

        response = []
        while True:
            try:
                buffer = self.ser.read_until( )
                buffer = buffer.decode()[:-1]
                print( "rcvd: ", buffer )
            except Exception as e:
                print( "rawread", e )
                break            
            if buffer.startswith( "DONE" ):
                break
            response.append( buffer )

        if key is not None:
            print( 'rawcommand scanning response for ', key )
            # return the selected line or None
            candidate = None
            for s in response:
                print( 'rawcommand line:', s )
                if s.startswith( key ):
                    candidate = s
            if candidate is not None:
                return candidate
            print( key, ' not found in response', response )
            return None
            
        return response

    # ===========================================
    def exit( self, ignored=None ):
        print( self.name + ' exit()' )
        self.write( 'stop\n' )
        self.close()

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

    # =======================================
    # this is used by the accumulators library
    def enqueueGraphics(self, record ):

        ycols \
            , frame_counter \
            , frame_counts \
            , frameset_counter \
            , frameset_counts \
            , frame_elapsed \
            , icg_elapsed \
            , timer_elapsed \
            , trigger_elapsed \
            , sh_period \
            , icg_period \
            , timer_period \
            , frameset_complete \
            , frame_mode \
            , frame_every \
            , accumulator_counter \
            , timestamp = record
        
        text = frame_mode
        text += '\ncounters: ' + str(frame_counter) + '/' + str(frame_counts)
        text += ' ' + str(frameset_counter) + str(frameset_counts)
        text += '\n' + timestamp.strftime('%Y-%m-%d.%H%M%S.%f')
        
        #print( "enqueue graphics" )
        self.GraphicsWindow.queue.put( [ ycols, text ] )
        #self.GraphicsWindow.queue.put( record  )
        
        return True
    
        
    # =============================================================================================
    # this is the asyncronous usb port monitoring thread
    def reader( self, name, flag, busyflag, baselineflag, localaccumulatorflag, errorflag, debug=False ):

        # ----------------------------------------------
        def sighandler( a, b ):
            print( "reader sighandler" )
            flag.value = 0
            
        signal.signal(signal.SIGINT, sighandler)

        # ----------------------------------------------
        # Fast access to queues
        dataqueue_get = self.dataqueue.get
        dataqueue_put = self.dataqueue.put
        dataqueue_empty = self.dataqueue.empty

        graphics_put = self.GraphicsWindow.queue.put

        # ----------------------------------------------
        # Fast acces to data parameters
        datalength = self.datalength
        darkstart = self.darkstart
        darklength = self.darklength
        darkstop = darkstart + darklength

        # ----------------------------------------------
        # Data stream is from firmware accumulator
        is_accumulating = False
        accumulator_counter = 0
        
        # ----------------------------------------------
        # local accumulators
        local_accumulator = np.zeros(self.datalength)
        local_accumulator_counter = 0
        
        # ----------------------------------------------
        # Record processing
        def recordProcessingInitialization():

            print("recordProcessingInitialization")
            
            if debug:
                print( 'lccd setting busyflag' )
            busyflag.value = 1
            try:
                while not dataqueue_empty():
                    dataqueue_get()
            except Exception as e:
                print( 'clearing dataqueue', e )

        def recordProcessing( ):

            nonlocal is_accumulating
            nonlocal accumulator_counter
            nonlocal data

            nonlocal local_accumulator
            nonlocal local_accumulator_counter

            #print("processing frame counter", frame_counter)
            
            if localaccumulatorflag.value:

                if frame_counter <= 1:
                    local_accumulator = data
                    local_accumulator_counter = 1
                else:
                    local_accumulator += data
                    local_accumulator_counter += 1
                
                    if local_accumulator_counter > 1:
                        data = local_accumulator/local_accumulator_counter
                
                if frame_counter == frame_counts:
                        
                    record = [ [data]
                               , frame_counter
                               , frame_counts
                               , frameset_counter
                               , frameset_counts
                               , frame_elapsed
                               , icg_elapsed
                               , timer_elapsed
                               , trigger_elapsed
                               , sh_period
                               , icg_period
                               , timer_period        
                               , frameset_complete
                               , frame_mode
                               , frame_every
                               , local_accumulator_counter,
                               timestamp ]

                    dataqueue_put( record )
                    
                    local_accumulator = np.zeros(self.datalength)
                    local_accumulator_counter = 0
                    
                #  ----------------------------------------
                if self.GraphicsWindow:
                    text = frame_mode
                    text += '\ncounters: '
                    text += str(frame_counter) + '/' + str(frame_counts)
                    text += ' ' + str(frameset_counter) + '/' + str(frameset_counts)
                    text += '\n' + timestamp.strftime('%Y-%m-%d.%H%M%S.%f')

                    #print( "enqueue graphics" )
                    graphics_put( [ [data], text ] )

                return

            # ---------------------------------
            # from here it is the hardware accumulator
            elif accumulator_counter > 1:
                data /= accumulator_counter

            if not is_accumulating:                   
                record = [ [data]
                           , frame_counter
                           , frame_counts
                           , frameset_counter
                           , frameset_counts
                           , frame_elapsed
                           , icg_elapsed
                           , timer_elapsed
                           , trigger_elapsed
                           , sh_period
                           , icg_period
                           , timer_period        
                           , frameset_complete
                           , frame_mode
                           , frame_every
                           , accumulator_counter,
                           timestamp ]

                dataqueue_put( record )

            #  ----------------------------------------
            if self.GraphicsWindow:
                text = frame_mode
                text += '\ncounters: '
                text += str(frame_counter) + '/' + str(frame_counts)
                text += ' ' + str(frameset_counter) + '/' + str(frameset_counts)
                text += '\n' + timestamp.strftime('%Y-%m-%d.%H%M%S.%f')

                #print( "enqueue graphics" )
                graphics_put( [ [data], text ] )

            #  ----------------------------------------
            is_accumulating = False
            accumulator_counter = 0;
        
        # -------------------------------------------------
        # The read loop parameters
        frame_counter = 0
        frame_counts = 0
        frameset_counter = 0
        frameset_counts = 0
        frame_elapsed = 0
        icg_elapsed = 0
        timer_elapsed = 0
        trigger_elapsed = 0
        sh_period = 0
        icg_period = 0
        timer_period = 0        
        frameset_complete = False
        frame_mode = ""
        frame_every = 0
        accumulator_counter = 0

        frameset_complete = False

        testdata = False

        crc = None
        
        def initialize_parameters():
            nonlocal frame_counter
            nonlocal frame_counts
            nonlocal frameset_counter
            nonlocal frameset_counts
            nonlocal frame_elapsed
            nonlocal icg_elapsed
            nonlocal timer_elapsed
            nonlocal trigger_elapsed
            nonlocal sh_period
            nonlocal icg_period
            nonlocal timer_period        
            nonlocal frameset_complete
            nonlocal frame_mode
            nonlocal frame_every
            nonlocal accumulator_counter

            frame_counter = 0
            frame_counts = 0
            frameset_counter = 0
            frameset_counts = 0
            frame_elapsed = 0
            icg_elapsed = 0
            timer_elapsed = 0
            trigger_elapsed = 0
            sh_period = 0
            icg_period = 0
            timer_period = 0        
            frameset_complete = False
            frame_mode = ""
            frame_every = 0
            accumulator_counter = 0
                
        # ----------------------------------------------
        # The read loop
        print( "lccd reader start", name, 'debug', debug )

        # fast access to the serial read until routine
        read_until = self.ser.read_until
        
        while flag.value:

            buffer = read_until( )
            
            if buffer is not None and len(buffer) >1 and flag.value:

                try:
                    buffer = buffer.decode()[:-1]
                except:
                    print( 'failed decode', buffer )
                    continue

                if debug:
                    print( "lccd reader: ", buffer )

                if buffer.startswith( "DONE" ):
                    self.textqueue.put( buffer )
                
                elif buffer.startswith( "READY" ):
                    self.write("send data\n")

                elif buffer.startswith( "CRC" ):
                    try:
                        crc = int( buffer[3:],0 )
                    except Exception as e:
                        print( 'error parsing', buffer )

                    print( "crc 0x%02x"%(crc) )
                    
                # Receive Ascii Formatted Data
                elif buffer.startswith( "DATA" ):
                        
                    ndata = int(buffer[4:])
                    print( 'ndata', ndata )

                    # Read the actual text format data buffer(s)
                    data_buffers = []
                    while len(data_buffers) < ndata:
                        data_buffers.append( self.ser.read_until( ) )

                    timestamp = datetime.now()

                    # Read(Expect) the end of data message
                    endbuffer = self.ser.read_until( )
                    endbuffer = endbuffer.decode()[:-1]

                    if debug:
                        print( "lccd endbuffer: ", endbuffer )
                        
                    # If valid, process the data
                    if endbuffer.startswith( "END" ):
                        
                        data = []
                        for b in data_buffers:
                            data.append( int(b.decode()) )
                            
                        data = np.array(data)

                        if self.vperbit:
                            data = data * self.vperbit

                        if self.scale_offset:
                            data = data - self.scale_offset

                        if baselineflag.value and self.darklength:
                            if debug:
                                print('baseline processing')
                            data -= np.median( data[darkstart:darkstop] )
                            #data -= np.sum( data[:self.darklength] ) / self.darklength

                        #  ----------------------------------------
                        recordProcessing()
                        #  ----------------------------------------

                    else:
                        print('reader ' + name +  ' ', buffer, len(data), ' without END')
                        self.textqueue.put( "ERROR: data not completed" )
                        if self.monitorqueue:
                            self.monitorqueue.put( "ERROR: data not completed\n" )  

                # --------------------------------
                # Receive Binary Formatted 16 bit Data Buffer
                elif buffer.startswith( "BINARY16" ):
                    ndata = int(buffer[8:])

                    # Read the data
                    data = self.ser.read( ndata*2 )

                    timestamp = datetime.now()

                    # Read(Expect) the end of data message
                    endbuffer = self.ser.read_until( )
                    endbuffer = endbuffer.decode()[:-1]

                    # Update the text display, begin and end mesages
                    if debug:
                        print( "endbuffer: ", endbuffer )
                    
                    if endbuffer.startswith( "END" ):
                        
                        data = struct.unpack( '<%dH'%(len(data)/2), data )

                        data = np.array(data)

                        if testdata:
                            for n,d in enumerate(data):
                                if d != n:
                                    printf( "aberrant test data at %d (0x%04x) read  %d (0x%04x)"%(n,n,d,d) )
                            testdata = False

                        
                        if self.vperbit:
                            data = data * self.vperbit

                        if self.scale_offset:
                            data = data - self.scale_offset
                            
                        if baselineflag.value and self.darklength:
                            if debug:
                                print('baseline processing')
                            data -= np.median( data[darkstart:darkstop] )
                            #data -= np.sum( data[:self.darklength] ) / self.darklength
                            
                        #data = self._mapdata( data )

                        #  ----------------------------------------
                        recordProcessing()
                        #  ----------------------------------------
                        
                    else:
                        print('reader ' + name +  ' ', buffer, len(data), ' without END')
                        
                        self.textqueue.put( "ERROR: data not completed" )
                        if self.monitorqueue:
                            self.monitorqueue.put( "ERROR: data not completed\n" )

                # --------------------------------
                # Receive Binary Formatted 32 bit Data Buffer
                elif buffer.startswith( "BINARY32" ):

                    ndata = int(buffer[8:])
                    
                    # Read the data

                    data = self.ser.read( ndata*4 )

                    timestamp = datetime.now()

                    # Read(Expect) the end of data message
                    endbuffer = self.ser.read_until( )
                    endbuffer = endbuffer.decode()[:-1]

                    # Update the text display, begin and end mesages
                    if endbuffer.startswith( "END" ):

                        data = struct.unpack( '<%dI'%(len(data)/4), data )

                        data = np.array(data)

                        if testdata:
                            for n,d in enumerate(data):
                                if d != n:
                                    printf( "aberrant test data at %d (0x%04x) read  %d (0x%04x)"%(n,n,d,d) )
                            testdata = False
                        
                        if self.vperbit:
                            data = data * self.vperbit

                        if self.scale_offset:
                            data = data - self.scale_offset
                            
                        if baselineflag.value and self.darklength:
                            if debug:
                                print('baseline processing')
                            data -= np.median( data[darkstart:darkstop] )
                            #data -= np.sum( data[:self.darklength] ) / self.darklength
                            
                        #data = self._mapdata( data )

                        #  ----------------------------------------
                        recordProcessing()
                        #  ----------------------------------------
                        
                    else:
                        print('reader ' + name +  ' ', buffer, len(data), ' without END')
                        
                        self.textqueue.put( "ERROR: data not completed" )
                        if self.monitorqueue:
                            self.monitorqueue.put( "ERROR: data not completed\n" )

                # ===================================================
                # Command flags               
                elif buffer.startswith( "ACCUMULATING" ):
                    is_accumulating = True
                    
                elif buffer.startswith( "TESTDATA" ):
                    testdata = True
                
                elif buffer.startswith( "FRAME COUNTER" ):

                    #print( buffer )
                    try:
                        frame_counter = int(buffer[13:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )

                elif buffer.startswith( "ACCUMULATOR" ):

                    #print( buffer )
                    try:
                        accumulator_counter = int(buffer[11:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )

                elif buffer.startswith( "FRAMESET COUNTER" ):

                    #print( buffer )
                    try:
                        frameset_counter = int(buffer[16:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )

                elif buffer.startswith( "ICG ELAPSED" ):
                    #print( buffer )
                    try:
                        icg_elapsed = float(buffer[11:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )

                elif buffer.startswith( "FRAME ELAPSED" ):
                    #print( buffer )
                    try:
                        frame_elapsed = float(buffer[13:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )

                elif buffer.startswith( "TIMER ELAPSED" ):
                    #print( buffer )
                    try:
                        timer_elapsed = float(buffer[13:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )
                        
                elif buffer.startswith( "TRIGGER ELAPSED" ):
                    #print( buffer )
                    try:
                        trigger_elapsed = float(buffer[15:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )
                        
                elif buffer.startswith( "SH PERIOD" ):
                    #print( buffer )
                    try:
                        sh_period = float(buffer[9:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )

                elif buffer.startswith( "ICG PERIOD" ):
                    #print( buffer )
                    try:
                        icg_period = float(buffer[10:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )

                elif buffer.startswith( "TIMER PERIOD" ):
                    #print( buffer )
                    try:
                        timer_period = float(buffer[12:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )

                # ---------------------------------------
                # This come at the start and end of each frameset
                elif buffer.startswith( "FRAMESET START" ):
                    frameset_complete = False

                elif buffer.startswith( "FRAMESET END" ):
                    frameset_complete = True

                elif buffer.startswith( "FRAME COUNTS" ):
                    try:
                        frame_counts = int(buffer[12:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )

                elif buffer.startswith( "FRAMESET COUNTS" ):
                    try:
                        frameset_counts = int(buffer[15:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )

                # ---------------------------------------
                # Mode keywords
                elif buffer.startswith( "START " ):
                    initialize_parameters()
                    recordProcessingInitialization()

                    frame_mode = buffer[6:]
                        
                # ---------------------------------------
                elif buffer.startswith( "ADC" ) or buffer.startswith( "CHIPTEMPERATURE" ):
                    if busyflag.value:
                        dataqueue_put( buffer )
                    else:
                        self.textqueue.put( buffer )
                        if self.monitorqueue:
                            self.monitorqueue.put( buffer.strip() + '\n' )
                        
                elif buffer.startswith( "COMPLETE" ):

                    # ---------------------
                    if debug:
                        print( 'lccd clearing busyflag' )
                    busyflag.value = 0
                    
                    
                elif buffer.startswith( "coefficient" ):

                    # this also generates self.xdata
                    self.parsecoefficients( buffer )
                    print( self.coefficients )
                    print( self.xdata )
                    
                elif buffer.startswith( "Identifier" ):

                    self.identifier = buffer.split(maxsplit=1)

                # --------------------------------
                # Receive Ascii Formatted Text
                #elif ( buffer[0] == '#' ):
                #    print( buffer[1:] )

                elif ( buffer.startswith( "Error:") ):

                    #print( buffer )
                    self.errorflag.value += 1
                    
                    self.textqueue.put( buffer )
                    if self.monitorqueue:
                        self.monitorqueue.put( buffer.strip() + '\n' )

                else:
                    
                    self.textqueue.put( buffer )
                    if self.monitorqueue:
                        self.monitorqueue.put( buffer.strip() + '\n' )

        print( "reader exit" )
        sys.exit()

    # --------------------------------------------
    def checkerrors( self ):

        counter = self.errorflag.value
        self.errorflag.value = 0

        return counter
        
    def read_all_( self ):
        resp = []
        while not self.textqueue.empty():
            resp.append( self.textqueue.get() )  
        return resp

    def read_to_done_( self ):
        resp = []
        while True:
            line = self.textqueue.get()
            if self.debug:
                print( "read_to_done_, got:", line )
            if line.startswith( "DONE"):
                break
            resp.append( line )
        return resp

    def read( self ):

        while self.textqueue.empty() and not input_ready():
            sleep(.1)

        return self.read_to_done_()

    def read_nowait( self ):
        return self.read_all_()

    def write( self, buffer ):
        self.ser.write( buffer.encode() )

    def writeread( self, line ):

        self.write( line + '\n' )
    
        response = self.read()
        for line in response:
            print( 'response: ', line )

        return response
        
    def clear( self ):

        print("clearing text and data queues and device accumulator(s)")
        
        while not self.textqueue.empty():
            self.textqueue.get()
            
        while not self.dataqueue.empty():
            self.dataqueue.get()

        return self.writeread("clear accumulator")
            
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

    # =============================================================================================
    def savetofile( self, file, timestamp=None, comments = None, write_ascii=True, framesetindex = None, records = None ):

        def formattedwrites( file, keys, vals, exclude=None ):
            for key,val in zip( keys,vals ):

                if exclude is not None:
                    if key in exclude:
                        continue

                key = key.strip()
                    
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
                
            try:
                file = open(file,"x")
                print( 'saveto', file )
                newfile = True
            except Exception as e:
                print( "open " + file )
                print( e )
                return False
            
        # ------------------------------------------------
            
        file.write( '# TCD1304Controller.py version %s\n'% __version__ )

        file.write( '# ' + timestamp.strftime('%Y-%m-%d %H:%M:%S.%f') + '\n' )

        if self.identifier is not None:
            s = self.identifier
            s = s.replace('\r', '')
            s = s.strip()
            file.write( '# identifier = "' + s + '"\n' )

        # Save all of the numerical and string values in the header portion of the class
        keys, vals = zip(*self.__dict__.items())
        formattedwrites( file, keys, vals, exclude=['identifier','filesuffix'] )
        
        if comments is not None:
            if type(comments) not in [list,tuple]:
                comments = [comments]

            for c in comments:
                file.write( '# comment = "' + c + '"\n' )

    
        file.write( '# header end\n' )

        if records is None:
            records = []
            while not self.dataqueue.empty():
                record = self.dataqueue.get()
                records.append(record )
                if self.dataqueue.empty():
                    sleep(0.2)

        print( 'lccd writing', len(records), 'records' )
            
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
                    
            else:
                ycols \
                    , frame_counter \
                    , frame_counts \
                    , frameset_counter \
                    , frameset_counts \
                    , frame_elapsed \
                    , icg_elapsed \
                    , timer_elapsed \
                    , trigger_elapsed \
                    , sh_period \
                    , icg_period \
                    , timer_period \
                    , frameset_complete \
                    , frame_mode \
                    , frame_every \
                    , accumulator_counter \
                    , timestamp = record

                vals = record[1:]
                
                keys = \
                    [ "frame_counter" \
                    , "frame_counts" \
                    , "frameset_counter" \
                    , "frameset_counts" \
                    , "frame_elapsed" \
                    , "icg_elapsed" \
                    , "timer_elapsed" \
                    , "trigger_elapsed" \
                    , "sh_period" \
                    , "icg_period" \
                    , "timer_period" \
                    , "frameset_complete" \
                    , "frame_mode" \
                    , "frame_every" \
                    , "accumulator_counter" \
                    , "timestamp" ]

                formattedwrites( file, keys, vals )

                if write_ascii:
                    for n, ycol in enumerate(ycols):
                        file.write( "# DATA ASCII %d COL %d\n"%(len(ycol),n) )
                        for y in ycol:
                            file.write( '%.8f\n'%(y) )
                        file.write( "# END DATA\n" )
                else:
                    for n, ycol in enumerate(ycols):
                        file.write( "# DATA %s %d COL %d\n"%(type(ycol[0]), len(ycol),n) )
                    file.write( "# END DATA\n" )                    
                
                wroterecord = True

        if newfile:
            file.close()
                  
    # =====================================================================
    def parseflexpwm(self,line):
        line=line.strip()
        if line.startswith('flexpwm:'):
            print("found flexpwm")
            pars=line.split(maxsplit=3)
            if len(pars)==4:
                parname="flexpwm_"+pars[1]+"_"+pars[2]
                print("parname ", parname)
                print("value ", pars[3])
                self.__dict__[parname]=pars[3]
            
    def commandlineprocessor( self, line, fileprefix=None ):

        # String ubstitutions
        if '%(' in line:
            parts = list( split_bracketed( line ) )
            for n,p in enumerate(parts):
                if p.startswith('"') and '"%(' in p:
                    try:
                        exec( "res="+p, self.__dict__, globals() )
                        parts[n] =res                    
                        line = ' '.join( parts )
                        print( "command with string substitution:", line )
                    except Exception as e:
                        print("cli string subst:", p, e)
                        return
        
        if self.monitorqueue:
            self.monitorqueue.put( 'command: ' + line + '\n')

            
        
        if line in [ 'h', 'help' ]:
            
            self.write( 'help\n' )           
            response = self.read()
            for r in response:
                r = r.rstrip()
                if r[0] == '#':
                    r = r[1:]
                print(r)

            print( "  " )

            print( "Commands implemented in the CLI/host computer:" )
            print( "   h|help                      - produces this help text" )
            print( "" )
            print( "   baseline on | off           - turn baseline correction on/off" )
            print( "" )
            print( "   clear                       - empty the data and text queues" )
            print( "" )
            print( "   save fileprefix comments... - save contents of data queue to diskfile" )
            print( "   wait                        - wait for completion of the active frameset" )
            print( "" )
            print( "   Execute commands from text file" )
            print( "     @filespec                 - read and execute commands from a file" )
            print( "        (in batch files,';' is a command separator, use \\; to escape for shell commands)" )
            print( "" )
            print( "   Pass command to operation system shell" )
            print( "     !command                  - execute shell command" )
            print( "" )
            print( "   Pass instruction to python intepretor" )
            print( "     a = 3                     - '=' causes evaluation as python" )
            print( "     = python statement        -  pass to python interprator" )
            print( "      these commands have access to local() and class name spaces" )
            print( "" )
            print( "   q[uit]                      - exit the cli program" )
            
        elif line.startswith( '#' ):
            print( "rcvd comment line" )
            print( line )

        elif line.startswith('wait'):

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

        elif line.startswith('clear'):
            self.clear()
            
            
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

        elif line.startswith( 'accumulator on' ):
            self.localaccumulatorflag.value = 1
            
        elif line.startswith( 'accumulator off' ):
            self.localaccumulatorflag.value = 0
            
        elif line.startswith( 'accumulator clear' ):
            self.localaccumulatorflag.value = -1
            
        elif line.startswith('@'):

            batchfile = line[1:].strip()
            
            try:
                with open( batchfile, 'r' ) as f:
                    script = f.readlines()
            except Exception as e:
                print(e)
                return False

            for line in script:
                line = line.strip()
                status = True

                # Protect the semicolons
                if ';' in line:
                    line = line.replace( ';', '{\\semicolon}' )
                    
                for line_ in line.split(';'):

                    # Restore the semicolons
                    line_ = line_.replace( '{\\semicolon}', ';' )
                    line_ = line_.strip()
                    print( "command:", line_ )
                    
                    if not self.commandlineprocessor( line_, fileprefix ):
                        status = False
                        break
                if not status:
                    break

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
                        exec( "line___ = " + line__, self.__dict__, globals() )
                        print( "command:", line___ )
                        if not self.commandlineprocessor( line___, fileprefix ):
                            return False                   
                    else:
                        print( "command:", line__ )
                        if not self.commandlineprocessor( line__, fileprefix ):
                            return False

            
        elif line.startswith( 'parse' ):
            try:
                print( list( split_bracketed( line ) ) )
            except Exception as e:
                print( e )

        elif line is not None and len(line) > 0:

            self.write( line + '\n' )
            
            response = self.read()
            for line in response:
                print( 'response: ', line )
                self.parseflexpwm(line)
                
        else:
            response = self.read_nowait()
            if len(response) :
                for line in response:
                    print( '>response: ', line )
                    self.parseflexpwm(line)

        if self.checkerrors():
            print( 'checkerrors found errors' )
            return False
        
        return True
        
        
    def commandloop( self, name="SerialMonitor", fileprefix=None ):
        
        while self.flag.value:

            line = input( name + ':' )

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

    parser.add_argument( 'ports', default=[ser0_default], nargs='*',
                         help = 'one or more serial or com ports,' +
                         ' the first is the control port, others are readonly' )

    parser.add_argument( '--historyfile', default = 'lccdcontroller.history', help='history file for the command line interface' )
    parser.add_argument( '--nohistoryfile' )
    
    parser.add_argument( '--pixels', action = 'store_true', help='graph pixel indices on x-axis' )
    
    parser.add_argument( '--guiwindow', action = 'store_true', help='gui version of the graphical display' )
    
    parser.add_argument( '--loggingwindow', action = 'store_true', help='display transactions in a scrolling text window' )

    parser.add_argument( '--raw', action = 'store_true', help='graph raw binary values' )

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
    
    if len(args.ports) > 1:
        dataport = TCD1304CONTROLLER( args.ports[1] )

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

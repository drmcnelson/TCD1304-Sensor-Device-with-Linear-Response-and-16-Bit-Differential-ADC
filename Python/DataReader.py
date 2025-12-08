#!/usr/bin/python

import os

import glob

import operator

import copy

import numpy as np

from scipy.signal import savgol_filter as savgol
from scipy.signal import argrelextrema
from scipy.interpolate import interp1d
from scipy.ndimage import generic_filter, median_filter

from scipy.optimize import curve_fit, least_squares, minimize

import matplotlib as mpl
import matplotlib.pyplot as plt

from matplotlib.ticker import FormatStrFormatter
from matplotlib.colors import LinearSegmentedColormap
from matplotlib import cm, colors
from matplotlib.patches import Rectangle
from matplotlib.patches import Circle

from matplotlib.animation import FFMpegWriter

from datetime import datetime

import itertools
            
def stringisint( s):
    try:
        val = int(s)
        return True
    except:
        return False

def stringisfloat( s):
    try:
        val = float(s)
        return True
    except:
        return False

def stringisstring( s):

    if stringisfloat(s):
        return False

    if stringisint(s):
        return False

    return True

def makeiterable(o):
    if hasattr(o,'__iter__'):
        return o
    return [0]

# ================================    

# for third y axis
def make_patch_spines_invisible(ax):
    ax.set_frame_on(True)
    ax.patch.set_visible(False)
    for sp in ax.spines.values():
        sp.set_visible(False)

# ==========================================================
def extract_linear_segment(data, xcoord, x1, x2):
    idx = np.where( (xcoord >= x1) & (xcoord <= x2) )[0]
    return data[idx]

def median_linear_segment(data, xcoord, x1, x2):
    segment = extract_linear_segment(data,xcoord,x1,x2)
    return np.median(segment)

# ==========================================================
# cleanup spectrum, cosmic ray, errant pixels, etc

def spectrum_median_filter( data, fp=13, passes = 2):
    #print( "spectrum_median_filter, fp", fp, "passes", passes )
    for n in range(passes):
        data = median_filter(data,footprint=fp)
        '''
        ydata = copy.deepcopy(data)
        data = median_filter(ydata,footprint=fp)
        '''
    return data

def spectrum_excursion_filter( data, span=11, gap=3, threshold=5., passes=2, verbose=False ):

    datalen = len(data)
    
    x = np.linspace(0,span,span,endpoint=False)

    y = copy.deepcopy(data)

    halfspan = int(span/2)
    halfgap = int(gap/2)
    n1 = int(halfspan-halfgap)
    n2 = int(n1 + gap)
    idx = list(range(n1)) + list(range(n2,span))
    
    for n, d in enumerate(data):

        if n < span:
            n1 = 0
            n2 = n1 + span;
        elif datalen - n < span:
            n1 = datalen - span
            n2 = datalen
        else:
            n1 = n - int(span/2)
            n2 = n + span
    

        #segment = copy.deepcopy(data[n1:n2])
        segment = data[n1:n2]
        median_ = np.median(segment[idx])
        stdev_  = np.std(segment[idx])

        m = passes - 1
        while m > 0:
            idx = np.where(np.abs(segment-median_)<threshold*stdev_)
            signal = np.median(segment[idx])
            stdev_ = np.std(segment[idx])
            m -= 1

        if abs(d - median_) > threshold * stdev_:
            y[n] = median_
    
    return y
    
# ===================================================================================================================
class FLEXPWM:

    def __init__( self, name, period, onA, offA, invertA, onB, offB, invertB):

        self.name = name
        
        self.period  = period

        self.onA = onA
        self.offA = offA
        self.invertA = invertA

        self.onB = onB
        self.offB = offB
        self.invertB = invertB

    def dump( self ):

        s = 'FLEXPWM: ' + self.name 
        for key, val in self.__dict__.items():
            s += ' ' + key + '=' + str(val)
            
        print(s)

        return True

    def draw(self,ax,stop,yoffset):

        def draw_(ax,on,off,stop,invert,yoffset):

            x = 0

            step1 = on
            step2 = off-on
            step3 = period-off

            if invert:
                offstate = 1 + yoffset
                onstate = yoffset
            else:
                onstate = 1 + yoffset
                offstate = yoffset                
            
            while x < stop:
                if step1 > 0:
                    ax.axhline(offstate,x,x+step1)
                    x = x+step1
                if step2 > 0:
                    ax.axvline(x,offstate,onstate)
                    ax.axhline(onstate,x,x+step2)
                    x = x+step2
                    ax.axvline(x,offstate,onstate)
                if step3 > 0:
                    ax.axhline(onstate,x,x+step3)
                    x = x+step3

        draw_(ax,self.onA,self.offA,self.period,self.invertA,yoffset)

        draw_(ax,self.onB,self.offB,self.period,self.invertB,yoffset)
            
        
# ===================================================================================================================
class DATAFrame:

    def __init__( self, lines, parentinstance, offsetdigits = 5 ):

        self.parent = parentinstance
        self.weight = 1
        self.isimage = False
        
        # ----------------------------------------------------------------
        # Read the file, load data and exec lines with '='

        self.isint = False
        
        rows = None
        for line in lines:

            line = line.strip()
            if not len(line):
                continue

            if '=' in line:
                if line[0] == '#':
                    line = line[1:].strip()
                exec( line, self.__dict__ )
                
            elif line.startswith( "# END" ) or line.startswith( "# end" ):
                break
            
            elif line.startswith( "# DATA" ) or line.startswith( "# data" ):
                rows = []
                self.isimage = False
                self.isint = ('int64' in line) or ('int32') in line or ('uint16' in line)

            elif line.startswith( "# IMAGE" ) or line.startswith( "# image" ):
                rows = []
                self.isimage = True
                self.isint = 'uint16' in line

            elif rows is not None:
                if self.isint:
                    rows.append( [ int(r) for r in line.split() ] )
                else:
                    rows.append( [ float(r) for r in line.split() ] )

        # ----------------------------------------------------------------
        # These are all historical, retained for backwards compatibility
        
        if 'INTERVAL' in self.__dict__:
            self.__dict__['interval'] = round( float(self.INTERVAL)*1.E-6, offsetdigits )

        if 'SHUTTER' in self.__dict__:
            self.__dict__['shutter'] = round( float(self.SHUTTER)*1.E-6, offsetdigits )

        if 'CLOCK' in self.__dict__:
            self.__dict__['clock'] = round( float(self.CLOCK)*1.E-6, offsetdigits )
            
        if 'TRIGGERELAPSED' in self.__dict__:
            self.__dict__['offset'] = round( float(self.TRIGGERELAPSED)*1.E-6, offsetdigits )
            self.__dict__['elapsed'] = round( float(self.TRIGGERELAPSED)*1.E-6, offsetdigits )

        # added 25/2/11
        if 'ELAPSED' in self.__dict__:
            self.__dict__['offset'] = round( float(self.ELAPSED)*1.E-6, offsetdigits )
            self.__dict__['elapsed'] = round( float(self.ELAPSED)*1.E-6, offsetdigits )

        if 'nanotimestamp' in self.__dict__:
            self.__dict__['offset'] = round( float(self.nanotimestamp)*1.E-9, offsetdigits )


        if 'ACCUMULATE' in self.__dict__:
            try:
                self.weight = max( self.ACCUMULATE, 1 )
            except Exception as e:
                print( 'ACCUMULATE', e )

        # ======================================================================
        # historical, image data support, else  from rows to columns 
        if self.isimage:
            self.data = np.array(rows)

        else:
            self.data = np.array(rows).T

            for n,d in enumerate(self.data):
                self.__dict__['chan%d'%n] = self.data[n]

            if 'LABELS' in self.__dict__:
                for n,s in enumerate(self.LABELS):
                    self.__dict__[s] = self.data[n]

    # Data in volts, if read as integers convert to volts and apply baseline
    def dataVolts(self,col=0,dark_subtraction=True,offset_subtraction=True):

        data = self.data[col]

        if self.isint:
            if self.parent.vperbit:
                data = data * self.parent.vperbit

            if offset_subtraction and 'offset' in self.__dict__:
                print("baseline offset subtraction", self.offset)
                data = data - self.offset

            elif self.parent.scale_offset:
                data = data - self.parent.scale_offset

            if dark_subtraction and self.parent.darklength:
                darkstart = self.parent.darkstart
                darklength = self.parent.darklength
                darkstop = darkstart + darklength
                darkoffset = np.median( data[darkstart:darkstop] )
                print("masked pixels (dark) subtraction", darkoffset)
                data = data - darkoffset
                

        return data

    # data in number of electrons counters (close to 1 per photon)
    def dataCounts(self,col=0):
        data = self.dataVolts(col)
        data *= 37.152*1000.
        return data
        
                    
    def get( self, name ):
        
        if name in self.__dict__:
            return self.__dict__[name]
        
        return None

    def median( self, n=0, idx=None ):

        if idx is None:
            return np.nanmedian(self.data,axis=1)
        else:
            return np.nanmedian(self.data[idx],axis=1)
    
    def nanmean( self, n=0, idx=None ):

        if idx is None:
            return np.nanmean(self.data,axis=1)
        else:
            return np.nanmean(self.data[idx],axis=1)
    
    def dump( self ):

        for key, val in self.__dict__.items():
            if type(val) is list and len(val) > 10:
                print( key,  '=', val[0], '...' )
            elif type(val) in [ dict ] :
                continue
            else:
                print( key, type(val), ' =', val )

        return True
                
    def weighteddata( self ):
        return self.data * self.weight
                    
# ===================================================================================================================
# DATA Class has header and a list of frames, of type class DATAFRAME

class DATA:

    def __init__( self, file, offsetdigits=5, verbose=False, debug=False ):

        self.associated = None
        
        if type(file) is str:
            file = open( file, 'r' )

        self.filename = file.name

        self.version = file.readline().strip()
        if self.version.startswith('#'):
            self.version = self.version[1:].strip()

        self.datetimestring = file.readline().strip()
        if self.datetimestring.startswith('# time:'):
            if debug:
                print( '*** legacy time' )
            self.datetimestring = self.datetimestring[7:].strip()

        elif self.datetimestring.startswith('# opened:'):
            if debug:
                print( '*** original legacy time' )
            self.datetimestring = self.datetimestring[9:].strip()
            
        else:
            self.datetimestring = self.datetimestring.strip('# ')
                        
        self.weightedadd = True

        self.frames = []

        for line in file:

            line = line.strip()
            if not len(line):
                continue
            
            if line[0] == '#':

                line = line[1:].strip()
                if line.startswith( 'header end' ):
                    break
                
                if '=' in line:
                    line = line.strip()
                    exec( line, self.__dict__ )


        if debug:
            for k, v in self.__dict__.items():
                if not k.startswith('__'):
                    print( k, v )

        # Now we read the data frames
        lines = []
        for line in file:
            lines.append(line)
            if line.startswith( '# END' ):
                self.frames.append( DATAFrame( lines, self, offsetdigits ) )
                lines = []

        file.close()

        # added 8/22/2024
        self.nframes = len(self.frames)

        # And here is a numpy array of all of the frames of data
        self.data = []
        for frame in self.frames:
            self.data.append(frame.data)
        self.data = np.array(self.data)
        
    def dump( self ):

        for key, val in self.__dict__.items():
            if type(val) is list and len(val) > 10:
                print( key,  '=', val[0], '...' )
            elif isinstance(val,FLEXPWM):
                val.dump()
            elif type(val) in [ dict ] :
                continue
            else:
                print( key, type(val), ' =', val )

        for f in self.frames:
            print( '---------------------------' )
            f.dump()

        return True
                
    def get( self, name ):
        
        if name in self.__dict__:
            return self.__dict__[name]

        return None

    def getlist(self, name):
        vals = []
        for f in self.frames:
            vals.append(f.__dict__[name])
        return vals

    def getset(self,name):
        return set(self.getlist(name))
    
# =========================================================================================================
# Load LCCD data
class LCCDDATA( DATA ):

    def __init__( self, file, offsetdigits=5, other=None, verbose=False ):
        
        DATA.__init__( self, file, offsetdigits, verbose=verbose )

        # other instrument if any,  associated with this measurement
        self.other = other

        # pixels, in millimeters
        if 'pixelpitch' in self.__dict__:
            self.pixelwidth_ = self.pixelpitch
        else:
            self.pixelwidth_ = 8.0E-3
            
        self.datalength = len(self.frames[0].data[0])

        print("creating xpixels")
        self.xpixels = np.linspace( 0, self.datalength, self.datalength )

        print("creating defaiult xdata")
        self.xdata = self.xpixels * self.pixelwidth_ 

        # Wavelength coefficients specified
        if 'coefficients' in self.__dict__:
            print("creating xdata from coefficients")
            self.xdata = np.polynomial.polynomial.polyval( self.xpixels, self.coefficients )
            
        if 'wavelength_coefficients' in self.__dict__:
            print("creating xdata from wavelength_coefficients")
            self.coefficients = self.wavelength_coefficients
            self.xdata = np.polynomial.polynomial.polyval( self.xpixels, self.coefficients )
        
        # integration intervals
        self.exposure = 0.
        
        try:
            self.mode = list(self.getset('MODE'))[0]
        except Exception as e:
            print( 'mode', e )
            
        try:
            self.interval = list(self.getset('interval'))[0]
        except Exception as e:
            print( 'interval', e )
            
        try:
            self.shutter = list(self.getset('shutter'))[0]
        except Exception as e:
            print( 'shutter', e )

        try:
            self.clock = list(self.getset('clock'))[0]
        except Exception as e:
            print( 'clock', e )
            
        try:
            self.elapsedtimes = self.getlist('elapsed')
        except Exception as e:
            print( 'elaspedtimes', e )
        
        try:
            self.shutter_periods = list(self.getset('sh_period'))
            #print( self.shutter_periods)
            s = list(set(self.shutter_periods))
            if len(s) == 1 and not self.exposure:
                self.exposure = float(s[0])*1.E-6
                print( "set exposure from shutter period", self.exposure)
        except Exception as e:
            print( 'shutter_periods', e )
            
        try:
            self.timer_periods = list(self.getset('timer_period'))
            print( self.timer_periods)
            s = list(set(self.timer_periods))
            if len(s) == 1 and s[0] > 0 and not self.exposure:
                # timer overrides exposure if present
                self.exposure = float(s[0])*1.E-6  
                print( "set exposure from timer period", self.exposure)
        except Exception as e:
            print( 'shutter_period', e )

        try:
            self.frame_exposures = [ round(f.frame_exposure,7) for f in self.frames ]
            print( "frame_exposures", self.frame_exposures)
            s = set(self.frame_exposures)
            s = list(s)
            if len(s) == 1:
                self.exposure = float(s[0])
                print( "set exposure from frame_exposure", self.exposure)
        except Exception as e:
            print( 'frame_exposures', e )

        for flexpwm_name in [ 'clk', 'sh', 'icg', 'cnvst', 'timer' ]:
            try:
                self.__dict__[ 'flexpwm_'+flexpwm_name ] = self.extractflexpwm(flexpwm_name)
            except Exception as e:
                print( 'extract '+flexpwm_name, e )
            
    def trim( self, left = 0, right = -1 ):

        self.xpixels = self.xpixels[left:right]
        self.datalength = len(self.xpixels)

        self.xdata = self.xdata[left:right]            
        self.angles = self.angles[left:right]
        
        for n,frame in enumerate(self.frames):
            frame.data = frame.data[:,left:right]
            
            if 'xdata' in frame.__dicit__:
                frame.__dict__['xdata'] = self.xdata
            if 'angles' in frame.__dicit__:
                frame.__dict__['angles'] = self.angles

    def trim_by_xdata(self, left=None, right=None ):
            
        idx1 = 0
        if left is not None:
            idx1 = np.argmax( self.xdata>=left )

        idx2 = -1
        if right is not None:
            idx2 = 1 + np.where(self.xdata<=right)[0][-1]

        self.trim( idx1, idx2 )

    def extractflexpwm(self,name):
    
        timestep = 1./150.E6
            
        try:
            key = "flexpwm_" + name+"_period"
            print( "key", key, self.__dict__[key] )
            if key in self.__dict__:
                vals = self.__dict__[key]
                vals   = vals.split()
                divider = int(vals[4])
                timestep = float(divider)/150.E6
                period = float(vals[0])*timestep

            key = "flexpwm_" + name+"_A"
            print( "key", key, self.__dict__[key] )
            if key in self.__dict__:
                vals = self.__dict__[key]
                vals = vals.split()
                onA     = float(vals[5])*timestep
                offA    = float(vals[7])*timestep
                invertA = 'noninverting' not in self.__dict__[key]
                    

            key = "flexpwm_" + name+"_B"
            print( "key", key, self.__dict__[key] )
            if key in self.__dict__:
                vals = self.__dict__[key]
                vals = vals.split()
                onB     = float(vals[5])*timestep
                offB    = float(vals[7])*timestep
                invertB = 'noninverting' not in self.__dict__[key]

        except Exception as e:
            print(name, 'not parsed', e)
            return None
                
        return FLEXPWM( name, period, onA, offA, invertA, onB, offB, invertB)
                
# =============================================================================================
def loaddata( filespec, verbose=False ):

    if filespec.endswith('lccd') or filespec.endswith('tcd1304'):
        if verbose:
            print( 'loaddata', filespec )
            print( 'is lccd' )
        data = LCCDDATA(filespec, verbose=verbose)

    else:
        data = DATA(filespec, verbose=verbose)
        
    return data

def loaddataset( filespecs, verbose=False ):

    if type(filespecs) is not list:
        filespecs = [ filespecs ]

    dataset = []
    for filespec in filespecs:
        for file in glob.glob(filespec):
            dataset.append(loaddata(file, verbose=verbose ))

    return dataset

def sortdataset( dataset, attr_name ):
    dataset.sort( key=operator.attrgetter( attr_name ) )

def set_xaxis( ax, xmin=None, xmax=None, label=None, color=None, ticfontsize=None, labelfontsize=None ):

    if xmin is not None:
        ax.set_xlim( left = float(xmin) )
        
    if xmax is not None:
        ax.set_xlim( right = float(xmax) )

    if ticfontsize:
        ax.tick_params(axis='x', labelsize=ticfontsize)
        
    if label is not None:        
        if color is not None and labelfontsize is not None:
            ax.set_xlabel( label, color=color, fontsize=labelfontsize )
        elif color is not None:
            ax.set_xlabel( label, color=color )
        elif labelfontsize is not None:
            ax.set_xlabel( label, fontsize=labelfontsize )
        else:
            ax.set_xlabel( label )
        
def set_yaxis( ax, ymin=None, ymax=None, label=None, color=None, ticfontsize=None, labelfontsize=None ):

    if ymin is not None:
        ax.set_ylim( bottom = float(ymin) )
        
    if ymax is not None:
        ax.set_ylim( top = float(ymax) )

    if ticfontsize:
        ax.tick_params(axis='y', labelsize=ticfontsize)
        
    if label is not None:
        print( "set_yaxis label", label, color)
        if color is not None and labelfontsize is not None:
            ax.set_ylabel( label, color=color, fontsize=labelfontsize )
        elif color is not None:
            ax.set_ylabel( label, color=color )
        elif labelfontsize is not None:
            ax.set_ylabel( label, fontsize=labelfontsize )
        else:
            ax.set_ylabel( label )

        
# =============================================================================================
if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser( description='Data reader' )

    parser.add_argument( 'inputs', nargs='*',
                         help = 'one or more datafiles or input statements' )

    parser.add_argument( '--figsize', nargs=2, type=float,  default=(8,5) )
    parser.add_argument( '--dpi', type=float )

    parser.add_argument( '--colors', nargs='*' )

    parser.add_argument( '--labelfont', \
                         choices=['xx-small', 'x-small', 'small', 'medium', 'large', 'x-large', 'xx-large'] )

    parser.add_argument( '--ticfont', \
                         choices=['xx-small', 'x-small', 'small', 'medium', 'large', 'x-large', 'xx-large'] )

    parser.add_argument( '--tight_layout', action='store_true' )

    parser.add_argument( '--sort' )

    parser.add_argument( '--output' )

    parser.add_argument( '--x' )
    parser.add_argument( '--y' )
    parser.add_argument( '--y2' )
    parser.add_argument( '--y3' )
    
    parser.add_argument( '--xlabel' )
    parser.add_argument( '--ylabel' )
    parser.add_argument( '--y2label' )
    parser.add_argument( '--y3label' )
    
    parser.add_argument( '--xmin', type=float )
    parser.add_argument( '--xmax', type=float )
    parser.add_argument( '--logx', action='store_true' )

    parser.add_argument( '--ymin', type=float )
    parser.add_argument( '--ymax', type=float )
    parser.add_argument( '--logy', action='store_true' )

    parser.add_argument( '--y2min', type=float )
    parser.add_argument( '--y2max', type=float )
    parser.add_argument( '--logy2', action='store_true' )
    
    parser.add_argument( '--y3min', type=float )
    parser.add_argument( '--y3max', type=float )
    parser.add_argument( '--logy3', action='store_true' )

    parser.add_argument( '--legend', nargs='*', help='applied  one for each of y,y2,...etc' )
    parser.add_argument( '--outside', action='store_true' )
    parser.add_argument( '--location' )
    
    parser.add_argument( '--title' )
    parser.add_argument( '--notitle', action='store_true' )

    parser.add_argument( '--print', nargs='*' )

    parser.add_argument( '--dump', action='store_true' )
    parser.add_argument( '--list', action='store_true' )

    parser.add_argument( '--grid', action='store_true' )
    parser.add_argument( '--minorgrid', action='store_true' )

    parser.add_argument( '--statements', nargs='*' )
    parser.add_argument( '--morestatements', nargs='*' )
    
    parser.add_argument( '--debug', action='store_true' )
    parser.add_argument( '--verbose', action='store_true' )
    parser.add_argument( '--quiet', action='store_true' )
    
    args = parser.parse_args()

    dataset = []
    statements = []
    n = 0
    for n,filespec in enumerate(args.inputs):
        if '=' in filespec:
            statements = args.inputs[n:]
            break
        for file in glob.glob(filespec):
            print( 'loading', file )
            dataset.append(loaddata(file, verbose=args.verbose ))
            
    print('dataset', len(dataset) )


    if args.statements is not None:
        for s in args.statements:
            s = s.strip()
            statements.append(s)

    if args.morestatements is not None:
        for s in args.morestatements:
            s = s.strip()
            statements.append(s)
    
    # ===============================================
    if args.list:
        print( 'list' )
        for d in dataset:
            print( d.filename )
            
    # ===============================================
    if args.dump:
        print( 'dataset', len(dataset) )
        for n,d in enumerate(dataset):
            print( '*********************' )
            d.dump()
            
    # ===============================================

    x = None
    y = None
    y2 = None
    y3 = None
    ydashed = None
    yscatter = None
    y2scatter = None
    y3scatter = None

    legend = None
    legendtitle = None

    xmin = None
    xmax = None
    xlabel = None

    ymin = None
    ymax = None
    ylabel = None

    y2min = None
    y2max = None
    y2label = None

    y3min = None
    y3max = None
    y3label = None

    text = None
    text_x = 0.98
    text_y = 0.98
    text_horizontal = 'right'
    text_vertical = 'top'

    log = False

    location=None

    for s in statements:

        s = s.strip()            
        if args.verbose:
            print( '* statement:', s )

        if '\\_' in s:
            s=s.replace('\\_', ' ')
            if args.verbose:
                print( '* statement =>', s )
            
        key = None
        value = None
        if '=' in s:
            key,value = s.split('=',maxsplit=1)
            key = key.strip()
            value = value.strip()

        if s.startswith( '#' ):
            print( s )

        elif key and len(key) and value == '?':
            found = False
            if key in globals():
                print( '# globals()' )
                dumpdict( globals(), key )
                found = True
            if key in locals():
                print( '# locals()' )
                dumpdict( locals(), key )
                found = True
            if not found:
                print( 'not found', key )

        elif key == '?':
            print( '#-----------------------' )
            print( '# globals()' )
            dumpdict( globals() )

            print( '#-----------------------' )
            print( '# locals()' )
            dumpdict( locals() )

        elif len(s):
            if args.verbose:
                print( '* exec:', s )
            if not args.quiet:
                print(s)
            exec( s )


    # ===============================================
    # ranges
    if x is not None:
        if xmin is None:
            xmin = float(np.nanmin(x))
        if xmax is None:
            xmax = float(np.nanmax(x))

    if y is not None:
        if ymin is None:
            ymin = float(np.nanmin(y))
        if ymax is None:
            if yscatter is None:
                ymax = float(np.nanmax(y))
            else:
                ymax = float(np.nanmax(yscatter))

    if y2 is not None:
        if y2min is None:
            y2min = float(np.nanmin(y2))
        if y2max is None:
            if y2scatter is None:
                y2max = float(np.nanmax(y2))
            else:
                y2max = float(np.nanmax(y2scatter))

    if y3 is not None:
        if y3min is None:
            y3min = float(np.nanmin(y3))
        if y3max is None:
            if y3scatter is None:
                y3max = float(np.nanmax(y3))
            else:
                y3max = float(np.nanmax(y3scatter))

    #  overrides
    if args.x:
        x = args.x
    if args.y:
        x = args.y
    if args.y2:
        x = args.y2
    if args.y3:
        x = args.y3
        
    if args.xmin:
        xmin = args.xmin
    if args.xmax:
        xmax = args.xmax
    if args.xlabel:
        xlabel = args.xlabel

    if args.ymin:
        ymin = args.ymin
    if args.ymax:
        ymax = args.ymax
    if args.ylabel:
        ylabel = args.ylabel

    if args.y2min:
        y2min = args.y2min
    if args.y2max:
        y2max = args.y2max
    if args.y2label:
        y2label = args.y2label

    if args.y3min:
        y3min = args.y3min
    if args.y3max:
        y3max = args.y3max
    if args.y3label:
        y3label = args.y3label

    if args.legend:
        legend = args.legend

    if args.location:
        location = args.location


    # ===============================================
    # graph

    if x is None:
        print( "need to specify x=something, try --list or --dump" )
        raise ValueError( "no x specified" )
    
    elif y is None:
        print( "need to specify y=something, try --list or --dump" )
        raise ValueError( "no y specified" )

    
    #fig = plt.figure( figsize=(8,5) )
    fig = plt.figure( figsize=args.figsize )
    
    ax1 = fig.add_subplot(111)

    if args.colors is not None and len(args.colors):
        ax1.set_prop_cycle(color=args.colors)
    
    lns = []
    
    if type(y) is not list:
        y = [y]
       
    for y_ in y:
        if y_ is not None:
            l_ = ax1.plot( x, y_ )
            lns += l_

    if yscatter is not None:
        if args.marker is None:
            ax1.scatter( x, yscatter, color=lns[-1].get_color(), alpha=args.alpha )
            lns[-1].set_linewidth(3.)
        else:
            ax1.scatter( x, yscatter, s=args.size, color=lns[-1].get_color(), marker=args.marker, alpha=args.alpha )
            lns[-1].set_linewidth(3.)

    if ydashed is not None:
        if type(ydashed) is not list:
            ydashed = [ydashed]
            
        for y_,l_ in zip(ydashed,lns):
            if y_ is not None:
                ax1.plot( x, y_, color=l_.get_color(), linestyle='dashed'  )

    set_xaxis( ax1, xmin, xmax, xlabel, ticfontsize=args.ticfont, labelfontsize=args.labelfont )
    if len(y) == 1:
        set_yaxis( ax1, ymin, ymax, ylabel, lns[-1].get_color(), ticfontsize=args.ticfont, labelfontsize=args.labelfont )
    else:
        set_yaxis( ax1, ymin, ymax, ylabel, None, ticfontsize=args.ticfont, labelfontsize=args.labelfont )

    if args.logy:
        ax1.set_yscale('log')
        
    if args.logx:
        ax1.set_xscale('log')
    #  ------------------------------------------------------------
    if y2 is not None:
        ax2 = ax1.twinx()
        #ax2._get_lines.prop_cycler = ax1._get_lines.prop_cycler
        
        if type(y2) is not list:
            y2 = [y2]
            
        for y_ in y2:
            l_ = ax2.plot( x, y_, ax1._get_lines.get_next_color() )
            lns += l_

        if y2scatter is not None:
            if args.marker is None:
                ax2.scatter( x, y2scatter, color=lns[-1].get_color(), alpha=args.alpha )
            else:
                ax2.scatter( x, y2scatter, s=args.size, color=lns[-1].get_color(), marker=args.marker, alpha=args.alpha )
            
        set_yaxis( ax2, y2min, y2max, y2label, lns[-1].get_color(), ticfontsize=args.ticfont, labelfontsize=args.labelfont )
                     
        if args.logy2:
            ax2.set_yscale('log')
            
        #  ------------------------------------------------------------
        if y3 is not None:
            
            fig.subplots_adjust(bottom=0.1,top=0.9,right=0.75)
    
            ax3 = ax1.twinx()
            ax3.spines["right"].set_position( ("axes",1.2) )
            make_patch_spines_invisible(ax3)
            ax3.spines["right"].set_visible(True)
            
            if type(y3) is not list:
                y3 = [y3]
            
            for y_ in y3:
                l_ = ax3.plot( x, y_, ax1._get_lines.get_next_color() )
                lns += l_

            if y3scatter is not None:
                if args.marker is None:
                    ax3.scatter( x, y3scatter, color=lns[-1].get_color(), alpha=args.alpha )
                else:
                    ax3.scatter( x, y3scatter, s=args.size, color=lns[-1].get_color(), marker=args.marker, alpha=args.alpha )
                    

            set_yaxis( ax3, y3min, y3max, y3label, lns[-1].get_color(), ticfontsize=args.ticfont, labelfontsize=args.labelfont )

            if args.logy3:
                ax3.set_yscale('log')
                
    #  ------------------------------------------------------------
    if text is not None:
        ax1.text( text_x, text_y, text,
                  horizontalalignment=text_horizontal,
                  verticalalignment=text_vertical,
                  transform=ax1.transAxes)

        
    if legend:
        if args.outside:
            if not args.tight_layout:
                box = ax1.get_position()
                ax1.set_position([box.x0, box.y0, box.width * 0.8, box.height])                
            ax1.legend(lns, legend, loc='center left', bbox_to_anchor=(1, 0.5), title=legendtitle)
        elif location:
            ax1.legend( lns, legend, loc=location, title=legendtitle)
        else:
            ax1.legend( lns, legend, title=legendtitle)
            
    if args.minorgrid:
        ax1.minorticks_on()
        ax1.grid(visible=True,which='both',axis='both',linestyle='--')
        
    elif args.grid:
        ax1.grid(visible=True,which='major',axis='both',linestyle='-')

    if args.title:
        plt.title(args.title)

    if args.tight_layout:
        fig.tight_layout(h_pad=1.0)

    if args.output:
        print( 'saving to', args.output )
        if args.dpi is not None:
            plt.savefig( args.output, dpi=args.dpi )
        else:
            plt.savefig( args.output )
            
    else:
        plt.show()
        

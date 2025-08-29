#!/usr/bin/python

# record structure  [ ycols,......, accumulation_counter, datetimestamp ]

import numpy as np

class Accumulators:

    def __init__( self, queue, ycol0 = 0, parentinstance = None ):

        self.accumulators = None
        self.naccumulators = None
        self.counter = 0
        
        self.queue = queue

        # ycol0 is the index for the first y vector, if we have [x,y1,y2,..etc], then set ycol0 = 1
        self.ycol0 = ycol0

        self.parent = parentinstance

    def __len__(self):
        return len(self.accumulators)

    def initialize( self, nrecords ):
        self.accumulators = None
        self.naccumulators = nrecords
        self.counter = 0
        return True

    def addrecords( self, newrecords ):

        if len(newrecords) != self.naccumulators:
            print( "number of new records != naccumulators" )
            return False

        # First records
        if self.accumulators is None:
            self.accumulators = newrecords                
            self.counter = 1            
            return True

        # Add
        for n, (newrecord,oldrecord) in enumerate( zip( newrecords, self.accumulators ) ):

            if isinstance( newrecord[0], list ):
                print( 'adding ycols, record', n, 'col0', self.ycol0 )

                # record format = [ [ycols]. ...., accumulator, timedatestamp ]
                new_ycols = newrecord[0]
                old_ycols = oldrecord[0]

                # ----------------------------------------
                #  ycol0 reserves columns, c.f. for tdaq 0,1 are time and dac
                for m,(newy,oldy) in enumerate( zip(new_ycols[self.ycol0:],old_ycols[self.ycol0:]), start=self.ycol0 ):
                    old_ycols[m] = (newy + oldy * self.counter)/(self.counter+1)
                # ----------------------------------------

                oldrecord[0] = old_ycols
                oldrecord[-2] = self.counter + 1
                self.accumulators[n] = oldrecord

            else:
                print( 'adding image, record', n )
                new_image = newrecord[0].astype(float)
                old_image = oldrecord[0]
                
                old_image = (new_image + old_image * self.counter)/(self.counter+1)

                oldrecord[0] = old_image
                oldrecord[-2] = self.counter + 1
                self.accumulators[n] = oldrecord
            
            
        self.counter += 1
        print( 'accumulators', len(self.accumulators), 'counter', self.counter )

        return True
    
        
    def pull( self, checkbusy=True ):
    
        # Busy collecting data
        if checkbusy and self.parent and self.parent.busyflag.value:
            print( "busy, try wait" )
            return False

        records = []
        for n in range(self.naccumulators):
            try:
                newrecord = self.queue.get(timeout=10)
                records.append(newrecord)
            except Exception as e:
                print( 'add - insufficient records', n, e )
                return False

        if not self.queue.empty():
            print( 'extra records in the queue' )
            return False

        return self.addrecords( records )


    def push( self ):

        for record in self.accumulators:
            self.queue.put(record)

        # partial init, keep naccumulators
        self.accumulators = None
        self.counter = 0

        return True


    def graph( self ):

        if self.parent is None:
            print( 'accumualators graph, parent instance is none' )
            return False

        if self.parent.GraphicsWindow is None:
            print( 'accumulators parent instance GraphicsWindow is None' )
            return False

        if self.accumulators is None:
            print( 'accumulators accumulators is none (not initialized?)' )
            return False

        print( self.accumulators )
        for record in self.accumulators:
            self.parent.enqueueGraphics( record )

        return True


    def avgvalue( self, colindex, xindices = None ):
        
        if len( self.accumulators ):
            rsum = 0.
            for record in self.accumulators:
                ycols = record[0]
                if xindices is None:
                    rsum += np.average( ycols[colindex] )
                else:
                    ycol = ycols[colindex]
                    rsum += np.average( ycol[xindices] )
            return rsum/len(self.accumulators)
        
        else:
            return 0.

    def avgvalue_above_zero( self, colindex, zero ):
        
        if len( self.accumulators ):
            rsum = 0.
            for record in self.accumulators:
                ycols = record[0]
                ycol = ycols[colindex]
                rsum += np.average( ycol[np.where(ycol>zero)] )
            return rsum/len(self.accumulators)
        
        else:
            return 0.


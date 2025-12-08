#!/usr/bin/bash

prgm=~/Projects/SPI_Instrumentation_Project/TCD1304_Boards/Python/DataReader.py

inf=$1
shift
if [ -z "$inf" ] ; then
    echo need filespec
    exit
fi
if [ ! -f $inf ] ; then
    echo nosuch $inf
    exit
fi

N=`grep "# DATA" $inf -c`
echo "$N frames"

dark=$1
if [ -n "$dark" ] ; then
    shift
    if [ ! -f $dark ] ; then
	echo nosuch $dark
	exit
    fi

    for n in `seq 0 $((N-1))`; do
	${prgm} \
	    $inf $dark \
	    x=dataset[0].xdata \
	    f=dataset[0].frames[$n] \
	    g=dataset[1].frames[$n] \
	    y="f.dataVolts()/f.frame_exposure - g.dataVolts()/g.frame_exposure" \
	    --output ${inf}.frame`printf "%02d" $n`-dark.png
    done
else
    for n in `seq 0 $((N-1))`; do
	${prgm} \
	    $inf $dark \
	    x=dataset[0].xdata \
	    f=dataset[0].frames[$n] \
	    g=dataset[1].frames[$n] \
	    y="f.dataVolts()/f.frame_exposure - g.dataVolts()/g.frame_exposure" \
	    --output ${inf}.frame`printf "%02d" $n`.png
    done
fi

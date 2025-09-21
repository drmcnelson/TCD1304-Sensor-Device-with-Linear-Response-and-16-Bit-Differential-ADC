#!/usr/bin/bash

here=`dirname $0`
here=`cd $here; pwd`

inf=$1
shift

nframe=$1
shift

hasdark=0
if [ -f "$1" ] ; then
    f2="$1"
    if [ "${f2#Dark}" != "${f2}" ] ; then
	dark="$f2"
	hasdark=1
	shift
    fi
fi

if [ -z $nframe ] ; then
    nframe=0
fi

if ((hasdark)) ; then
    ${here}/DataReader.py \
	   $inf $dark \
	   "text=\"int:%.3gs N:%d\"%(dataset[0].exposure,dataset[0].frames[$nframe].frame_counter)" \
	   x=dataset[0].xdata \
	   y="dataset[0].data[${nframe}][0]/dataset[0].exposure" \
	   ydark="dataset[1].data[-1][0]/dataset[0].exposure" \
	   y=y-ydark \
	   ylabel="\"v-dark/sec\"" \
	   $@

else

    ${here}/DataReader.py \
	   $inf \
	   "text=\"int:%.3gs N:%d\"%(dataset[0].exposure,dataset[0].frames[$nframe].frame_counter)" \
	   x=dataset[0].xdata \
	   y="dataset[0].data[${nframe}][0]/dataset[0].exposure" \
	   ylabel="\"v/sec\"" \
	   $@
fi



#!/usr/bin/bash

here=`dirname $0`
here=`cd $here; pwd`

inf=$1
shift

nframe=$1
shift

if [ -z $nframe ] ; then
    nframe=0
fi

nframes=( `grep -i frames $inf` )
nframes=${nframes[-1]}
${here}/DataReader.py \
       $inf \
       "text=\"int:%.3gs N:%d\"%(dataset[0].exposure,dataset[0].frames[$nframe].frame_counter)" \
       x=dataset[0].xdata \
       y="dataset[0].data[${nframe}][0]/dataset[0].exposure" \
       ylabel="\"v/sec\"" \
       $@


#	    "text=\"%.3gs %.3gs %d\"%(dataset[0].frames[0].SHUTTER*1.E-6,dataset[0].frames[0].INTERVAL*1.E-6,($nframes-$n0))" \

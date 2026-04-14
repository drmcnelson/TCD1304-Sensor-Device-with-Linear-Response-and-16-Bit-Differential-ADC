#!/usr/bin/bash

here=`dirname $0`
here=`cd $here; pwd`

if (( $# < 2 )); then
    >&2 echo "Need filespec frame_index"
fi

inf=$1
shift

if [ ! -f $inf ] ; then
    echo nosuch file $inf
    exit
fi

nframe=$1
shift

${here}/DataReader.py \
       $inf $dark \
       d=dataset[0] \
       t=d.frames[${nframe}].frame_exposure \
       n=d.frames[${nframe}].frame_counter \
       x=d.xdata \
       y=d.data[${nframe},0,:]/t \
       "text=\"%d exposure:%.3gs\"%(n,t)" \
       ylabel="\"ADU/sec\"" \
       $@



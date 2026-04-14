#!/usr/bin/bash

here=`dirname $0`
here=`cd $here; pwd`

if (( $# < 2 )); then
    >&2 echo "Need filespec first_frame_index (default=10)"
fi

inf=$1
shift

if [ ! -f $inf ] ; then
    echo nosuch file $inf
    exit
fi

nframes=`grep "^DATA" $inf -c`

((frame_start = (nframes > 10)? 10 : 0))
if [ -d "$1" ] ; then
    frame_start=$1
    shift
fi

${here}/DataReader.py \
       $inf $dark \
       d=dataset[0] \
       exposures="np.array([f.frame_exposure for f in d.frames[$frame_start:]])" \
       data=d.data[$frame_start:,0,:]/exposures \
       t="np.median([f.frame_exposure for f in d.frames])" \
       N="np.median([f.frame_counter for f in d.frames])" \
       x=d.xdata \
       y=list(data) \
       "text=\"int:%.3gs N:%d\"%(t,N)" \
       xlabel="Wavelength(nm)"
       ylabel="\"ADU/sec\"" \
       $@



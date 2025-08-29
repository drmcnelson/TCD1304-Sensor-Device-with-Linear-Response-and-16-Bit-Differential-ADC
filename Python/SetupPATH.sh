#!/usr/bin/bash

here=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
echo here $here

export PATH=${here%/}/:$PATH

echo $PATH

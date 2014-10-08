#!/bin/bash
exiftool '-filename<CreateDate' -d %y%m%d_%H%M%S%%-c.%%e -r -ext cr2 -ext mrw ./

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
python ${DIR}/move.py

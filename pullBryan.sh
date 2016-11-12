#!/bin/bash
echo You are copying from $1.

scriptpath="`dirname \"$0\"`"
scriptpath="`( cd \"$scriptpath\" && pwd )`"
sourcedir=/home/robolab/aalab/robotis/Linux/project/FIRA
cd $scriptpath

rsync -avh robolab@$1:$sourcedir/$offsetdir \
    --exclude "do.sh" \
    --exclude ".git" \
    --exclude "sprint/*.o" \
    --exclude "sprint_two_colors/*.o" \
    --exclude "sprint/sprint" \
    --exclude "sprint_two_colors/sprint" \
    --exclude "pullBryan.sh" \
    --exclude "pullDell.sh" \
    --exclude "labcpy" \
    --exclude "sprint/www" \
    --exclude "sprint/README.md" \
    --exclude "sprint_two_colors/www" \
    --exclude "sprint_two_colors/README.md" \
    \
    --exclude "TODO.txt" \
    ./
# rsync -avh robolab@$1:$sourcedir/visionmodule/www ./visionmodule/bin/debug
exit $?

#!/bin/bash
# Script Organizer 0.2.1
# Created by Julien Becirovski

function checkArgument {
    # Check if arguments are available
    if [ -z "$1" ]; then
        echo -e "\e[0;31mERROR: Invalid path: $1\e[0m"
        exit 1
    else
        if [ -d "$1" ]; then
            echo "FOUND: Path $1"
        else
            echo -e "\e[0;31mERROR: Invalid path: $1\e[0m"
            exit 2
        fi
    fi
}

function checkFilesExtension {
    # check if processing source folder contains raw pictures
    echo $1
    COUNT_RAW_DATA=$(find $1 -type f -name "*.CR2" | wc -l)
    if [ ${COUNT_RAW_DATA} == 0 ]; then
        echo -e "\e[0;31mERROR: No raw pictures in folder: $1.\e[0m"
        exit 3
    else
        echo "FOUND: $COUNT_RAW_DATA raw pictures in folder: $1"
    fi
}

function checkOptions {
    # Check and apply options
    eval COND_GET_PIX=false
    eval COND_ORG_PIX=false
    eval COND_SND_PIX=false
    eval COND_RPI_PIX=false

    while getopts ":" opt; do
        case $OPTARG in
            g) COND_GET_PIX=true ;; # Get pictures from camera to src path
            o) COND_ORG_PIX=true ;; # Organize pictures inside src path
            s) COND_SND_PIX=true ;; # Send pictures from src to dst path
            p) COND_RPI_PIX=true ;; # Get pictures serie per serie
            # TODO Make helper
            \?) echo "fuck all" >&2 ;;
        esac
    done

    if [ $OPTIND -eq 1 ]; then
        COND_GET_PIX=true
        COND_ORG_PIX=true
        COND_SND_PIX=true
        COND_RPI_PIX=true
    fi
}

function getRawSerieFromCamera {
    echo -e "Getting raw pictures serie from camera ..."
    cd $1
    COUNT_RAW_CAM=$(gphoto2 --list-files | grep '.CR2' | wc -l)
    if [ ${COUNT_RAW_CAM} -gt ${SERIE_MAX} ]; then
        COUNT_RAW_CAM=${SERIE_MAX}
    fi

    for range in $(seq 1 ${COUNT_RAW_CAM}); do
        gphoto2 --get-file=${range} --force-overwrite
    done
    cd ..
    echo -e "Raw pictures serie are gotten"

}

function getRawDataFromCamera {
    if [ ${COND_RPI_PIX} = true ]; then
        getRawSerieFromCamera $1
        purgeCamera $1

    else
        echo -e "Getting raw pictures from camera ..."
        MSG=$(cd $1 && gphoto2 --get-all-raw-data --force-overwrite)
        if [[ "Error" != *${MSG}* ]]; then
            exit 5
        fi
        echo -e "Raw pictures are gotten"
    fi

}

function purgeCamera {
    echo -e "Purge beginning ..."
    COUNT_RAW_CAM=$(gphoto2 --list-files | grep '.CR2' | wc -l)
    COUNT_RAW_DST=$(find $1 -type f -name '*.CR2' | wc -l)

    if [ ${COUNT_RAW_CAM} -eq ${COUNT_RAW_DST} ]; then
        gphoto2 --delete-all-files --recurse
    else
        cd $1
        INCR=1
        for range in $(seq 1 ${COUNT_RAW_CAM}); do
            FIRST_FILE=$(gphoto2 --list-files | egrep -o '[[:digit:]]{1}[[:alpha:]]{1}[[:digit:]]{1}[[:alpha:]]{1}[[:digit:]]{4}.CR2' | sed -n ${INCR}p)

            if [ -e ${FIRST_FILE} ]; then
                gphoto2 --delete-file=${INCR} --recurse
            else
                let INCR++
            fi
        done
        cd ..
    fi

    echo -e "Camera purged"

}

function sendRawDataToVictoria {

    echo -e "Folder sending ..."
    rsync -vrz --progress --remove-source-files --no-owner --no-group --chmod=ugo+rwx,Dugo+rwx $1/processed/ $2

    COUNT_RAW_DATA=$(find $1/processed/ -type f -name "*.CR2" | wc -l)
    if [ ${COUNT_RAW_DATA} == 0 ]; then
        echo -e "Folder sent"
        rm -rf $1/processed
        echo -e "Remove source folders."
    else
        echo -e "\e[0;31mERROR: ${COUNT_RAW_DATA} Raw data not sent\e[0m"
        exit 6
    fi

}

function main {
    # initialisation watchdog and current path
    WATCH_DOG=0
    SERIE_MAX=7
    DIR_PATH="`dirname \"$0\"`"
    DIR_PATH="`( cd \"${DIR_PATH}\" && pwd )`"
    if [ -z "$DIR_PATH" ]; then
        echo -e "\e[0;31mERROR: script path not found.\e[0m"
        exit 4
    fi

    # launch processes to download and upload pictures with options
    checkOptions $1
    checkArgument $2
    while [ $(gphoto2 --list-files | grep '.CR2' | wc -l) != 0 -o ${WATCH_DOG} -eq 200 ]; do
        if [ ${COND_GET_PIX} = true ]; then
            getRawDataFromCamera $2
        fi
        if [ ${COND_ORG_PIX} = true ]; then
            echo $2
            checkFilesExtension $2
            python ${DIR_PATH}/organizer.py $2 $3
        fi
        if [ ${COND_SND_PIX} = true ]; then
            sendRawDataToVictoria $2 $3
        fi
        let WATCH_DOG++
    done
    }

# ---------------------------------------------------------------
#   Launch transfer from camera to unprocess folder in victoria
# ---------------------------------------------------------------

# TODO Bug when no arg
main $@ $1 $2
exit 0
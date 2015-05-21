#!/bin/bash
# Script Organizer 0.2.2
# Created by Julien Becirovski

function display_error {
    # Just format error message
    echo -e "\e[0;31mERROR: $1\e[0m"
}

function checkArgument {
    # Check if arguments are available
    if [ -z "$1" ]; then
        display_error "Invalid path: $1"
        exit 1
    else
        if [ -d "$1" ]; then
            echo "FOUND: Path $1"
        else
            display_error "Invalid path: $1"
            exit 2
        fi
    fi
}

function checkFilesExtension {
    # check if processing source folder contains raw pictures
    COUNT_RAW_DATA=$(find $1 -type f -name "*.CR2" | wc -l)
    if [ ${COUNT_RAW_DATA} == 0 ]; then
        display_error "No raw pictures in folder: $1"
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
        case ${OPTARG} in
            g)  COND_GET_PIX=true ;; # Get pictures from camera to src path
            o)  COND_ORG_PIX=true ;; # Organize pictures inside src path
            s)  COND_SND_PIX=true ;; # Send pictures from src to dst path
            p)  COND_RPI_PIX=true ;; # Get pictures serie per serie
            a)  COND_GET_PIX=true    # Standard use
                COND_ORG_PIX=true
                COND_SND_PIX=true ;;
            # TODO Make helper
            \?|h) echo -e "Helper: blablabal" >&2 ;;
            *)  display_error "Invalid arguments ${OPTARG}"
                exit ${ER_BAD_ARGS} ;;
        esac
    done
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
        if [[ $(echo ${MSG} | grep -o 'Error' | wc -w) -ne 0 ]]; then
            display_error "Impossible to check transfer"
            exit 5
        fi
        echo -e "Raw pictures are gotten"
        purgeCamera $1
    fi

}

function purgeCamera {
    echo -e "Purge beginning ..."
    COUNT_RAW_CAM=$(gphoto2 --list-files | grep '.CR2' | wc -l)
    COUNT_RAW_DST=$(find $1 -type f -name '*.CR2' | wc -l)

    echo -e "FOUND: ${COUNT_RAW_CAM} pictures inside camera and ${COUNT_RAW_DST} inside folder."

    INCR=1
    for range in $(seq 1 ${COUNT_RAW_CAM}); do
        FIRST_FILE=$(gphoto2 --list-files | egrep -o '[[:digit:]]{1}[[:alpha:]]{1}[[:digit:]]{1}[[:alpha:]]{1}[[:digit:]]{4}.CR2' | sed -n ${INCR}p)
        IS_EXIST_FILE=$(find $1 -type f -name "${FIRST_FILE}" | wc -l)
        if [ ${IS_EXIST_FILE} = 1 ]; then
            gphoto2 --delete-file=${INCR} --recurse
        else
            let INCR++
        fi
    done


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
        display_error "${COUNT_RAW_DATA} Raw data not sent."
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
        display_error "Script path not found."
        exit 4
    fi

    # launch processes to download and upload pictures with options
    checkOptions $1
    checkArgument $2
    while [ ${WATCH_DOG} -le 500 ]; do
        if [ ${COND_GET_PIX} = true ]; then
            getRawDataFromCamera $2
        fi
        if [ ${COND_ORG_PIX} = true ]; then
            checkFilesExtension $2
            python ${DIR_PATH}/organizer.py $2
        fi
        if [ ${COND_SND_PIX} = true ]; then
            sendRawDataToVictoria $2 $3
        fi

        # loop breaker
        if [ ${COND_RPI_PIX} = true ]; then
            if [ $(gphoto2 --list-files | egrep '.CR2' | wc -l) = 0 ]; then
                break
            fi
        else
            break
        fi
        let WATCH_DOG++
    done
    }

# ---------------------------------------------------------------
#   Launch transfer from camera to unprocess folder in victoria
# ---------------------------------------------------------------

echo -e "-----------------------------------"
echo -e " Camera Network Transfer begin ... "
echo -e "-----------------------------------"

ER_BAD_ARGS=65

if [ $# -eq 2 ]; then
    PARAM="-a"
    SRC_PATH=$1
    DST_PATH=$2
else
    if [ $# -eq 3 ]; then
        PARAM=$1
        SRC_PATH=$2
        DST_PATH=$3
    else
        display_error "Need 2 or 3 arguments."
        exit ${ER_BAD_ARGS}
    fi
fi

main ${PARAM} ${SRC_PATH} ${DST_PATH}

echo -e "----------------------------------"
echo -e " Camera Network Transfer finished "
echo -e "----------------------------------"
exit 0
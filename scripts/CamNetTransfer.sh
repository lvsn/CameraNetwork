#!/bin/bash
# Script Organizer 0.2.3
# Created by Julien Becirovski

function displayError {
    # Just format error message
    echo -e "\e[0;31mERROR: $1\e[0m"
}

function helpText {

    echo -e "Usage: CamNetTransfer [OPTION...] [SRC_PATH] [DST_PATH]

Common options
 -?, -h             Print complet help message on program usage
 -g                 Get raw pictures from camera to source
                    folder and delete it if raw pictures exist
                    in source folder.
 -o                 Organize source folder with raw pictures:
                    |_ YYYYMMDD
                     |_ HHMMSS (7 pictures per time folder)
                       |_ YYMMDD_HHMMSS_1.CR2
                       |_ ...
                       |_ YYMMDD_HHMMSS_n.CR2
                       ...
 -s                 Send source folder to destination folder
                    thanks to rsync. SSH transfer is possible.

Miscellaneous options
 -p                 Process serial raw pictures (currently 7).
                    This is specific option for low data storage
                    like Raspberry Pi.
"
    exit 0

}
function checkArgument {
    # Check if arguments are available
    if [ -z "$1" ]; then
        displayError "Invalid path: $1"
        exit 1
    else
        if [ -d "$1" ]; then
            echo "FOUND: Path $1"
        else
            displayError "Invalid path: $1"
            exit 2
        fi
    fi
}

function checkFilesExtension {
    # check if processing source folder contains raw pictures
    COUNT_RAW_DATA=$(find $1 -type f -name "*.CR2" | wc -l)
    if [ ${COUNT_RAW_DATA} == 0 ]; then
        displayError "No raw pictures in folder: $1"
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
            \?|h) helpText >&2 ;;
            *)  displayError "Invalid arguments ${OPTARG}"
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
            displayError "Impossible to check transfer"
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
        displayError "${COUNT_RAW_DATA} Raw data not sent."
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
        displayError "Script path not found."
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

echo -e "-------------------------"
echo -e " Camera Network Transfer "
echo -e "-------------------------"

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
        displayError "Need 2 or 3 arguments."
        exit ${ER_BAD_ARGS}
    fi
fi

main ${PARAM} ${SRC_PATH} ${DST_PATH}

echo -e "----------------------------------"
echo -e " Camera Network Transfer finished "
echo -e "----------------------------------"
exit 0
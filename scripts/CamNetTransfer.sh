#!/bin/bash
# Script Organizer 0.1.0
# Created by Julien Becirovski

function checkArgument {
    # Check if arguments are available
    if [ -z "$1" ]; then
        echo -e "\e[0;31mERROR: Invalid path.\e[0m"
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
    # check if source folder contains raw pictures
    COUNT_RAW_DATA=$(find $1 -type f -name "*.CR2" | wc -l)
    if [ ${COUNT_RAW_DATA} == 0 ]; then
        echo -e "\e[0;31mERROR: No raw pictures in folder: $1.\e[0m"
        exit 3
    else
        echo "FOUND: $COUNT_RAW_DATA raw pictures in folder: $1"
    fi
}

function checkOptions {
    eval COND_GET_PIX=false
    eval COND_ORG_PIX=false
    eval COND_SND_PIX=false

    while getopts ":" opt; do
        case $OPTARG in
            g) COND_GET_PIX=true ;;
            o) COND_ORG_PIX=true ;;
            s) COND_SND_PIX=true ;;
            # TODO Make helper
            \?) echo "fuck all" >&2 ;;
        esac
    done

    if [ $OPTIND -eq 1 ]; then
        COND_GET_PIX=true
        COND_ORG_PIX=true
        COND_SND_PIX=true
    fi
}

function getRawDataFromCamera {
    echo -e "Getting raw pictures from camera ..."
    MSG=$(cd $1 && gphoto2 --get-all-raw-data)
    if [[ "Error" != *${MSG}* ]]; then
        exit 5
    fi
    echo -e "Raw pictures are gotten"

}

function sendRawDataToVictoria {

    echo -e "Folder sending ..."
    rsync -avrz --progress --remove-source-files $1 $2

    COUNT_RAW_DATA=$(find $1 -type f -name "*.CR2" | wc -l)
    if [ ${COUNT_RAW_DATA} == 0 ]; then
        echo -e "Folder sent"
        rm -rf $1/*
        echo -e "Remove source folders."
    else
        echo -e "\e[0;31mERROR: ${COUNT_RAW_DATA} Raw data not sent\e[0m"
        exit 6
    fi

}

function main {
    # launch processes to download and upload pictures with options
    DIR_PATH="`dirname \"$0\"`"
    DIR_PATH="`( cd \"${DIR_PATH}\" && pwd )`"
    if [ -z "$DIR_PATH" ]; then
        echo -e "\e[0;31mERROR: script path not found.\e[0m"
        exit 4
    fi

    checkOptions $1
    checkArgument $2
    if [ ${COND_GET_PIX} = true ]; then
        getRawDataFromCamera $2
    fi
    if [ ${COND_ORG_PIX} = true ]; then
        checkFilesExtension $2
        python ${DIR_PATH}/organizer.py $2 $3
    fi
    if [ ${COND_SND_PIX} = true ]; then
        sendRawDataToVictoria $2 $3
    fi
}

# ---------------------------------------------------------------
#   Launch transfer from camera to unprocess folder in victoria
# ---------------------------------------------------------------

main $@ $1 $2
exit 0
#!/bin/bash
USERNAME="user"
HOSTS="rosPI"
SCRIPT="sudo poweroff"
SSHTIMEOUT="ConnectTimeout=10"

delayCount=15

for HOSTNAME in ${HOSTS} ; do

  echo "Attempting to connect to host..."
  ssh -l ${USERNAME} ${HOSTNAME} -o ${SSHTIMEOUT} "${SCRIPT}"

  while [ $delayCount -gt 0 ]
  do
    echo "Waiting $delayCount seconds for $HOSTS to shutdown..."
    delayCount=$(( $delayCount - 1))
    sleep 1
  done

done

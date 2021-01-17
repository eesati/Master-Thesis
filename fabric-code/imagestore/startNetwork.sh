#!/bin/bash
#
# Copyright IBM Corp All Rights Reserved
#
# SPDX-License-Identifier: Apache-2.0
#
# Exit on first error
set -e

# don't rewrite paths for Windows Git Bash users
export MSYS_NO_PATHCONV=1
starttime=$(date +%s)
CC_SRC_LANGUAGE=${1:-"go"}
CC_SRC_LANGUAGE=`echo "$CC_SRC_LANGUAGE" | tr [:upper:] [:lower:]`

if [ "$CC_SRC_LANGUAGE" = "go" -o "$CC_SRC_LANGUAGE" = "golang" ] ; then
	CC_SRC_PATH="../chaincode/imagestore/go/"
elif [ "$CC_SRC_LANGUAGE" = "javascript" ]; then
	CC_SRC_PATH="../chaincode/imagestore/javascript/"
elif [ "$CC_SRC_LANGUAGE" = "java" ]; then
	CC_SRC_PATH="../chaincode/imagestore/java"
elif [ "$CC_SRC_LANGUAGE" = "typescript" ]; then
	CC_SRC_PATH="../chaincode/imagestore/typescript/"
else
	echo The chaincode language ${CC_SRC_LANGUAGE} is not supported by this script
	echo Supported chaincode languages are: go, java, javascript, and typescript
	exit 1
fi

# clean out any old identites in the wallets
rm -rf javascript/wallet/*
rm -rf java/wallet/*
rm -rf typescript/wallet/*
rm -rf go/wallet/*

# launch network; create channel and join peer to channel
# save the current directory and move to the test-network
pushd ../test-network
./network.sh down
./network.sh up createChannel -ca -s couchdb
./network.sh deployCC -ccn imagecontractupdated -ccv 1 -cci initLedger -ccl ${CC_SRC_LANGUAGE} -ccp ${CC_SRC_PATH}
popd
# come back to fabric directory with the popd
cat <<EOF

Total setup execution time : $(($(date +%s) - starttime)) secs ...

Next, use the this applications to interact with the deployed contract.


JavaScript:

  Start by changing into the "javascript" directory:
    cd javascript

  Next, install all required packages:
    npm install

  Then run the following applications to enroll the admin user, and register a new user
  called appUser which will be used by the other applications to interact with the deployed
  Image contract:
    node enrollAdmin
    node registerUser

  You can run the invoke application as follows. By default, the invoke application will
  create a new image and wait for new images, but you can update the application to submit other transactions:
    node invoke

  You can run the query application as follows. By default, the query application will
  return all images, but you can update the application to evaluate other transactions:
    node query


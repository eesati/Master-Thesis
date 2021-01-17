/*
 * Copyright IBM Corp. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

'use strict';

var express = require('express');    //Express Web Server 
var busboy = require('connect-busboy'); //middleware for form/file upload
var pathing = require('path');     //used for file path
var fsextra = require('fs-extra');       //File System - for file manipulation



var app = express();
app.use(express.json());
app.use(express.urlencoded());

app.use(busboy());
app.use(express.static(pathing.join(__dirname, 'public')));


const { Gateway, Wallets } = require('fabric-network');
const fs = require('fs');
const path = require('path');

async function main(x) {
    try {
        // load the network configuration
        const ccpPath = path.resolve(__dirname, '..', '..', 'test-network', 'organizations', 'peerOrganizations', 'org1.example.com', 'connection-org1.json');
        let ccp = JSON.parse(fs.readFileSync(ccpPath, 'utf8'));

        // Create a new file system based wallet for managing identities.
        const walletPath = path.join(process.cwd(), 'wallet');
        const wallet = await Wallets.newFileSystemWallet(walletPath);
        console.log(`Wallet path: ${walletPath}`);

        // Check to see if we've already enrolled the user.
        const identity = await wallet.get('appUser');
        if (!identity) {
            console.log('An identity for the user "appUser" does not exist in the wallet');
            console.log('Run the registerUser.js application before retrying');
            return;
        }

        // Create a new gateway for connecting to our peer node.
        const gateway = new Gateway();
        await gateway.connect(ccp, { wallet, identity: 'appUser', discovery: { enabled: true, asLocalhost: true } });

        // Get the network (channel) our contract is deployed to.
        const network = await gateway.getNetwork('mychannel');

        // Get the contract from the network.
        const contract = network.getContract('imagecontract');

        // Submit the specified transaction.

        await contract.submitTransaction('createImage', 'IMAGE3', 234234234, x);
        console.log('Transaction has been submitted');

        // Disconnect from the gateway.
        await gateway.disconnect();

    } catch (error) {
        console.error(`Failed to submit transaction: ${error}`);
        process.exit(1);
    }
}

//main();


app.route('/')
    .post(function (req, res, next) {

        var fstream;
        req.pipe(req.busboy);
        req.busboy.on('file', function (fieldname, file, filename) {
            console.log("Uploading: " + filename);
            console.log(req.body);
            //var base64data = new Buffer(req.body, 'binary').toString('base64');
            var myimage = req.body.toString();
            // var request = myimage.toString('base64');
            main(myimage);
            //Path where image will be uploaded
            //fstream = fsextra.createWriteStream(__dirname + '/img/' + filename + '.jpg');
            //file.pipe(fstream);
            //fstream.on('close', function () {
            console.log("Upload Finished of " + filename);
            res.redirect('back');           //where to go next
        });
        //});
    });

var server = app.listen(9000, function () {
    console.log('Listening on port %d', server.address().port);
});
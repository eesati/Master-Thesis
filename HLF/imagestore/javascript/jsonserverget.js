
'use strict';

//Express Web Server 
var busboy = require('connect-busboy'); //middleware for form/file upload
//used for file path





var express = require("express");
var cors = require("cors");


const path = require('path');
//var fs = require("fs");
// const authenticate = require("./routes/authentication");

const PORT = 8585;
var app = express();

app.use(cors());
app.use(express.json({ limit: "50mb" }));
app.use(express.urlencoded({ extended: false, limit: "50mb" }));
app.use(express.static(path.join(__dirname, "public")));
// app.use(express.bodyParser({limit: '50mb'}));


const { Gateway, Wallets } = require('fabric-network');
const fs = require('fs');
const util = require('util');


async function main(faceid, deviceid, timestamp, imagebase64) {
    try {
        // load the network configuration
        const ccpPath = path.resolve(__dirname, '..', '..', 'test-network', 'organizations', 'peerOrganizations', 'org1.example.com', 'connection-org1.json');
        let ccp = JSON.parse(fs.readFileSync(ccpPath, 'utf8'));

        // Create a new file system based wallet for managing identities.
        const walletPath = path.join(process.cwd(), 'wallet');
        const wallet = await Wallets.newFileSystemWallet(walletPath);
        //console.log(`Wallet path: ${walletPath}`);

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
        const contract = network.getContract('imagecontractupdated');

        // Submit the specified transaction.

        await contract.submitTransaction('createImage', faceid, deviceid, timestamp, imagebase64);
        console.log('Transaction has been submitted');

        // Disconnect from the gateway.
        await gateway.disconnect();

    } catch (error) {
        console.error(`Failed to submit transaction: ${error}`);
        process.exit(1);
    }
}

//main();
function encode_base64(filename) {


    //fs.readFile(filename, function (error, data) {
    //   if (error) {
    //       console.log(error);
    //         throw error;
    //    } else {
    //        var buf = Buffer.from(data);
    //        var base64 = buf.toString("base64");
    //       console.log(base64);
    //       return base64;
    //    }
    //});
}

app.get("/images", async (req, res) => {
    const contents = util.readFileSync('getrequest.jpg', { encoding: 'base64' });
    //const contents = fs.readFileSync('getrequest.jpg', { encoding: 'base64' });
    console.log("Here");
    //let base64_data = await encode_base64("getrequest.jpg");
    console.log(contents);
    res.send({ "imageData": contents });

    //res.send({ "Name": "Something" });
    //console.log(req.body);
    //res.send(200);
});

app.post("/esptest", async (req, res) => {

    if (req.body.ImageData) {
        console.log(" Device Id is:-", req.body.DeviceId);
        console.log(" Face id is :-", req.body.Face_Id);
        console.log(" Base64 Image is:-", req.body.ImageData);
        console.log(" Date Time is:-", req.body.Timestamp);
    }

    res.sendStatus(200);
    //console.log(req.body);
    console.log("Receided image and uploading to blockchain");
    main(req.body.Face_Id, req.body.DeviceId, req.body.Timestamp, req.body.ImageData);
    //res.send(200);
});

app.listen(PORT, () => {
    console.log(`Example app listening at http://localhost:${PORT}`);
});
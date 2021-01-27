/*
 * Copyright IBM Corp. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

'use strict';

const { Contract } = require('fabric-contract-api');

class FabImages extends Contract {

    async initLedger(ctx) {
        console.info('============= START : Initialize Ledger ===========');
        const images = [
            {
                device_id: 'Device 1',
                timestamp: '2020 - 12 - 13T01: 47: 38Z',
                imageitself: 'AAAFCAYAAACNbybl',

            }
        ];

        for (let i = 0; i < images.length; i++) {
            images[i].docType = 'image';
            await ctx.stub.putState('IMAGE' + i, Buffer.from(JSON.stringify(images[i])));
            console.info('Added <--> ', images[i]);
        }
        console.info('============= END : Initializing Ledger ===========');
    }

    async queryImage(ctx, imageid) {
        const imageAsBytes = await ctx.stub.getState(imageid); // get the image from chaincode state
        if (!imageAsBytes || imageAsBytes.length === 0) {
            throw new Error(`${imageid} does not exist`);
        }
        console.log(imageAsBytes.toString());
        return imageAsBytes.toString();
    }

    async createImage(ctx, imageid, timestamp, imageitself) {
        console.info('============= START : Create Image ===========');

        const image = {
            timestamp,
            docType: 'image',
            imageitself,

        };

        await ctx.stub.putState(imageid, Buffer.from(JSON.stringify(image)));
        console.info('============= END : Create Image ===========');
    }

    async queryAllImages(ctx) {
        const startKey = '';
        const endKey = '';
        const allResults = [];
        for await (const { key, value } of ctx.stub.getStateByRange(startKey, endKey)) {
            const strValue = Buffer.from(value).toString('utf8');
            let record;
            try {
                record = JSON.parse(strValue);
            } catch (err) {
                console.log(err);
                record = strValue;
            }
            allResults.push({ Key: key, Record: record });
        }
        console.info(allResults);
        return JSON.stringify(allResults);
    }

}

module.exports = FabImages;

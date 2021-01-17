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
                docType: 'image',
                imageitself: 'AAAFCAYAAACNbybl',

            }
        ];

        for (let i = 0; i < images.length; i++) {
            images[i].docType = 'image';
            await ctx.stub.putState('Face id 0', Buffer.from(JSON.stringify(images[i])));
            console.info('Added <--> ', images[i]);
        }
        console.info('============= END : Initializing Ledger ===========');
    }

    async queryImage(ctx, faceid) {
        const imageAsBytes = await ctx.stub.getState(faceid); // get the image from chaincode state
        if (!imageAsBytes || imageAsBytes.length === 0) {
            throw new Error(`${faceid} does not exist`);
        }
        console.log(imageAsBytes.toString());
        return imageAsBytes.toString();
    }

    async createImage(ctx, faceid, device_id, timestamp, imageitself) {
        console.info('============= START : Create Image ===========');

        const image = {
            device_id,
            timestamp,
            docType: 'image',
            imageitself


        };

        await ctx.stub.putState(faceid, Buffer.from(JSON.stringify(image)));
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


    async retrieveHistory(ctx, key) {
        console.info('getting history for key: ' + key);
        let iterator = await ctx.stub.getHistoryForKey(key);
        let result = [];
        let res = await iterator.next();
        while (!res.done) {
            if (res.value) {
                console.info(`found state update with value: ${res.value.value.toString('utf8')}`);
                const obj = JSON.parse(res.value.value.toString('utf8'));
                result.push(obj);
            }
            res = await iterator.next();
        }
        await iterator.close();
        return result;
    }


    async query(ctx, key) {
        console.info('querying for key: ' + key);
        let returnAsBytes = await ctx.stub.getState(key);
        let result = JSON.parse(returnAsBytes);
        return JSON.stringify(result);
    }

    async querySpecificKey(ctx, key) {
        console.info('querying for key: ' + key);
        let returnAsBytes = await ctx.stub.getState(key);
        let result = JSON.parse(returnAsBytes);
        return result;
    }
}

module.exports = FabImages;

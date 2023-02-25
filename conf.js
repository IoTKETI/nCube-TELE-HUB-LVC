/**
 * Created by Wonseok Jung in KETI on 2020-08-02.
 */

const fs = require('fs');
const {nanoid} = require("nanoid");

global.my_drone_name = 'LVC_Drone';
global.my_simul = 'on';
global.my_sysid = 250;
global.my_gcs_name = '';

let conf = {};
let cse = {};
let ae = {};
let cnt_arr = [];
let sub_arr = [];
let acp = {};

conf.useprotocol = 'http'; // select one for 'http' or 'mqtt' or 'coap' or 'ws'

// build cse
let ae_name = {};
try {
    ae_name = JSON.parse(fs.readFileSync('../flight.json', 'utf8'));
} catch (e) {
    console.log('can not find [ ../flight.json ] file');
    ae_name.host = '127.0.0.1';
    ae_name.gcs = 'KETI_LVC';
    ae_name.drone_name = "LVC_Drone";
    ae_name.sysid = 251;
    ae_name.simul = "on";

    fs.writeFileSync('../flight.json', JSON.stringify(ae_name, null, 4), 'utf8');
}

my_gcs_name = ae_name.gcs;
my_drone_name = ae_name.drone_name;
my_sysid = ae_name.sysid;
my_simul = ae_name.simul;

cse.host        = ae_name.host;
cse.port        = '7579';
cse.name        = 'Mobius';
cse.id          = '/Mobius2';
cse.mqttport    = '1883';
cse.wsport      = '7577';

// build ae
ae.name = my_gcs_name;

ae.id = 'S' + ae.name;

ae.parent       = '/' + cse.name;
ae.appid        = nanoid(9);
ae.port         = '9727';
ae.bodytype     = 'json'; // select 'json' or 'xml' or 'cbor'
ae.tas_mav_port = '3105';
ae.tas_sec_port = '3105';

// build acp: not complete
acp.parent = '/' + cse.name + '/' + ae.name;
acp.name = 'acp-' + ae.name;
acp.id = ae.id;

conf.cnt = [];
conf.sub = [];

conf.usesecure = 'disable';

if (conf.usesecure === 'enable') {
    cse.mqttport = '8883';
}

conf.cse = cse;
conf.ae = ae;
conf.cnt = cnt_arr;
conf.sub = sub_arr;
conf.acp = acp;

module.exports = conf;

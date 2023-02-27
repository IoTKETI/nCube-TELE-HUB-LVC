/**
 * Created by Wonseok Jung in KETI on 2022-12-27.
 */
const mqtt = require("mqtt");
const {nanoid} = require("nanoid");
const fs = require("fs");
const {exec} = require("child_process");
const {SerialPort} = require("serialport");
require("moment-timezone");
const moment = require('moment');
moment.tz.setDefault("Asia/Seoul");
const dgram = require("dgram");
const net = require("net");

const mavlink = require('./mavlibrary/mavlink.js');

let mavUdpComLink = {};
let mavTcpComLink = {};

let MP_protocol = 'udp'
if (MP_protocol.toLowerCase() === 'udp') {
    createUdpCommLink(my_sysid, 10000 + parseInt(my_sysid, 10));
} else if (MP_protocol.toLowerCase() === 'tcp') {
    createTcpCommLink(my_sysid, 9000 + parseInt(my_sysid, 10));
}

let rfPort = null;
let rfPort_info = {
    Path: 'COM5',
    BaudRate: '115200'
}

let mobius_sub_rc_topic = '/Mobius/';

let MQTT_SUBSCRIPTION_ENABLE = 0;

global.my_parent_cnt_name = '';
global.my_cnt_name = '';
global.pre_my_cnt_name = '';
global.my_sortie_name = 'disarm'
global.my_command_parent_name = '';
global.my_command_name = '';

const retry_interval = 2500;
const normal_interval = 100;

global.pub_start_init = '/LVC/init';

global.init_flag = false;

let mission_topic = '/Mobius/' + my_gcs_name + '/Mission_Data/' + my_drone_name;

global.getType = function (p) {
    var type = 'string';
    if (Array.isArray(p)) {
        type = 'array';
    } else if (typeof p === 'string') {
        try {
            var _p = JSON.parse(p);
            if (typeof _p === 'object') {
                type = 'string_object';
            } else {
                type = 'string';
            }
        } catch (e) {
            type = 'string';
            return type;
        }
    } else if (p != null && typeof p === 'object') {
        type = 'object';
    } else {
        type = 'other';
    }

    return type;
};

var return_count = 0;
var request_count = 0;

function ae_response_action(status, res_body, callback) {
    var aeid = res_body['m2m:ae']['aei'];
    conf.ae.id = aeid;
    callback(status, aeid);
}

function create_cnt_all(count, callback) {
    if (conf.cnt.length == 0) {
        callback(2001, count);
    } else {
        if (conf.cnt.hasOwnProperty(count)) {
            var parent = conf.cnt[count].parent;
            var rn = conf.cnt[count].name;
            sh_adn.crtct(parent, rn, count, function (rsc, res_body, count) {
                if (rsc == 5106 || rsc == 2001 || rsc == 4105) {
                    create_cnt_all(++count, function (status, count) {
                        callback(status, count);
                    });
                } else {
                    callback(9999, count);
                }
            });
        } else {
            callback(2001, count);
        }
    }
}

function delete_sub_all(count, callback) {
    if (conf.sub.length == 0) {
        callback(2001, count);
    } else {
        if (conf.sub.hasOwnProperty(count)) {
            var target = conf.sub[count].parent + '/' + conf.sub[count].name;
            sh_adn.delsub(target, count, function (rsc, res_body, count) {
                if (rsc == 5106 || rsc == 2002 || rsc == 2000 || rsc == 4105 || rsc == 4004) {
                    delete_sub_all(++count, function (status, count) {
                        callback(status, count);
                    });
                } else {
                    callback(9999, count);
                }
            });
        } else {
            callback(2001, count);
        }
    }
}

function create_sub_all(count, callback) {
    if (conf.sub.length == 0) {
        callback(2001, count);
    } else {
        if (conf.sub.hasOwnProperty(count)) {
            var parent = conf.sub[count].parent;
            var rn = conf.sub[count].name;
            var nu = conf.sub[count].nu;
            sh_adn.crtsub(parent, rn, nu, count, function (rsc, res_body, count) {
                if (rsc == 5106 || rsc == 2001 || rsc == 4105) {
                    create_sub_all(++count, function (status, count) {
                        callback(status, count);
                    });
                } else {
                    callback('9999', count);
                }
            });
        } else {
            callback(2001, count);
        }
    }
}

function retrieve_my_cnt_name() {
    mobius_sub_rc_topic = mobius_sub_rc_topic + my_gcs_name + '/RC_Data';

    console.log("gcs host is " + conf.cse.host);

    let info = {};
    info.parent = '/Mobius/' + my_gcs_name;
    info.name = 'Drone_Data';
    conf.cnt.push(JSON.parse(JSON.stringify(info)));

    info = {};
    info.parent = '/Mobius/' + my_gcs_name + '/Drone_Data';
    info.name = my_drone_name;
    conf.cnt.push(JSON.parse(JSON.stringify(info)));

    info.parent = '/Mobius/' + my_gcs_name + '/Drone_Data/' + my_drone_name;
    info.name = my_sortie_name;
    conf.cnt.push(JSON.parse(JSON.stringify(info)));

    my_parent_cnt_name = info.parent;
    my_cnt_name = my_parent_cnt_name + '/' + info.name;

    info = {};
    info.parent = '/Mobius/' + my_gcs_name;
    info.name = 'GCS_Data';
    conf.cnt.push(JSON.parse(JSON.stringify(info)));

    info = {};
    info.parent = '/Mobius/' + my_gcs_name + '/GCS_Data';
    info.name = my_drone_name;
    conf.cnt.push(JSON.parse(JSON.stringify(info)));

    my_command_parent_name = info.parent;
    my_command_name = my_command_parent_name + '/' + info.name;

    MQTT_SUBSCRIPTION_ENABLE = 1;
    sh_state = 'crtae';
    setTimeout(http_watchdog, normal_interval);

    mqtt_connect('127.0.0.1');

    rfPortOpening();
}

function http_watchdog() {
    if (sh_state === 'rtvct') {
        retrieve_my_cnt_name();
    } else if (sh_state === 'crtae') {
        console.log('[sh_state] : ' + sh_state);
        sh_adn.crtae(conf.ae.parent, conf.ae.name, conf.ae.appid, function (status, res_body) {
            console.log(res_body);
            if (status == 2001) {
                ae_response_action(status, res_body, function (status, aeid) {
                    console.log('x-m2m-rsc : ' + status + ' - ' + aeid + ' <----');
                    sh_state = 'rtvae';
                    request_count = 0;
                    return_count = 0;

                    setTimeout(http_watchdog, normal_interval);
                });
            } else if (status == 5106 || status == 4105) {
                console.log('x-m2m-rsc : ' + status + ' <----');
                sh_state = 'rtvae';

                setTimeout(http_watchdog, normal_interval);
            } else {
                console.log('x-m2m-rsc : ' + status + ' <----');
                setTimeout(http_watchdog, retry_interval);
            }
        });
    } else if (sh_state === 'rtvae') {
        if (conf.ae.id === 'S') {
            conf.ae.id = 'S' + nanoid(9);
        }

        console.log('[sh_state] : ' + sh_state);
        sh_adn.rtvae(conf.ae.parent + '/' + conf.ae.name, function (status, res_body) {
            if (status == 2000) {
                var aeid = res_body['m2m:ae']['aei'];
                console.log('x-m2m-rsc : ' + status + ' - ' + aeid + ' <----');

                if (conf.ae.id != aeid && conf.ae.id != ('/' + aeid)) {
                    console.log('AE-ID created is ' + aeid + ' not equal to device AE-ID is ' + conf.ae.id);
                } else {
                    sh_state = 'crtct';
                    request_count = 0;
                    return_count = 0;

                    setTimeout(http_watchdog, normal_interval);
                }
            } else {
                console.log('x-m2m-rsc : ' + status + ' <----');
                setTimeout(http_watchdog, retry_interval);
            }
        });
    } else if (sh_state === 'crtct') {
        console.log('[sh_state] : ' + sh_state);
        create_cnt_all(request_count, function (status, count) {
            if (status == 9999) {
                setTimeout(http_watchdog, retry_interval);
            } else {
                request_count = ++count;
                return_count = 0;
                if (conf.cnt.length <= count) {
                    sh_state = 'delsub';
                    request_count = 0;
                    return_count = 0;

                    setTimeout(http_watchdog, normal_interval);
                }
            }
        });
    } else if (sh_state === 'delsub') {
        console.log('[sh_state] : ' + sh_state);
        delete_sub_all(request_count, function (status, count) {
            if (status == 9999) {
                setTimeout(http_watchdog, retry_interval);
            } else {
                request_count = ++count;
                return_count = 0;
                if (conf.sub.length <= count) {
                    sh_state = 'crtsub';
                    request_count = 0;
                    return_count = 0;

                    setTimeout(http_watchdog, normal_interval);
                }
            }
        });
    } else if (sh_state === 'crtsub') {
        console.log('[sh_state] : ' + sh_state);
        create_sub_all(request_count, function (status, count) {
            if (status == 9999) {
                setTimeout(http_watchdog, retry_interval);
            } else {
                request_count = ++count;
                return_count = 0;
                if (conf.sub.length <= count) {
                    sh_state = 'crtci';

                    setTimeout(http_watchdog, normal_interval);
                }
            }
        });
    } else if (sh_state === 'crtci') {
        console.log('[sh_state] : ' + sh_state);
    }
}

setTimeout(http_watchdog, normal_interval);

const RC_RATE = 0.64;

function SBUS2RC(x) {
    return Math.round((x * 8 + 1 - 1000) * RC_RATE + 1500);
}

function mavlinkGenerateMessage(src_sys_id, src_comp_id, type, params) {
    const mavlinkParser = new MAVLink(null/*logger*/, src_sys_id, src_comp_id);

    let mavMsg
    let genMsg

    try {
        mavMsg = null;
        genMsg = null;

        switch (type) {
            case mavlink.MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
                mavMsg = new mavlink.messages.rc_channels_override(params.target_system,
                    params.target_component,
                    params.ch1_raw,
                    params.ch2_raw,
                    params.ch3_raw,
                    params.ch4_raw,
                    params.ch5_raw,
                    params.ch6_raw,
                    params.ch7_raw,
                    params.ch8_raw,
                    params.ch9_raw,
                    params.ch10_raw,
                    params.ch11_raw,
                    params.ch12_raw,
                    params.ch13_raw,
                    params.ch14_raw,
                    params.ch15_raw,
                    params.ch16_raw,
                    // params.ch17_raw,
                    // params.ch18_raw,
                );
                break;
        }
    } catch (e) {
        console.log('MAVLINK EX:' + e);
    }

    if (mavMsg) {
        genMsg = Buffer.from(mavMsg.pack(mavlinkParser));
        //console.log('>>>>> MAVLINK OUTGOING MSG: ' + genMsg.toString('hex'));
    }

    return genMsg;
}

function mqtt_connect(serverip) {
    if (mqtt_client === null) {
        if (conf.usesecure === 'disable') {
            var connectOptions = {
                host: serverip,
                port: conf.cse.mqttport,
                protocol: "mqtt",
                keepalive: 10,
                clientId: 'TELE_RF_' + nanoid(15),
                protocolId: "MQTT",
                protocolVersion: 4,
                clean: true,
                reconnectPeriod: 2000,
                connectTimeout: 2000,
                rejectUnauthorized: false
            }
        } else {
            connectOptions = {
                host: serverip,
                port: conf.cse.mqttport,
                protocol: "mqtts",
                keepalive: 10,
                clientId: 'TELE_RF_' + nanoid(15),
                protocolId: "MQTT",
                protocolVersion: 4,
                clean: true,
                reconnectPeriod: 2000,
                connectTimeout: 2000,
                key: fs.readFileSync("./server-key.pem"),
                cert: fs.readFileSync("./server-crt.pem"),
                rejectUnauthorized: false
            }
        }

        mqtt_client = mqtt.connect(connectOptions);

        mqtt_client.on('connect', () => {
            console.log('mqtt is connected to ( ' + serverip + ' )')

            if (mobius_sub_rc_topic !== '') {
                mqtt_client.subscribe(mobius_sub_rc_topic, () => {
                    console.log('[mqtt] mobius_sub_rc_topic is subscribed: ' + mobius_sub_rc_topic);
                });
            }
            if (my_command_name !== '') {
                mqtt_client.subscribe(my_command_name, () => {
                    console.log('[mqtt] my_command_name is subscribed: ' + my_command_name);
                });
            }
        })

        mqtt_client.on('message', (topic, message) => {
            if (topic === mobius_sub_rc_topic) {
                let ver = message.toString().substring(0, 2);
                if (ver === 'ff') {
                    let rc_data = message.toString('hex');

                    let mission_value = {};
                    mission_value.target_system = my_sysid;
                    mission_value.target_component = 1;
                    mission_value.ch1_raw = SBUS2RC(parseInt(rc_data.substring(36, 38), 16));   // CH 18 - Tilt
                    mission_value.ch2_raw = SBUS2RC(parseInt(rc_data.substring(34, 36), 16));   // CH 17 - Pan
                    mission_value.ch3_raw = SBUS2RC(parseInt(rc_data.substring(38, 40), 16));   // CH 19 - Zoom
                    mission_value.ch4_raw = SBUS2RC(parseInt(rc_data.substring(54, 56), 16));   // CH 27 - Gun
                    // mission_value.ch4_raw = SBUS2RC(parseInt(rc_data.substring(40, 42), 16));   // CH 20
                    mission_value.ch5_raw = SBUS2RC(parseInt(rc_data.substring(12, 14), 16));   // CH 6 - Drop
                    mission_value.ch6_raw = SBUS2RC(parseInt(rc_data.substring(42, 44), 16));   // CH 21 - Camera direction
                    mission_value.ch7_raw = SBUS2RC(parseInt(rc_data.substring(44, 46), 16));   // CH 22 - camera mode
                    mission_value.ch8_raw = SBUS2RC(parseInt(rc_data.substring(46, 48), 16));   // CH 23 - sub
                    mission_value.ch9_raw = SBUS2RC(parseInt(rc_data.substring(48, 50), 16));   // CH 24
                    mission_value.ch10_raw = SBUS2RC(parseInt(rc_data.substring(50, 52), 16));   // CH 25
                    mission_value.ch11_raw = SBUS2RC(parseInt(rc_data.substring(52, 54), 16));   // CH 26
                    mission_value.ch12_raw = SBUS2RC(parseInt(rc_data.substring(56, 58), 16));   // CH 28
                    mission_value.ch13_raw = SBUS2RC(parseInt(rc_data.substring(58, 60), 16));   // CH 29
                    mission_value.ch14_raw = SBUS2RC(parseInt(rc_data.substring(60, 62), 16));   // CH 30
                    mission_value.ch15_raw = SBUS2RC(parseInt(rc_data.substring(62, 64), 16));   // CH 31
                    mission_value.ch16_raw = SBUS2RC(parseInt(rc_data.substring(64, 66), 16));   // CH 32
                    console.log(mission_value);

                    try {
                        let mission_signal = mavlinkGenerateMessage(255, 0xbe, mavlink.MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, mission_value);
                        if (mission_signal == null) {
                            console.log("mavlink message is null");
                        } else {
                            if (mqtt_client !== null) {
                                mqtt_client.publish(mission_topic, mission_signal, () => {
                                    // console.log('publish ' + mission_signal.toString('hex') + ' to ' + mission_topic);
                                });
                            }
                        }
                    } catch (ex) {
                        console.log('[ERROR] ' + ex);
                    }
                }
            } else if (topic === my_command_name) {
                if (rfPort !== null) {
                    rfPort.write(message, () => {
                        console.log('Send FC Command (' + message.toString('hex') + ')to drone')
                    });
                }
                sh_adn.crtci(topic + '?rcn=0', 0, message.toString('hex'), null, function () {
                });
            } else {
                console.log('Received Message ' + message.toString('hex') + ' From ' + topic);
            }
        })

        mqtt_client.on('error', function (err) {
            console.log('[mqtt] (error) ' + err.message);
            mqtt_client = null;
            mqtt_connect(serverip);
        })
    }
}

function rfPortOpening() {
    if (rfPort === null) {
        rfPort = new SerialPort({
            path: rfPort_info.Path,
            baudRate: parseInt(rfPort_info.BaudRate, 10),
        });

        rfPort.on('open', rfPortOpen);
        rfPort.on('close', rfPortClose);
        rfPort.on('error', rfPortError);
        rfPort.on('data', rfPortData);
    } else {
        if (rfPort.isOpen) {
            console.log('This is an already open RF port.');
            rfPort.close();
            rfPort = null;
            setTimeout(rfPortOpening, 2000);
        } else {
            rfPort.open();
        }
    }
}

function rfPortOpen() {
    console.log('rfPort(' + rfPort.path + '), rfPort rate: ' + rfPort.baudRate + ' open.');
}

function rfPortClose() {
    console.log('rfPort closed.');

    setTimeout(rfPortOpening, 2000);
}

function rfPortError(error) {
    console.log('[rfPort error]: ' + error.message);

    setTimeout(rfPortOpening, 2000);
}

var mavStrFromDrone = '';
var mavStrFromDroneLength = 0;
var mavVersion = 'unknown';
var mavVersionCheckFlag = false;

function rfPortData(data) {
    mavStrFromDrone += data.toString('hex').toLowerCase();

    while (mavStrFromDrone.length > 20) {
        if (!mavVersionCheckFlag) {
            var stx = mavStrFromDrone.substring(0, 2);
            if (stx === 'fe') {
                var len = parseInt(mavStrFromDrone.substring(2, 4), 16);
                var mavLength = (6 * 2) + (len * 2) + (2 * 2);
                var sysid = parseInt(mavStrFromDrone.substring(6, 8), 16);
                var msgid = parseInt(mavStrFromDrone.substring(10, 12), 16);

                if (msgid === 0 && len === 9) { // HEARTBEAT
                    mavVersionCheckFlag = true;
                    mavVersion = 'v1';
                }

                if ((mavStrFromDrone.length) >= mavLength) {
                    var mavPacket = mavStrFromDrone.substring(0, mavLength);

                    mavStrFromDrone = mavStrFromDrone.substring(mavLength);
                    mavStrFromDroneLength = 0;
                } else {
                    break;
                }
            } else if (stx === 'fd') {
                len = parseInt(mavStrFromDrone.substring(2, 4), 16);
                mavLength = (10 * 2) + (len * 2) + (2 * 2);

                sysid = parseInt(mavStrFromDrone.substring(10, 12), 16);
                msgid = parseInt(mavStrFromDrone.substring(18, 20) + mavStrFromDrone.substring(16, 18) + mavStrFromDrone.substring(14, 16), 16);

                if (msgid === 0 && len === 9) { // HEARTBEAT
                    mavVersionCheckFlag = true;
                    mavVersion = 'v2';
                }
                if (mavStrFromDrone.length >= mavLength) {
                    mavPacket = mavStrFromDrone.substring(0, mavLength);

                    mavStrFromDrone = mavStrFromDrone.substring(mavLength);
                    mavStrFromDroneLength = 0;
                } else {
                    break;
                }
            } else {
                mavStrFromDrone = mavStrFromDrone.substring(2);
            }
        } else {
            stx = mavStrFromDrone.substring(0, 2);
            if (mavVersion === 'v1' && stx === 'fe') {
                len = parseInt(mavStrFromDrone.substring(2, 4), 16);
                mavLength = (6 * 2) + (len * 2) + (2 * 2);

                if ((mavStrFromDrone.length) >= mavLength) {
                    mavPacket = mavStrFromDrone.substring(0, mavLength);
                    // console.log('v1', mavPacket);

                    if (mqtt_client !== null) {
                        mqtt_client.publish(my_cnt_name, Buffer.from(mavPacket, 'hex'));
                    }
                    send_aggr_to_Mobius(my_cnt_name, mavPacket, 2000);

                    setTimeout(parseMavFromDrone, 0, mavPacket);

                    mavStrFromDrone = mavStrFromDrone.substring(mavLength);
                    mavStrFromDroneLength = 0;
                } else {
                    break;
                }
            } else if (mavVersion === 'v2' && stx === 'fd') {
                len = parseInt(mavStrFromDrone.substring(2, 4), 16);
                mavLength = (10 * 2) + (len * 2) + (2 * 2);

                if (mavStrFromDrone.length >= mavLength) {
                    mavPacket = mavStrFromDrone.substring(0, mavLength);
                    // console.log('v2', mavPacket);

                    if (mqtt_client !== null) {
                        mqtt_client.publish(my_cnt_name, Buffer.from(mavPacket, 'hex'));
                    }
                    send_aggr_to_Mobius(my_cnt_name, mavPacket, 2000);

                    setTimeout(parseMavFromDrone, 0, mavPacket);

                    mavStrFromDrone = mavStrFromDrone.substring(mavLength);
                    mavStrFromDroneLength = 0;
                } else {
                    break;
                }
            } else {
                mavStrFromDrone = mavStrFromDrone.substring(2);
            }
        }
    }
}

var fc = {}
var flag_base_mode = 0

function parseMavFromDrone(mavPacket) {
    try {
        var ver = mavPacket.substring(0, 2);
        if (ver === 'fd') {
            var cur_seq = parseInt(mavPacket.substring(8, 10), 16);
            var sys_id = parseInt(mavPacket.substring(10, 12).toLowerCase(), 16);
            var msg_id = parseInt(mavPacket.substring(18, 20) + mavPacket.substring(16, 18) + mavPacket.substring(14, 16), 16);
            var base_offset = 20;
        } else {
            cur_seq = parseInt(mavPacket.substring(4, 6), 16);
            sys_id = parseInt(mavPacket.substring(6, 8).toLowerCase(), 16);
            msg_id = parseInt(mavPacket.substring(10, 12).toLowerCase(), 16);
            base_offset = 12;
        }

        if (MP_protocol.toLowerCase() === 'udp') {
            if (mavUdpComLink.hasOwnProperty(sys_id)) {
                mavUdpComLink[sys_id].socket.send(Buffer.from(mavPacket, 'hex'), mavUdpComLink[sys_id].port, '127.0.0.1', (error) => {
                    if (error) {
                        mavUdpComLink[sys_id].socket.close();
                        console.log('udpCommLink[' + sys_id + '].socket is closed');
                    }
                });
            }
        } else if (MP_protocol.toLowerCase() === 'tcp') {
            if (mavTcpComLink.hasOwnProperty(sys_id)) {
                mavTcpComLink[sys_id].socket.write(Buffer.from(mavPacket, 'hex'), () => {
                    // console.log('[RF] Written through TCP - ' + data);
                });
            }
        }

        if (msg_id === mavlink.MAVLINK_MSG_ID_HEARTBEAT) { // #00 : HEARTBEAT
            var custom_mode = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 8;
            var type = mavPacket.substring(base_offset, base_offset + 2).toLowerCase();
            base_offset += 2;
            var autopilot = mavPacket.substring(base_offset, base_offset + 2).toLowerCase();
            base_offset += 2;
            var base_mode = mavPacket.substring(base_offset, base_offset + 2).toLowerCase();
            base_offset += 2;
            var system_status = mavPacket.substring(base_offset, base_offset + 2).toLowerCase();
            base_offset += 2;
            var mavlink_version = mavPacket.substring(base_offset, base_offset + 2).toLowerCase();

            fc.heartbeat = {};
            fc.heartbeat.type = Buffer.from(type, 'hex').readUInt8(0);
            fc.heartbeat.autopilot = Buffer.from(autopilot, 'hex').readUInt8(0);
            fc.heartbeat.base_mode = Buffer.from(base_mode, 'hex').readUInt8(0);
            fc.heartbeat.custom_mode = Buffer.from(custom_mode, 'hex').readUInt32LE(0);
            fc.heartbeat.system_status = Buffer.from(system_status, 'hex').readUInt8(0);
            fc.heartbeat.mavlink_version = Buffer.from(mavlink_version, 'hex').readUInt8(0);

            if (fc.heartbeat.base_mode & 0x80) {
                if (flag_base_mode === 3) {
                    flag_base_mode++;
                    my_sortie_name = moment().format('YYYY_MM_DD_T_HH_mm');
                    my_cnt_name = my_parent_cnt_name + '/' + my_sortie_name;
                    sh_adn.crtct(my_parent_cnt_name + '?rcn=0', my_sortie_name, 0, function (rsc, res_body, count) {
                    });
                } else {
                    flag_base_mode++
                    if (flag_base_mode > 16) {
                        flag_base_mode = 16;
                    }
                }
            } else {
                flag_base_mode = 0;

                my_sortie_name = 'disarm';
                my_cnt_name = my_parent_cnt_name + '/' + my_sortie_name;
            }
        } else if (msg_id == mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT) { // #33
            var time_boot_ms = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 8;
            var lat = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 8;
            var lon = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 8;
            var alt = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 8;
            var relative_alt = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 8;
            var vx = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 4;
            var vy = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 4;
            var vz = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 4;
            var hdg = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();

            fc.global_position_int = {};
            fc.global_position_int.lat = Buffer.from(lat, 'hex').readInt32LE(0);
            fc.global_position_int.lon = Buffer.from(lon, 'hex').readInt32LE(0);
            fc.global_position_int.alt = Buffer.from(alt, 'hex').readInt32LE(0);
            fc.global_position_int.relative_alt = Buffer.from(relative_alt, 'hex').readInt32LE(0);
            // fc.global_position_int.vx = Buffer.from(vx, 'hex').readInt16LE(0);
            // fc.global_position_int.vy = Buffer.from(vy, 'hex').readInt16LE(0);
            // fc.global_position_int.vz = Buffer.from(vz, 'hex').readInt16LE(0);
            fc.global_position_int.hdg = Buffer.from(hdg, 'hex').readUInt16LE(0);

            if (my_simul.toLowerCase() === 'off') {
                if (!init_flag) {
                    // TODO: 추후 실드론에서도 init_flag 값 변경하여 지속적으로 보내는 것 방지
                    fc.global_position_int.drone_name = my_drone_name;
                    mqtt_client.publish(pub_start_init, JSON.stringify(fc.global_position_int));
                }
            }
        }
    } catch (e) {
        console.log('[parseMavFromDrone Error]', msg_id + '\n' + e);
    }
}

let aggr_content = {};

function send_aggr_to_Mobius(topic, content_each, gap) {
    if (aggr_content.hasOwnProperty(topic)) {
        var timestamp = moment().format('YYYY-MM-DDTHH:mm:ssSSS');
        aggr_content[topic][timestamp] = content_each;
    } else {
        aggr_content[topic] = {};
        timestamp = moment().format('YYYY-MM-DDTHH:mm:ssSSS');
        aggr_content[topic][timestamp] = content_each;

        setTimeout(function () {
            sh_adn.crtci(topic + '?rcn=0', 0, aggr_content[topic], null, function () {
            });

            delete aggr_content[topic];
        }, gap, topic);
    }
}

function createUdpCommLink(sys_id, port) {
    if (!mavUdpComLink.hasOwnProperty(sys_id)) {
        var udpSocket = dgram.createSocket('udp4');

        udpSocket.id = sys_id;

        mavUdpComLink[sys_id] = {};
        mavUdpComLink[sys_id].socket = udpSocket;
        mavUdpComLink[sys_id].port = port;

        console.log('UDP socket created on port ' + port + ' [' + sys_id + ']');

        udpSocket.on('message', (msg) => {
            if (rfPort !== null) {
                rfPort.write(msg);
            }
        });

        udpSocket.on('close', function () {
            console.log('close');

            if (mavUdpComLink.hasOwnProperty(this.id)) {
                delete mavUdpComLink[this.id];
            }

            createTcpCommLink(this.id, mavUdpComLink[this.id].port);
        });
    }
}

function createTcpCommLink(sys_id, port) {
    if (!mavTcpComLink.hasOwnProperty(sys_id)) {
        var tcpSocket = net.createServer(function (socket) {
            console.log('TCP socket connected [' + sys_id + ']');

            socket.id = sys_id;

            mavTcpComLink[sys_id] = {};
            mavTcpComLink[sys_id].socket = socket;
            mavTcpComLink[sys_id].port = port;

            socket.on('data', (data) => {
                console.log('TCP Received GCS data - ' + data.toString('hex'))
                if (rfPort !== null) {
                    rfPort.write(data);
                }
            });

            socket.on('end', function () {
                console.log('end');

                if (mavTcpComLink.hasOwnProperty(this.id)) {
                    delete mavTcpComLink[this.id];
                }
            });

            socket.on('close', function () {
                console.log('close');

                if (mavTcpComLink.hasOwnProperty(this.id)) {
                    delete mavTcpComLink[this.id];
                }
            });

            socket.on('error', function (e) {
                console.log('error ', e);

                if (mavTcpComLink.hasOwnProperty(this.id)) {
                    delete mavTcpComLink[this.id];
                }
            });
        });

        tcpSocket.listen(port, '127.0.0.1', function () {
            console.log('TCP Server for secure is listening on port ' + port);
        });
    }
}

# nCube-TELE-LVC
Start Guide

### Install dependencies
```shell
$ curl -sL https://deb.nodesource.com/setup_16.x | sudo -E bash -

$ sudo apt-get install -y nodejs

$ node -v

$ git clone https://github.com/IoTKETI/nCube-TELE-LVC

$ cd /home/pi/nCube-TELE-LVC

$ npm install
```

### Connect with FC (ex. CubePilot Cube Orange)
1. Set FC
   - Serial baudrate : 115200
   - Change parameter
     - SRx: 2hz
     - SYSID_THISMAV: change to a value other than 1
2. Connect MC and FC
   - Connect UART1(/dev/ttyAMA0) of MC and TELEMx of FC via Serial.

### Mobius address

```shell
$ nano conf.json
```   
```javascript
   // line 13
   approval_host.ip = '127.0.0.1'; // change to Mobius address
   ```

### Define Drone ID
```shell
$ nano flight.json
```
```json
{
    "approval_gcs": "LVC",
    "flight": "Dione"
}
```

### Run
```shell
$ node thyme.js
```

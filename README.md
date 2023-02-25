# nCube-TELE-HUB-LVC
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

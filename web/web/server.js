const amqp = require('amqplib/callback_api');
const app = require('express')();
const http = require('http').Server(app);
const io = require('socket.io')(http,{
    cors: {
        origin: '*',
    }
});
const speech = require('@google-cloud/speech');
const client = new speech.SpeechClient();
const fs = require('fs');
const mongoose = require('mongoose');
const express = require('express');
const User = require('./models/User');
const Util = require('./util');
const bcrypt = require('bcrypt');
const bodyParser = require('body-parser');
const cors = require('cors');
app.use(bodyParser.json());
app.use(cors());

RABBIT_URL = "amqp://admin:password@rabbit:5672"

mongoose.connect('mongodb://admin:password@mongo:27017/senior?authSource=admin', {useNewUrlParser: true, useUnifiedTopology: true});
amqp.connect(RABBIT_URL, function (error0, connection) {
    if (error0) {
        throw error0;
    }
    connection.createChannel(function (error1, channel) {
        if (error1) {
            throw error1;
        }

        const queue_robotState = 'robotState';
        const queue_robotImage = 'robotImage';
        const queue_robotLidar = 'robotLidar';
        const queue_robotOccupancyGrid = 'robotOccupancyGrid';

        channel.assertQueue(queue_robotState, {
            durable: false
        });

        channel.consume(queue_robotState, function (msg) {
            var json = JSON.parse(msg.content.toString());
            io.emit(queue_robotState, json);
        }, {
            noAck: true
        });

        channel.assertQueue(queue_robotImage, {
            durable: false
        });

        channel.consume(queue_robotImage, function (msg) {
            io.emit(queue_robotImage, msg.content.toString());
        }, {
            noAck: true
        });

        channel.assertQueue(queue_robotLidar, {
            durable: false
        });

        channel.consume(queue_robotLidar, function (msg) {
            var json = JSON.parse(msg.content.toString());
            console.log(json);
            io.emit(queue_robotLidar, json);
        }, {
            noAck: true
        });

        channel.assertQueue(queue_robotOccupancyGrid, {
            durable: false
        });

        channel.consume(queue_robotOccupancyGrid, function (msg) {
            var json = JSON.parse(msg.content.toString());
            console.log(json);
            io.emit(queue_robotOccupancyGrid, json);
        }, {
            noAck: true
        });
    });
});

const sendRabbit = (queue, json_msg) => {
    amqp.connect(RABBIT_URL, function(error0, connection) {
    if (error0) {
        throw error0;
    }
    connection.createChannel(function(error1, channel) {
        if (error1) {
        throw error1;
        }
        var msg = JSON.stringify(json_msg);
        console.log(msg)

        channel.assertQueue(queue, {
            durable: false
        });

        channel.sendToQueue(queue, Buffer.from(msg));
        console.log(" [x] Sent %s", msg);
    });
    });
}

app.get('/api/whoami', Util.authmiddleware, async(req, res) => {
    try {
        return res.json({
            username: req.user
        })
    } catch(err) {
        return res.sendStatus(401);
    }
});

app.post('/api/auth', async(req, res) => {
    let username = req.body.username;
    let password = req.body.password;
    let user = await User.findOne({username});
    if(!user)
        return res.sendStatus(401);
    try {
        let passwordcheck = await bcrypt.compare(password, user.password);
        if(!passwordcheck)
            return res.sendStatus(401);
    } catch (err) {
        return res.sendStatus(401);
    }
    let token = Util.getToken(username);
    return res.json({
        token
    });
});

app.use('/static', express.static('build/static'));

app.get('*', (req, res) => {
    res.sendFile(__dirname + '/build/index.html');
});



const encoding = 'LINEAR16';
const sampleRateHertz = 44100;
const languageCode = 'en-US';


const request = {
    config: {
      encoding: encoding,
      sampleRateHertz: sampleRateHertz,
      languageCode: languageCode,
      speechContexts: [{
        phrases: ["pause delivery", "resume delivery", "stop delivery", "deliver object to room", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10"],
        boost: 10
      }]  
    },
    interimResults: true,
  };
  
io.on('connection', async(socket) => {

    var recognizeStream;
    var authenticated = false;
    var username = null;

    var token = socket.handshake.query.token;
    var verified = await Util.verifyJWT(token);
    if(verified) {
        authenticated = true;
        username = verified.username;
        console.log('a user connected:', username);
        socket.emit('authenticated', username);
    }
    else {
        socket.emit('401');
        socket.disconnect();
    }

    socket.on('disconnect', () => {
        if(recognizeStream)
            recognizeStream.end();
    });

    socket.on('audio', (data) => {
        if(!authenticated)
            return;
        const aa = new Int16Array(data, 0, data.byteLength / 2);
        recognizeStream.write(aa);
    });

    socket.on('stoprec', () => {
        if(!authenticated)
            return;
        recognizeStream.end();
    });

    socket.on('startnewrec', () => {
        if(!authenticated)
            return;
        recognizeStream =  client
        .streamingRecognize(request)
        .on('error', console.error)
        .on('data', data => {
            var t = data.results[0].alternatives[0].transcript;
            socket.emit('transcribe', t);
        });
        socket.emit('transcribestart');
    });

    socket.on('stopCommand', () => {
        if(!authenticated)
            return;
        sendRabbit("webCommand", {
            "type": 1,
            "status": 2           
        })
    });

    socket.on('pauseCommand', () => {
        if(!authenticated)
            return;
        sendRabbit("webCommand", {
            "type": 1,
            "status": 0           
        })
    });

    socket.on('resumeCommand', () => {
        if(!authenticated)
            return;
        sendRabbit("webCommand", {
            "type": 1,
            "status": 1           
        })
    });

    socket.on('moveCommand', (data) => {
        if(!authenticated)
            return;
        sendRabbit("webCommand", {
            "type": 0,
            "goal": data.destination           
        })

        setTimeout(()=>{
            sendRabbit("webCommand", {
                "type": 1,
                "status": 1           
            })
        }, 1000)
    });
});

http.listen(3001, () => {
    console.log('listening on *:3001');
});
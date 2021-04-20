const amqp = require('amqplib/callback_api');
const app = require('express')();
const http = require('http').Server(app);
const io = require('socket.io')(http);
const speech = require('@google-cloud/speech');
const client = new speech.SpeechClient();
const fs = require('fs');



amqp.connect('amqp://admin:password@rabbit:5672', function (error0, connection) {
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
            console.log(json);
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
    amqp.connect('amqp://admin:password@rabbit:5672', function(error0, connection) {
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

app.get('/', (req, res) => {
    res.sendFile(__dirname + '/index.html');
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
        phrases: ["pause delivery", "resume delivery", "stop delivery", "pick up object at room", "deliver object to room", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10"],
        boost: 10
      }]  
    },
    interimResults: true,
  };
  
io.on('connection', (socket) => {

    var recognizeStream;


    console.log('a user connected');
    socket.on('disconnect', () => {
        if(recognizeStream)
            recognizeStream.end();
    });

    socket.on('audio', (data) => {
        const aa = new Int16Array(data, 0, data.byteLength / 2);
        recognizeStream.write(aa);
    });

    socket.on('stoprec', () => {
        recognizeStream.end();
    });

    socket.on('startnewrec', () => {
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
        sendRabbit("webCommand", {
            "type": 1,
            "status": 2           
        })
    });

    socket.on('pauseCommand', () => {
        sendRabbit("webCommand", {
            "type": 1,
            "status": 0           
        })
    });

    socket.on('resumeCommand', () => {
        sendRabbit("webCommand", {
            "type": 1,
            "status": 1           
        })
    });

    socket.on('moveCommand', (data) => {
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

http.listen(3000, () => {
    console.log('listening on *:3000');
});
//Load packages
var rpio = require('rpio');
var json2csv = require('json2csv');
var http = require('http');
var fs = require('fs');
var path = require('path');

var dataReadyPin = 13;
var pin2 = 11;  // data ready pin for initial spi transactions
var pin3 = 15;  // reset/emergency stopp

var inputs = {};
var wstream;
var commandBits = {
	'0':['Motor 1 Command Pos',1],
	'8':['Motor 1 Command Pos',-1],
	'1':['Motor 1 Actual Pos',1],
	'9':['Motor 1 Actual Pos',-1],
	'2':['Motor 2 Command Pos',1],
	'A':['Motor 2 Command Pos',-1],
	'3':['Motor 2 Actual Pos',1],
	'B':['Motor 2 Actual Pos',-1],
	'4':['five',1],
	'C':['five',-1],
	'5':['six',1],
	'D':['six',-1],
	'6':['seven',1],
	'E':['seven',-1],
	'7':['eight',1],
	'F':['eight',-1]
};

//Setup web server
var server = http.createServer(function (req, res) {
    // requesting files
    var file = '/home/pi/Project/index.html'
	//var file = '.'+((req.url=='/')?'/index.html':req.url);
    var fileExtension = path.extname(file);
    var contentType = 'text/html';
    fs.exists(file, function(exists){
        if(exists){
            fs.readFile(file, function(error, content){
                if(!error){
                    // Page found, write content
                    res.writeHead(200,{'content-type':contentType});
                    res.end(content);
                }
            })
        }
        else{
            // Page not found
            res.writeHead(404);
            res.end('Page not found');
        }
    })
}).listen(443);

var io = require('socket.io').listen(server);

rpio.spiBegin();
rpio.spiChipSelect(0);
rpio.spiSetClockDivider(256);
rpio.spiSetDataMode(0);

var tx = new Buffer(1);
var rx = new Buffer(1);
var counter = 0;
var sampleTime = 0;
var outputs = {
	'Motor 1 Command Pos':[],
	'Motor 1 Actual Pos':[],
	'Motor 2 Command Pos':[],
	'Motor 2 Actual Pos':[]
};

function translateInc(buff){
	//takes a 16 bit value and splits it into a float of up to 2^8 with a tenth's place and the label being the first hex digit
	// 0000 0000 0000 0000
	// ^^^^ command byte
	//		^^^^ pre decial value
	//			 ^^^^ ^^^^ post decimal value
	
	// console.log("Recieved: ", buff);
	label = buff.toString('hex').slice(0,1);
	predec = parseInt(buff.toString(16).slice(1,2),16);
	postdec = parseInt(buff.toString('hex').slice(2,4),16);
	if (postdec < 10) {
		floatval = commandBits[label][1] * parseFloat(predec.toString() + '.' + "0" + postdec.toString());
	}
	else {
		floatval = commandBits[label][1] * parseFloat(predec.toString() + '.' + postdec.toString());
	}

	// console.log("Command Bit: ",commandBits[label][0],"Value: ", floatval);
	//wstream.write(commandBits[label][0]+" : "+floatval.toString() + "\n");

	outputs[commandBits[label][0]].push(floatval);
	
	outputs['Time'] = [...Array(outputs['Motor 1 Command Pos'].length).keys()].map(function(x) {return (x*(1/sampleTime).toFixed(4))}); 
	io.sockets.emit('updateOutputData',{'label':commandBits[label][0],'value':floatval,'sampletime':sampleTime,'time':outputs['Time']});
}

function spi16xfer(writeBuffer){
	//Splits the input into two 8 bit words then does two transactions. Returns the concatenated bit
	tx1 = writeBuffer.slice(0,1);
	tx2 = writeBuffer.slice(1,2);

	rx1 = new Buffer(1);
	rx2 = new Buffer(1);

	rpio.spiTransfer(tx1,rx1,1);
	
	//rpio.usleep(1);

	rpio.spiTransfer(tx2,rx2,1);
	
	rx16 = Buffer.concat([rx1,rx2]);
	return rx16.toString('hex');	
}

function initSpi(){
	//Transfers all the input data recieved from the client side. After everything is transfered, reads the sample Time.
	tx1 = (counter+1).toString(16);
	if (tx1.length == 1){
		tx1 = '0'+tx1;
	}
	
	tx2 = (parseInt(inputs[Object.keys(inputs)[counter]])).toString(16);

	if (tx2.length == 1){
		tx2 = '0'+tx2;
	}

	commStr = tx1+tx2;
	
	commBuff = Buffer.from(tx1+tx2,"hex")

	val = spi16xfer(commBuff);
	//console.log("Sent",commBuff,"String",commStr,"Received",val);
	
	counter +=1;

	if (counter >= Object.keys(inputs).length) {
		counter = 0;
		rpio.poll(pin2,null);
		junk = new Buffer("0000","hex");
		sampleTime  = parseInt(spi16xfer(junk).slice(2,4),16);
		
		//console.log("Sample time: ", sampleTime);
		//console.log("Command Inputs Ending");
		//console.log("Awaiting Data");
		rpio.poll(dataReadyPin,runSpi,rpio.POLL_LOW);
	}
}

function runSpi(){
	//run when the datareadypin is pulled low.
	//Writes junk data while recording data
	junk = new Buffer("0000","hex");
	val = spi16xfer(junk);
	val = translateInc(val);
	//console.log(val);

	io.sockets.emit('outputdata',json2csv({data:outputs,fields:Object.keys(outputs),quotes:''}));
	io.sockets.emit('updateBlob');
	
	//outputs['Time'] = [...Array(outputs['Motor 1 Command Pos'].length).keys()].map(function(x) {return (x*(1/sampleTime).toFixed(4))}); 
	//io.sockets.emit('updateOutputData',outputs);

}

rpio.open(dataReadyPin, rpio.INPUT);
rpio.open(pin2,rpio.INPUT);
rpio.open(pin3,rpio.OUTPUT);
rpio.write(pin3,rpio.HIGH);

//Documentation states that poll has max frequency of 1kHz, however we recroded max freq at 500-~650 hz
//rpio.poll(dataReadyPin,spi16xfer,rpio.POLL_LOW);

//connect socket
io.on('connection',function(socket) {
	console.log('Connection Established');
	socket.on('sendInput', function(data){
		outputs = {'Motor 1 Command Pos':[],'Motor 1 Actual Pos':[],'Motor 2 Command Pos':[],'Motor 2 Actual Pos':[],'Time':[]};
		inputs = data;
		//console.log(inputs);
		try{rpio.poll(pin2,null);} catch(err) {}
		
		try{rpio.poll(dataReadyPin,null);} catch(err) {}

		//split inputs into the 6 16 bit data buffers. Transfer those data streams to the xmega
		try{rpio.poll(pin2,initSpi,rpio.POLL_LOW);} catch(err) {};
		//filename = new Date();
		//filename = filename.toLocaleDateString("en-US",{hour:"2-digit",minute:"2-digit",second:"2-digit"});
		//filename = filename.replace(/\//g,"-");
		//wstream = fs.createWriteStream(filename +'.txt');
	});
	
	socket.on('emButtPress',function(){
		rpio.write(pin3,rpio.LOW);
		rpio.sleep(1);
		rpio.write(pin3,rpio.HIGH);
		try{rpio.poll(dataReadyPin,null);} catch(err) {}
		try{rpio.poll(pin2,null);} catch(err) {}

		outputs = {'Motor 1 Command Pos':[],'Motor 1 Actual Pos':[],'Motor 2 Command Pos':[],'Motor 2 Actual Pos':[],'Time':[]};
	});
});

//setInterval(function(){console.log(rpio.read(pin2))},1);
//Print console message
console.log('Server running')

var express = require('express');
var server = express();
var bodyParser = require('body-parser');
server.use(bodyParser.json());
server.use(bodyParser.urlencoded({ extended: true }));
var mysql = require('mysql');
var http = require('http');

const con = mysql.createConnection({
    host: 'localhost',
    user: 'root',
    password: 'toor',
});

con.connect(function(err) {
    if (err){ 
        console.log("Could not connect to the database.");
        throw err;
    }
    console.log("Connected to Database!");
}); 


// var database = "CREATE DATABASE rover";
// con.query(database, function (err, result) {
//     if (err) throw err;
//     console.log("database created!");
// });


/*con.end((err) => {
    // The connection is terminated gracefully
    // Ensures all remaining queries are executed
    // Then sends a quit packet to the MySQL server.
}); */ 
    
// Client's browser performs GET request to ask server to display HTML web page
server.get('/', function(req, res) {
    res.sendFile('/mnt/d/Control_Github/Project/Command/index.html');
    //Change due to yiwei laptop
});

// index.html requires styles.css so you have to add a GET request in node.js
server.get('/styles.css', function(req, res) {
    res.sendFile('/mnt/d/Control_Github/Project/Command/styles.css');
    //Change due to yiwei laptop
});

// when a user clicks a button on the web app, data is sent to the server using a POST request
server.post("/datastream", function (req, res) {
    
    res.send(req.body.data);
    console.log(req.body.data);
});

server.get("/datastream", function (req, res) {
    
    console.log(req.body.data);
    //Below is the script used to add data in the sql
    con.query("SELECT * FROM TABLE radar;", function (err, result) {
        if (err) throw err;
        console.log("stssttttstsgs");//This function is used to debug the function
        console.log(result);
        res.send(result);//Return the result object to the client
    });
    
});

// testing 
//console.log(movement);

/* esp32 accesses the data using a GET request
server.get("/datastream", function (req, res) {
    res.send(req.body.data);
    console.log('esp32 get request');
    res.end('Hello');
});
*/

console.log('Server is running on port 8000');
server.listen(8000,'0.0.0.0');
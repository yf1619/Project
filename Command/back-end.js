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
    database: 'rover'
});

con.connect(function(err) {
    if (err){ 
        console.log("Could not connect to the database.");
        throw err;
    }
    console.log("Connected to Database!");
}); 

/*
var radar = "CREATE TABLE radar (x_pos INT, y_pos INT)";
con.query(radar, function (err, result) {
    if (err) throw err;
    console.log("radar table created!");
});
*/

con.end((err) => {
    // The connection is terminated gracefully
    // Ensures all remaining queries are executed
    // Then sends a quit packet to the MySQL server.
});  
    

server.get('/', function(req, res) {
 res.sendFile('/Users/Owner/Documents/GitHub/2022Project_Rover/Command/index.html');
});
server.get('/styles.css', function(req, res) {
 res.sendFile('/Users/Owner/Documents/GitHub/2022Project_Rover/Command/styles.css');
});
server.post("/datastream", function (req, res) {
    res.send(req.body.data);
    console.log(req.body.data);
});
console.log('Server is running on port 8000');
server.listen(8000,'0.0.0.0');
"use strict";

var Client = require("./node_modules/github/index");

var github = new Client({
    debug: true,
    version: "3.0.0"
});
//console.log('Loading function');

exports.handler = function(event, context) {
    
    gitActivate("custom title","custom body");
    //console.log('Received event:', JSON.stringify(event, null, 2));
    //console.log('value1 =', event.key1);
    //console.log('value2 =', event.key2);
    //console.log('value3 =', event.key3);
    //context.succeed(event.key1);  // Echo back the first key value
    // context.fail('Something went wrong');
};

function gitActivate(title, body){
	github.authenticate(
	{
	type : 'basic',
	username : 'USERNAME HERE',
	password : 'INSERT PASSWORD HERE' 
	}
	);
	github.issues.create({
	"title" : title,
	"body" : body,
	"assignee" : "USERNAME HERE",
	"user" : "USERNAME HERE",
	"repo" : "AWSLambdaGitIssueGenerator"
	});

}

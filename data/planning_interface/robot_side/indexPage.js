//Local memory usage
if (window.localStorage){
  window.localStorage.clear();
  console.log('Local Storage is here')
}
else
  alert('Browser not supports localStorage; please try changing browser.');

// Connecting to ROS
// -----------------

var ros = new ROSLIB.Ros({
  url : 'ws://localhost:9090'
});

ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});
// Subscribing to a Topic
// ----------------------

var listener = new ROSLIB.Topic({
  ros : ros,
  name : '/config_robot_side_sender',
  messageType : 'robot_knowledge_base/StringArray'
});

listener.subscribe(function(message) {
  feedbackTopic(message.data);
  listener.unsubscribe();
});

function printList(list){
  /*
  * Used for testing.
  */
  console.log('activated printing')
  var div = document.getElementById("textDiv");
  for (index in list){
    div.innerHTML+=('<p>'+list[index]+'</p>')
  }
}

function feedbackTopic(data){
  /*
  Reads all topics and types and saves all in @localStorage, for later use
  */
  var elements=[] //Need for a string to save data on @localStorage
  for (index in data){
    line = data[index].split('-');
    elements.push({topic: line[0], message: line[1]})
  }
  elements.push({topic: 'None', message: 'None'})
  window.localStorage.setItem('feedback',JSON.stringify(elements));
}

function updateEdited(){
  /* 
  * Uses all data computed in various editing topic pages, and retrieves the data to ROS in list of String
  * Pushing the button confirms that all data are ok and sends them to @back_robot_side, 
  * that will persist them on MongoDB.
  */
  returnMessage=[]
  d3.select('#edited').
  selectAll('p').remove();
  for(var i=0, len=window.localStorage.length; i<len; i++) {
    var key = window.localStorage.key(i);
    if (key == 'feedback')
      continue;//if it is not a topic jump iteration
    var value = window.localStorage[key];
    console.log(key, value );
    var obj=JSON.parse(value);
    if(obj.flag=='edited'){
      d3.select('#edited')
      .append('p')
      .append('text')
      .text(JSON.stringify(obj));
      returnMessage.push(value);
    }
  }
  var to_publish = new ROSLIB.Message({data: returnMessage});
  publisher.publish(to_publish)
}

function resume(){
  /* 
  * Opens a new page that displays all the SV edited, and gives opportunity to user to add
  * synchronization constraints.
  */
  var editWindows = window.open('./resume.html','_blank');
      editWindows.addEventListener("load", function() {
        editWindows.document.title = 'Synchronization';
      });
}


function addSV2workspace(){
	/*
  * Adds current value in textbox to the list of State Variables needed to configuration.
  * Initializes the SV with empty values.
  * Pushing the button confirms that the value is correct and needs to be added 
  */
  //MAYBE WE CAN ADD CHECK ON INPUT
  var state = d3.select('#namesInput')
  	.select('#stateVar')
  	.property('value');

  d3.select('#workspace')
  	.append('a')
  	.attr('class', 'list-group-item')
  	.text(state)
  	.style('font-size', '18px');

  d3.select('#workspace')
	.selectAll('a')
	.on('click', function(){
		var data = d3.select(this);
    data.style('background-color','lightblue');
		var actionId = data.text();
		window.localStorage.setItem(actionId, JSON.stringify({stateVar: actionId, flag: 'unedited', states: [], links: []}));
      var editWindows = window.open('./edit.html','_blank');
      editWindows.addEventListener("load", function() {
        editWindows.document.title = actionId;
      });
	})
}
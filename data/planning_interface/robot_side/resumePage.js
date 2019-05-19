//Local memory usage
if (window.localStorage){
  //window.localStorage.clear();
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

//Defining the publisher to retrieve data to ROS
/************************************/
var publisher = new ROSLIB.Topic({
  ros : ros,
  name : '/config_robot_side_receiver',
  messageType : 'robot_knowledge_base/StringArray'
});

editedSV=[]; //contains all SV in objects, easy to access
allPossibleStates=[]; //All the state for synchronization
window.onload = function(){
  /* 
  * Displays all the SV edited to show the users which he edited. Then adds the possibility to create 
  * synchronization rules of type BEFORE, MEETS, DURING. 
  */
  for(var i=0, len=window.localStorage.length; i<len; i++) {
    var key = window.localStorage.key(i);
    if (key == 'feedback')
      continue;//if it is not a topic jump iteration
    var value = window.localStorage[key];
    //console.log(key, value ); OKOK
    var obj=JSON.parse(value);
    if(obj.flag=='edited'){
      /*d3.select('#edited')
      .append('a')
      .attr('class', 'list-group-item')
      .text(obj.stateVar)
      .style('font-size', '18px');*/
      editedSV.push(obj);
    }
  }
  d3.select('#edited').selectAll('ul')
  	.data(editedSV).enter()
  	.append('ul')
  	.text(function(d){return d.stateVar})
  	.selectAll('li')
  	.data(function(d){return d.links;}).enter()
  	.append('li')
  	.text(function(d){return (d.source+ ' => '+d.target+', topic: '+d.topic+', role: '+d.type)})
  	.style("background-color", function(d, i) { return i % 2 ? "#eee" : "#ddd"; });
  allPossibleStates = getAllStates(editedSV);
}

function getAllStates(svList){
	allStates = [];
	for (index in svList){
		var temp = svList[index];
		//console.log(temp);
		for (ind in temp.states){
			var state = temp.states[ind];
			//console.log(state);
			if (state.controlTag == 'u')
				allStates.push(temp.stateVar+' -> '+'_'+state.id); 
			if (state.controlTag == 'c')
				allStates.push(temp.stateVar+' -> '+state.id);
		}
	}
	return allStates;
}

/*****************************************************************************/
/****** This part contains functions for synchronization constraints *********/
//rules = d3.select('body').select('#synchronization');

function addBeforeSync(){
	var line = d3.select('body').select('#synchronization').append('p');
	
	var selectA = line.append('select')
						.attr('id','selA')
						.selectAll('option')
						.data(allPossibleStates).enter()
						.append('option')
						.text(function(d){return d;});

	var label = line.append('label')
							.attr('id','lab')
							.append('text')
							.text(' BEFORE ');

	var selectB = line.append('select')
						.attr('id','selB')
						.selectAll('option')
						.data(allPossibleStates).enter()
						.append('option')
						.text(function(d){return d;});

	var button = line.append('input')
		.attr('type','button')
    .attr('class','button')
    .style('background-color','#468499')
    .attr('value', 'Remove Rule')
    .on('click',function(d){
    	d3.select(this.parentNode).remove();
    });

  var confirmButton = line.append('input')
		.attr('type','button')
    .attr('class','button')
    .style('background-color','#468499')
    .attr('value', 'Add Rule')
    .on('click',function(d){
    	var color = d3.rgb(d3.select(this).style('background-color'));
    	if (color.r == 255 && color.g == 0 && color.b == 0)
    		return
    	var valA = d3.select(this.parentNode).select('#selA').property('value');
    	var valB = d3.select(this.parentNode).select('#selB').property('value');
    	var rule = valA+' BEFORE '+valB;
    	//console.log(rule);
    	d3.select(this).style('background-color','red');
    	addSyncRule(editedSV, rule);
    });

}

function addDuringSync(){
  var line = d3.select('body').select('#synchronization').append('p');
	
	var selectA = line.append('select')
						.attr('id','selA')
						.selectAll('option')
						.data(allPossibleStates).enter()
						.append('option')
						.text(function(d){return d;});
	var label = line.append('label')
							.attr('id','lab')
							.append('text')
							.text(' DURING ');
	var selectB = line.append('select')
						.attr('id','selB')
						.selectAll('option')
						.data(allPossibleStates).enter()
						.append('option')
						.text(function(d){return d;});
	var button = line.append('input')
		.attr('type','button')
    .attr('class','button')
    .style('background-color','#468499')
    .attr('value', 'Remove Rule')
    .on('click',function(d){
    	d3.select(this.parentNode).remove();
    });

    var confirmButton = line.append('input')
		.attr('type','button')
    .attr('class','button')
    .style('background-color','#468499')
    .attr('value', 'Add Rule')
    .on('click',function(d){
    	var color = d3.rgb(d3.select(this).style('background-color'));
    	if (color.r == 255 && color.g == 0 && color.b == 0)
    		return
    	var valA = d3.select(this.parentNode).select('#selA').property('value');
    	var valB = d3.select(this.parentNode).select('#selB').property('value');
    	var rule = valA+' DURING '+valB;
    	//console.log(rule);
    	d3.select(this).style('background-color','red');
    	addSyncRule(editedSV, rule);
    });

}

function addMeetsSync(){
  var line = d3.select('body').select('#synchronization').append('p');
	
	var selectA = line.append('select')
						.attr('id','selA')
						.selectAll('option')
						.data(allPossibleStates).enter()
						.append('option')
						.text(function(d){return d;});
	var label = line.append('label')
							.attr('id','lab')
							.append('text')
							.text(' MEETS ');
	var selectB = line.append('select')
						.attr('id','selB')
						.selectAll('option')
						.data(allPossibleStates).enter()
						.append('option')
						.text(function(d){return d;});

	var button = line.append('input')
		.attr('type','button')
    .attr('id','button')
    .style('background-color','#468499')
    .attr('value', 'Remove Rule')
    .on('click',function(d){
    	d3.select(this.parentNode).remove();
    });

    var confirmButton = line.append('input')
		.attr('type','button')
    .attr('class','button')
    .style('background-color','#468499')
    .attr('value', 'Add Rule')
    .on('click',function(d){
    	var color = d3.rgb(d3.select(this).style('background-color'));
    	if (color.r == 255 && color.g == 0 && color.b == 0)
    		return
    	var valA = d3.select(this.parentNode).select('#selA').property('value');
    	var valB = d3.select(this.parentNode).select('#selB').property('value');
    	var rule = valA+' MEETS '+valB;
    	d3.select(this).style('background-color','red');
    	//console.log(rule);
    	addSyncRule(editedSV, rule);
    });
}

function addSyncRule(sv, rule){
	/*
	* Reads all the State Variables and inserts the Synchronization rule
	* based on their first state.
	* @params: array of StateVar object and the sync rule.
	*/
	var elements = rule.split(' ');
	/*How is elements composed: ["ION", "->", "I", "BEFORE", "Akka", "->","_K"]
	 *elements[0] = SV Stato 1
	 *elements[2] = Stato 1
	 *elements[3] = Synchronization rule
	 *elements[4] = SV Stato 2
	 *elements[6] = Stato 2
	*/
	var svLabel = elements[0];
	for (index in sv){
		var temp = sv[index];
		if (temp.stateVar == svLabel)
			if (temp.hasOwnProperty('syncRule'))
				temp.syncRule.push(rule);
			else temp.syncRule = [rule];
	}
}

function sendData2DB(){
	/*
	* Reads the state variables and builds the StringArray to create and send
	* the ROS Message.
	*/
	returnMessage = [];
	//alert('Implementare azione');
	for (i in editedSV){
		var temp = editedSV[i];
		returnMessage.push(JSON.stringify(temp));
	}

	  var to_publish = new ROSLIB.Message({data: returnMessage});
	  alert('Data will be sent to MongoDB');
  	publisher.publish(to_publish);
}
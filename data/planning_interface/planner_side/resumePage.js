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

var publisher = new ROSLIB.Topic({
  ros : ros,
  name : '/config_plan_side_receiver',
  messageType : 'robot_knowledge_base/StringArray'
});

var usedStateVar = []; //the state var we want to send to the DB
var stateVars = []; //State var to save on MongoDB

window.onload = function(){
  /*
  * On loading displays all the possible State Variables. The user can choose the SV he needs and
  * add them to the final State Variables. If button is hit,it sends data to
  * @back_planner_side that will persist all final SV on the MongoDB
  */
	var stateVariables = JSON.parse(window.localStorage.getItem('stateVariables'));

	d3.select('body').select('#allStateVar')
  .selectAll('a')
	.data(stateVariables).enter()
	.append('a')
  .attr('class', 'list-group-item')
	.text(function(d){return d.stateVar})
  .style('font-size', '18px');

  d3.select('body').select('#allStateVar')
  .selectAll('a')
  .on('click', function(d){
    putOnSelected(removeSync(d));
  });

	function putOnSelected(data){
    console.log(data)
    usedStateVar.push(data);
    d3.select('body').select('#selectedStateVar')
      //.selectAll('a')
      //.data(data).enter()
      .append('a')
      .attr('class', 'list-group-item')
      .text(function(){return JSON.stringify(data)})
      .style('font-size', '18px');
  }

  function removeSync(sv){
    if (sv.hasOwnProperty('syncRule'))
      delete sv['syncRule'];
    return sv;
  }

  d3.select('body').append('div')
		.append('input')
		.attr('type','button')
    .attr('class','button')
    .style('background-color','#FF851B')
    .attr('value', 'Send data to python')
    .on('click', function(d){
      usedStateVar.forEach(function(sv){
        stateVars.push(JSON.stringify(sv));
      });
    	var to_publish = new ROSLIB.Message({data: stateVars});
      var resume = ''
      stateVars.forEach(function(data){
        var d = JSON.parse(data);
        resume += d.stateVar + ' states: '+d.states.length+' links: '+d.links.length;
      });
      if (confirm('Your data:\n'+resume)){
            publisher.publish(to_publish);
            alert('Data have been published.')
            } 
      else stateVars=[];
    });
}

/*****************************************************************************/
/****** This part contains functions for synchronization constraints *********/
//rules = d3.select('body').select('#synchronization');

//var allPossibleStates = getAllStates(usedStateVar); //To create Sync rules

function addBeforeSync(){
  var line = d3.select('body').select('#synchronization').append('p');
  var allPossibleStates = getAllStates(usedStateVar);
  
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
      addSyncRule(usedStateVar, rule);
    });

}

function addDuringSync(){
  var line = d3.select('body').select('#synchronization').append('p');
  var allPossibleStates = getAllStates(usedStateVar);
  
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
      addSyncRule(usedStateVar, rule);
    });

}

function addMeetsSync(){
  var line = d3.select('body').select('#synchronization').append('p');
  var allPossibleStates = getAllStates(usedStateVar);
  
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
      var rule = valA+' MEETS '+valB;
      d3.select(this).style('background-color','red');
      //console.log(rule);
      addSyncRule(usedStateVar, rule);
    });
}

/*MODIFICARE QUESTA FUNZIONE PER AGGIUNGERE I */
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

function getAllStates(svList){
  console.log(svList);
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
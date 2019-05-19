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
// Subscribing to a Topic
// ----------------------

var listener = new ROSLIB.Topic({
  ros : ros,
  name : '/config_plan_side_sender',
  messageType : 'robot_knowledge_base/StringArray'
});

listener.subscribe(function(message) {
  //If it is the first time we use the page
  if (window.localStorage.getItem('stateVariables')==null){
    var stateVarsList = extractSVfromMsg(message);
    window.localStorage.setItem('stateVariables',JSON.stringify(stateVarsList))//We save state variables on accessible memory
     var allStatesInfo = [];
    var allSV = JSON.parse(window.localStorage.getItem('stateVariables'));
    allSV.forEach(function(d){
      allStatesInfo = allStatesInfo.concat(d.states);
    })
    window.localStorage.setItem('allStatesInfo',JSON.stringify(allStatesInfo));
    window.localStorage.setItem('editedStateVar','[]')
    }
 
  robotSideConf();

  listener.unsubscribe();

});

//Defining the publisher to retrieve data to ROS
/************************************/
var publisher = new ROSLIB.Topic({
  ros : ros,
  name : '/kb_topic_receiver',
  messageType : 'robot_knowledge_base/StringArray'
});

//Working on received message
/************************************/

function extractSVfromMsg(message){
  /* 
  * Reads the message from ROS and creates object list. Each object represents a State Variable
  */
  stateVarList = []
  stateVars = message.data
  for (index in stateVars){
    var sv = JSON.parse(stateVars[index]);//Converts the string in json obj for a better handling 
    stateVarList.push(sv)
  }
  return stateVarList
}

function robotSideConf(){
  /*  
  * Creates buttons to select state variables; if clicked button disappears
  * Clicking on buttons draws the corresponding graph, using the function @drawInitialGraphs
  */
  var stateVarList = JSON.parse(window.localStorage.getItem('stateVariables'));
  var newStateVar = JSON.parse(window.localStorage.getItem('editedStateVar'));
  var buttonArea =  d3.select('body')
    .select('#super')
    .select('#panelContainer')
    .select('#buttonPan')

  var w = buttonArea.style('width');

  stateVarList = stateVarList.concat(newStateVar);
  var buttonPanel = buttonArea.selectAll('input')
    .data(stateVarList).enter()
    .append('input')
    .attr('type','button')
    .attr('class','button')
    .attr('id','stateVarButton')
    .attr('status','on')
    .style('width', w)
    .style('background-color','#FF851B')//color if State Var is still to edit
    .attr('value', function(d){return d.stateVar})
    .on('click',function(d){
      //console.log(d);
      convertingDictionary.push({key:d.stateVar, value:'', flag:'stateVar'});
      if (d3.select(this).attr('status') == 'on') {
        d3.select(this).attr('status','off');
        d3.select(this).style('visibility','hidden')
        //console.log(d) stampa le cose giuste
        drawInitialGraphs(d);
      }
    });
}

var initialLinks = [];
var initialNodes = {};
var convertingDictionary = []; //Contains the mapping needed to adapt sync rules to new State Variables
var modifiedSyncRule = []; //All the sync rule for the state Variables.

function drawInitialGraphs(sv){
  /*
  * Draws the graph using a Sticky Force Layout. Why sticky? To better handle and understand the graph
  */
  var allStatesInfo = JSON.parse(window.localStorage.getItem('allStatesInfo'));//Utilized to display states info

  if (sv.hasOwnProperty('syncRule'))
    modifiedSyncRule = modifiedSyncRule.concat(sv.syncRule)
  //console.log(modifiedSyncRule);
  var workArea = d3.select('#workingPan');
  var links = sv.links;
  var w = 870,
  h = 450;

  var svg = workArea.select("svg")
    .attr("width", w)
   .attr("height", h);
  
  links.forEach(function(link) {
    link.source = initialNodes[link.source] || (initialNodes[link.source] = {name: link.source});
    link.target = initialNodes[link.target] || (initialNodes[link.target] = {name: link.target});
  });
  initialLinks = initialLinks.concat(links);

  var force = d3.layout.force()
  .nodes(d3.values(initialNodes))
  .links(initialLinks)
  .size([w, h])
  .gravity(0.5)
  .linkDistance(200)
  .charge(-400)
  .on("tick", tick)
  .start();

  // build the arrow.
  svg.append("svg:defs").selectAll("marker")
    .data(["end"])      // Different link/path types can be defined here
  .enter().append("svg:marker")    // This section adds in the arrows
    .attr("id", String)
    .attr("viewBox", "0 -5 10 10")
    .attr("refX", 15)
    .attr("refY", -1.5)
    .attr("markerWidth", 6)
    .attr("markerHeight", 6)
    .attr("orient", "auto")
  .append("svg:path")
    .attr("d", "M0,-5L10,0L0,5");

  // add the links and the arrows
  var link = svg.append("svg:g").selectAll("path")
    .data(force.links());

  var node = svg.selectAll('.node')
  .data(d3.values(initialNodes));

  node.exit().remove();
  link.exit().remove();

  link.enter().append("svg:path")
    .attr("class", "link")
    .attr("marker-end", "url(#end)");

  //sticky force layout
  var drag = force.drag()
              .on('dragstart', function(d){
                d3.select(this).classed("fixed", d.fixed = true);
              })


  node.enter().append('g')
  .attr('class','node')
  .call(drag);

  node.append('circle')
  .style("fill", "red")
  .attr("r", 9);

  node.append('text')
  .attr("x", 25)
  .attr('dy','.80em')
  .text(function(d){ return d.name; });

  var tooltip = d3.select('#workingPan')
      .append('div')
      .attr('class','tooltip')
      .style('opacity',0);

  node.on('mouseover', function(d){
    var infos;
    allStatesInfo.forEach(function(s){
      if (d.name == s.id){
        infos = JSON.stringify(s);
      }
    })

    tooltip.html('<strong>Node Info</strong><br>'+infos)
      .style('top', d3.event.pageY - 2 + 'px')
      .style('left', d3.event.pageX + 2 + 'px')
      .style("opacity", 1);
  })
  .on('mouseout', function(d){
    tooltip.style('opacity',0);
  })

  link.append('text')
  .attr("x", 25)
  .attr('dy','.80em')
  .text(function(d){ return d.topic; });

  /*link.on('mouseover', function(d) {
    d3.select('#topicEdge').property('value',d.topic)
  });*/

  function tick() {

    link
    .attr("d", function(d) {
        var dx = d.target.x - d.source.x,
            dy = d.target.y - d.source.y,
            dr = Math.sqrt(dx * dx + dy * dy);
        return "M" + 
            d.source.x + "," + 
            d.source.y + "A" + 
            dr + "," + dr + " 0 0,1 " + 
            d.target.x + "," + 
            d.target.y;
    });
    node
        .attr("transform", function(d) { 
        return "translate(" + d.x + "," + d.y + ")"; });
  }
}

/*
* These 2 functions were used to modify names on merging, preventing conflicts due to 2 same names.
* In the last version we advice to NOT give states same names. Thats more convenient than modify.

function updateStateInfo(id, newId){
  
  var allStates = JSON.parse(window.localStorage.getItem('allStatesInfo'));
  console.log(allStates);
  var state = allStates.find(function(el){return el.id==id});
  state.id = newId;
  allStates.push(state);
  window.localStorage.setItem('allStatesInfo',JSON.stringify(allStates));

}
function updateNameVersion(id){
  var newId = ''
  newId = id + id.length
  updateStateInfo(id, newId);
  return newId;
}
*/

/*********************************************************/
/***************** IMPLEMENTATION OF OPERATIONS ************/
var modifiedLinks = []; //saves the links to modify
var nodesToMerge;
function mergeOperation(){
  /*
  * Prepares the form to get information on the merging.
  */

  nodesToMerge = []
  var div = d3.select('#operations')
      .append('h3')
      .text('Click on Node to Merge');

      div.append('input')
      .attr('id', 'newStateName')
      .attr('type','text')
      .attr('placeholder', 'New node name')
      .attr('class','col-md-3');

      div.append('input')
      .attr('id', 'newSVName')
      .attr('type','text')
      .attr('placeholder', 'New state variable name')
      .attr('class','col-md-3');

      div.append('input')
      .attr('type','button')
      .attr('class','button col-md-3')
      .attr('value', 'Do Merging')
      .style('background-color','#ff6f69')
      .on('click', activateMerging);

  d3.selectAll('circle')
    .on('click',function(d){
      var color = d3.rgb(d3.select(this).style('fill'));
      if (color.r == 255){
        d3.select(this).style('fill','blue');
        nodesToMerge.push(d.name);
       //Can merge no more than 2 nodes
        }  
      if (color.b == 255){
        d3.select(this).style('fill','red');
        nodesToMerge = nodesToMerge.filter(function(el){ return el!=d.name})
        }
      }); 
}
function activateMerging(){
  /*
  * Performs merging, changing adequatelly all the parameters we need to change.
  * If list of nodes contains less than 2, alerts the error.
  */
    if (nodesToMerge.length<2){
      alert('You have not enough nodes for merging. Please, select more');
      return
    }

    var tempLinks = initialLinks.slice();
    var newNodeName = d3.select('#newStateName')
    .property('value');
    var newSVName = d3.select('#newSVName')
    .property('value');

    //If enough nodes then add them to the dictionary for later conversion of sync rules
    for (index in nodesToMerge)
      convertingDictionary.push({key: nodesToMerge[index], value:newNodeName, flag:'state'})

    for (index in convertingDictionary){
      var temp = convertingDictionary[index];
      if (temp.flag == 'stateVar')
        temp.value = newSVName;
    }

    for (index in tempLinks){
      var link = tempLinks[index]; 

      var src = link.source.name,
          tgt = link.target.name;
      
      /* Still code for previous version.Reccomend not to use it
      var src2 = updateNameVersion(src),
          tgt2 = updateNameVersion(tgt);
          */
      if (nodesToMerge.indexOf(src) >= 0 ){
        modifiedLinks.push({source: newNodeName, target: tgt, topic: link.topic, message: link.message, type:link.type});
      }
      if (nodesToMerge.indexOf(tgt) >= 0 ){
        modifiedLinks.push({source: src, target: newNodeName, topic: link.topic, message: link.message, type:link.type})
        }
      if (nodesToMerge.indexOf(tgt) < 0 && nodesToMerge.indexOf(src) < 0 ) 
        modifiedLinks.push({source: src, target: tgt, topic: link.topic, message: link.message, type:link.type});
      }

      //Saving in memory data for drawing the new state variable and all its data
      var newStateVar = JSON.parse(window.localStorage.getItem('newStateVar'));
      var obj = {stateVar: newSVName, links: modifiedLinks, flag: 'unedited', syncRule: modifiedSyncRule};
      window.localStorage.setItem('newStateVar',JSON.stringify(obj));
      window.localStorage.setItem('conversionDict',JSON.stringify(convertingDictionary));
      //Reset these variables to modify for other possible changing
      modifiedLinks = [];
      modifiedSyncRule = []; 
      convertingDictionary = [];
      window.open('./edit.html','_blank');
}

function resetStateVariables(){
  /*
  * Deletes all the SV computed. Restarts from connecting to ROS
  */
  window.localStorage.clear();
  window.location.reload();
}

function reloadPanel(){
  /*
  * Reloads panel inserting the new edited state variable
  */
  d3.select('#workingPan').selectAll('*').remove();
  d3.select('#workingPan').append('svg');
  d3.select('#operations').selectAll('*').remove();
  initialLinks = [];
  initialNodes = {};
  robotSideConf();
}

function resumeVariables(){
  /*
  * Opens a page that presents all the SV edited at their final state. These SV will be saved 
  * on MongoDB and used to build the ddl file.
  */
  window.open('./resume.html','_blank');
}

function printAll(list){
  for (index in list)
    console.log(list[index]);
}